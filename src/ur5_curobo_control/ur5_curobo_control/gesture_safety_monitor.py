#!/usr/bin/env python3
"""
Gesture-based Safety Monitor Node

Uses MediaPipe Gesture Recognition to control robot safety state:
- STOP gestures (safety_triggered=True): Open Palm, Pointing Up
- RESUME gestures (safety_triggered=False): Thumbs Up, Victory

Publishes to /human_safety topic for integration with curobo_control_node
and program_executor_node.

MediaPipe recognizes these gestures:
- Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou, None
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import cv2
import numpy as np
import os

# MediaPipe imports
try:
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    from mediapipe.framework.formats import landmark_pb2
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False
    print("WARNING: mediapipe not installed. Run: pip install mediapipe")

# Optional RealSense support
try:
    import pyrealsense2 as rs
    HAS_REALSENSE_LIB = True
except ImportError:
    HAS_REALSENSE_LIB = False


class GestureSafetyMonitor(Node):
    """ROS 2 node for gesture-based safety monitoring."""
    
    # Gestures that trigger safety stop
    STOP_GESTURES = {'Open_Palm', 'Pointing_Up'}
    
    # Gestures that clear safety (resume operation)
    RESUME_GESTURES = {'Thumb_Up', 'Victory'}
    
    def __init__(self):
        super().__init__('gesture_safety_monitor')
        
        if not HAS_MEDIAPIPE:
            self.get_logger().error("MediaPipe not installed! Run: pip install mediapipe")
            raise RuntimeError("MediaPipe required for gesture recognition")
        
        # Publisher for safety status (same topic as face_safety_monitor)
        self.publisher_ = self.create_publisher(Bool, '/human_safety', 10)
        
        # Publisher for detected gesture (for debugging/visualization)
        self.gesture_pub = self.create_publisher(String, '~/detected_gesture', 10)
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('model_path', '')  # Path to gesture_recognizer.task
        self.declare_parameter('use_realsense', False)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('num_hands', 2)
        self.declare_parameter('gesture_hold_frames', 3)  # Frames to hold gesture before triggering
        
        self.camera_id = self.get_parameter('camera_id').value
        model_path = self.get_parameter('model_path').value
        self.use_realsense = self.get_parameter('use_realsense').value
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self.num_hands = self.get_parameter('num_hands').value
        self.gesture_hold_frames = self.get_parameter('gesture_hold_frames').value
        
        self.get_logger().info("Initializing Gesture Safety Monitor...")
        self.get_logger().info(f"Camera ID: {self.camera_id}")
        self.get_logger().info(f"Use RealSense: {self.use_realsense}")
        self.get_logger().info(f"Stop gestures: {self.STOP_GESTURES}")
        self.get_logger().info(f"Resume gestures: {self.RESUME_GESTURES}")
        
        # Find or download the model
        if not model_path:
            # Check common locations
            possible_paths = [
                os.path.expanduser('~/.mediapipe/gesture_recognizer.task'),
                os.path.join(os.path.dirname(__file__), 'gesture_recognizer.task'),
                '/tmp/gesture_recognizer.task',
                'gesture_recognizer.task',
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    model_path = path
                    break
            
            if not model_path:
                # Download the model
                model_path = self._download_model()
        
        self.get_logger().info(f"Model path: {model_path}")
        
        # Initialize MediaPipe Gesture Recognizer
        try:
            base_options = python.BaseOptions(model_asset_path=model_path)
            options = vision.GestureRecognizerOptions(
                base_options=base_options,
                num_hands=self.num_hands,
                min_hand_detection_confidence=self.min_detection_confidence,
                min_tracking_confidence=self.min_tracking_confidence,
            )
            self.recognizer = vision.GestureRecognizer.create_from_options(options)
            self.get_logger().info("MediaPipe Gesture Recognizer initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize gesture recognizer: {e}")
            raise e
        
        # Initialize Camera
        self.pipeline = None
        self.cap = None
        self._init_camera()
        
        # MediaPipe drawing utilities
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # State tracking
        self.safety_triggered = False
        self.gesture_counter = {}  # Track consecutive frames for each gesture
        self.last_gesture = "None"
        
        # Timer for processing loop (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Gesture Safety Monitor ready!")
        self.get_logger().info("Show 'Open Palm' or 'Pointing Up' to STOP robot")
        self.get_logger().info("Show 'Thumbs Up' or 'Victory' to RESUME robot")
    
    def _download_model(self) -> str:
        """Download the gesture recognizer model if not found."""
        import urllib.request
        
        model_url = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
        model_dir = os.path.expanduser('~/.mediapipe')
        model_path = os.path.join(model_dir, 'gesture_recognizer.task')
        
        os.makedirs(model_dir, exist_ok=True)
        
        self.get_logger().info(f"Downloading gesture recognizer model to {model_path}...")
        try:
            urllib.request.urlretrieve(model_url, model_path)
            self.get_logger().info("Model downloaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to download model: {e}")
            raise RuntimeError(f"Could not download gesture recognizer model: {e}")
        
        return model_path
    
    def _init_camera(self):
        """Initialize camera (RealSense or standard webcam)."""
        if self.use_realsense:
            if HAS_REALSENSE_LIB:
                try:
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    self.pipeline.start(config)
                    self.get_logger().info("Intel RealSense initialized successfully.")
                    return
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize RealSense: {e}")
                    self.get_logger().warn("Falling back to standard webcam...")
                    self.use_realsense = False
            else:
                self.get_logger().warn("pyrealsense2 library not found. Using standard webcam...")
                self.use_realsense = False
        
        # Standard webcam
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {self.camera_id}")
            raise RuntimeError(f"Could not open video device {self.camera_id}")
        self.get_logger().info(f"Standard webcam (ID {self.camera_id}) initialized.")
    
    def timer_callback(self):
        """Main processing loop."""
        frame = self._capture_frame()
        if frame is None:
            return
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Ensure the array is contiguous for MediaPipe
        rgb_frame = np.ascontiguousarray(rgb_frame)
        mp_image = mp.Image(mp.ImageFormat.SRGB, rgb_frame)
        
        # Run gesture recognition
        try:
            result = self.recognizer.recognize(mp_image)
        except Exception as e:
            self.get_logger().warn(f"Recognition error: {e}")
            return
        
        # Process results
        detected_gesture = "None"
        gesture_score = 0.0
        
        if result.gestures and len(result.gestures) > 0:
            # Get the top gesture from the first hand
            top_gesture = result.gestures[0][0]
            detected_gesture = top_gesture.category_name
            gesture_score = top_gesture.score
        
        # Update gesture counter for debouncing
        if detected_gesture != "None":
            self.gesture_counter[detected_gesture] = self.gesture_counter.get(detected_gesture, 0) + 1
            # Reset other gesture counters
            for g in list(self.gesture_counter.keys()):
                if g != detected_gesture:
                    self.gesture_counter[g] = 0
        else:
            # Decay all counters
            for g in list(self.gesture_counter.keys()):
                self.gesture_counter[g] = max(0, self.gesture_counter[g] - 1)
        
        # Check for stable gesture (held for enough frames)
        stable_gesture = None
        for gesture, count in self.gesture_counter.items():
            if count >= self.gesture_hold_frames:
                stable_gesture = gesture
                break
        
        # Update safety state based on stable gesture
        prev_state = self.safety_triggered
        
        if stable_gesture in self.STOP_GESTURES:
            self.safety_triggered = True
            if not prev_state:
                self.get_logger().warn(f"SAFETY TRIGGERED! Gesture: {stable_gesture}")
        elif stable_gesture in self.RESUME_GESTURES:
            self.safety_triggered = False
            if prev_state:
                self.get_logger().info(f"SAFETY CLEARED! Gesture: {stable_gesture}")
        
        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = self.safety_triggered
        self.publisher_.publish(safety_msg)
        
        # Publish detected gesture
        gesture_msg = String()
        gesture_msg.data = f"{detected_gesture} ({gesture_score:.2f})"
        self.gesture_pub.publish(gesture_msg)
        
        # Draw visualization
        annotated_frame = self._draw_results(frame, result, detected_gesture, gesture_score)
        
        # Display
        cv2.imshow("Gesture Safety Monitor", annotated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quit requested")
            rclpy.shutdown()
    
    def _capture_frame(self):
        """Capture a frame from the camera."""
        if self.use_realsense:
            try:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    return None
                return np.asanyarray(color_frame.get_data())
            except Exception as e:
                self.get_logger().warn(f"RealSense capture error: {e}")
                return None
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                return None
            return frame
    
    def _draw_results(self, frame, result, gesture_name, gesture_score):
        """Draw gesture recognition results on frame."""
        annotated_frame = frame.copy()
        
        # Draw hand landmarks if available
        if result.hand_landmarks:
            for hand_landmarks in result.hand_landmarks:
                # Convert to proto format for drawing
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend([
                    landmark_pb2.NormalizedLandmark(
                        x=landmark.x, 
                        y=landmark.y, 
                        z=landmark.z
                    ) for landmark in hand_landmarks
                ])
                
                self.mp_drawing.draw_landmarks(
                    annotated_frame,
                    hand_landmarks_proto,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
        
        # Draw status bar at top
        color = (0, 0, 255) if self.safety_triggered else (0, 255, 0)
        status_text = "STOP" if self.safety_triggered else "SAFE"
        
        # Draw background rectangle for text
        cv2.rectangle(annotated_frame, (0, 0), (annotated_frame.shape[1], 60), (0, 0, 0), -1)
        
        # Status
        cv2.putText(annotated_frame, f"Status: {status_text}", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        # Detected gesture
        cv2.putText(annotated_frame, f"Gesture: {gesture_name} ({gesture_score:.2f})", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Instructions at bottom
        h = annotated_frame.shape[0]
        cv2.rectangle(annotated_frame, (0, h-50), (annotated_frame.shape[1], h), (0, 0, 0), -1)
        cv2.putText(annotated_frame, "STOP: Open Palm / Point Up  |  RESUME: Thumbs Up / Victory", 
                   (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return annotated_frame
    
    def __del__(self):
        """Cleanup resources."""
        if hasattr(self, 'cap') and self.cap and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GestureSafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
