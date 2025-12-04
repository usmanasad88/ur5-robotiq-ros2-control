#!/home/rml/miniconda3/envs/ur5_python/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import numpy as np
from ultralytics import YOLO

try:
    import pyrealsense2 as rs
    HAS_REALSENSE_LIB = True
except ImportError:
    HAS_REALSENSE_LIB = False

class FaceSafetyMonitor(Node):
    def __init__(self):
        super().__init__('face_safety_monitor')
        
        # Publisher for safety status
        self.publisher_ = self.create_publisher(Bool, '/human_safety', 10)
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('threshold_area', 0.15) # Fraction of screen area
        self.declare_parameter('model_type', 'yolov8n.pt') # Standard YOLOv8 model
        self.declare_parameter('use_realsense', False) # Use Intel RealSense if available
        self.declare_parameter('depth_threshold', 2.7) # Safety distance in meters
        
        self.camera_id = self.get_parameter('camera_id').value
        self.threshold_area = self.get_parameter('threshold_area').value
        model_type = self.get_parameter('model_type').value
        self.use_realsense = self.get_parameter('use_realsense').value
        self.depth_threshold = self.get_parameter('depth_threshold').value
        
        self.get_logger().info(f"Initializing Face/Person Safety Monitor...")
        self.get_logger().info(f"Camera ID: {self.camera_id}")
        self.get_logger().info(f"Safety Threshold (Area): {self.threshold_area}")
        self.get_logger().info(f"Use RealSense: {self.use_realsense}")
        
        # Load YOLO model
        # Note: 'yolov8n.pt' detects 'person' (class 0). 
        # For specific face detection, you would need a face-trained model like 'yolov8n-face.pt'
        try:
            self.model = YOLO(model_type)
            self.get_logger().info(f"Model {model_type} loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise e

        # Initialize Camera (RealSense or Webcam)
        self.pipeline = None
        self.cap = None
        
        if self.use_realsense:
            if HAS_REALSENSE_LIB:
                try:
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    self.pipeline.start(config)
                    self.align = rs.align(rs.stream.color)
                    self.get_logger().info("Intel RealSense initialized successfully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize RealSense: {e}")
                    self.get_logger().warn("Falling back to standard webcam...")
                    self.use_realsense = False
            else:
                self.get_logger().warn("pyrealsense2 library not found. Falling back to standard webcam...")
                self.use_realsense = False
        
        if not self.use_realsense:
            # Open Standard Webcam
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                self.get_logger().error("Could not open video device")
                raise RuntimeError("Could not open video device")
            self.get_logger().info(f"Standard webcam (ID {self.camera_id}) initialized.")

        # Timer for processing loop
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        
        self.safety_triggered = False

    def timer_callback(self):
        frame = None
        depth_frame = None
        
        # Capture Frame
        if self.use_realsense:
            try:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame_rs = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame_rs:
                    return
                    
                frame = np.asanyarray(color_frame.get_data())
                depth_frame = depth_frame_rs # Keep RS object for distance queries
            except Exception as e:
                self.get_logger().warn(f"RealSense capture error: {e}")
                return
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                return

        # Run inference
        # class 0 is 'person' in COCO dataset used by standard YOLOv8
        results = self.model(frame, classes=[0], verbose=False) 
        
        annotated_frame = results[0].plot()
        
        should_stop = False
        min_distance = float('inf')
        max_area_ratio = 0.0
        
        height, width, _ = frame.shape
        frame_area = height * width
        
        # Check bounding boxes
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # box.xyxy is [x1, y1, x2, y2]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Calculate Area Ratio (Fallback safety)
                box_area = (x2 - x1) * (y2 - y1)
                ratio = box_area / frame_area
                if ratio > max_area_ratio:
                    max_area_ratio = ratio
                
                # Calculate Depth (Primary safety if RealSense)
                if self.use_realsense and depth_frame:
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    # Ensure within bounds
                    cx = min(max(0, cx), width - 1)
                    cy = min(max(0, cy), height - 1)
                    
                    # Get distance at center
                    dist = depth_frame.get_distance(cx, cy)
                    
                    # Filter invalid 0.0 readings (try a small patch if needed)
                    if dist > 0 and dist < min_distance:
                        min_distance = dist
                        
                    # Draw distance on frame
                    cv2.putText(annotated_frame, f"{dist:.2f}m", (int(x1), int(y1)-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Determine safety status
        if self.use_realsense:
            # Trigger if ANY person is FURTHER than threshold (or not detected)
            if min_distance > self.depth_threshold:
                should_stop = True
                self.get_logger().info(f"Depth Safety: Min Dist {min_distance:.2f}m > {self.depth_threshold}m")
        else:
            # Trigger if area ratio exceeds threshold
            if max_area_ratio > self.threshold_area:
                should_stop = True
                self.get_logger().info(f"Area Safety: Ratio {max_area_ratio:.2f} > {self.threshold_area}")
        
        if should_stop and not self.safety_triggered:
            self.get_logger().warn(f"SAFETY TRIGGERED! Stopping robot.")
            self.safety_triggered = True
        elif not should_stop and self.safety_triggered:
            self.get_logger().info("Safety Reset. Area clear.")
            self.safety_triggered = False
            
        # Publish status
        msg = Bool()
        msg.data = self.safety_triggered
        self.publisher_.publish(msg)
        
        # Display feed
        color = (0, 0, 255) if self.safety_triggered else (0, 255, 0)
        text = "STOP" if self.safety_triggered else "SAFE"
        
        info_text = f"Status: {text}"
        if self.use_realsense:
            info_text += f" (Dist: {min_distance:.2f}m)" if min_distance != float('inf') else " (No Target)"
        else:
            info_text += f" (Area: {max_area_ratio:.2f})"
            
        cv2.putText(annotated_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        cv2.imshow("Safety Monitor", annotated_frame)
        cv2.waitKey(1)

    def __del__(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = FaceSafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
