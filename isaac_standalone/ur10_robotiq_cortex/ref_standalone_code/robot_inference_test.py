"""
Standalone GR00T Inference Client Test Script

This script demonstrates how to send observations to a running GR00T inference server
and receive actions, without depending on the gr00t package.

PREREQUISITE: Start the inference server in another terminal:
    python scripts/inference_service.py --server --embodiment-tag oxe_droid --data-config oxe_droid

This script can be used as a template for external clients that need to interface
with the GR00T inference server from outside this repository.
"""

import io
import numpy as np
import msgpack
import zmq

"""
REFERENCE FRAME DOCUMENTATION FOR OXE_DROID EMBODIMENT

Based on the pretrained model metadata, here are the reference frame specifications:

STATE (Observations you provide):
- state.eef_position: ABSOLUTE position in robot base frame
  - Shape: (1, 3) - [x, y, z] in meters
  - "absolute": true means coordinates are relative to robot base, NOT deltas
  
- state.eef_rotation: ABSOLUTE orientation in robot base frame  
  - Shape: (1, 3) - [roll, pitch, yaw] in radians (Euler angles RPY)
  - "absolute": true means actual orientation, NOT relative to previous
  - rotation_type: "euler_angles_rpy"
  
- state.gripper_position: ABSOLUTE gripper state
  - Shape: (1, 1) - 0.0 = fully open, 1.0 = fully closed
  - "absolute": true

ACTIONS (Model outputs):
- action.eef_position_delta: RELATIVE position change
  - Shape: (16, 3) - [dx, dy, dz] in meters
  - "absolute": false means these are DELTAS to apply to current position
  - Apply as: new_pos = current_pos + delta
  
- action.eef_rotation_delta: RELATIVE rotation change
  - Shape: (16, 3) - axis-angle representation in radians
  - "absolute": false means this is a DELTA rotation
  - Apply by composing with current rotation (see RotationHelper in ur5_robot_control.py)
  - Note: rotation_type in metadata says "euler_angles_rpy" but actually outputs axis-angle
  
- action.gripper_position: ABSOLUTE gripper command
  - Shape: (16, 1) - 0.0 = open, 1.0 = closed
  - "absolute": true means direct gripper state command

IMPORTANT COORDINATE FRAME NOTES:
1. Robot Base Frame: All absolute positions/orientations are w.r.t. robot base
   - Origin: Robot base mounting point
   - X-axis: Typically forward
   - Y-axis: Typically left  
   - Z-axis: Typically up
   - Convention may vary by robot manufacturer

2. For UR5 specifically:
   - Base frame origin is at the center of the robot base
   - Right-handed coordinate system
   - You should use UR5's get_pose() or forward kinematics for current EEF state
   
3. Deltas are in the SAME frame as absolute coordinates (robot base frame)

EXAMPLE USAGE:
  # Get current state from robot
  current_pos, current_euler = ur5.get_ee_pose()  # Both in base frame
  
  # Send to policy
  obs = {
      "state.eef_position": current_pos.reshape(1, 3),      # Absolute base frame
      "state.eef_rotation": current_euler.reshape(1, 3),    # Absolute base frame
      ...
  }
  
  # Get action
  action = policy.get_action(obs)
  
  # Apply delta (still in base frame)
  target_pos = current_pos + action["action.eef_position_delta"][0]
  target_euler = apply_rotation_delta(current_euler, action["action.eef_rotation_delta"][0])
"""


class StandaloneInferenceClient:
    """
    Standalone client for GR00T inference server.
    
    This client uses ZMQ and msgpack to communicate with the server without
    requiring any gr00t package imports. It can be copied to external projects.
    """
    
    def __init__(self, host="localhost", port=5555, api_token=None):
        self.host = host
        self.port = port
        self.api_token = api_token
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{host}:{port}")
        
    @staticmethod
    def _encode_data(obj):
        """Encode custom objects (numpy arrays) for msgpack."""
        if isinstance(obj, np.ndarray):
            output = io.BytesIO()
            np.save(output, obj, allow_pickle=False)
            return {"__ndarray_class__": True, "as_npy": output.getvalue()}
        return obj
    
    @staticmethod
    def _decode_data(obj):
        """Decode custom objects (numpy arrays) from msgpack."""
        if "__ndarray_class__" in obj:
            return np.load(io.BytesIO(obj["as_npy"]), allow_pickle=False)
        return obj
    
    def _send_request(self, endpoint, data=None, requires_input=True):
        """Send a request to the server and return the response."""
        request = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token
            
        # Serialize and send
        message = msgpack.packb(request, default=self._encode_data)
        self.socket.send(message)
        
        # Receive and deserialize
        response_bytes = self.socket.recv()
        response = msgpack.unpackb(response_bytes, object_hook=self._decode_data)
        
        if "error" in response:
            raise RuntimeError(f"Server error: {response['error']}")
        
        return response
    
    def get_action(self, observations):
        """
        Get action from the inference server.
        
        Args:
            observations (dict): Dictionary containing:
                - Video observations as numpy arrays (H, W, C) uint8
                - State observations as numpy arrays
                - Language instructions as numpy arrays of strings
                
        Returns:
            dict: Action dictionary with numpy arrays
        """
        return self._send_request("get_action", observations)
    
    def ping(self):
        """Ping the server to check if it's running."""
        try:
            response = self._send_request("ping", requires_input=False)
            return response.get("status") == "ok"
        except Exception:
            return False
    
    def close(self):
        """Clean up resources."""
        self.socket.close()
        self.context.term()


# Create client instance
print("Connecting to GR00T inference server...")
client = StandaloneInferenceClient(host="localhost", port=5555)

# Test connection
if not client.ping():
    print("ERROR: Cannot connect to inference server!")
    print("Please start the server first:")
    print("  python scripts/inference_service.py --server --embodiment-tag oxe_droid --data-config oxe_droid")
    exit(1)

print("✓ Connected to server!")
print()

# Example observation for OXE_DROID (batch size 1, time horizon 1)
obs = {
    # videos: shapes (1, H, W, C) uint8. The model will resize to 224x224 on server-side transforms.
    "video.exterior_image_1": np.zeros((1, 256, 256, 3), dtype=np.uint8),
    "video.exterior_image_2": np.zeros((1, 256, 256, 3), dtype=np.uint8),
    "video.wrist_image": np.zeros((1, 256, 256, 3), dtype=np.uint8),

    # state: end-effector position (1, 3)
    "state.eef_position": np.array([[0.5, 0.0, 0.2]], dtype=np.float32),

    # state: end-effector rotation as EULER ANGLES (roll, pitch, yaw) in RADIANS - shape (1, 3)
    # This is the format expected by the OXE_DROID pretrained checkpoint
    # Identity rotation (no rotation):
    "state.eef_rotation": np.array([[0.0, 0.0, 0.0]], dtype=np.float32),

    # gripper state (1,1) : 0.0 = open, 1.0 = closed
    "state.gripper_position": np.array([[0.0]], dtype=np.float32),

    # Optional language instruction(s)
    "annotation.language.language_instruction": np.array(["pick up the red cube"])
}

action = client.get_action(obs)
print("Action keys:", action.keys())
print("\nAction shapes:")
for key, value in action.items():
    print(f"  {key}: {value.shape}")

print("\n" + "="*80)
print("ACTION HORIZON EXPLAINED")
print("="*80)

# Show first few timesteps
print("\nFirst 5 timesteps of position delta (dx, dy, dz in meters):")
for i in range(min(5, len(action["action.eef_position_delta"]))):
    pos_delta = action["action.eef_position_delta"][i]
    print(f"  Step {i}: [{pos_delta[0]:+.6f}, {pos_delta[1]:+.6f}, {pos_delta[2]:+.6f}]")

print("\nFirst 5 timesteps of gripper command (0=open, 1=closed):")
for i in range(min(5, len(action["action.gripper_position"]))):
    gripper = action["action.gripper_position"][i, 0]
    print(f"  Step {i}: {gripper:.3f}")

print("\n" + "="*80)
print("HOW TO USE")
print("="*80)
print("""
Shape (16, 3) means:
  - 16 timesteps into the future
  - 3 values per timestep (e.g., dx, dy, dz for position)

Common approaches:
  1. RECEDING HORIZON (most common):
     - Execute only step 0 (first timestep)
     - Re-query policy for next action
     - Repeat
     
  2. MULTI-STEP EXECUTION:
     - Execute steps 0-3 (first 4 timesteps)
     - Re-query policy
     - Provides smoother motion but less reactive
     
  3. TEMPORAL SMOOTHING:
     - Use predictions from multiple timesteps to smooth trajectory
     - Helps reduce jitter

Example code for step 0 only:
""")
print("  pos_delta = action['action.eef_position_delta'][0]  # Shape: (3,)")
print("  rot_delta = action['action.eef_rotation_delta'][0]  # Shape: (3,)")
print("  gripper = action['action.gripper_position'][0, 0]   # Scalar")
print("\n" + "="*80)

# Cleanup
client.close()