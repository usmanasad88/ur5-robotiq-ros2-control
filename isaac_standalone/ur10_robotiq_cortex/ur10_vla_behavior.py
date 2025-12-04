# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
VLA (Vision Language Action) behavior for UR10 with Robotiq gripper.
Receives joint action targets from a VLA model (e.g., Octo) via HTTP and moves the end-effector
relative to the current position.

The VLA provides actions in the format:
{
    "joints": {
        "x": dx, "y": dy, "z": dz,           # Translation deltas in meters
        "roll": droll, "pitch": dpitch, "yaw": dyaw,  # Rotation deltas in radians
        "gripper": gripper_action           # Gripper command (0-1 or open/close)
    }
}
"""

import json
import numpy as np
import requests
import zmq
import msgpack
from typing import Any, Dict, Optional, Tuple
from collections import OrderedDict
import tempfile
import os
import time
import io

from isaacsim.cortex.framework.df import (
    DfDecider,
    DfDecision,
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
)
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from isaacsim.cortex.framework.motion_commander import MotionCommand, PosePq
import isaacsim.cortex.framework.math_util as math_util

# Mock setuptools_scm if missing to allow curobo import from source
try:
    import setuptools_scm
except ImportError:
    import sys
    from unittest.mock import MagicMock
    mock_scm = MagicMock()
    mock_scm.get_version.return_value = "0.0.0-dev"
    sys.modules["setuptools_scm"] = mock_scm

# Configuration for Motion Planning
USE_CUROBO = True  # Set to True to use Curobo for motion generation

# Curobo imports
try:
    import torch
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.util.logger import setup_curobo_logger
    from omni.isaac.core.utils.types import ArticulationAction
    CUROBO_AVAILABLE = True
except Exception as e:
    CUROBO_AVAILABLE = False
    print(f"[VLA] Curobo not available: {e}")
    if USE_CUROBO:
        raise e

# Global camera instance (lazily initialized)
_vla_camera = None

# Global VLA context reference for dynamic task updates
_vla_context = None

# Model configurations: Different VLA models have different rate limits and latency
VLA_MODEL_CONFIGS = {
    "octo": {
        "timeout": 4.0,           # Fast local inference
        "min_call_interval": 4,  # 10 requests/sec (600 req/min)
        "description": "Octo (local, fast)"
    },
    "gemini-flash": {
        "timeout": 10.0,          # Cloud API with moderate latency
        "min_call_interval": 5.0,  # 12 requests/min (under 15 req/min limit)
        "description": "Gemini 2.0 Flash (cloud, 15 req/min limit)"
    },
    "gemini-pro": {
        "timeout": 10.0,          # Cloud API with higher latency
        "min_call_interval": 10.0, # 2 requests/min (very conservative)
        "description": "Gemini Pro (cloud, expensive, slow)"
    },
    "gr00t": {
        "timeout": 5.0,           # Local inference server
        "min_call_interval": 0.1, # Fast inference, ~10 Hz
        "description": "GR00T (local, pretrained policy)"
    }
}

# Default model to use (can be changed)
DEFAULT_VLA_MODEL = "gemini-pro"

# HARDCODED CONFIGURATION - Change these values as needed
VLA_SERVER_URL = "http://localhost:5000"
VLA_MODEL = "gr00t"  # Options: "octo", "gemini-flash", "gemini-pro", "gr00t"

# GR00T USAGE:
# 1. Start the GR00T inference server first:
#    python scripts/inference_service.py --server --embodiment-tag oxe_droid --data-config oxe_droid
# 2. Set VLA_MODEL = "gr00t" above
# 3. VLA_SERVER_URL should point to the host (e.g., "localhost" or "192.168.1.100")
#    The GR00T client will connect via ZMQ on port 5555


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions (scalar-first format [w, x, y, z]).
    Result = q1 * q2 (applies q1 then q2 rotation)
    
    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]
    
    Returns:
        Result quaternion [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def transform_gripper_to_world_frame(delta_gripper: np.ndarray, gripper_quat: np.ndarray) -> np.ndarray:
    """
    Transform a position delta from gripper-centric frame to world frame.
    
    For Octo model, actions are ego-centric (relative to the gripper):
    - +delta_x: Forward along gripper's forward axis
    - +delta_y: Left along gripper's left axis  
    - +delta_z: Up along gripper's up axis
    
    Args:
        delta_gripper: Position delta in gripper frame [dx, dy, dz]
        gripper_quat: Current gripper orientation quaternion [w, x, y, z]
    
    Returns:
        Position delta in world frame [dx_world, dy_world, dz_world]
    """
    # Convert quaternion to rotation matrix
    from isaacsim.core.utils.rotations import quat_to_rot_matrix
    
    # Get rotation matrix from gripper quaternion
    R = quat_to_rot_matrix(gripper_quat)
    
    # Transform the delta from gripper frame to world frame
    delta_world = R @ delta_gripper
    
    return delta_world


def capture_viewport_frame(viewport_name: str = "Viewport") -> Optional[np.ndarray]:
    """
    Capture RGB frame from a camera in the scene (third-person perspective).
    Creates a camera on first call and reuses it for subsequent captures.
    
    Args:
        viewport_name: Not used (kept for compatibility)
    
    Returns:
        RGB image as numpy array (H, W, 3) uint8 or None on failure
    """
    global _vla_camera
    
    try:
        # Initialize camera on first call
        if _vla_camera is None:
            from omni.isaac.sensor import Camera
            import omni.isaac.core.utils.numpy.rotations as rot_utils
            
            # Create third-person camera positioned for workspace view
            # Position: x=2.72, y=4.77, z=2.52
            # Orientation (Euler): x=-65°, y=24°, z=169°
            _vla_camera = Camera(
                prim_path="/World/vla_camera",
                position=np.array([2.72, 4.77, 2.52]),
                frequency=20,
                resolution=(256, 256),
                # Orientation in Euler angles (degrees)
                orientation=rot_utils.euler_angles_to_quats(
                    np.array([-65, 24, 169]), 
                    degrees=True
                ),
            )
            
            # Initialize the camera
            try:
                _vla_camera.initialize()
                
                # Set transform properties via prim
                prim = _vla_camera.prim
                
                # Set position
                from pxr import Gf
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(2.72, 4.77, 2.52))
                
                # Set focal length
                if prim.HasAttribute("focalLength"):
                    prim.GetAttribute("focalLength").Set(40.0)
                
                print("[VLA] Camera initialized:")
                print(f"  Position: (2.72, 4.77, 2.52)")
                print(f"  Orientation (Euler): (-65°, 24°, 169°)")
                print(f"  Focal Length: 40.0")
                
                # Camera needs at least one render cycle before get_rgba() works
                # Return None on first call to allow camera to initialize
                print("[VLA] Camera created, waiting for first frame render...")
                return None
                
            except Exception as e:
                print(f"[VLA] Error initializing camera: {e}")
                _vla_camera = None
                return None
        
        # Capture RGBA frame and convert to RGB
        try:
            rgba_frame = _vla_camera.get_rgba()
            
            if rgba_frame is None or rgba_frame.size == 0:
                # Camera might still be initializing, this is expected on first few calls
                return None
            
            # Convert RGBA to RGB (drop alpha channel)
            rgb_frame = rgba_frame[:, :, :3]
            
            return rgb_frame.astype(np.uint8)
            
        except Exception as e:
            print(f"[VLA] Error capturing frame from camera: {e}")
            return None
        
    except Exception as e:
        print(f"[VLA] Error in capture_viewport_frame: {e}")
        return None


class VLAHTTPClient:
    """HTTP client for fetching VLA actions from a server"""
    
    def __init__(self, server_url: str, endpoint: str = "/joint_actions", timeout: float = 10.0, model: str = DEFAULT_VLA_MODEL):
        self.server_url = server_url.rstrip('/')
        self.endpoint = endpoint if endpoint.startswith('/') else f'/{endpoint}'
        self.full_url = f"{self.server_url}{self.endpoint}"
        self.model = model
        self.timeout = timeout
        self.connection_ok = False
        self.error_count = 0
        self.last_response = None
        
        print(f"[VLA] VLA HTTP Client initialized")
        print(f"[VLA] Model: {model} ({VLA_MODEL_CONFIGS.get(model, {}).get('description', 'unknown')})")
        print(f"[VLA] Server: {self.full_url}")
        print(f"[VLA] Timeout: {timeout}s")
    
    @staticmethod
    def _coerce_float(value: Any, default: float = 0.0) -> float:
        """Convert mixed numeric types to float"""
        try:
            if value is None:
                return default
            if isinstance(value, (float, int, np.floating, np.integer)):
                return float(value)
            if isinstance(value, str):
                return float(value.strip())
            if isinstance(value, (list, tuple, np.ndarray)) and len(value) > 0:
                return VLAHTTPClient._coerce_float(value[0], default)
        except (TypeError, ValueError):
            pass
        return default
    
    @staticmethod
    def _extract_from_dict(source: Dict, keys: Tuple[str, ...], default: float = 0.0) -> float:
        """Extract value from dict, trying multiple key names"""
        if not isinstance(source, dict):
            return default
        for key in keys:
            if key in source and source[key] is not None:
                return VLAHTTPClient._coerce_float(source[key], default)
        return default
    
    def parse_response(self, data: Dict) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Parse VLA response to extract action.
        
        Expected format:
        {
            "joints": {
                "x": dx, "y": dy, "z": dz,
                "roll": droll, "pitch": dpitch, "yaw": dyaw,
                "gripper": gripper_action
            }
        }
        
        Returns:
            Tuple of (position_delta, orientation_quat, gripper_action) or None
        """
        try:
            if not data:
                return None
            
            if "joints" not in data:
                return None
            
            joints = data["joints"]
            if not isinstance(joints, dict):
                return None
            
            # Extract position deltas
            position_delta = np.array([
                self._extract_from_dict(joints, ("x", "dx"), 0.0),
                self._extract_from_dict(joints, ("y", "dy"), 0.0),
                self._extract_from_dict(joints, ("z", "dz"), 0.0),
            ], dtype=np.float64)
            
            # Extract rotation deltas (in radians)
            roll = self._extract_from_dict(joints, ("roll", "droll", "rx"), 0.0)
            pitch = self._extract_from_dict(joints, ("pitch", "dpitch", "ry"), 0.0)
            yaw = self._extract_from_dict(joints, ("yaw", "dyaw", "rz"), 0.0)
            
            # Convert Euler angles to quaternion
            from isaacsim.core.utils.rotations import euler_angles_to_quat
            orientation_quat = euler_angles_to_quat(np.array([roll, pitch, yaw]))
            
            # Extract gripper action
            gripper_action = self._extract_from_dict(joints, ("gripper",), 0.0)
            
            return (position_delta, orientation_quat, gripper_action)
        
        except Exception as e:
            print(f"[VLA] Error parsing response: {e}")
            return None
    
    def fetch_action(self) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Fetch next VLA action from server"""
        try:
            response = requests.get(self.full_url, timeout=self.timeout)
            
            if response.status_code == 200:
                data = response.json()
                self.last_response = data
                
                if not self.connection_ok:
                    print(f"[VLA] Connected to VLA server: {self.full_url}")
                    self.connection_ok = True
                self.error_count = 0
                
                action = self.parse_response(data)
                if action is not None:
                    return action
            
            return None
        
        except requests.exceptions.Timeout:
            self.error_count += 1
            if self.error_count == 1:
                print(f"[VLA] Request timeout (waiting for server...)")
            return None
        
        except requests.exceptions.ConnectionError:
            self.error_count += 1
            if self.error_count == 1:
                print(f"[VLA] Cannot connect to {self.full_url}")
            self.connection_ok = False
            return None
        
        except Exception as e:
            self.error_count += 1
            if self.error_count <= 3:
                print(f"[VLA] Error fetching action: {e}")
            return None


class GR00TClient:
    """ZMQ client for GR00T inference server (based on robot_inference_test.py)"""
    
    def __init__(self, host: str = "localhost", port: int = 5555, api_token: Optional[str] = None, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.api_token = api_token
        self.timeout = timeout
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))  # milliseconds
        self.socket.setsockopt(zmq.SNDTIMEO, int(timeout * 1000))
        self.socket.connect(f"tcp://{host}:{port}")
        self.connection_ok = False
        self.error_count = 0
        
        print(f"[VLA] GR00T Client initialized")
        print(f"[VLA] Server: tcp://{host}:{port}")
        print(f"[VLA] Timeout: {timeout}s")
        
        # Test connection
        if self.ping():
            self.connection_ok = True
            print(f"[VLA] ✓ Connected to GR00T server")
        else:
            print(f"[VLA] ✗ Cannot connect to GR00T server")
    
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
        # Handle both byte and string keys for __ndarray_class__
        if isinstance(obj, dict):
            if b"__ndarray_class__" in obj or "__ndarray_class__" in obj:
                npy_key = b"as_npy" if b"as_npy" in obj else "as_npy"
                return np.load(io.BytesIO(obj[npy_key]), allow_pickle=False)
        return obj
    
    def _send_request(self, endpoint: str, data: Optional[Dict] = None, requires_input: bool = True):
        """Send a request to the server and return the response."""
        request = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token
            
        # Serialize and send - use strict_types=False for compatibility
        message = msgpack.packb(request, default=self._encode_data, strict_types=False, use_bin_type=True)
        self.socket.send(message)
        
        # Receive and deserialize
        response_bytes = self.socket.recv()
        response = msgpack.unpackb(response_bytes, object_hook=self._decode_data, raw=True)
        
        # Check for error (handle byte keys from raw=True)
        if b"error" in response:
            raise RuntimeError(f"Server error: {response[b'error'].decode('utf-8') if isinstance(response[b'error'], bytes) else response[b'error']}")
        
        # Convert byte keys to strings for easier access
        decoded_response = {}
        for key, value in response.items():
            str_key = key.decode('utf-8') if isinstance(key, bytes) else key
            decoded_response[str_key] = value
        
        return decoded_response
    
    def ping(self) -> bool:
        """Ping the server to check if it's running."""
        try:
            response = self._send_request("ping", requires_input=False)
            return response.get("status") == "ok"
        except Exception:
            return False
    
    def get_action(self, observations: Dict) -> Optional[Dict]:
        """
        Get action from the GR00T inference server.
        
        Args:
            observations (dict): Dictionary containing:
                - Video observations as numpy arrays (H, W, C) uint8
                - State observations as numpy arrays
                - Language instructions as numpy arrays of strings
                
        Returns:
            dict: Action dictionary with numpy arrays or None on error
        """
        try:
            response = self._send_request("get_action", observations)
            self.connection_ok = True
            self.error_count = 0
            return response
        except zmq.error.Again:
            print(f"[VLA] GR00T server timeout after {self.timeout}s")
            self.error_count += 1
            return None
        except Exception as e:
            print(f"[VLA] GR00T error: {e}")
            self.error_count += 1
            self.connection_ok = False
            return None
    
    def parse_response(self, action_dict: Dict) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """
        Parse GR00T response to extract action.
        
        GR00T format (OXE_DROID):\n        {
            "action.eef_position_delta": (16, 3),  # Position deltas in meters
            "action.eef_rotation_delta": (16, 3),  # Axis-angle rotation deltas
            "action.gripper_position": (16, 1)     # Absolute gripper state
        }
        
        Returns:
            Tuple of (position_delta, rotation_delta_axisangle, gripper_action) or None
        """
        try:
            if not action_dict:
                print("[VLA] Empty action dict")
                return None
            
            # Extract first timestep (step 0 - receding horizon control)
            pos_delta = action_dict.get("action.eef_position_delta")
            rot_delta = action_dict.get("action.eef_rotation_delta")  # axis-angle format
            gripper = action_dict.get("action.gripper_position")
            
            if pos_delta is None or rot_delta is None or gripper is None:
                print(f"[VLA] Missing action components in GR00T response")
                print(f"[VLA] Received keys: {list(action_dict.keys())}")
                # Check if there's an error in the response
                if 'error' in action_dict or b'error' in action_dict:
                    error_key = 'error' if 'error' in action_dict else b'error'
                    error_msg = action_dict[error_key]
                    if isinstance(error_msg, bytes):
                        error_msg = error_msg.decode('utf-8')
                    print(f"[VLA]   Error from server: {error_msg}")
                elif action_dict:
                    for key, value in action_dict.items():
                        if hasattr(value, 'shape'):
                            print(f"[VLA]   {key}: shape={value.shape}, dtype={value.dtype}")
                        else:
                            print(f"[VLA]   {key}: {type(value)}")
                return None
            
            # Use only the first timestep (receding horizon)
            pos_delta_0 = pos_delta[0]  # Shape: (3,)
            rot_delta_0 = rot_delta[0]  # Shape: (3,) - axis-angle
            gripper_0 = float(gripper[0, 0])  # Scalar
            
            # Convert axis-angle to quaternion for consistency with other VLA models
            # Axis-angle format: [ax, ay, az] where magnitude is rotation angle
            angle = np.linalg.norm(rot_delta_0)
            if angle > 1e-6:
                axis = rot_delta_0 / angle
                # Convert to quaternion [w, x, y, z]
                half_angle = angle / 2.0
                w = np.cos(half_angle)
                xyz = axis * np.sin(half_angle)
                rot_quat = np.array([w, xyz[0], xyz[1], xyz[2]])
            else:
                # Identity rotation
                rot_quat = np.array([1.0, 0.0, 0.0, 0.0])
            
            return (pos_delta_0, rot_quat, gripper_0)
            
        except Exception as e:
            print(f"[VLA] Error parsing GR00T response: {e}")
            return None
    
    def fetch_action(self, observations: Dict) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Fetch and parse action from GR00T server."""
        action_dict = self.get_action(observations)
        if action_dict is None:
            return None
        return self.parse_response(action_dict)
    
    def close(self):
        """Clean up resources."""
        self.socket.close()
        self.context.term()


class VLAContext(DfRobotApiContext):
    """Context for VLA-controlled behavior"""
    
    def __init__(self, robot, server_url: str = "http://localhost:5000", endpoint: str = "/joint_actions", 
                 task_description: str = "Move to the left", model: str = DEFAULT_VLA_MODEL):
        super().__init__(robot)
        self.robot = robot
        self.task_description = task_description
        self.model = model
        
        # Get model-specific configuration
        model_config = VLA_MODEL_CONFIGS.get(model, VLA_MODEL_CONFIGS[DEFAULT_VLA_MODEL])
        self.timeout = model_config["timeout"]
        self.min_call_interval = model_config["min_call_interval"]
        
        # Create appropriate client based on model type
        if model == "gr00t":
            # GR00T uses ZMQ on port 5555 by default
            # Extract host from server_url if provided
            if server_url.startswith("http://"):
                host = server_url.replace("http://", "").split(":")[0]
            elif server_url.startswith("tcp://"):
                host = server_url.replace("tcp://", "").split(":")[0]
            else:
                host = server_url.split(":")[0] if ":" in server_url else server_url
            
            port = 5555  # Default GR00T port
            self.vla_client = GR00TClient(host=host, port=port, timeout=self.timeout)
        else:
            # HTTP-based VLA (Octo, Gemini, etc.)
            self.vla_client = VLAHTTPClient(server_url, endpoint, timeout=self.timeout, model=model)
        
        # Current desired pose (accumulated)
        self.desired_position = None
        self.desired_orientation = None
        self.desired_gripper = 0.0
        
        # Curobo motion generator
        self.motion_gen = None
        self.plan_config = None
        if USE_CUROBO and CUROBO_AVAILABLE:
            self._init_curobo()
        
        # Scaling parameters
        self.delta_scale = 1.0
        self.max_delta = 0.02  # meters
        self.rotation_scale = 1.0
        self.max_rotation = 0.05  # radians
        
        # Diagnostics
        self.last_action = None
        self.action_count = 0
        
        print(f"[VLA] VLAContext initialized")
        print(f"[VLA] Model: {model}")
        print(f"[VLA] Timeout: {self.timeout}s, Min interval: {self.min_call_interval}s")
    
    def _init_curobo(self):
        """Initialize Curobo motion generator"""
        # try:
        setup_curobo_logger("warn")
        
        # Load robot config
        config_file = "ur10e.yml"
        if self.robot and hasattr(self.robot, "name") and "ur5" in self.robot.name.lower():
            config_file = "ur5e.yml"
            print(f"[VLA] Auto-detected UR5, using config: {config_file}")
            
        robot_cfg = load_yaml(join_path(get_robot_configs_path(), config_file))["robot_cfg"]
        
        # Configure world (ground plane)
        world_cfg = WorldConfig.from_dict({
            "cuboid": {
                "ground_plane": {
                    "pose": [0, 0, -0.05, 1, 0, 0, 0], # slightly below 0
                    "dims": [10, 10, 0.1],
                }
            }
        })
            
        tensor_args = TensorDeviceType()
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            world_cfg,
            tensor_args,
            interpolation_dt=0.05,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup()
        
        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            max_attempts=1,
            time_dilation_factor=1.0
        )
        print("[VLA] Curobo initialized")
        # except Exception as e:
        #     print(f"[VLA] Failed to initialize Curobo: {e}")
        #     self.motion_gen = None

    def get_curobo_joint_state(self):
        """Get current joint state for Curobo"""
        if not self.motion_gen:
            return None
            
        # curobo joint names:
        joint_names = self.motion_gen.kinematics.joint_names
        
        # Get from robot articulation
        articulation = None
        if hasattr(self.robot, "articulation"):
             articulation = self.robot.articulation
        elif hasattr(self.robot, "dof_names"):
             # self.robot IS the articulation wrapper (e.g. CortexRobot/SingleArticulation)
             articulation = self.robot
        elif hasattr(self.robot, "arm") and hasattr(self.robot.arm, "articulation"):
             articulation = self.robot.arm.articulation
        
        if articulation is None:
            print(f"[VLA] Robot has no articulation. Dir: {dir(self.robot)}")
            return None
            
        full_dof_names = articulation.dof_names
        full_positions = articulation.get_joint_positions()
        full_velocities = articulation.get_joint_velocities()
        
        positions = []
        velocities = []
        
        for name in joint_names:
            if name in full_dof_names:
                idx = full_dof_names.index(name)
                positions.append(full_positions[idx])
                velocities.append(full_velocities[idx])
            else:
                # Fallback
                positions.append(0.0)
                velocities.append(0.0)
                
        return JointState(
            position=torch.tensor(positions, device=self.motion_gen.tensor_args.device).unsqueeze(0),
            velocity=torch.tensor(velocities, device=self.motion_gen.tensor_args.device).unsqueeze(0),
            acceleration=torch.zeros(1, len(joint_names), device=self.motion_gen.tensor_args.device),
            jerk=torch.zeros(1, len(joint_names), device=self.motion_gen.tensor_args.device),
        )

    def step_curobo(self):
        """Plan and execute one step using Curobo"""
        if not self.motion_gen or self.plan_config is None or self.desired_position is None:
            print("[VLA] step_curobo skipped: missing components")
            return
            
        try:
            # Get current state
            start_state = self.get_curobo_joint_state()
            if start_state is None:
                print("[VLA] step_curobo skipped: could not get joint state")
                return

            # Create goal pose
            # desired_position is [x, y, z]
            # desired_orientation is [w, x, y, z]
            goal_pose = Pose(
                position=torch.tensor(self.desired_position, device=self.motion_gen.tensor_args.device).unsqueeze(0),
                quaternion=torch.tensor(self.desired_orientation, device=self.motion_gen.tensor_args.device).unsqueeze(0),
            )
            
            # Plan
            result = self.motion_gen.plan_single(start_state, goal_pose, self.plan_config)
            
            if result.success.item():
                # Get next point (index 1, as 0 is current state)
                # If trajectory is short, take the last one
                traj = result.optimized_plan
                if traj.position.shape[1] > 1:
                    cmd_joints = traj.position[0, 1].cpu().numpy()
                else:
                    cmd_joints = traj.position[0, 0].cpu().numpy()
                
                # Send to robot
                # Map back to robot indices
                joint_names = self.motion_gen.kinematics.joint_names
                
                # Get articulation
                articulation = None
                if hasattr(self.robot, "articulation"):
                     articulation = self.robot.articulation
                elif hasattr(self.robot, "dof_names"):
                     articulation = self.robot
                elif hasattr(self.robot, "arm") and hasattr(self.robot.arm, "articulation"):
                     articulation = self.robot.arm.articulation
                
                if articulation:
                    full_dof_names = articulation.dof_names
                    
                    indices = []
                    values = []
                    
                    for i, name in enumerate(joint_names):
                        if name in full_dof_names:
                            indices.append(full_dof_names.index(name))
                            values.append(cmd_joints[i])
                    
                    action = ArticulationAction(
                        joint_positions=np.array(values),
                        joint_indices=np.array(indices)
                    )
                    articulation.apply_action(action)
                else:
                    print("[VLA] Could not apply action: articulation not found")
                
                # Also handle gripper
                if self.desired_gripper > 0.5:
                    self.robot.gripper.close()
                else:
                    self.robot.gripper.open()
            else:
                print(f"[VLA] Curobo planning failed. Valid query: {result.valid_query}")
                    
        except Exception as e:
            print(f"[VLA] Error in step_curobo: {e}")

    def reset(self):
        """Reset context for new episode"""
        print("[VLA] Resetting VLA context")
        
        # Get current end-effector pose as initial desired pose
        try:
            eff_T = self.robot.arm.get_fk_T()
            from isaacsim.core.utils.rotations import quat_to_euler_angles
            self.desired_position, self.desired_orientation = math_util.T2pq(eff_T)
            self.desired_gripper = 0.0
        except Exception as e:
            print(f"[VLA] Error getting initial pose: {e}")
            # Fallback to home
            aji = self.robot.arm.aji
            home_config = self.robot.get_joints_default_state().positions[aji]
            eff_T = self.robot.arm.get_fk_T(config=home_config)
            self.desired_position, self.desired_orientation = math_util.T2pq(eff_T)
            self.desired_gripper = 0.0
        
        print(f"[VLA] Initial pose: pos={self.desired_position}, ori={self.desired_orientation}")
    
    def send_inference_request(self, image_path: str):
        """Send inference request to Flask server with task and image (HTTP-based VLA only)"""
        # This method only works for HTTP-based VLA models (not GR00T)
        if self.model == "gr00t":
            print("[VLA] send_inference_request not supported for GR00T (use fetch_and_apply_action instead)")
            return None
        
        try:
            # Type checking doesn't know vla_client is VLAHTTPClient here, but we do
            inference_url = self.vla_client.server_url.rstrip('/') + '/inference'  # type: ignore
            payload = {
                "task": self.task_description,
                "image_path": image_path
            }
            
            response = requests.post(inference_url, json=payload, timeout=self.vla_client.timeout)  # type: ignore
            response.raise_for_status()
            data = response.json()
            
            # Parse the response
            if "joint_actions" in data:
                joint_actions = data["joint_actions"]
                # Ensure it's a list of 7 values [x, y, z, roll, pitch, yaw, gripper]
                if isinstance(joint_actions, (list, tuple, np.ndarray)) and len(joint_actions) >= 7:
                    position_delta = np.array(joint_actions[0:3], dtype=np.float32)
                    euler_delta = np.array(joint_actions[3:6], dtype=np.float32)
                    gripper_action = float(joint_actions[6])
                    
                    # Convert Euler to quaternion
                    from isaacsim.core.utils.rotations import euler_angles_to_quat
                    orientation_quat = euler_angles_to_quat(euler_delta)
                    
                    return (position_delta, orientation_quat, gripper_action)
            
            return None
            
        except requests.exceptions.Timeout:
            print(f"[VLA] Inference timeout")
            return None
        except requests.exceptions.ConnectionError:
            print(f"[VLA] Cannot connect to inference server")
            return None
        except Exception as e:
            print(f"[VLA] Error in inference request: {e}")
            return None
    
    def capture_and_infer(self):
        """Capture frame from viewport and send to VLA for inference"""
        try:
            # Read task description from text file at every step
            task_file_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur10_robotiq_cortex/text_command.txt"
            try:
                if os.path.exists(task_file_path):
                    with open(task_file_path, 'r') as f:
                        new_task = f.read().strip()
                        if new_task and new_task != self.task_description:
                            self.task_description = new_task
                            print(f"[VLA] Task updated from file: {self.task_description}")
            except Exception as e:
                print(f"[VLA] Warning: Could not read task file: {e}")
            
            # Capture frame from viewport
            frame = capture_viewport_frame()
            if frame is None:
                return None
            
            # Save frame to temporary file
            temp_dir = tempfile.gettempdir()
            image_path = os.path.join(temp_dir, "vla_input_frame.png")
            
            try:
                from PIL import Image
                img = Image.fromarray(frame)
                img.save(image_path)
            except ImportError:
                print(f"[VLA] PIL not available, trying cv2")
                try:
                    import cv2
                    # Convert RGB to BGR for OpenCV
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(image_path, frame_bgr)
                except ImportError:
                    print(f"[VLA] No image library available for frame saving")
                    return None
            
            # Send inference request with captured frame
            action = self.send_inference_request(image_path)
            
            # Clean up temp file
            try:
                os.remove(image_path)
            except:
                pass
            
            return action
            
        except Exception as e:
            print(f"[VLA] Error in capture_and_infer: {e}")
            return None
    
    def apply_action(self, action: Tuple[np.ndarray, np.ndarray, float]):
        """Apply a VLA action to the desired pose"""
        try:
            if action is None:
                return False
            
            position_delta, orientation_quat, gripper_action = action
            
            # For Octo model: Transform position delta from gripper frame to world frame
            # Octo actions are ego-centric (relative to gripper orientation)
            if self.model == "octo":
                # Transform from gripper-centric to world coordinates
                position_delta_world = transform_gripper_to_world_frame(
                    position_delta, 
                    self.desired_orientation
                )
                print(f"[VLA] Octo frame transform: gripper={position_delta} -> world={position_delta_world}")
                position_delta = position_delta_world
            
            # Apply scaling and clipping to deltas
            position_delta = position_delta * self.delta_scale
            position_delta = np.clip(
                position_delta,
                -self.max_delta,
                self.max_delta
            )
            
            # Apply scaling and clipping to rotation
            from isaacsim.core.utils.rotations import quat_to_euler_angles
            ori_euler = quat_to_euler_angles(orientation_quat)
            ori_euler = ori_euler * self.rotation_scale
            ori_euler = np.clip(
                ori_euler,
                -self.max_rotation,
                self.max_rotation
            )
            from isaacsim.core.utils.rotations import euler_angles_to_quat
            orientation_quat = euler_angles_to_quat(ori_euler)
            
            # Accumulate into desired pose
            self.desired_position = self.desired_position + position_delta
            
            # Apply rotation delta (multiply quaternions)
            self.desired_orientation = quat_multiply(self.desired_orientation, orientation_quat)
            
            # Update gripper
            old_gripper = self.desired_gripper
            self.desired_gripper = np.clip(gripper_action, 0.0, 1.0)
            
            # Log action
            self.last_action = {
                "position_delta": position_delta,
                "new_position": self.desired_position.copy(),
                "gripper": self.desired_gripper,
            }
            self.action_count += 1
            
            if self.action_count % 10 == 0:
                print(f"[VLA] Action #{self.action_count}: pos_delta={position_delta}, "
                      f"gripper={self.desired_gripper:.2f}")
            
            return True
        
        except Exception as e:
            print(f"[VLA] Error in apply_action: {e}")
            return False
    
    def fetch_and_apply_action(self):
        """Fetch VLA action from client and apply it"""
        try:
            # For GR00T, need to prepare observations
            if self.model == "gr00t":
                # Get current robot state
                try:
                    # Use Cortex robot API to get end-effector transform
                    eff_T = self.robot.arm.get_fk_T()
                    current_pos, current_quat = math_util.T2pq(eff_T)
                    
                    # Convert quaternion to Euler angles (roll, pitch, yaw) for GR00T
                    from isaacsim.core.utils.rotations import quat_to_euler_angles
                    current_euler = quat_to_euler_angles(current_quat)
                    
                    # Get gripper state (0.0 = open, 1.0 = closed)
                    gripper_state = self.desired_gripper
                    
                except Exception as e:
                    print(f"[VLA] Error getting robot state: {e}")
                    return False
                
                # Capture video frames from camera
                frame = capture_viewport_frame()
                if frame is None:
                    print("[VLA] Failed to capture viewport frame")
                    # Use dummy frame as fallback
                    frame = np.zeros((256, 256, 3), dtype=np.uint8)
                
                # Prepare observations in GR00T format
                # GR00T expects observations with batch size 1, time horizon 1
                observations = {
                    # Video observations (1, H, W, C) uint8
                    "video.exterior_image_1": np.expand_dims(frame, axis=0),
                    "video.exterior_image_2": np.expand_dims(frame, axis=0),  # Reuse same view
                    "video.wrist_image": np.expand_dims(frame, axis=0),  # Reuse same view
                    
                    # State: end-effector position (1, 3) - ABSOLUTE in robot base frame
                    "state.eef_position": current_pos.reshape(1, 3).astype(np.float32),
                    
                    # State: end-effector rotation as EULER ANGLES (roll, pitch, yaw) in RADIANS (1, 3)
                    # ABSOLUTE in robot base frame
                    "state.eef_rotation": current_euler.reshape(1, 3).astype(np.float32),
                    
                    # Gripper state (1, 1): 0.0 = open, 1.0 = closed - ABSOLUTE
                    "state.gripper_position": np.array([[gripper_state]], dtype=np.float32),
                }
                
                # Add language instruction - try as plain list first to avoid encoding issues
                # The server will convert it to numpy array internally if needed
                observations["annotation.language.language_instruction"] = [self.task_description]
                
                # Debug: log observation shapes
                print("[VLA] Sending observations to GR00T:")
                for key, val in observations.items():
                    if hasattr(val, 'shape'):
                        print(f"[VLA]   {key}: shape={val.shape}, dtype={val.dtype}")
                    else:
                        print(f"[VLA]   {key}: {type(val)}")
                
                # Fetch action from GR00T with observations (cast to GR00TClient)
                # Type checking can't infer this but we know vla_client is GR00TClient when model=="gr00t"
                action = self.vla_client.fetch_action(observations)  # type: ignore
            else:
                # HTTP-based VLA (Octo, Gemini, etc.) - no observations needed
                # Type checking can't infer this but we know vla_client is VLAHTTPClient when model!="gr00t"
                action = self.vla_client.fetch_action()  # type: ignore
            
            return self.apply_action(action) if action is not None else False
        except Exception as e:
            print(f"[VLA] Error in fetch_and_apply_action: {e}")
            return False


class VLAControlState(DfState):
    """Main VLA control state: fetches actions and moves end-effector"""
    
    def __init__(self):
        super().__init__()
        self.step_count = 0
        self.last_api_call_time = 0.0  # Time-based throttling (seconds since epoch)
    
    def enter(self):
        self.step_count = 0
        self.last_api_call_time = 0.0  # Reset to allow immediate first call
        
        # Get min_call_interval from context (set based on model)
        ct = self.context
        min_interval = getattr(ct, 'min_call_interval', 5.0)
        model = getattr(ct, 'model', 'unknown')
        
        print("[VLA] >>> ENTER VLAControlState")
        print(f"[VLA] Model: {model}")
        print(f"[VLA] API call throttling: {min_interval}s minimum interval (max {60.0/min_interval:.1f} req/min)")
    
    def step(self):
        self.step_count += 1
        ct = self.context
        
        # Get model-specific min_call_interval from context
        min_interval = getattr(ct, 'min_call_interval', 5.0)
        
        # Time-based throttling: fetch new action only if enough time has passed
        current_time = time.time()
        time_since_last_call = current_time - self.last_api_call_time
        
        if time_since_last_call >= min_interval:
            self.last_api_call_time = current_time
            
            # Log API call for debugging
            if self.step_count > 1:  # Don't log on first step
                print(f"[VLA] Making API call (last call was {time_since_last_call:.2f}s ago)")
            
            # For GR00T, use fetch_and_apply_action which handles observations
            # For HTTP-based VLA (Octo, Gemini), use capture_and_infer
            if ct.model == "gr00t":
                success = ct.fetch_and_apply_action()
                if not success:
                    print(f"[VLA] GR00T action fetch/apply failed")
            else:
                # Use capture_and_infer to get action from VLA with task + image
                action = ct.capture_and_infer()
                if action is not None:
                    ct.apply_action(action)
                else:
                    print(f"[VLA] No action returned from API")
        
        # Send motion command to desired pose
        try:
            if ct.desired_position is not None and ct.desired_orientation is not None:
                if USE_CUROBO and getattr(ct, 'motion_gen', None):
                    ct.step_curobo()
                else:
                    command = MotionCommand(target_pose=PosePq(ct.desired_position, ct.desired_orientation))
                    ct.robot.arm.send(command)
                    
                    # Update gripper based on desired state
                    if ct.desired_gripper > 0.5:
                        ct.robot.gripper.close()
                    else:
                        ct.robot.gripper.open()
        
        except Exception as e:
            print(f"[VLA] Error in step: {e}")
        
        return self
    
    def exit(self):
        print(f"[VLA] <<< EXIT VLAControlState (ran for {self.step_count} steps)")


class VLABehavior(DfDecider):
    """VLA-controlled behavior dispatcher"""
    
    def __init__(self, server_url: str = "http://localhost:5000"):
        super().__init__()
        self.vla_control_state = VLAControlState()
        self.add_child("control", DfStateMachineDecider(DfStateSequence([self.vla_control_state])))
    
    def decide(self):
        """Always return the VLA control state"""
        return DfDecision("control")


def make_decider_network(robot, server_url: str = None, task_description: str = "Move to the left", 
                        model: str = None, monitor_fn=None):
    """
    Create the decider network for VLA behavior.
    
    Args:
        robot: Robot instance
        server_url: URL of the VLA server (uses hardcoded VLA_SERVER_URL if None)
        task_description: Task description for the VLA (read from file during execution)
        model: VLA model type (uses hardcoded VLA_MODEL if None)
        monitor_fn: Optional monitoring function
    
    Returns:
        DfNetwork instance
    """
    global _vla_context
    
    # Use hardcoded values if not provided
    if server_url is None:
        server_url = VLA_SERVER_URL
    if model is None:
        model = VLA_MODEL
    
    print("[VLA] Creating VLA behavior network")
    print(f"[VLA] Server URL: {server_url}")
    print(f"[VLA] Task: {task_description}")
    print(f"[VLA] Model: {model}")
    
    # Validate model
    if model not in VLA_MODEL_CONFIGS:
        print(f"[VLA] WARNING: Unknown model '{model}', using default '{DEFAULT_VLA_MODEL}'")
        model = DEFAULT_VLA_MODEL
    
    # Create context with VLA client and task description
    context = VLAContext(robot, server_url=server_url, task_description=task_description, model=model)
    
    # Store context reference for dynamic updates
    _vla_context = context
    
    # Create behavior and network
    behavior = VLABehavior(server_url=server_url)
    network = DfNetwork(behavior, context=context)
    
    return network


def set_vla_task_description(new_task: str) -> bool:
    """
    Update the VLA task description dynamically.
    This allows changing the task without restarting the behavior.
    
    Args:
        new_task: New task description to send to VLA
    
    Returns:
        True if update was successful, False if no context available
    """
    global _vla_context
    
    if _vla_context is None:
        print("[VLA] Warning: VLA context not initialized yet")
        return False
    
    old_task = _vla_context.task_description
    _vla_context.task_description = new_task
    
    print(f"[VLA] Task description updated:")
    print(f"  Old: {old_task}")
    print(f"  New: {new_task}")
    
    return True
