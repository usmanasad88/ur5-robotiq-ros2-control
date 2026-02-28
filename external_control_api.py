#!/usr/bin/env python3
"""
UR5 External Control API — Flask REST Server

Exposes the same robot commands available in the Streamlit UI
(programs, named positions, gripper, execution control) as a
REST API so that an external autonomous service can drive the robot.

Endpoints
─────────
  GET  /api/commands          List every available command
  GET  /api/status            Executor & joint-state status

  POST /api/program/execute   Load + execute a .prog file
  POST /api/program/load      Load a .prog without executing
  POST /api/program/pause     Pause running program
  POST /api/program/resume    Resume paused program
  POST /api/program/stop      Stop & hold at current position

  POST /api/gripper           Gripper open / close / set position
  POST /api/move/named        Move to a named position
  POST /api/move/joints       Move to raw joint angles
  POST /api/move/pose         Move to a Cartesian pose (cuRobo)
  POST /api/move/relative     Move by Cartesian offset (left/right/up/down/…)

  POST /api/position/save     Save current position as a named position

Usage
─────
  ./run_external_api.sh                   # default port 5050
  ./run_external_api.sh --port 5050       # explicit port
"""

from __future__ import annotations

import argparse
import os
import sys
import json
import threading
import time
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# ROS 2 bootstrap (same trick as the Streamlit UI)
# ---------------------------------------------------------------------------
ros_paths = [
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]
for p in ros_paths:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

# Flask
from flask import Flask, jsonify, request as flask_request

# ROS 2
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Float64
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS_AVAILABLE = True
except ImportError as exc:
    print(f"⚠  ROS 2 imports failed ({exc}) — running in demo/dry-run mode")

# Named-position parser (from the ur5_curobo_control package)
try:
    from ur5_curobo_control.program_parser import NamedPositionsParser, PositionType
except ImportError:
    # Fallback: import directly from source tree
    workspace_root = Path(__file__).resolve().parent
    sys.path.insert(0, str(workspace_root / "src/ur5_curobo_control/ur5_curobo_control"))
    from program_parser import NamedPositionsParser, PositionType

import psutil

# ═══════════════════════════════════════════════════════════════════════════
# ROS 2 controller (mirrors ROSProgramController in program_selector_ui.py)
# ═══════════════════════════════════════════════════════════════════════════

class ROSBridge:
    """Thin wrapper around ROS 2 services/topics used by the program executor."""

    JOINT_NAMES = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]

    def __init__(self):
        self.node = None
        self.latest_joint_state: JointState | None = None
        self.latest_gripper_state: JointState | None = None
        self._spin_lock = threading.Lock()
        self._spin_thread: threading.Thread | None = None
        self._running = False

        if not ROS_AVAILABLE:
            return

        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("external_control_api")

        # Service clients --------------------------------------------------
        prefix = "/ur5_program_executor"
        self.cli_load = self.node.create_client(Trigger, f"{prefix}/load_program")
        self.cli_exec = self.node.create_client(Trigger, f"{prefix}/execute_program")
        self.cli_pause = self.node.create_client(SetBool, f"{prefix}/pause")
        self.cli_stop = self.node.create_client(Trigger, f"{prefix}/stop")
        self.cli_move_pose = self.node.create_client(Trigger, f"{prefix}/move_to_pose")
        self.cli_save_position = self.node.create_client(Trigger, f"{prefix}/save_position")
        self.cli_move_relative = self.node.create_client(Trigger, f"{prefix}/move_relative")
        self.cli_set_params = self.node.create_client(SetParameters, f"{prefix}/set_parameters")

        # Publishers -------------------------------------------------------
        self.pub_gripper = self.node.create_publisher(String, "/gripper_command_simple", 10)
        self.pub_gripper_viz = self.node.create_publisher(Float64, "/gripper_position_command", 10)
        self.pub_trajectory = self.node.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )

        # Subscriber -------------------------------------------------------
        self.node.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

        # Background spin so callbacks fire
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    # -- internals ---------------------------------------------------------

    def _joint_cb(self, msg: JointState):
        if len(msg.name) == 6:
            self.latest_joint_state = msg
        elif len(msg.name) == 1:
            self.latest_gripper_state = msg

    def _spin_loop(self):
        while self._running and rclpy.ok():
            with self._spin_lock:
                rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call_service(self, client, request, timeout: float = 5.0):
        """Block until the service returns or *timeout* expires."""
        if not ROS_AVAILABLE or self.node is None:
            return None
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        # Wait without holding the spin lock (spin thread handles callbacks)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        return future.result() if future.done() else None

    # -- public helpers ----------------------------------------------------

    def set_parameter(self, name: str, value, ptype):
        req = SetParameters.Request()
        p = Parameter()
        p.name = name
        p.value = ParameterValue(type=ptype)
        if ptype == ParameterType.PARAMETER_STRING:
            p.value.string_value = value
        elif ptype == ParameterType.PARAMETER_DOUBLE:
            p.value.double_value = float(value)
        elif ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
            p.value.double_array_value = [float(v) for v in value]
        req.parameters = [p]
        res = self._call_service(self.cli_set_params, req)
        return res is not None and res.results[0].successful

    def load_program(self, program_name: str):
        if not self.set_parameter("program_file", program_name, ParameterType.PARAMETER_STRING):
            return False, "Failed to set program_file parameter"
        res = self._call_service(self.cli_load, Trigger.Request())
        if res is None:
            return False, "load_program service timeout"
        return res.success, res.message

    def execute_program(self):
        res = self._call_service(self.cli_exec, Trigger.Request())
        if res is None:
            return False, "execute_program service timeout"
        return res.success, res.message

    def pause_program(self, pause: bool):
        req = SetBool.Request()
        req.data = pause
        res = self._call_service(self.cli_pause, req)
        if res is None:
            return False, "pause service timeout"
        return res.success, res.message

    def stop_program(self):
        res = self._call_service(self.cli_stop, Trigger.Request())
        if res is None:
            return False, "stop service timeout"
        return res.success, res.message

    def gripper(self, action: str, position: float | None = None):
        """Control the physical Robotiq gripper via ros2 action send_goal.

        The /gripper_command_simple topic is only for logging/visualization.
        The real gripper is driven by the Robotiq action server at
        /robotiq_2f_urcap_adapter/gripper_command.

        Robotiq 2F-85 mapping:
            position 0.085 m  →  fully open
            position 0.0   m  →  fully closed
        """
        import subprocess

        if action == "open":
            robotiq_pos = 0.085
            viz_pos = 0.0
        elif action == "close":
            robotiq_pos = 0.0
            viz_pos = 0.8
        elif action == "position" and position is not None:
            position = max(0.0, min(1.0, position))
            robotiq_pos = (1.0 - position) * 0.085
            viz_pos = position * 0.8
        else:
            return False, f"Unknown gripper action: {action}"

        # Publish visualization so RViz updates immediately
        viz = Float64()
        viz.data = viz_pos
        self.pub_gripper_viz.publish(viz)

        # Also publish to the simple topic for logging / program-executor state
        simple = String()
        simple.data = action if action in ("open", "close") else f"position:{position}"
        self.pub_gripper.publish(simple)

        # Drive the physical gripper through the Robotiq action server
        cmd = (
            'unset LD_PRELOAD && '
            'source /opt/ros/humble/setup.bash && '
            'source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && '
            'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
            'robotiq_2f_urcap_adapter/action/GripperCommand '
            f"'{{command: {{position: {robotiq_pos}, max_effort: 255.0, max_speed: 0.1}}}}'"
        )

        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=15,
                executable='/bin/bash',
            )
            if result.returncode == 0:
                return True, f"Gripper {action} (pos={robotiq_pos:.3f}m)"
            else:
                return False, f"Gripper action failed: {result.stderr.strip()}"
        except subprocess.TimeoutExpired:
            return False, "Gripper action timed out"
        except Exception as e:
            return False, f"Gripper action error: {e}"

    def move_to_joints(self, positions: list[float], duration: float = 3.0):
        if len(positions) != 6:
            return False, f"Expected 6 joint values, got {len(positions)}"
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * 6
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(pt)
        self.pub_trajectory.publish(msg)
        return True, f"Moving to joint target (duration={duration:.1f}s)"

    def move_to_pose(self, position: list[float], quaternion: list[float]):
        if len(position) != 3 or len(quaternion) != 4:
            return False, "Need position[3] and quaternion[4]"
        ok1 = self.set_parameter("target_pose_position", position, ParameterType.PARAMETER_DOUBLE_ARRAY)
        ok2 = self.set_parameter("target_pose_quaternion", quaternion, ParameterType.PARAMETER_DOUBLE_ARRAY)
        if not (ok1 and ok2):
            return False, "Failed to set pose parameters"
        res = self._call_service(self.cli_move_pose, Trigger.Request(), timeout=10.0)
        if res is None:
            return False, "move_to_pose service timeout"
        return res.success, res.message

    def save_position(self, name: str, pos_type: str = "joint"):
        """Save the current position as a named position."""
        ok1 = self.set_parameter("save_position_name", name, ParameterType.PARAMETER_STRING)
        ok2 = self.set_parameter("save_position_type", pos_type, ParameterType.PARAMETER_STRING)
        if not (ok1 and ok2):
            return False, "Failed to set save_position parameters"
        res = self._call_service(self.cli_save_position, Trigger.Request(), timeout=5.0)
        if res is None:
            return False, "save_position service timeout"
        return res.success, res.message

    def move_relative(self, direction: str, distance: float = 0.05):
        """Move end-effector by a Cartesian offset in the given direction."""
        ok1 = self.set_parameter("relative_move_direction", direction, ParameterType.PARAMETER_STRING)
        ok2 = self.set_parameter("relative_move_distance", distance, ParameterType.PARAMETER_DOUBLE)
        if not (ok1 and ok2):
            return False, "Failed to set move_relative parameters"
        res = self._call_service(self.cli_move_relative, Trigger.Request(), timeout=10.0)
        if res is None:
            return False, "move_relative service timeout"
        return res.success, res.message

    def set_speed(self, speed: float):
        """Set the executor speed factor (0.0-1.0)."""
        speed = max(0.01, min(1.0, speed))
        ok = self.set_parameter("default_speed", speed, ParameterType.PARAMETER_DOUBLE)
        if not ok:
            return False, "Failed to set speed parameter"
        return True, f"Speed set to {speed:.2f}"

    def get_speed(self) -> float | None:
        """Read the current speed factor from the executor node."""
        if not ROS_AVAILABLE or self.node is None:
            return None
        try:
            from rcl_interfaces.srv import GetParameters
            cli = self.node.create_client(GetParameters, "/ur5_program_executor/get_parameters")
            if not cli.wait_for_service(timeout_sec=1.0):
                return None
            req = GetParameters.Request()
            req.names = ["default_speed"]
            future = cli.call_async(req)
            end = time.time() + 2.0
            while not future.done() and time.time() < end:
                time.sleep(0.02)
            if future.done() and future.result() and future.result().values:
                return future.result().values[0].double_value
        except Exception:
            pass
        return None

    def get_joint_state_dict(self) -> dict | None:
        js = self.latest_joint_state
        if js is None:
            return None
        return {
            "names": list(js.name),
            "positions": list(js.position),
            "velocities": list(js.velocity) if js.velocity else [],
            "efforts": list(js.effort) if js.effort else [],
        }

    def get_gripper_state_dict(self) -> dict | None:
        gs = self.latest_gripper_state
        if gs is None:
            return None
        return {
            "position": float(gs.position[0]) if gs.position else 0.0,
            "velocity": float(gs.velocity[0]) if gs.velocity else 0.0,
            "effort": float(gs.effort[0]) if gs.effort else 0.0,
        }

    def shutdown(self):
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        if self.node:
            self.node.destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
# Helpers — programs & named positions
# ═══════════════════════════════════════════════════════════════════════════

WORKSPACE_ROOT = Path(__file__).resolve().parent


def _programs_dir() -> Path:
    d = WORKSPACE_ROOT / "install/ur5_curobo_control/share/ur5_curobo_control/programs"
    if not d.exists():
        d = WORKSPACE_ROOT / "src/ur5_curobo_control/programs"
    return d


def _named_positions_file() -> Path | None:
    for rel in [
        "install/ur5_curobo_control/share/ur5_curobo_control/config/named_positions.txt",
        "src/ur5_curobo_control/config/named_positions.txt",
    ]:
        p = WORKSPACE_ROOT / rel
        if p.exists():
            return p
    return None


def _read_description(prog_path: Path) -> str:
    try:
        line = prog_path.open().readline().strip()
        if line.startswith("#"):
            return line[1:].strip()
    except Exception:
        pass
    return ""


def get_available_commands() -> dict[str, Any]:
    """Build the full command catalogue returned by GET /api/commands."""
    commands: dict[str, Any] = {
        "programs": [],
        "named_positions": {"joint": [], "pose": []},
        "gripper_actions": ["open", "close", "position"],
        "execution_control": ["pause", "resume", "stop"],
        "relative_directions": ["left", "right", "forward", "back", "up", "down"],
        "speed": {"min": 0.01, "max": 1.0, "default": 0.5,
                  "description": "Velocity scaling factor (0.01-1.0). Use POST /api/speed to change."},
    }

    # Programs
    pdir = _programs_dir()
    if pdir.exists():
        for p in sorted(pdir.glob("*.prog")):
            commands["programs"].append({
                "name": p.name,
                "description": _read_description(p),
            })

    # Named positions
    nf = _named_positions_file()
    if nf:
        parser = NamedPositionsParser()
        parser.parse_file(str(nf))
        for pos in parser.get_joints():
            commands["named_positions"]["joint"].append({
                "name": pos.name,
                "joint_positions": list(pos.joint_positions),
                "description": pos.description or "",
            })
        for pos in parser.get_poses():
            commands["named_positions"]["pose"].append({
                "name": pos.name,
                "position": list(pos.position),
                "quaternion": list(pos.quaternion),
                "description": pos.description or "",
            })

    return commands


def is_executor_running() -> bool:
    for proc in psutil.process_iter(["cmdline"]):
        try:
            cl = proc.info.get("cmdline") or []
            if "program_executor_node" in " ".join(cl):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return False


# ═══════════════════════════════════════════════════════════════════════════
# Flask app
# ═══════════════════════════════════════════════════════════════════════════

app = Flask(__name__)
bridge: ROSBridge | None = None


def _ok(msg: str = "ok", **extra):
    return jsonify({"success": True, "message": msg, **extra})


def _err(msg: str, code: int = 400):
    return jsonify({"success": False, "message": msg}), code


# ---------- discovery / status -------------------------------------------

@app.route("/api/commands", methods=["GET"])
def api_commands():
    """Return every command the robot can accept."""
    return jsonify(get_available_commands())


@app.route("/api/status", methods=["GET"])
def api_status():
    """Return executor process status + current joint state."""
    data: dict[str, Any] = {
        "executor_running": is_executor_running(),
        "ros_available": ROS_AVAILABLE,
        "joint_state": None,
        "gripper_state": None,
        "speed": None,
    }
    if bridge:
        data["joint_state"] = bridge.get_joint_state_dict()
        data["gripper_state"] = bridge.get_gripper_state_dict()
        data["speed"] = bridge.get_speed()
    return jsonify(data)


# ---------- program control ----------------------------------------------

@app.route("/api/program/execute", methods=["POST"])
def api_program_execute():
    """Load and execute a program.

    Body: {"program": "pick_and_place_object.prog"}
    """
    body = flask_request.get_json(silent=True) or {}
    prog = body.get("program")
    if not prog:
        return _err("Missing 'program' field")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)

    ok, msg = bridge.load_program(prog)
    if not ok:
        return _err(f"Load failed: {msg}")
    ok, msg = bridge.execute_program()
    if not ok:
        return _err(f"Execute failed: {msg}")
    return _ok(msg, program=prog)


@app.route("/api/program/load", methods=["POST"])
def api_program_load():
    """Load a program without executing.

    Body: {"program": "inspect_object.prog"}
    """
    body = flask_request.get_json(silent=True) or {}
    prog = body.get("program")
    if not prog:
        return _err("Missing 'program' field")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    ok, msg = bridge.load_program(prog)
    return (_ok(msg, program=prog) if ok else _err(msg))


@app.route("/api/program/pause", methods=["POST"])
def api_program_pause():
    """Pause the running program."""
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    ok, msg = bridge.pause_program(True)
    return (_ok(msg) if ok else _err(msg))


@app.route("/api/program/resume", methods=["POST"])
def api_program_resume():
    """Resume a paused program."""
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    ok, msg = bridge.pause_program(False)
    return (_ok(msg) if ok else _err(msg))


@app.route("/api/program/stop", methods=["POST"])
def api_program_stop():
    """Stop the running program."""
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    ok, msg = bridge.stop_program()
    return (_ok(msg) if ok else _err(msg))


# ---------- gripper -------------------------------------------------------

@app.route("/api/gripper", methods=["POST"])
def api_gripper():
    """Control the gripper.

    Body: {"action": "open"}
          {"action": "close"}
          {"action": "position", "position": 0.5}
    """
    body = flask_request.get_json(silent=True) or {}
    action = body.get("action")
    if not action:
        return _err("Missing 'action' field (open | close | position)")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    pos = body.get("position")
    ok, msg = bridge.gripper(action, pos)
    return (_ok(msg) if ok else _err(msg))


# ---------- movement ------------------------------------------------------

@app.route("/api/move/named", methods=["POST"])
def api_move_named():
    """Move to a named position (from named_positions.txt).

    Body: {"name": "Home"}                       # auto-detects type
          {"name": "Home", "duration": 4.0}       # joint-type with duration
    """
    body = flask_request.get_json(silent=True) or {}
    name = body.get("name")
    if not name:
        return _err("Missing 'name' field")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)

    # Normalise name: spaces → underscores (save_position does the same)
    name = name.strip().replace(' ', '_')

    # Look up in named positions
    nf = _named_positions_file()
    if not nf:
        return _err("Named positions file not found")

    parser = NamedPositionsParser()
    parser.parse_file(str(nf))

    # Search joints first, then poses
    for pos in parser.get_joints():
        if pos.name.lower() == name.lower():
            dur = body.get("duration", 3.0)
            ok, msg = bridge.move_to_joints(list(pos.joint_positions), duration=float(dur))
            return (_ok(msg, type="joint", name=pos.name) if ok else _err(msg))

    for pos in parser.get_poses():
        if pos.name.lower() == name.lower():
            ok, msg = bridge.move_to_pose(list(pos.position), list(pos.quaternion))
            return (_ok(msg, type="pose", name=pos.name) if ok else _err(msg))

    return _err(f"Named position '{name}' not found")


@app.route("/api/move/joints", methods=["POST"])
def api_move_joints():
    """Move to raw joint positions.

    Body: {"positions": [0, -1.57, 1.57, -1.57, -1.57, 0], "duration": 3.0}
    """
    body = flask_request.get_json(silent=True) or {}
    positions = body.get("positions")
    if not positions or len(positions) != 6:
        return _err("Need 'positions' list with 6 values")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    dur = body.get("duration", 3.0)
    ok, msg = bridge.move_to_joints(positions, duration=float(dur))
    return (_ok(msg) if ok else _err(msg))


@app.route("/api/move/pose", methods=["POST"])
def api_move_pose():
    """Move to a Cartesian pose via cuRobo motion planning.

    Body: {"position": [0.4, 0.0, 0.4], "quaternion": [0.0, 1.0, 0.0, 0.0]}
    """
    body = flask_request.get_json(silent=True) or {}
    pos = body.get("position")
    quat = body.get("quaternion")
    if not pos or len(pos) != 3:
        return _err("Need 'position' list with 3 values [x, y, z]")
    if not quat or len(quat) != 4:
        return _err("Need 'quaternion' list with 4 values [qw, qx, qy, qz]")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    ok, msg = bridge.move_to_pose(pos, quat)
    return (_ok(msg) if ok else _err(msg))


@app.route("/api/move/relative", methods=["POST"])
def api_move_relative():
    """Move the end-effector by a Cartesian offset.

    Directions are relative to the robot base frame:
      left/right  → Y axis,  forward/back → X axis,  up/down → Z axis (gravity)

    Body: {"direction": "left", "distance": 0.05}
          {"direction": "up",   "distance": 0.1}
    """
    body = flask_request.get_json(silent=True) or {}
    direction = body.get("direction")
    if not direction:
        return _err("Missing 'direction' field (left|right|forward|back|up|down)")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    distance = body.get("distance", 0.05)
    ok, msg = bridge.move_relative(direction, distance=float(distance))
    return (_ok(msg, direction=direction, distance=distance) if ok else _err(msg))


# ---------- position saving -----------------------------------------------

@app.route("/api/position/save", methods=["POST"])
def api_position_save():
    """Save the current robot position as a named position.

    Body: {"name": "MyPosition"}                  # saves as joint (default)
          {"name": "MyPosition", "type": "pose"}  # saves as Cartesian pose
    """
    body = flask_request.get_json(silent=True) or {}
    name = body.get("name")
    if not name:
        return _err("Missing 'name' field")
    if bridge is None:
        return _err("ROS bridge not initialised", 503)
    pos_type = body.get("type", "joint")
    if pos_type not in ("joint", "pose"):
        return _err("'type' must be 'joint' or 'pose'")
    ok, msg = bridge.save_position(name, pos_type)
    return (_ok(msg, name=name, type=pos_type) if ok else _err(msg))


# ---------- speed control -------------------------------------------------

@app.route("/api/speed", methods=["GET", "POST"])
def api_speed():
    """Get or set the robot speed factor.

    GET  → returns current speed
    POST → sets speed
      Body: {"speed": 0.3}
    """
    if bridge is None:
        return _err("ROS bridge not initialised", 503)

    if flask_request.method == "GET":
        spd = bridge.get_speed()
        if spd is None:
            return _err("Could not read speed from executor")
        return _ok(f"Current speed: {spd:.2f}", speed=spd)

    # POST
    body = flask_request.get_json(silent=True) or {}
    speed = body.get("speed")
    if speed is None:
        return _err("Missing 'speed' field (0.01 – 1.0)")
    try:
        speed = float(speed)
    except (TypeError, ValueError):
        return _err("'speed' must be a number (0.01 – 1.0)")
    if speed < 0.01 or speed > 1.0:
        return _err("'speed' must be between 0.01 and 1.0")
    ok, msg = bridge.set_speed(speed)
    return (_ok(msg, speed=speed) if ok else _err(msg))


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    global bridge

    parser = argparse.ArgumentParser(description="UR5 External Control REST API")
    parser.add_argument("--port", type=int, default=5050, help="HTTP port (default 5050)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    parser.add_argument("--no-ros", action="store_true", help="Run without ROS (dry-run / dev mode)")
    args = parser.parse_args()

    if not args.no_ros and ROS_AVAILABLE:
        print("🤖 Initialising ROS 2 bridge …")
        bridge = ROSBridge()
    else:
        print("⚠  Running without ROS 2 bridge (dry-run mode)")

    print(f"\n{'='*50}")
    print(f"  UR5 External Control API")
    print(f"  http://{args.host}:{args.port}/api/commands")
    print(f"  http://{args.host}:{args.port}/api/status")
    print(f"{'='*50}\n")

    try:
        app.run(host=args.host, port=args.port, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        if bridge:
            bridge.shutdown()
        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
