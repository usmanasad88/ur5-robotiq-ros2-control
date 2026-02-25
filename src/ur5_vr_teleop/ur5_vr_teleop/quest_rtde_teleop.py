#!/usr/bin/env python3
"""
Meta Quest 3S VR teleop for UR5 via ur_rtde (Cartesian velocity / speedL).

Coexistence with the ROS 2 UR driver
--------------------------------------
ur_rtde can run alongside the existing ROS 2 driver:
  - RTDEReceiveInterface   → read-only, multiple clients OK, no conflict
  - RTDEControlInterface   → sends URScript commands directly; safe as long as
                             the ROS 2 trajectory controller is not actively
                             executing a trajectory (idle during teleop)

So: keep launch_all.sh running for joint_states, gripper adapter, cuRobo etc.
Start this script any time — it takes over motion only while grip is held.

Gripper
-------
Uses Robotiq2fSocketAdapter (already in robotiq_2f_urcap_adapter_socket) which
connects to the Robotiq URCap server on the robot at port 63352 — the same
socket interface shown in the ur_rtde robotiq_gripper.py example.

Control law  (mirrors DROID VRPolicy._calculate_action)
---------------------------------------------------------
    target_pos  = robot_origin_pos  + (vr_pos  - vr_origin_pos)
    target_quat = dR_vr             * robot_origin_quat
    lin_vel     = (target_pos  - current_pos)  * pos_gain
    ang_vel     = quat_to_rotvec(quat_error)   * rot_gain
    speedL([lin_vel, ang_vel], accel, 2*dt)

Requirements
------------
    pip install ur-rtde
    pip install git+https://github.com/rail-berkeley/oculus_reader.git
    sudo apt install android-tools-adb

Quest setup
-----------
    1. Enable Developer Mode: Meta Quest app → Devices → Developer Mode ON
    2. Connect Quest 3S via USB-C, put on headset, accept ADB prompt
    3. Verify: adb devices

Controls
--------
    Grip  (side button)  hold  → robot moves
    Joystick click             → reset "forward" direction  ← do this first
    Trigger (front)      hold  → close gripper; release → open
"""

import argparse
import signal
import sys
import threading
import time

import numpy as np

try:
    import rtde_control as _rc
    import rtde_receive as _rr
except ImportError:
    print("ERROR: ur_rtde not found.  pip install ur-rtde")
    sys.exit(1)

try:
    from oculus_reader.reader import OculusReader
except ImportError:
    print("ERROR: oculus_reader not found.")
    print("  pip install git+https://github.com/rail-berkeley/oculus_reader.git")
    print("  sudo apt install android-tools-adb")
    sys.exit(1)

try:
    from robotiq_2f_urcap_adapter_socket.robotiq_2f_socket_adapter import (
        Robotiq2fSocketAdapter,
    )
    _GRIPPER_AVAILABLE = True
except ImportError:
    _GRIPPER_AVAILABLE = False


# ---------------------------------------------------------------------------
# Rotation helpers  (no scipy / tf2 dependency)
# ---------------------------------------------------------------------------

def rotvec_to_quat(rv):
    """Rotation vector (axis×angle) → quaternion [w, x, y, z]."""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), axis[0]*s, axis[1]*s, axis[2]*s])


def quat_to_rotvec(q):
    """Quaternion [w, x, y, z] → rotation vector (axis×angle)."""
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    w, x, y, z = q
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return np.array([x, y, z]) / s * angle


def rmat_to_quat(R):
    """3×3 rotation matrix → quaternion [w, x, y, z]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25/s,
                         (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([(R[2,1]-R[1,2])/s, 0.25*s,
                         (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s,
                         0.25*s, (R[1,2]+R[2,1])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s,
                         (R[1,2]+R[2,1])/s, 0.25*s])


def quat_mult(q1, q2):
    w1,x1,y1,z1 = q1;  w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_diff(q1, q2):
    """Rotation from q2 to q1: q1 * conj(q2)  — same as DROID."""
    return quat_mult(q1, quat_conj(q2))


def vec_to_reorder_mat(vec):
    """Signed-index → 4×4 axis permutation matrix (DROID convention)."""
    n = len(vec)
    M = np.zeros((n, n))
    for i, v in enumerate(vec):
        M[i, int(abs(v))-1] = np.sign(v)
    return M


# ---------------------------------------------------------------------------
# Gripper helper
# ---------------------------------------------------------------------------

class GripperController:
    """
    Thin wrapper around Robotiq2fSocketAdapter for trigger-driven open/close.

    Trigger [0 → 1] maps to gripper width [open → closed].
    Commands are throttled — only sent when the trigger moves more than
    `deadband` from the last commanded position, to avoid socket spam.
    """

    # Robotiq URCap port (same as robotiq_2f_adapter_node.py default)
    URCAP_PORT = 63352
    # Normalised position range the adapter uses
    POS_OPEN   = 0    # fully open  (0 mm)
    POS_CLOSED = 255  # fully closed

    def __init__(self, robot_ip: str, deadband: float = 0.05):
        self._adapter = Robotiq2fSocketAdapter()
        self._adapter.connect(hostname=robot_ip, port=self.URCAP_PORT)
        self._adapter.activate(auto_calibrate=False)
        self._last_trig = -1.0   # force send on first call
        self._deadband  = deadband
        print("Gripper connected and activated.")

    def update(self, trigger: float):
        """Send a move command if trigger changed more than deadband."""
        trigger = float(np.clip(trigger, 0.0, 1.0))
        if abs(trigger - self._last_trig) < self._deadband:
            return
        self._last_trig = trigger
        pos = int(trigger * self.POS_CLOSED)
        self._adapter.move(position=pos, speed=200, force=100)

    def open(self):
        self._adapter.move(position=self.POS_OPEN, speed=200, force=100)

    def disconnect(self):
        try:
            self._adapter.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main teleop class
# ---------------------------------------------------------------------------

class QuestRTDETeleop:
    """
    DROID-style VR teleop using ur_rtde for Cartesian velocity control.

    Two threads:
      _reader_loop   — reads Quest at 50 Hz (mirrors DROID _update_internal_state)
      _control_loop  — P-controller + speedL at 50 Hz (mirrors _calculate_action)
    """

    def __init__(
        self,
        robot_ip: str,
        pos_gain: float       = 3.0,
        rot_gain: float       = 2.0,
        max_lin_vel: float    = 0.15,   # m/s   — conservative for first use
        max_rot_vel: float    = 0.75,   # rad/s
        control_hz: float     = 50.0,
        acceleration: float   = 0.5,    # m/s²
        right_controller: bool = True,
        rmat_reorder: list    = None,
        gripper: bool         = True,
    ):
        self.robot_ip    = robot_ip
        self.pos_gain    = pos_gain
        self.rot_gain    = rot_gain
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.dt          = 1.0 / control_hz
        self.accel       = acceleration

        rmat_reorder = rmat_reorder or [-2, -1, -3, 4]
        self.global_to_env = vec_to_reorder_mat(rmat_reorder)

        self.cid      = 'r' if right_controller else 'l'
        self.grip_key = self.cid.upper() + 'G'
        self.joy_key  = self.cid.upper() + 'J'
        self.trig_key = 'rightTrig' if self.cid == 'r' else 'leftTrig'

        # ---- shared state ----
        self._lock             = threading.Lock()
        self._poses            = {}
        self._buttons          = {}
        self._enabled          = False
        self._vr_to_global     = np.eye(4)
        self._reset_orientation = True

        # DROID-style origins (set on first tick after grip pressed)
        self._robot_origin_pos  = None
        self._robot_origin_quat = None
        self._vr_origin_pos     = None
        self._vr_origin_quat    = None

        self._running = False

        # ---- ur_rtde connections ----
        print(f"Connecting to UR5 at {robot_ip} ...")
        self.rtde_r = _rr.RTDEReceiveInterface(robot_ip)
        # RTDEControlInterface: safe alongside ROS 2 driver when no trajectory
        # is actively executing — it sends URScript directly via port 30004.
        self.rtde_c = _rc.RTDEControlInterface(robot_ip)
        print("Robot connected.")

        # ---- Gripper ----
        self.gripper: GripperController | None = None
        if gripper and _GRIPPER_AVAILABLE:
            try:
                self.gripper = GripperController(robot_ip)
            except Exception as exc:
                print(f"WARNING: gripper init failed ({exc}). Continuing without gripper.")
        elif gripper and not _GRIPPER_AVAILABLE:
            print("WARNING: robotiq_2f_urcap_adapter_socket not found — gripper disabled.")

        # ---- OculusReader ----
        print("Initialising OculusReader (Quest 3S via ADB) ...")
        self.reader = OculusReader()
        print("OculusReader ready — put on headset and accept ADB prompt if shown.")

    # ------------------------------------------------------------------
    # Background reader thread  (DROID _update_internal_state)
    # ------------------------------------------------------------------

    def _reader_loop(self, hz: float = 50.0):
        while self._running:
            time.sleep(1.0 / hz)
            try:
                poses, buttons = self.reader.get_transformations_and_buttons()
            except Exception:
                continue
            if not poses:
                continue

            with self._lock:
                prev_enabled = self._enabled
                now_enabled  = bool(buttons.get(self.grip_key, False))
                toggled      = prev_enabled != now_enabled

                # Orientation alignment — keep updating until joystick clicked or grip pressed
                if self._reset_orientation or buttons.get(self.joy_key, False):
                    raw  = np.asarray(poses.get(self.cid, np.eye(4)))
                    stop = buttons.get(self.joy_key, False) or now_enabled
                    if stop:
                        self._reset_orientation = False
                    try:
                        self._vr_to_global = np.linalg.inv(raw)
                    except np.linalg.LinAlgError:
                        self._vr_to_global = np.eye(4)
                        self._reset_orientation = True

                # Clear origins on every grip toggle (forces re-baseline)
                if toggled:
                    self._robot_origin_pos  = None
                    self._robot_origin_quat = None
                    self._vr_origin_pos     = None
                    self._vr_origin_quat    = None

                self._poses   = poses
                self._buttons = buttons
                self._enabled = now_enabled

    # ------------------------------------------------------------------
    # Control loop  (DROID _calculate_action mapped to speedL)
    # ------------------------------------------------------------------

    def _control_loop(self):
        print("\n--- Quest RTDE Teleop running ---")
        print("  Joystick click  : reset forward direction")
        print("  Hold GRIP       : move robot")
        print("  Hold TRIGGER    : close gripper")
        print("  Ctrl-C          : quit\n")

        prev_enabled = False
        log_count    = 0

        while self._running:
            t0 = time.time()

            with self._lock:
                poses        = dict(self._poses)
                buttons      = dict(self._buttons)
                enabled      = self._enabled
                vr_to_global = self._vr_to_global.copy()
                origin_ready = self._robot_origin_pos is not None

            # ---- gripper (always active, independent of grip deadman) ----
            if self.gripper is not None:
                trig = float(buttons.get(self.trig_key, [0.0])[0])
                self.gripper.update(trig)

            # ---- not enabled: stop and idle ----
            if not enabled:
                if prev_enabled:
                    self.rtde_c.speedStop(self.accel)
                    print("Grip released — robot stopped.")
                prev_enabled = False
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            if not prev_enabled:
                print("Grip pressed — setting origins ...")
            prev_enabled = True

            # ---- current TCP pose from RTDE receive ----
            tcp          = self.rtde_r.getActualTCPPose()   # [x,y,z, rx,ry,rz]
            current_pos  = np.array(tcp[:3])
            current_quat = rotvec_to_quat(np.array(tcp[3:]))

            # ---- VR pose in env frame (DROID _process_reading) ----
            if self.cid not in poses:
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            raw    = np.asarray(poses[self.cid])
            mat    = self.global_to_env @ vr_to_global @ raw
            vr_pos  = mat[:3, 3]
            vr_quat = rmat_to_quat(mat[:3, :3])

            # ---- set origins on first tick (DROID reset_origin) ----
            if not origin_ready:
                with self._lock:
                    self._robot_origin_pos  = current_pos.copy()
                    self._robot_origin_quat = current_quat.copy()
                    self._vr_origin_pos     = vr_pos.copy()
                    self._vr_origin_quat    = vr_quat.copy()
                print(f"  Origin set at TCP {current_pos}")
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            with self._lock:
                ro_pos  = self._robot_origin_pos.copy()
                ro_quat = self._robot_origin_quat.copy()
                vo_pos  = self._vr_origin_pos.copy()
                vo_quat = self._vr_origin_quat.copy()

            # ---- DROID P-controller ----

            # Position: robot should be displaced from its origin by the same
            # amount the controller moved from its origin
            target_pos = ro_pos + (vr_pos - vo_pos)
            pos_error  = target_pos - current_pos
            lin_vel    = pos_error * self.pos_gain

            # Rotation: apply the VR displacement rotation onto the robot origin orientation
            dR_vr       = quat_diff(vr_quat, vo_quat)          # rotation from vr_origin → vr_now
            target_quat = quat_mult(dR_vr, ro_quat)            # apply to robot origin
            quat_error  = quat_diff(target_quat, current_quat) # remaining rotation error
            if quat_error[0] < 0:                              # shortest path
                quat_error = -quat_error
            ang_vel = quat_to_rotvec(quat_error) * self.rot_gain

            # ---- velocity limits ----
            ln = np.linalg.norm(lin_vel)
            rn = np.linalg.norm(ang_vel)
            if ln > self.max_lin_vel:
                lin_vel = lin_vel * self.max_lin_vel / ln
            if rn > self.max_rot_vel:
                ang_vel = ang_vel * self.max_rot_vel / rn

            # ---- send to robot ----
            # time = 2×dt: robot auto-stops if commands stop for one period
            self.rtde_c.speedL(
                np.concatenate([lin_vel, ang_vel]).tolist(),
                self.accel,
                2.0 * self.dt,
            )

            # ---- periodic log ----
            log_count += 1
            if log_count % 50 == 0:
                trig = float(buttons.get(self.trig_key, [0.0])[0])
                print(
                    f"lin=[{lin_vel[0]:+.3f},{lin_vel[1]:+.3f},{lin_vel[2]:+.3f}] "
                    f"ang=[{ang_vel[0]:+.3f},{ang_vel[1]:+.3f},{ang_vel[2]:+.3f}] "
                    f"trig={trig:.2f}"
                )

            time.sleep(max(0.0, self.dt - (time.time() - t0)))

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def run(self):
        self._running = True
        reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='quest_reader')
        reader_thread.start()
        try:
            self._control_loop()
        finally:
            self.stop()

    def stop(self):
        self._running = False
        try:
            self.rtde_c.speedStop(self.accel)
            self.rtde_c.stopScript()
        except Exception:
            pass
        if self.gripper is not None:
            self.gripper.open()
            self.gripper.disconnect()
        print("Teleop stopped.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Quest 3S → UR5 VR teleop via ur_rtde (DROID-style P-controller)')
    parser.add_argument('--robot-ip',      default='172.17.66.105')
    parser.add_argument('--controller',    choices=['right','left'], default='right')
    parser.add_argument('--pos-gain',      type=float, default=3.0)
    parser.add_argument('--rot-gain',      type=float, default=2.0)
    parser.add_argument('--max-lin-vel',   type=float, default=0.15,
                        help='Max linear speed [m/s] (default 0.15 — conservative)')
    parser.add_argument('--max-rot-vel',   type=float, default=0.75,
                        help='Max angular speed [rad/s]')
    parser.add_argument('--hz',            type=float, default=50.0)
    parser.add_argument('--no-gripper',    action='store_true',
                        help='Disable gripper control')
    parser.add_argument('--rmat-reorder',  nargs=4, type=int, default=[-2,-1,-3,4],
                        metavar='N',
                        help='Axis reorder (default: -2 -1 -3 4). '
                             'Flip sign to negate an axis, swap indices to swap axes.')
    args = parser.parse_args()

    teleop = QuestRTDETeleop(
        robot_ip         = args.robot_ip,
        pos_gain         = args.pos_gain,
        rot_gain         = args.rot_gain,
        max_lin_vel      = args.max_lin_vel,
        max_rot_vel      = args.max_rot_vel,
        control_hz       = args.hz,
        right_controller = (args.controller == 'right'),
        rmat_reorder     = args.rmat_reorder,
        gripper          = not args.no_gripper,
    )

    def _sigint(sig, frame):
        print("\nCtrl-C — stopping ...")
        teleop.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    teleop.run()


if __name__ == '__main__':
    main()
