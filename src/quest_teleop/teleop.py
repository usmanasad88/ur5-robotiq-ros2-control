"""DROID-style VR teleop loop: Quest poses → UR5 Cartesian velocity (speedL).

Control law (mirrors DROID VRPolicy._calculate_action):
    target_pos  = robot_origin_pos  + (vr_pos  - vr_origin_pos)
    target_quat = dR_vr             * robot_origin_quat
    lin_vel     = (target_pos  - current_pos)  * pos_gain
    ang_vel     = quat_to_rotvec(quat_error)   * rot_gain
    speedL([lin_vel, ang_vel], accel, 2*dt)

Two threads:
  _reader_loop   — polls the Quest at 50 Hz (mirrors DROID _update_internal_state)
  _control_loop  — P-controller + speedL at 50 Hz (mirrors _calculate_action)

Reader watchdog:
  If the controller pose goes stale (no change for > watchdog_timeout) while
  teleop is enabled — Quest asleep, Wi-Fi drop, ADB hiccup — the robot is
  stopped immediately and a trip latch is set. The latch clears only once the
  grip is seen released, so the operator must consciously re-press grip
  (which also re-baselines origins) before motion resumes.
"""

import threading
import time

import numpy as np

from .filters import VRPoseFilter
from .transforms import (
    limit_velocity,
    quat_conj,
    quat_diff,
    quat_mult,
    quat_to_rmat,
    quat_to_rotvec,
    rmat_to_quat,
    rotvec_to_quat,
    twist_about_axis,
    vec_to_reorder_mat,
)


class QuestTeleop:
    """Quest → robot teleop. `robot`, `gripper`, `reader` are injected so the
    same loop runs against the real robot or the dry-run back-end."""

    def __init__(
        self,
        robot,
        reader,
        gripper=None,
        mode: str = 'speed',
        pos_gain: float = 3.0,
        rot_gain: float = 2.0,
        max_lin_vel: float = 0.15,   # m/s   — conservative
        max_rot_vel: float = 0.75,   # rad/s
        control_hz: float = 50.0,
        acceleration: float = 0.5,   # m/s²
        right_controller: bool = True,
        rmat_reorder: list = None,
        watchdog_timeout: float = 0.25,  # s without a fresh pose → stop
        filter_alpha: float = 0.8,   # one-pole low-pass on VR pose (0 = off)
        pos_scale: float = 1.0,      # motion scaling (vr delta → robot delta)
        precision_scale: float = 0.5,  # pos_scale multiplier in precision mode
    ):
        if mode != 'speed':
            raise ValueError(f"unknown mode '{mode}' (servo arrives in Phase 3)")
        self.mode        = mode
        self.robot       = robot
        self.reader      = reader
        self.gripper     = gripper
        self.pos_gain    = pos_gain
        self.rot_gain    = rot_gain
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.dt          = 1.0 / control_hz
        self.accel       = acceleration
        self.watchdog_timeout = watchdog_timeout
        self.pos_scale        = pos_scale
        self.precision_scale  = precision_scale

        rmat_reorder = rmat_reorder or [-2, -1, -3, 4]
        self.global_to_env = vec_to_reorder_mat(rmat_reorder)
        # The VR-frame axis that the reorder maps to env "up" (+Z): used as the
        # yaw axis for the forward reset so gravity stays vertical.
        self._vr_up = self.global_to_env[:3, :3].T @ np.array([0.0, 0.0, 1.0])

        self.cid      = 'r' if right_controller else 'l'
        self.grip_key = self.cid.upper() + 'G'
        self.joy_key  = self.cid.upper() + 'J'
        self.trig_key = 'rightTrig' if self.cid == 'r' else 'leftTrig'
        # Secondary face button (A / X) toggles precision (slow) mode.
        self.precision_key = 'A' if self.cid == 'r' else 'X'

        # ---- shared state ----
        self._lock             = threading.Lock()
        self._poses            = {}
        self._buttons          = {}
        self._enabled          = False
        self._vr_to_global     = np.eye(4)
        self._reset_orientation = True
        self._watchdog_tripped  = False
        self._filter           = VRPoseFilter(alpha=filter_alpha)
        self._precision        = False
        self._prev_precision_btn = False

        # DROID-style origins (set on first tick after grip pressed)
        self._robot_origin_pos  = None
        self._robot_origin_quat = None
        self._vr_origin_pos     = None
        self._vr_origin_quat    = None

        self._running = False

    # ------------------------------------------------------------------
    # Background reader thread  (DROID _update_internal_state)
    # ------------------------------------------------------------------

    def _reader_loop(self, hz: float = 50.0):
        while self._running:
            time.sleep(1.0 / hz)
            poses, buttons = self.reader.poll()
            if not poses:
                continue

            with self._lock:
                prev_enabled = self._enabled
                now_enabled  = bool(buttons.get(self.grip_key, False))
                toggled      = prev_enabled != now_enabled

                # A watchdog trip is only cleared by a deliberate grip release
                if self._watchdog_tripped and not now_enabled:
                    self._watchdog_tripped = False
                    print("Watchdog cleared — press grip to resume.")

                # Precision (slow) mode: edge-detected toggle on A / X
                precision_btn = bool(buttons.get(self.precision_key, False))
                if precision_btn and not self._prev_precision_btn:
                    self._precision = not self._precision
                    print(f"Precision mode {'ON' if self._precision else 'OFF'} "
                          f"(pos_scale x{self.precision_scale if self._precision else 1.0:.2f})")
                self._prev_precision_btn = precision_btn

                # Orientation alignment — keep updating until joystick clicked or grip pressed.
                # Yaw-only: keep only the controller's rotation about the env-vertical
                # axis so tilt at click time doesn't tilt the mapped workspace.
                if self._reset_orientation or buttons.get(self.joy_key, False):
                    raw  = np.asarray(poses.get(self.cid, np.eye(4)))
                    stop = buttons.get(self.joy_key, False) or now_enabled
                    if stop:
                        self._reset_orientation = False
                    try:
                        q_raw  = rmat_to_quat(raw[:3, :3])
                        yaw_q  = twist_about_axis(q_raw, self._vr_up)
                        inv_yaw = quat_to_rmat(quat_conj(yaw_q))
                        V = np.eye(4)
                        V[:3, :3] = inv_yaw
                        self._vr_to_global = V
                    except (np.linalg.LinAlgError, ValueError):
                        self._vr_to_global = np.eye(4)
                        self._reset_orientation = True

                # Clear origins on every grip toggle (forces re-baseline)
                if toggled:
                    self._robot_origin_pos  = None
                    self._robot_origin_quat = None
                    self._vr_origin_pos     = None
                    self._vr_origin_quat    = None
                    self._filter.reset()

                self._poses   = poses
                self._buttons = buttons
                self._enabled = now_enabled

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _stop_motion(self):
        """Mode-appropriate immediate stop (speedStop now, servoStop in Phase 3)."""
        self.robot.speed_stop(self.accel)

    def _check_watchdog(self) -> bool:
        """Returns True (and stops the robot) if the pose stream went stale."""
        stale = self.reader.staleness(self.cid)
        if stale <= self.watchdog_timeout:
            return False
        with self._lock:
            self._watchdog_tripped = True
        self._stop_motion()
        print(f"WATCHDOG: no fresh Quest pose for {stale:.2f}s — robot stopped. "
              "Release and re-press grip to resume.")
        return True

    # ------------------------------------------------------------------
    # Control loop  (DROID _calculate_action mapped to speedL)
    # ------------------------------------------------------------------

    def _control_loop(self):
        print("\n--- Quest Teleop running (mode: %s) ---" % self.mode)
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
                enabled      = self._enabled and not self._watchdog_tripped
                vr_to_global = self._vr_to_global.copy()
                origin_ready = self._robot_origin_pos is not None
                precision    = self._precision

            # ---- gripper (always active, independent of grip deadman) ----
            if self.gripper is not None:
                trig = float(buttons.get(self.trig_key, [0.0])[0])
                self.gripper.update(trig)

            # ---- not enabled: stop and idle ----
            if not enabled:
                if prev_enabled:
                    self._stop_motion()
                    print("Grip released — robot stopped.")
                prev_enabled = False
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            # ---- reader watchdog (Quest sleep / Wi-Fi drop / ADB hiccup) ----
            if self._check_watchdog():
                prev_enabled = False
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            if not prev_enabled:
                print("Grip pressed — setting origins ...")
            prev_enabled = True

            # ---- current TCP pose ----
            tcp          = self.robot.get_tcp_pose()   # [x,y,z, rx,ry,rz]
            current_pos  = np.array(tcp[:3])
            current_quat = rotvec_to_quat(np.array(tcp[3:]))

            # ---- VR pose in env frame (DROID _process_reading) ----
            if self.cid not in poses:
                time.sleep(max(0.0, self.dt - (time.time() - t0)))
                continue

            raw     = np.asarray(poses[self.cid])
            mat     = self.global_to_env @ vr_to_global @ raw
            vr_pos  = mat[:3, 3]
            vr_quat = rmat_to_quat(mat[:3, :3])

            # ---- one-pole low-pass on the VR pose (Open-Teach Filter) ----
            # First sample after a re-baseline passes through unfiltered, so the
            # origin below is captured from the same (filtered) stream.
            vr_pos, vr_quat = self._filter(vr_pos, vr_quat)

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

            # Position: robot is displaced from its origin by the controller's
            # displacement from its origin, scaled by pos_scale (× precision).
            scale      = self.pos_scale * (self.precision_scale if precision else 1.0)
            target_pos = ro_pos + scale * (vr_pos - vo_pos)
            pos_error  = target_pos - current_pos
            lin_vel    = pos_error * self.pos_gain

            # Rotation: apply the VR displacement rotation onto the robot origin orientation
            dR_vr       = quat_diff(vr_quat, vo_quat)          # rotation from vr_origin → vr_now
            target_quat = quat_mult(dR_vr, ro_quat)            # apply to robot origin
            quat_error  = quat_diff(target_quat, current_quat) # remaining rotation error
            if quat_error[0] < 0:                              # shortest path
                quat_error = -quat_error
            ang_vel = quat_to_rotvec(quat_error) * self.rot_gain

            # ---- uniform saturation (DROID): scale both by one factor ----
            lin_vel, ang_vel = limit_velocity(
                lin_vel, ang_vel, self.max_lin_vel, self.max_rot_vel)

            # ---- send to robot ----
            # time = 2×dt: robot auto-stops if commands stop for one period
            self.robot.speed_l(
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
        # Ordering matters: halt motion first, then kill the URScript program,
        # then release peripherals.
        try:
            self._stop_motion()
            self.robot.stop_script()
        except Exception:
            pass
        if self.gripper is not None:
            self.gripper.open()
            self.gripper.disconnect()
        try:
            self.robot.disconnect()
        except Exception:
            pass
        self.reader.stop()
        print("Teleop stopped.")
