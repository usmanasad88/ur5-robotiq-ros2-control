"""Robot back-ends: real ur_rtde connection, or a dry-run simulator.

Both expose the same minimal surface the teleop loop uses:
    get_tcp_pose() -> [x, y, z, rx, ry, rz]
    speed_l(velocity6, accel, t)
    speed_stop(accel)
    stop_script()
    disconnect()

ur_rtde is imported lazily so --dry-run and the unit tests work on machines
without it installed.
"""

import time

import numpy as np

from .transforms import quat_mult, quat_to_rotvec, rotvec_to_quat


class URRobot:
    """Real robot via ur_rtde.

    RTDEReceiveInterface is read-only (coexists with the ROS 2 driver);
    RTDEControlInterface sends URScript directly and is safe while the ROS 2
    trajectory controller is idle.
    """

    def __init__(self, robot_ip: str):
        try:
            import rtde_control
            import rtde_receive
        except ImportError:
            raise SystemExit(
                "ur_rtde not found (pip install ur-rtde) — "
                "use --dry-run to run without a robot.")
        print(f"Connecting to UR5 at {robot_ip} ...")
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        print("Robot connected.")

    def get_tcp_pose(self):
        return self.rtde_r.getActualTCPPose()

    def speed_l(self, velocity, accel, t):
        self.rtde_c.speedL(list(velocity), accel, t)

    def speed_stop(self, accel):
        self.rtde_c.speedStop(accel)

    def stop_script(self):
        self.rtde_c.stopScript()

    def disconnect(self):
        try:
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
        except Exception:
            pass


class DryRunRobot:
    """No-connection back-end: integrates commanded velocities and logs.

    Integration makes the P-controller behave realistically (errors shrink as
    the virtual TCP chases the target), so dry-run sessions exercise the full
    control law, watchdog, and shutdown ordering.
    """

    # A plausible UR5 ready pose: tool pointing down, 40 cm out, 40 cm up.
    START_POSE = [0.4, 0.0, 0.4, 0.0, np.pi, 0.0]

    def __init__(self, robot_ip: str = "", log_period: float = 0.5):
        self._pos = np.array(self.START_POSE[:3])
        self._quat = rotvec_to_quat(np.array(self.START_POSE[3:]))
        self._last_integrate = None
        self._log_period = log_period
        self._last_log = 0.0
        print("DRY RUN — no RTDE connection; commands are logged, "
              f"virtual TCP starts at {self.START_POSE}")

    def get_tcp_pose(self):
        rv = quat_to_rotvec(self._quat)
        return [*self._pos.tolist(), *rv.tolist()]

    def speed_l(self, velocity, accel, t):
        now = time.monotonic()
        dt = 0.0 if self._last_integrate is None else now - self._last_integrate
        self._last_integrate = now
        v = np.asarray(velocity, dtype=float)
        self._pos = self._pos + v[:3] * dt
        dq = rotvec_to_quat(v[3:] * dt)
        self._quat = quat_mult(dq, self._quat)
        self._quat = self._quat / np.linalg.norm(self._quat)
        if now - self._last_log >= self._log_period:
            self._last_log = now
            print(f"[dry-run] speedL lin=[{v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f}] "
                  f"ang=[{v[3]:+.3f},{v[4]:+.3f},{v[5]:+.3f}] "
                  f"accel={accel:.2f} t={t:.3f} | tcp={np.round(self._pos, 3)}")

    def speed_stop(self, accel):
        self._last_integrate = None
        print(f"[dry-run] speedStop(accel={accel:.2f})")

    def stop_script(self):
        print("[dry-run] stopScript()")

    def disconnect(self):
        print("[dry-run] disconnect()")
