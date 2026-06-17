"""Integration tests for the QuestTeleop control loop in speed vs servo mode.

Drives the real threaded loop with a scripted fake reader and a recording
DryRunRobot — no Quest, no robot — to verify mode branching, the servo
per-cycle displacement clamp, and stop ordering.
"""

import threading
import time

import numpy as np

from quest_teleop.robot import DryRunRobot
from quest_teleop.teleop import QuestTeleop


class FakeReader:
    """Grip held the whole time; right controller translates +X each poll."""

    def __init__(self, step=0.02):
        self._n = 0
        self._step = step

    def poll(self):
        self._n += 1
        mat = np.eye(4)
        mat[:3, 3] = [self._step * self._n, 0.0, 0.0]   # moves → never stale
        return {'r': mat}, {'RG': True}

    def staleness(self, cid):
        return 0.0

    def stop(self):
        pass


class PulseTriggerReader:
    """Grip held; trigger pulses high for `hi` polls then low for `lo`, N times."""

    def __init__(self, pulses=3, hi=4, lo=4):
        seq = []
        for _ in range(pulses):
            seq += [1.0] * hi + [0.0] * lo
        self._seq = seq
        self._i = 0

    def poll(self):
        trig = self._seq[min(self._i, len(self._seq) - 1)]
        self._i += 1
        mat = np.eye(4)
        mat[:3, 3] = [0.001 * self._i, 0.0, 0.0]   # tiny motion → never stale
        return {'r': mat}, {'RG': True, 'rightTrig': (trig,)}

    def staleness(self, cid):
        return 0.0

    def stop(self):
        pass


class RecordGripper:
    def __init__(self):
        self.toggles = 0
        self.updates = 0

    def update(self, trigger):
        self.updates += 1

    def toggle(self):
        self.toggles += 1

    def open(self):
        pass

    def disconnect(self):
        pass


class RecordRobot(DryRunRobot):
    def __init__(self):
        super().__init__(log_period=1e9)   # suppress logging
        self.servo_calls = []
        self.speed_calls = []
        self.servo_stops = 0
        self.speed_stops = 0

    def servo_l(self, pose, dt, lookahead_time, gain):
        self.servo_calls.append(np.asarray(pose, float))
        super().servo_l(pose, dt, lookahead_time, gain)

    def speed_l(self, velocity, accel, t):
        self.speed_calls.append(np.asarray(velocity, float))
        super().speed_l(velocity, accel, t)

    def servo_stop(self):
        self.servo_stops += 1
        super().servo_stop()

    def speed_stop(self, accel):
        self.speed_stops += 1
        super().speed_stop(accel)


def _run(teleop, seconds=0.4):
    th = threading.Thread(target=teleop.run, daemon=True)
    th.start()
    time.sleep(seconds)
    teleop.stop()
    th.join(timeout=1.0)


def test_servo_mode_calls_servo_l_not_speed_l():
    robot = RecordRobot()
    teleop = QuestTeleop(robot, FakeReader(), gripper=None, mode='servo',
                         control_hz=100.0, max_lin_vel=0.15, max_rot_vel=0.75)
    _run(teleop)
    assert robot.servo_calls, "servo mode should call servo_l"
    assert not robot.speed_calls, "servo mode must not call speed_l"
    assert robot.servo_stops >= 1 and robot.speed_stops == 0


def test_speed_mode_calls_speed_l_not_servo_l():
    robot = RecordRobot()
    teleop = QuestTeleop(robot, FakeReader(), gripper=None, mode='speed',
                         control_hz=100.0)
    _run(teleop)
    assert robot.speed_calls, "speed mode should call speed_l"
    assert not robot.servo_calls, "speed mode must not call servo_l"
    assert robot.speed_stops >= 1 and robot.servo_stops == 0


def test_servo_per_cycle_step_is_clamped():
    # Fast controller motion + small clamp: consecutive commanded TCP positions
    # must not jump more than max_pos_step (DryRunRobot snaps to each command).
    robot = RecordRobot()
    hz = 100.0
    max_lin = 0.15
    teleop = QuestTeleop(robot, FakeReader(step=0.05), gripper=None, mode='servo',
                         control_hz=hz, max_lin_vel=max_lin, pos_scale=1.0)
    _run(teleop, seconds=0.5)
    max_step = max_lin / hz
    poses = robot.servo_calls
    assert len(poses) >= 3
    deltas = [np.linalg.norm(poses[i+1][:3] - poses[i][:3])
              for i in range(len(poses) - 1)]
    assert max(deltas) <= max_step + 1e-6, f"max step {max(deltas)} > {max_step}"
    # and motion actually happened (not stuck)
    assert max(deltas) > 0.0


def test_gripper_toggle_fires_once_per_press():
    grip = RecordGripper()
    teleop = QuestTeleop(RecordRobot(), PulseTriggerReader(pulses=3, hi=5, lo=5),
                         gripper=grip, mode='speed', control_hz=200.0,
                         gripper_mode='toggle')
    _run(teleop, seconds=0.6)
    # 3 trigger pulses → exactly 3 toggles (edge-detected, not per-tick)
    assert grip.toggles == 3, f"expected 3 toggles, got {grip.toggles}"
    assert grip.updates == 0


def test_gripper_proportional_calls_update_not_toggle():
    grip = RecordGripper()
    teleop = QuestTeleop(RecordRobot(), PulseTriggerReader(pulses=2),
                         gripper=grip, mode='speed', control_hz=200.0,
                         gripper_mode='proportional')
    _run(teleop, seconds=0.4)
    assert grip.toggles == 0
    assert grip.updates > 0


def test_explicit_max_pos_step_overrides_default():
    robot = RecordRobot()
    teleop = QuestTeleop(robot, FakeReader(step=0.05), gripper=None, mode='servo',
                         control_hz=100.0, max_pos_step=0.001)
    _run(teleop, seconds=0.4)
    deltas = [np.linalg.norm(robot.servo_calls[i+1][:3] - robot.servo_calls[i][:3])
              for i in range(len(robot.servo_calls) - 1)]
    assert deltas and max(deltas) <= 0.001 + 1e-6
