"""Tests for the freshness tracking that feeds the teleop watchdog."""

import numpy as np

from quest_teleop.reader import FreshnessTracker


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def pose(seed):
    rng = np.random.default_rng(seed)
    m = np.eye(4)
    m[:3, 3] = rng.normal(size=3)
    return m


def test_unseen_controller_is_infinitely_stale():
    tracker = FreshnessTracker(clock=FakeClock())
    assert tracker.staleness('r') == float('inf')


def test_changing_pose_stays_fresh():
    clock = FakeClock()
    tracker = FreshnessTracker(clock=clock)
    for i in range(5):
        clock.t = i * 0.02
        tracker.update({'r': pose(i)})
    assert tracker.staleness('r') == 0.0


def test_frozen_pose_goes_stale():
    clock = FakeClock()
    tracker = FreshnessTracker(clock=clock)
    frozen = pose(1)
    tracker.update({'r': frozen})
    # Stream dies: same cached matrix re-served for 300 ms
    for i in range(1, 16):
        clock.t = i * 0.02
        tracker.update({'r': frozen.copy()})
    assert tracker.staleness('r') >= 0.25


def test_recovery_resets_staleness():
    clock = FakeClock()
    tracker = FreshnessTracker(clock=clock)
    frozen = pose(1)
    tracker.update({'r': frozen})
    clock.t = 1.0
    tracker.update({'r': frozen})
    assert tracker.staleness('r') == 1.0
    tracker.update({'r': pose(2)})
    assert tracker.staleness('r') == 0.0


def test_controllers_tracked_independently():
    clock = FakeClock()
    tracker = FreshnessTracker(clock=clock)
    frozen_l = pose(10)
    tracker.update({'r': pose(0), 'l': frozen_l})
    clock.t = 0.5
    tracker.update({'r': pose(1), 'l': frozen_l})
    assert tracker.staleness('r') == 0.0
    assert tracker.staleness('l') == 0.5
