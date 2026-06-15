"""Stateful pose filtering for VR teleop (one-pole low-pass).

Ported from Open-Teach's FrankaArmOperator.Filter
(References/Open-Teach/openteach/components/operators/franka.py:22-33):
position is an EMA, orientation a slerp toward the new quat. Open-Teach uses
scipy Slerp; here we use the scipy-free transforms.slerp so the runtime has no
scipy dependency. Applied to the VR pose (not the final robot pose).
"""

import numpy as np

from .transforms import slerp


class VRPoseFilter:
    """One-pole low-pass on a (position, quaternion) stream.

    alpha is the smoothing constant in [0, 1):
      - alpha = 0    → no smoothing (pass-through)
      - alpha → 1    → very smooth / laggy
    new = alpha * prev + (1 - alpha) * sample   (position)
    new = slerp(prev, sample, 1 - alpha)         (orientation)

    The first sample after a reset is passed through unfiltered so origins are
    captured from the same stream that is later smoothed.
    """

    def __init__(self, alpha: float = 0.8):
        if not 0.0 <= alpha < 1.0:
            raise ValueError("alpha must be in [0, 1)")
        self.alpha = alpha
        self._pos = None
        self._quat = None

    def reset(self):
        self._pos = None
        self._quat = None

    def __call__(self, pos, quat):
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
        if self.alpha == 0.0:
            return pos.copy(), quat.copy()
        if self._pos is None:
            self._pos = pos.copy()
            self._quat = quat.copy()
            return self._pos.copy(), self._quat.copy()
        self._pos = self.alpha * self._pos + (1.0 - self.alpha) * pos
        self._quat = slerp(self._quat, quat, 1.0 - self.alpha)
        return self._pos.copy(), self._quat.copy()
