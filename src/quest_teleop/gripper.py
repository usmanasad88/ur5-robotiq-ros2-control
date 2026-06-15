"""Robotiq 2F gripper via the URCap socket adapter (port 63352).

Moved unchanged from quest_rtde_teleop.py; adds a dry-run stub with the same
interface so the teleop loop never special-cases the gripper.
"""

import numpy as np

try:
    from robotiq_2f_urcap_adapter_socket.robotiq_2f_socket_adapter import (
        Robotiq2fSocketAdapter,
    )
    GRIPPER_AVAILABLE = True
except ImportError:
    GRIPPER_AVAILABLE = False


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


class DryRunGripper:
    """Logs gripper commands instead of sending them."""

    def __init__(self, deadband: float = 0.05):
        self._last_trig = -1.0
        self._deadband = deadband

    def update(self, trigger: float):
        trigger = float(np.clip(trigger, 0.0, 1.0))
        if abs(trigger - self._last_trig) < self._deadband:
            return
        self._last_trig = trigger
        print(f"[dry-run] gripper move to {int(trigger * 255)}/255")

    def open(self):
        print("[dry-run] gripper open")

    def disconnect(self):
        pass


def make_gripper(robot_ip: str, enabled: bool, dry_run: bool):
    """Build the right gripper object for the session (or None)."""
    if not enabled:
        return None
    if dry_run:
        return DryRunGripper()
    if not GRIPPER_AVAILABLE:
        print("WARNING: robotiq_2f_urcap_adapter_socket not found — "
              "gripper disabled.")
        return None
    try:
        return GripperController(robot_ip)
    except Exception as exc:
        print(f"WARNING: gripper init failed ({exc}). Continuing without gripper.")
        return None
