"""Quest input: OculusReader wrapper with ADB preflight and freshness tracking.

Why this exists
---------------
- oculus_reader's own error paths call exit(1) from inside the library, so we
  validate the ADB connection *before* constructing OculusReader and raise a
  QuestConnectionError with actionable hints instead.
- OculusReader.get_transformations_and_buttons() returns the last *cached*
  values with no timestamp — if the Quest sleeps or Wi-Fi drops it silently
  keeps returning a frozen pose. Real controller tracking jitters at ~72 Hz,
  so two bit-identical consecutive pose matrices mean the stream is dead.
  QuestReader stamps the last time each controller's matrix changed and
  exposes staleness() for the teleop watchdog.
"""

import shutil
import subprocess
import time

import numpy as np


class QuestConnectionError(RuntimeError):
    """ADB / Quest connectivity problem, with setup hints in the message."""


def _run_adb(args, timeout=10.0):
    try:
        out = subprocess.run(
            ["adb"] + args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise QuestConnectionError(
            f"adb {' '.join(args)} timed out after {timeout}s. "
            "Try `adb kill-server && adb start-server`.")
    return out.stdout + out.stderr


def _adb_stdout(args, timeout=10.0):
    """adb stdout only (keeps `* daemon ...` startup chatter out of parsing)."""
    try:
        out = subprocess.run(
            ["adb"] + args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise QuestConnectionError(
            f"adb {' '.join(args)} timed out after {timeout}s. "
            "Try `adb kill-server && adb start-server`.")
    return out.stdout


def adb_preflight(quest_ip=None, port=5555):
    """Verify a Quest is reachable over ADB before handing off to OculusReader.

    Raises QuestConnectionError with setup instructions on failure.
    """
    if shutil.which("adb") is None:
        raise QuestConnectionError(
            "adb not found on PATH.\n"
            "  sudo apt install android-tools-adb")

    # Start the server explicitly so its startup chatter doesn't land in the
    # device-list parsing below.
    _run_adb(["start-server"])

    if quest_ip is not None:
        serial = f"{quest_ip}:{port}"
        out = _run_adb(["connect", serial])
        if "connected" not in out:  # matches both "connected to" / "already connected"
            raise QuestConnectionError(
                f"adb connect {serial} failed:\n  {out.strip()}\n"
                "Wireless ADB setup (one-time per reboot of the Quest):\n"
                f"  1. Connect the Quest via USB, accept the prompt in the headset\n"
                f"  2. adb tcpip {port}\n"
                f"  3. Find the IP: adb shell ip route   (or Settings → Wi-Fi)\n"
                f"  4. adb connect <quest-ip>:{port}  — then unplug USB\n"
                "Make sure the Quest and this host are on the same network and "
                "the headset is awake (proximity sensor uncovered).")
        # `adb connect` can report success yet leave the device "offline";
        # check the actual state.
        state = _device_state(serial)
        if state != "device":
            raise QuestConnectionError(
                f"{serial} is '{state or 'missing'}' (want 'device').\n"
                f"Try `adb disconnect {serial}`, wake the headset, then rerun. "
                f"If it persists, redo `adb tcpip {port}` over USB.")
        print(f"Quest preflight OK — wireless ADB at {serial}")
        return serial

    # USB path: first device whose serial is not an ip:port
    devices = _list_devices()
    usb = [(s, st) for s, st in devices if ":" not in s]
    if not usb:
        raise QuestConnectionError(
            "No USB ADB device found.\n"
            "  1. Connect the Quest via USB-C and check the cable supports data\n"
            "  2. Enable Developer Mode (Meta Quest app → Devices)\n"
            "  3. Put on the headset and accept the 'Allow USB debugging' prompt\n"
            "  4. Verify with: adb devices\n"
            "(For wireless operation pass --quest-ip <ip> instead.)")
    serial, state = usb[0]
    if state == "unauthorized":
        raise QuestConnectionError(
            f"Device {serial} is unauthorized — put on the headset and accept "
            "the 'Allow USB debugging' prompt, then rerun.")
    if state != "device":
        raise QuestConnectionError(
            f"Device {serial} is '{state}' (want 'device'). "
            "Reconnect the cable or run `adb kill-server && adb start-server`.")
    print(f"Quest preflight OK — USB ADB device {serial}")
    return serial


_VALID_STATES = {"device", "unauthorized", "offline", "authorizing",
                 "no permissions", "recovery", "bootloader"}


def _list_devices():
    out = _adb_stdout(["devices"])
    devices = []
    for line in out.splitlines():
        line = line.strip()
        # Skip the header and any "* daemon ... *" chatter that reached stdout.
        if not line or line.startswith("*") or line.startswith("List of devices"):
            continue
        parts = line.split(None, 1)
        if len(parts) == 2 and parts[1].strip() in _VALID_STATES:
            devices.append((parts[0], parts[1].strip()))
    return devices


def _device_state(serial):
    for s, state in _list_devices():
        if s == serial:
            return state
    return None


class FreshnessTracker:
    """Stamps the last time each controller's raw pose matrix *changed*.

    update() compares against the previous matrix with exact equality: real
    tracking data jitters every frame, so a bit-identical repeat means the
    underlying stream stopped (cached value being re-served).
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._prev_mats = {}
        self._last_fresh = {}

    def update(self, poses):
        now = self._clock()
        for cid, mat in poses.items():
            prev = self._prev_mats.get(cid)
            if prev is None or not np.array_equal(prev, mat):
                self._last_fresh[cid] = now
                self._prev_mats[cid] = np.array(mat, copy=True)

    def staleness(self, cid):
        """Seconds since controller `cid`'s pose last changed (inf if never)."""
        t = self._last_fresh.get(cid)
        return float("inf") if t is None else self._clock() - t


class QuestReader:
    """OculusReader + per-controller freshness tracking.

    poll() must be called regularly (the teleop reader thread does); it
    timestamps each controller whose raw 4x4 pose matrix changed since the
    previous poll. staleness(cid) is the watchdog input.
    """

    def __init__(self, quest_ip=None, port=5555):
        adb_preflight(quest_ip, port)
        # Import here so --help / preflight errors don't require the package
        from oculus_reader.reader import OculusReader
        self.reader = OculusReader(ip_address=quest_ip, port=port)
        self._freshness = FreshnessTracker()

    def poll(self):
        """Return (poses, buttons); update freshness stamps. Never raises."""
        try:
            poses, buttons = self.reader.get_transformations_and_buttons()
        except Exception:
            return {}, {}
        poses = poses or {}
        self._freshness.update(poses)
        return poses, (buttons or {})

    def staleness(self, cid):
        """Seconds since controller `cid`'s pose last changed (inf if never)."""
        return self._freshness.staleness(cid)

    def stop(self):
        try:
            self.reader.stop()
        except Exception:
            pass
