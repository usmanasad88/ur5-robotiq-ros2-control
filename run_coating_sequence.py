#!/home/mani/Repos/ur_ws/.venv/bin/python3
"""
Coating Sequence Runner
=======================
Executes the coating preparation programs one-by-one, advancing to the
next program on each "Next" press of the wireless slide changer.

Sequence
--------
  1. move_hardener_from_storage_to_workplace
  2. move_resin_from_storage_to_workplace
  3. move_roller_from_storage_to_workplace
  4. move_hardener_from_workplace_to_storage
  5. move_resin_from_workplace_to_storage
  6. move_roller_from_workplace_to_storage

Controls (Logitech presenter or any USB slide changer)
------------------------------------------------------
  Next  (PageDown / Right / Space / F5)  → advance to next step
  Prev  (PageUp  / Left)                 → go back one step
  Stop  (S / Esc)                        → stop current motion & pause sequence
  Q / Ctrl-C                             → quit

Usage
-----
  python3 run_coating_sequence.py           # uses default API at localhost:5050
  python3 run_coating_sequence.py --port 5050
  python3 run_coating_sequence.py --host 192.168.1.10 --port 5050
"""

from __future__ import annotations

import argparse
import selectors
import sys
import threading
import time
from enum import Enum, auto

import requests

# ---------------------------------------------------------------------------
# Program sequence
# ---------------------------------------------------------------------------

SEQUENCE = [
    "move_hardener_from_storage_to_workplace.prog",
    "move_resin_from_storage_to_workplace.prog",
    "move_roller_from_storage_to_workplace.prog",
    "move_hardener_from_workplace_to_storage.prog",
    "move_resin_from_workplace_to_storage.prog",
    "move_roller_from_workplace_to_storage.prog",
]

# ---------------------------------------------------------------------------
# Presenter key codes (mirrored from program_executor_node.py)
# ---------------------------------------------------------------------------

NEXT_KEYS: set[int] = set()
PREV_KEYS: set[int] = set()
STOP_KEYS: set[int] = set()

try:
    import evdev
    NEXT_KEYS = {
        evdev.ecodes.KEY_PAGEDOWN,
        evdev.ecodes.KEY_RIGHT,
        evdev.ecodes.KEY_SPACE,
        evdev.ecodes.KEY_N,
        evdev.ecodes.KEY_F5,
    }
    PREV_KEYS = {
        evdev.ecodes.KEY_PAGEUP,
        evdev.ecodes.KEY_LEFT,
        evdev.ecodes.KEY_P,
        evdev.ecodes.KEY_BACKSPACE,
    }
    STOP_KEYS = {
        evdev.ecodes.KEY_S,
        evdev.ecodes.KEY_ESC,
    }
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("⚠  evdev not found — falling back to stdin keyboard input")
    print("   Install with: pip install evdev")

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class RunnerState(Enum):
    WAITING   = auto()   # waiting for "next" press to start/continue
    EXECUTING = auto()   # program is running on the robot
    STOPPED   = auto()   # user-requested stop / emergency


class SequenceRunner:
    def __init__(self, api_base: str):
        self.api_base = api_base.rstrip("/")
        self.state = RunnerState.WAITING
        self.current_idx = 0          # index into SEQUENCE
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ API helpers

    def _post(self, path: str, body: dict | None = None, timeout: float = 10.0):
        try:
            r = requests.post(f"{self.api_base}{path}", json=body or {}, timeout=timeout)
            return r.json()
        except Exception as e:
            print(f"  [API ERROR] POST {path}: {e}")
            return {"success": False, "message": str(e)}

    def _get(self, path: str, timeout: float = 5.0):
        try:
            r = requests.get(f"{self.api_base}{path}", timeout=timeout)
            return r.json()
        except Exception as e:
            print(f"  [API ERROR] GET {path}: {e}")
            return {}

    def _executor_is_idle(self) -> bool:
        """Return True when the executor node reports IDLE (program done/stopped)."""
        data = self._get("/api/status")
        # The executor publishes "<STATE>: <message>", bridged into the REST status
        # The external_control_api doesn't expose executor_state directly, but
        # executor_running tells us the process is alive.  To detect "done" we
        # poll until the last POST /api/program/execute returns and then just wait
        # a generous fixed time, OR we can rely on a short poll of /api/status.
        # The executor node publishes status on ~/status topic; the REST API does
        # not forward it, so we use a simple approach: keep polling /api/status and
        # watch executor_running.  When the program finishes the executor goes back
        # to IDLE — but executor_running stays True (the process is still up).
        #
        # Best we can do via the REST layer: issue execute, then poll status every
        # 0.5 s checking that joint velocities have settled.  A cleaner heuristic:
        # the execute service call already blocks until the program starts; we then
        # poll for idle by issuing a lightweight GET /api/status and checking if
        # the executor process is still marked running (it always is while the node
        # lives).  Instead, we subscribe to the ROS topic indirectly: after
        # issuing execute we just poll /api/status and when joint velocities stay
        # near zero for 1 s we call it done.
        js = data.get("joint_state")
        if js is None:
            return False
        vels = js.get("velocities", [])
        if not vels:
            return True
        return all(abs(v) < 0.005 for v in vels)

    def _wait_for_completion(self, program: str, poll_interval: float = 0.5, settle_time: float = 1.5):
        """Poll until robot joints have settled (program finished)."""
        print(f"  ⏳ Waiting for '{program}' to complete …")
        settled_since: float | None = None
        # Give the motion time to actually start before we check velocities
        time.sleep(2.0)
        while True:
            with self._lock:
                if self.state == RunnerState.STOPPED:
                    return

            if self._executor_is_idle():
                if settled_since is None:
                    settled_since = time.time()
                elif time.time() - settled_since >= settle_time:
                    print(f"  ✓ '{program}' finished (joints settled)")
                    return
            else:
                settled_since = None

            time.sleep(poll_interval)

    # ------------------------------------------------------------------ sequence control

    def _run_step(self, idx: int):
        """Load and execute the program at *idx*; block until done."""
        program = SEQUENCE[idx]
        print(f"\n[{idx + 1}/{len(SEQUENCE)}] Executing: {program}")

        result = self._post("/api/program/execute", {"program": program}, timeout=15.0)
        if not result.get("success"):
            print(f"  ✗ Failed to start: {result.get('message', '?')}")
            with self._lock:
                self.state = RunnerState.WAITING
            return

        print(f"  ▶ Started")
        with self._lock:
            self.state = RunnerState.EXECUTING

        self._wait_for_completion(program)

        with self._lock:
            if self.state == RunnerState.EXECUTING:
                self.state = RunnerState.WAITING

    def handle_next(self):
        with self._lock:
            state = self.state
            idx   = self.current_idx

        if state == RunnerState.STOPPED:
            print("[Runner] Stopped — press S/Esc to reset stop, then Next to continue")
            return

        if state == RunnerState.EXECUTING:
            # Presenter "next" while running → pause robot
            print("[Runner] Pausing …")
            self._post("/api/program/pause")
            with self._lock:
                self.state = RunnerState.WAITING
            return

        if state == RunnerState.WAITING:
            if idx >= len(SEQUENCE):
                print("[Runner] Sequence complete — press Prev to go back or restart")
                return

            # Kick off execution in a background thread so the key listener stays live
            t = threading.Thread(target=self._run_and_advance, args=(idx,), daemon=True)
            t.start()

    def _run_and_advance(self, idx: int):
        self._run_step(idx)
        with self._lock:
            if self.state != RunnerState.STOPPED:
                self.current_idx = idx + 1
                if self.current_idx >= len(SEQUENCE):
                    print("\n🎉 All programs complete! Press Prev to go back or Ctrl-C to quit.")
                else:
                    print(f"\n[Runner] Ready — press Next to run step {self.current_idx + 1}/{len(SEQUENCE)}: "
                          f"{SEQUENCE[self.current_idx]}")

    def handle_prev(self):
        with self._lock:
            state = self.state

        if state == RunnerState.EXECUTING:
            print("[Runner] Stopping current program …")
            self._post("/api/program/stop")
            with self._lock:
                self.state = RunnerState.WAITING

        with self._lock:
            self.current_idx = max(0, self.current_idx - 1)
            print(f"[Runner] Stepped back — ready at step {self.current_idx + 1}/{len(SEQUENCE)}: "
                  f"{SEQUENCE[self.current_idx]}")

    def handle_stop(self):
        print("[Runner] STOP requested — stopping robot")
        self._post("/api/program/stop")
        with self._lock:
            self.state = RunnerState.STOPPED
        print("[Runner] Robot stopped. Press S/Esc again to clear STOP, or Ctrl-C to quit.")

    def clear_stop(self):
        """Toggle STOP → WAITING so the sequence can be resumed."""
        with self._lock:
            if self.state == RunnerState.STOPPED:
                self.state = RunnerState.WAITING
                print("[Runner] STOP cleared — ready to continue")

    def status_line(self) -> str:
        with self._lock:
            idx   = self.current_idx
            state = self.state
        prog = SEQUENCE[idx] if idx < len(SEQUENCE) else "(done)"
        return f"State={state.name}  Step={idx + 1}/{len(SEQUENCE)}  Next={prog}"


# ---------------------------------------------------------------------------
# Input listeners
# ---------------------------------------------------------------------------

def evdev_listener(runner: SequenceRunner, stop_event: threading.Event):
    """Read key events from evdev devices (works without a terminal)."""
    keyboard_devices = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
            caps = device.capabilities()
            if evdev.ecodes.EV_KEY in caps:
                key_caps = caps[evdev.ecodes.EV_KEY]
                if evdev.ecodes.KEY_PAGEDOWN in key_caps or evdev.ecodes.KEY_RIGHT in key_caps:
                    keyboard_devices.append(device)
                    print(f"  Found presenter device: {device.name} ({path})")
        except Exception:
            continue

    if not keyboard_devices:
        print("⚠  No presenter/keyboard evdev devices found — using stdin fallback")
        stdin_listener(runner, stop_event)
        return

    sel = selectors.DefaultSelector()
    for dev in keyboard_devices:
        sel.register(dev, selectors.EVENT_READ)

    stop_was_pressed = False

    try:
        while not stop_event.is_set():
            events = sel.select(timeout=0.1)
            for key, _ in events:
                device = key.fileobj
                try:
                    for event in device.read():
                        if event.type == evdev.ecodes.EV_KEY and event.value == 1:
                            if event.code in NEXT_KEYS:
                                runner.handle_next()
                            elif event.code in PREV_KEYS:
                                runner.handle_prev()
                            elif event.code in STOP_KEYS:
                                # First press stops; second press clears stop
                                if runner.state == RunnerState.STOPPED:
                                    runner.clear_stop()
                                else:
                                    runner.handle_stop()
                except BlockingIOError:
                    pass
    finally:
        sel.close()
        for dev in keyboard_devices:
            dev.close()


def stdin_listener(runner: SequenceRunner, stop_event: threading.Event):
    """Fallback stdin key listener for terminals."""
    import termios
    import tty
    import select as _select

    print("Keyboard controls: [N/Space/→/PgDn] Next  [P/←/PgUp] Prev  [S/Esc] Stop  [Q] Quit")

    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        while not stop_event.is_set():
            if _select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)

                if key == '\x1b':  # escape sequence
                    if _select.select([sys.stdin], [], [], 0.05)[0]:
                        k2 = sys.stdin.read(1)
                        if k2 == '[':
                            if _select.select([sys.stdin], [], [], 0.05)[0]:
                                k3 = sys.stdin.read(1)
                                if k3 == 'C':    runner.handle_next()          # →
                                elif k3 == 'D':  runner.handle_prev()          # ←
                                elif k3 == '5':  sys.stdin.read(1); runner.handle_prev()  # PgUp
                                elif k3 == '6':  sys.stdin.read(1); runner.handle_next()  # PgDn
                        else:
                            # bare Esc
                            if runner.state == RunnerState.STOPPED:
                                runner.clear_stop()
                            else:
                                runner.handle_stop()
                elif key in (' ', 'n'):
                    runner.handle_next()
                elif key == 'p':
                    runner.handle_prev()
                elif key == 's':
                    if runner.state == RunnerState.STOPPED:
                        runner.clear_stop()
                    else:
                        runner.handle_stop()
                elif key in ('q', '\x03'):   # q or Ctrl-C
                    print("\nQuitting …")
                    stop_event.set()
                    break
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Coating sequence runner via slide changer")
    parser.add_argument("--host", default="localhost", help="API host (default: localhost)")
    parser.add_argument("--port", type=int, default=5050, help="API port (default: 5050)")
    args = parser.parse_args()

    api_base = f"http://{args.host}:{args.port}"

    print("=" * 60)
    print("  Coating Sequence Runner")
    print(f"  API: {api_base}")
    print("=" * 60)
    print()

    # Verify API is reachable
    try:
        r = requests.get(f"{api_base}/api/status", timeout=5)
        r.raise_for_status()
        print("✓ API reachable")
    except Exception as e:
        print(f"✗ Cannot reach API at {api_base}: {e}")
        print("  Is run_external_api.sh running?")
        sys.exit(1)

    runner = SequenceRunner(api_base)

    print()
    print("Sequence:")
    for i, prog in enumerate(SEQUENCE):
        print(f"  {i + 1}. {prog}")
    print()
    print(f"Ready — press Next to start step 1: {SEQUENCE[0]}")
    print()

    stop_event = threading.Event()

    try:
        if EVDEV_AVAILABLE:
            evdev_listener(runner, stop_event)
        else:
            stdin_listener(runner, stop_event)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping robot …")
        runner._post("/api/program/stop")

    print("Done.")


if __name__ == "__main__":
    main()
