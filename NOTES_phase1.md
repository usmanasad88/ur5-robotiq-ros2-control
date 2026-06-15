# Phase 1 notes — wireless operation + reader watchdog

## What I read

### oculus_reader (installed package, the source of truth for Phase 1)
`~/miniconda3/envs/ur5_python/lib/python3.10/site-packages/oculus_reader/reader.py`

- `OculusReader(ip_address=..., port=5555)` already supports wireless: with
  `ip_address` set it calls `client.remote_connect(ip, port)` and looks up the
  device as `"<ip>:<port>"` (`get_network_device`, reader.py:57-75). USB is the
  default (`ip_address=None` → `get_usb_device`, reader.py:77-88, which picks
  the first serial containing fewer than 3 dots, i.e. not an ip:port serial).
- Its own error handling calls `exit(1)` inside the library on failure
  (reader.py:72, 88) — unacceptable mid-session, so we run our **own ADB
  preflight** (`quest_teleop/reader.py: adb_preflight`) before constructing
  `OculusReader`, producing actionable messages (`adb tcpip 5555`,
  `adb connect <ip>:5555`, "accept the prompt in the headset") and raising a
  normal exception instead of exiting.
- **Key fact for the watchdog**: `get_transformations_and_buttons()`
  (reader.py:180-182) returns the *last cached* `last_transforms` /
  `last_buttons` with **no timestamp**. The logcat thread only overwrites them
  when a new line arrives (reader.py:184-199). If the Quest sleeps or Wi-Fi
  drops, the API silently keeps returning a *frozen* pose — with grip held the
  robot would keep chasing a stale target. There is no liveness signal in the
  API at all.

## What I'm porting / building

1. **Freshness by change-detection.** Real controller tracking jitters at
   ~72 Hz, so two bit-identical consecutive 4×4 pose matrices mean "no new
   data". `QuestReader.poll()` stamps `time.monotonic()` whenever the raw
   matrix for a controller *changes* (`np.array_equal` against previous);
   `staleness(cid)` returns seconds since the last change. This is the only
   way to get freshness without forking the oculus_reader APK/logcat path.
2. **Watchdog policy** (`teleop.py`): while teleop is enabled, if staleness
   exceeds `--watchdog-timeout` (default 0.25 s) → immediately command the
   mode-appropriate stop (`speedStop` now; `servoStop` when servo mode lands
   in Phase 3), and latch a trip flag. The latch clears only after the grip is
   seen *released* — so the operator must consciously re-press grip (which
   also re-baselines origins, existing behavior) before motion resumes.

### Quest2ROS2 (`References/Quest2ROS2/q2r2_bringup/robot_arm_controller_base.py`)
Checked for prior art on stale-input safety: it has none of this kind — its
timeouts are ROS-plumbing (TF `lookup_transform` timeout, gripper action
server `wait_for_server`). Its input arrives as ROS topics from the Quest2ROS
APK, where liveness is implicit in message arrival. Nothing to port for
Phase 1; its clutch/relative-motion logic is Phase 2 material.

### Open-Teach / openteach-controller-fork
Not relevant to Phase 1 (ZMQ transport from a Unity APK, no ADB). Deferred to
Phases 3-5.

## Environment note (this session)
`ur-rtde` is not currently importable in any local python env, and the robot
is unreachable — Phase 1 is validated via `--dry-run` (no RTDE connection,
`DryRunRobot` integrates commanded velocities and logs) and pytest unit tests
for the math moved into `quest_teleop/transforms.py`.
