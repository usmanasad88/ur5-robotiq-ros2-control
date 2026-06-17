# Phase 3 notes — servoL position-streaming mode

Study of `References/openteach-controller-fork` (deoxys/Franka operator) before
writing code, and what I'm porting to a ur_rtde `servoL` mode.

## openteach-controller-fork operator loop
`openteach/components/operators/franka.py`

- **Constants** (lines 28-35): `CONTROL_FREQ = 60`, `STATE_FREQ = 200`,
  `CONTROLLER_TYPE = "OSC_POSE"`, `TRANSLATION_VELOCITY_LIMIT = 0.1`,
  `ROTATION_VELOCITY_LIMIT = 0.5`. It streams an **absolute target pose** to
  deoxys' OSC position controller at 60 Hz; deoxys does the low-level servo.
  The ur_rtde analogue is `servoL(target_pose, ...)`.
- **Reset-on-engage** (`_apply_retargeted_angles`, lines 348-354): re-anchors
  `robot_init_H` and `hand_init_H` on `is_first_frame` or the STOP→CONT edge —
  same baseline-on-engage my grip deadman already does.
- **Per-cycle displacement clamp** (`arm_control`, lines 451-460) — the key
  port. Each cycle it computes the pose error target↔current, then
  `clip_translation(action_pos, TRANSLATION_VELOCITY_LIMIT)` (scale the
  translation *vector* so its norm ≤ limit) and
  `np.clip(action_axis_angle, ±ROTATION_VELOCITY_LIMIT)`. This bounds how far
  the robot is asked to move **per control cycle** — a step clamp, not a
  velocity clamp. Exactly Phase 3's "max step in pos/rot".
- **comp_filter** = the one-pole `Filter(comp_ratio=0.8)` I already ported in
  Phase 2 (here applied to the robot pose; I apply it to the VR pose).
- **Motion scale** (`_get_scaled_cart_pose`, line 311): `* 0.25` on the
  translation delta — note for the open motion-tuning issue
  ([[quest-teleop-motion-tuning]]); the fork scales translation *down* a lot.
- `deoxys_obs_cmd_history` dict (lines ~462+) logs cartesian cmd / action /
  eef pose / joints / timestamp per cycle — the structure to mirror in Phase 5.

## What I'm porting / building (Phase 3)
1. **`--mode servo`**: target pose from the *same* DROID map (`target_pos`,
   `target_quat`), sent via `rtde_c.servoL(pose, 0, 0, dt, lookahead, gain)` at
   the control rate. Flags `--lookahead-time` (0.1) and `--servo-gain` (300).
2. **Per-cycle displacement clamp** (from `arm_control`): scale the translation
   step to `--max-pos-step` and the rotation step (axis-angle) to
   `--max-rot-step`, defaulting to `max_lin_vel*dt` / `max_rot_vel*dt` so the
   conservative velocity limits carry over. Pure helper
   `transforms.scale_to_max_norm`, unit-tested.
3. **Stop ordering / safety**: `_stop_motion()` becomes mode-aware —
   `servoStop()` in servo mode, `speedStop()` in speed mode — called on grip
   release, watchdog trip, and in `stop()` **before** `stopScript()` and before
   any peripheral teardown. servoStop must precede stopScript for both modes'
   cleanup.
4. **Keep `--mode speed`** unchanged (default for now).

## Scope / caveats
- servoL is a **ur_rtde (RTDE) concept** → servo mode is valid only with
  `--backend rtde` (and `--dry-run`); it errors on `--backend ros` (the ROS
  forward_velocity_controller path has no servoL). The sim the user tests on
  uses the ros backend, so servo can't be exercised there.
- **Default stays `speed`.** The mission says make servo the default *after
  testing*; ur-rtde isn't installed and no real robot/URSim is reachable this
  session, so servoL can't be validated live. Validated here by unit tests
  (step clamp) + `--dry-run` (DryRunRobot tracks the commanded pose) only.
  Flip the default once it's confirmed on hardware.
