# Phase 2 notes — motion quality

Study of DROID VRPolicy, Quest2ROS2, and Open-Teach *before* writing code, and
what each does differently from my current `quest_teleop` loop.

## My current code (`src/quest_teleop/teleop.py`, end of Phase 1)
- Absolute position map: `target_pos = ro + (vr_pos - vo)`; `lin_vel = err*pos_gain`; `speedL`.
- Velocity limits: lin and ang clamped **independently** (`teleop.py` saturation
  block) → when only one saturates, the screw axis is bent.
- **No** low-pass on the VR pose — raw controller jitter passes straight through.
- Forward reset: `_vr_to_global = inv(raw_click)` — removes the controller's
  **full** orientation (yaw *and* pitch/roll) and translation at click time.
- Motion scale fixed at 1:1, no precision mode.

## DROID VRPolicy (mirrored from the paper; not vendored in References/)
- Same absolute position map and `dR_vr * robot_origin_quat` rotation — my code
  already follows this (header docstring cites `_calculate_action`).
- **`_limit_velocity`: scales lin AND ang by the *same* factor** = `min` of the
  two per-axis ratios, so the commanded twist keeps its direction (screw axis)
  when saturated. ← this is the uniform-saturation requirement.
- `rmat_reorder` axis remap — already ported (`vec_to_reorder_mat`).
- Reset uses the full inverse of the controller rotation (no yaw-only step);
  the yaw-only refinement below is mine, motivated by gravity-vertical safety.

## Quest2ROS2 (`References/Quest2ROS2/q2r2_bringup/robot_arm_controller_base.py`)
- **Clutch / relative motion**: `offset = quest_now - first_quest`; `target =
  robot_init + offset` (`_pose_callback`, lines 246-267). Position streaming to a
  Cartesian controller `target_frame`, *no* velocity command and *no* velocity
  limit at all — so nothing to port for uniform saturation, but it confirms the
  origin/anchor pattern I already use.
- **Pause-and-reset** (`_inputs_callback`, lines 356-381): lower button *latches*
  pose streaming on/off and re-anchors **both** robot and quest origins
  (sets `initial_orientation=None` to force re-anchor on the next sample) and
  clears the filter history. My grip deadman already re-baselines origins on
  every toggle; I add the matching **filter reset** on re-baseline.
- **Filter**: moving-average over a deque window on position and on quaternion
  (linear mean + renormalize, lines 166-208) — heavier latency than a one-pole;
  I use the Open-Teach one-pole instead (below).
- Orientation: `q_rel = q_now * inv(q_init)`, `q_target = q_rel * q_robot_init`
  (lines 253-262) — identical to my `dR_vr` formulation.
- **No yaw-only decomposition.** Gripper is an edge-detected open/close *toggle*
  (`_toggle_gripper`), unlike my proportional trigger — I keep proportional.

## Open-Teach FrankaArmOperator (`References/Open-Teach/openteach/components/operators/franka.py`)
- **One-pole complementary filter** (`Filter`, lines 22-33): position EMA
  `p = c*p_prev + (1-c)*p_new`; orientation `scipy Slerp` from prev toward new by
  `(1-c)`; `comp_ratio=0.8`. ← exactly the Phase 2 low-pass. They filter the
  *final robot* pose; the spec wants it on the *VR* pose, so I apply it there.
  I reimplement `slerp` with my own quats (no scipy at runtime; scipy stays a
  test-only reference).
- **Resolution scaling** (`_get_scaled_cart_pose`, lines 147-165): scales the
  translation delta by `resolution_scale`, toggled `1.0` (high) / `0.6` (low) by
  a button (`_get_resolution_scale_mode`). ← motion scaling + precision mode.
  They scale delta-from-current; I scale delta-from-origin (cleaner with my
  absolute map): `target_pos = ro + scale*(vr_pos - vo)`.
- Re-anchors robot + hand init on the STOP→CONT transition (`_reset_teleop`).

## What I'm porting (and where)
1. **Uniform saturation** ← DROID `_limit_velocity`: `transforms.limit_velocity`
   (pure, scales both by `min` ratio), unit-tested.
2. **One-pole low-pass** ← Open-Teach `Filter`: `transforms.slerp` (pure, tested
   vs `scipy ... Slerp`) + stateful `filters.VRPoseFilter` (pos EMA + quat slerp,
   `--filter-alpha` default 0.8), applied to the VR pose; reset on re-baseline.
3. **Yaw-only forward reset** (mine — neither reference does it): swing-twist
   decomposition `transforms.twist_about_axis` about the env-vertical axis
   derived from the reorder matrix (`global_to_env.T @ [0,0,1]`), keeping only
   the yaw so controller tilt at click doesn't tilt the workspace. Unit-tested.
4. **Motion scaling + precision toggle** ← Open-Teach resolution scale:
   `--pos-scale` (default 1.0), precision button (`A`/`X`, edge-detected) →
   `--precision-scale` (default 0.5).
5. **Keep `--mode speed`** working unchanged (servoL is Phase 3).

Validated by pytest (slerp/limit_velocity/twist vs scipy + VRPoseFilter
behavior) and a live `--dry-run` Quest session.
