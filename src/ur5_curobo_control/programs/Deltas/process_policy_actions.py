#!/usr/bin/env python3
"""
Process policy action JSONL files into robot .prog files with jointdelta instructions.

The JSONL format (from LeRobot / cuRobo policy rollouts) contains one JSON
object per step.  The field ``target_joint_values`` holds the absolute
6-DOF joint positions in radians that the policy wants to reach at each
step.  This script computes the per-step incremental deltas and writes them
as a sequence of ``jointdelta`` instructions that the UR5 program executor
can replay.

Gripper handling
----------------
The ``gripper_action`` field encodes the desired gripper state (negative =
open, positive = close).  Since gripper commands are sent via a slow
subprocess call, we use a threshold approach: an ``opengripper`` or
``closegripper`` command is emitted only when ``gripper_action`` crosses
zero.  This naturally splits the trajectory into batches separated by
gripper commands — the executor batches consecutive ``jointdelta`` lines
into a single smooth multi-waypoint trajectory, so each batch remains
jerk-free.

Usage
-----
    python process_policy_actions.py latest_policy_actions.jsonl
    python process_policy_actions.py latest_policy_actions.jsonl \\
        --start lerobot_start --step-time 0.1 --output my_traj.prog

Output .prog file structure
---------------------------
    movetonamed(<start_position>)
    set_step_time(<step_time>)
    jointdelta([d1, d2, d3, d4, d5, d6])
    ...
    closegripper              # emitted only at transitions
    jointdelta([d1, d2, d3, d4, d5, d6])
    ...
"""

import argparse
import json
import os
import sys


# gripper_action > 0 → close, < 0 → open.
# A threshold of 0.5 adds comfortable hysteresis against noise near zero.
GRIPPER_CLOSE_THRESHOLD = 0.5


def load_steps(jsonl_path: str) -> list:
    steps = []
    with open(jsonl_path, "r") as fh:
        for line in fh:
            line = line.strip()
            if line:
                steps.append(json.loads(line))
    return steps


def is_gripper_closed(step: dict) -> bool:
    return float(step.get("gripper_action", -1.0)) > GRIPPER_CLOSE_THRESHOLD


def build_prog(steps: list, start_position: str, step_time: float) -> str:
    if not steps:
        sys.exit("ERROR: JSONL file is empty")

    if "target_joint_values" not in steps[0]:
        sys.exit("ERROR: 'target_joint_values' field missing from JSONL entries")

    lines = [
        f"# Auto-generated from policy JSONL ({len(steps)} steps)",
        f"# Task: {steps[0].get('task', 'unknown')}",
        "",
        f"movetonamed({start_position})",
        f"set_step_time({step_time})",
        "",
        "# --- Policy trajectory (joint deltas in radians) ---",
    ]

    prev_joints = steps[0]["target_joint_values"]
    prev_closed = is_gripper_closed(steps[0])
    gripper_transitions = 0

    for step in steps[1:]:
        # Check for gripper transition before emitting the delta
        curr_closed = is_gripper_closed(step)
        if curr_closed != prev_closed:
            gripper_transitions += 1
            if curr_closed:
                lines.append("closegripper")
            else:
                lines.append("opengripper")
            prev_closed = curr_closed

        curr_joints = step["target_joint_values"]
        delta = [curr_joints[j] - prev_joints[j] for j in range(6)]
        delta_str = ", ".join(f"{d:.6f}" for d in delta)
        lines.append(f"jointdelta([{delta_str}])")
        prev_joints = curr_joints

    lines.append("")
    lines.append("# --- End of policy trajectory ---")

    return "\n".join(lines) + "\n", gripper_transitions


def main():
    parser = argparse.ArgumentParser(
        description="Convert a policy-action JSONL file to a UR5 .prog trajectory."
    )
    parser.add_argument(
        "jsonl_file",
        help="Path to the input .jsonl file (e.g. latest_policy_actions.jsonl)",
    )
    parser.add_argument(
        "--start",
        default="lerobot_start",
        metavar="NAMED_POSITION",
        help="Named position in named_positions.txt to move to before replaying "
             "(default: lerobot_start)",
    )
    parser.add_argument(
        "--step-time",
        type=float,
        default=0.1,
        metavar="SECONDS",
        help="Duration in seconds for each jointdelta step (default: 0.1)",
    )
    parser.add_argument(
        "--output",
        metavar="FILE",
        help="Output .prog file path.  Defaults to <jsonl_basename>.prog in the "
             "same directory as the input file.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.jsonl_file):
        sys.exit(f"ERROR: File not found: {args.jsonl_file}")

    steps = load_steps(args.jsonl_file)
    print(f"Loaded {len(steps)} steps from {args.jsonl_file}")

    prog_text, gripper_transitions = build_prog(steps, args.start, args.step_time)

    if args.output:
        out_path = args.output
    else:
        base = os.path.splitext(args.jsonl_file)[0]
        out_path = base + ".prog"

    with open(out_path, "w") as fh:
        fh.write(prog_text)

    print(f"Written: {out_path}  ({len(steps) - 1} jointdelta steps, "
          f"{gripper_transitions} gripper transitions)")

    # Print a brief summary of the joint range covered
    all_joints = [s["target_joint_values"] for s in steps]
    for j in range(6):
        vals = [q[j] for q in all_joints]
        print(f"  Joint {j+1}: {min(vals):.4f} → {max(vals):.4f} rad "
              f"(range {max(vals)-min(vals):.4f} rad)")


if __name__ == "__main__":
    main()
