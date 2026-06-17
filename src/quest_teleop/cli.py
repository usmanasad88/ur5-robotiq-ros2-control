"""CLI entry point. All flags from quest_rtde_teleop.py are preserved;
Phase 1 adds --quest-ip/--quest-port (wireless ADB), --watchdog-timeout,
and --dry-run."""

import argparse
import signal
import sys

from .gripper import make_gripper
from .reader import QuestConnectionError, QuestReader
from .robot import DryRunRobot, URRobot
from .teleop import QuestTeleop


def build_parser():
    parser = argparse.ArgumentParser(
        prog='quest_teleop',
        description='Quest 3S → UR5 VR teleop via ur_rtde (DROID-style P-controller)')
    parser.add_argument('--robot-ip',      default='172.17.66.105')
    parser.add_argument('--backend',       choices=['rtde', 'ros'], default='rtde',
                        help="rtde: direct RTDE to a real robot / URSim (default). "
                             "ros: Jacobian IK → /forward_velocity_controller, "
                             "drives the launch_all.sh sim (fake or real hardware).")
    parser.add_argument('--controller',    choices=['right', 'left'], default='right')
    parser.add_argument('--mode',          choices=['speed', 'servo'], default='speed',
                        help="speed: speedL Cartesian velocity (default). "
                             "servo: servoL position streaming (rtde backend only).")
    parser.add_argument('--pos-gain',      type=float, default=3.0)
    parser.add_argument('--rot-gain',      type=float, default=2.0)
    parser.add_argument('--max-lin-vel',   type=float, default=0.15,
                        help='Max linear speed [m/s] (default 0.15 — conservative)')
    parser.add_argument('--max-rot-vel',   type=float, default=0.75,
                        help='Max angular speed [rad/s]')
    parser.add_argument('--hz',            type=float, default=50.0)
    parser.add_argument('--no-gripper',    action='store_true',
                        help='Disable gripper control')
    parser.add_argument('--gripper-mode',  choices=['toggle', 'proportional'],
                        default='toggle',
                        help="toggle: trigger press flips open/close (default). "
                             "proportional: trigger 0->1 maps to open->closed.")
    parser.add_argument('--rmat-reorder',  nargs=4, type=int, default=[-2, -1, -3, 4],
                        metavar='N',
                        help='Axis reorder (default: -2 -1 -3 4). '
                             'Flip sign to negate an axis, swap indices to swap axes.')
    # ---- Phase 1 ----
    parser.add_argument('--quest-ip',      default=None, metavar='IP',
                        help='Quest IP for wireless ADB (default: USB). '
                             'One-time setup: adb tcpip 5555 over USB, then unplug.')
    parser.add_argument('--quest-port',    type=int, default=5555,
                        help='Wireless ADB port (default 5555)')
    parser.add_argument('--watchdog-timeout', type=float, default=0.25,
                        help='Stop the robot if no fresh Quest pose for this many '
                             'seconds while enabled (default 0.25)')
    # ---- Phase 2 (motion quality) ----
    parser.add_argument('--filter-alpha',  type=float, default=0.8,
                        help='One-pole low-pass on the VR pose: 0 = off, '
                             '→1 = smoother/laggier (default 0.8)')
    parser.add_argument('--pos-scale',     type=float, default=1.0,
                        help='Motion scaling: robot delta = scale x controller '
                             'delta (default 1.0)')
    parser.add_argument('--precision-scale', type=float, default=0.5,
                        help='pos-scale multiplier in precision mode, toggled by '
                             'the A/X button (default 0.5)')
    # ---- servo mode (servoL) ----
    parser.add_argument('--lookahead-time', type=float, default=0.1,
                        help='[--mode servo] servoL lookahead [s] (default 0.1)')
    parser.add_argument('--servo-gain',     type=float, default=300.0,
                        help='[--mode servo] servoL proportional gain (default 300)')
    parser.add_argument('--max-pos-step',   type=float, default=None,
                        help='[--mode servo] per-cycle translation clamp [m] '
                             '(default max_lin_vel/hz)')
    parser.add_argument('--max-rot-step',   type=float, default=None,
                        help='[--mode servo] per-cycle rotation clamp [rad] '
                             '(default max_rot_vel/hz)')
    # ---- ROS backend (Jacobian IK) ----
    parser.add_argument('--max-joint-vel', type=float, default=1.5,
                        help='[--backend ros] max joint speed [rad/s] (default 1.5)')
    parser.add_argument('--damping',       type=float, default=0.05,
                        help='[--backend ros] damped-least-squares λ (default 0.05)')
    parser.add_argument('--dry-run',       action='store_true',
                        help='No robot connection: log commands, integrate a '
                             'virtual TCP. Quest still required.')
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.mode == 'servo' and args.backend == 'ros':
        parser.error("--mode servo requires --backend rtde (servoL has no ROS "
                     "forward_velocity_controller equivalent). Use --mode speed "
                     "for the ros backend.")

    print("Initialising Quest reader (%s) ..." %
          (f"wireless {args.quest_ip}:{args.quest_port}" if args.quest_ip else "USB ADB"))
    try:
        reader = QuestReader(quest_ip=args.quest_ip, port=args.quest_port)
    except QuestConnectionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("OculusReader ready — put on headset and accept ADB prompt if shown.")

    if args.dry_run:
        robot = DryRunRobot()
        gripper = make_gripper(args.robot_ip, enabled=not args.no_gripper,
                               dry_run=True)
    elif args.backend == 'ros':
        from .robot_ros import ROSControlRobot
        robot = ROSControlRobot(max_joint_vel=args.max_joint_vel,
                                damping=args.damping)
        gripper = robot.make_gripper() if not args.no_gripper else None
    else:
        robot = URRobot(args.robot_ip)
        gripper = make_gripper(args.robot_ip, enabled=not args.no_gripper,
                               dry_run=False)

    teleop = QuestTeleop(
        robot            = robot,
        reader           = reader,
        gripper          = gripper,
        mode             = args.mode,
        pos_gain         = args.pos_gain,
        rot_gain         = args.rot_gain,
        max_lin_vel      = args.max_lin_vel,
        max_rot_vel      = args.max_rot_vel,
        control_hz       = args.hz,
        right_controller = (args.controller == 'right'),
        rmat_reorder     = args.rmat_reorder,
        watchdog_timeout = args.watchdog_timeout,
        filter_alpha     = args.filter_alpha,
        pos_scale        = args.pos_scale,
        precision_scale  = args.precision_scale,
        lookahead_time   = args.lookahead_time,
        servo_gain       = args.servo_gain,
        max_pos_step     = args.max_pos_step,
        max_rot_step     = args.max_rot_step,
        gripper_mode     = args.gripper_mode,
    )

    def _sigint(sig, frame):
        print("\nCtrl-C — stopping ...")
        teleop.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    teleop.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
