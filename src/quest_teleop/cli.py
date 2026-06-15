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
    parser.add_argument('--controller',    choices=['right', 'left'], default='right')
    parser.add_argument('--mode',          choices=['speed'], default='speed',
                        help="Control mode (servo mode arrives in Phase 3)")
    parser.add_argument('--pos-gain',      type=float, default=3.0)
    parser.add_argument('--rot-gain',      type=float, default=2.0)
    parser.add_argument('--max-lin-vel',   type=float, default=0.15,
                        help='Max linear speed [m/s] (default 0.15 — conservative)')
    parser.add_argument('--max-rot-vel',   type=float, default=0.75,
                        help='Max angular speed [rad/s]')
    parser.add_argument('--hz',            type=float, default=50.0)
    parser.add_argument('--no-gripper',    action='store_true',
                        help='Disable gripper control')
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
    parser.add_argument('--dry-run',       action='store_true',
                        help='No robot connection: log commands, integrate a '
                             'virtual TCP. Quest still required.')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    print("Initialising Quest reader (%s) ..." %
          (f"wireless {args.quest_ip}:{args.quest_port}" if args.quest_ip else "USB ADB"))
    try:
        reader = QuestReader(quest_ip=args.quest_ip, port=args.quest_port)
    except QuestConnectionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print("OculusReader ready — put on headset and accept ADB prompt if shown.")

    robot = DryRunRobot() if args.dry_run else URRobot(args.robot_ip)
    gripper = make_gripper(args.robot_ip, enabled=not args.no_gripper,
                           dry_run=args.dry_run)

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
