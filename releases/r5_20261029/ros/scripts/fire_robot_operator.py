#!/usr/bin/env python3
"""Open the installed R5 intent observer; does not launch or control the robot."""
import argparse


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--show-gui', action='store_true', help='Show camera, status and intent-only keys')
    return parser.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    from fire_robot_mission.operational_session import main as operator_main
    return operator_main(['--ros-args', '-p', 'show_gui:='+str(args.show_gui).lower()])


if __name__ == '__main__':
    raise SystemExit(main())
