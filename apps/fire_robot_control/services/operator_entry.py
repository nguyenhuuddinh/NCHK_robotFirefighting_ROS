"""Candidate operator command; not installed in the sealed R5 runtime."""
import argparse
import os
import sys
from pathlib import Path

from .operator_bootstrap import read_bootstrap_fd
from .operator_runner import run_integrated


def main(argv=None, *, run=run_integrated):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bootstrap-fd", type=int, required=True)
    parser.add_argument("--enable-integrated-candidate", action="store_true", required=True)
    parser.add_argument("--ui-diagnostic", type=Path)
    args, ros_args = parser.parse_known_args(argv)
    try:
        bootstrap = read_bootstrap_fd(args.bootstrap_fd)
    except ValueError:
        print("Bootstrap operator không hợp lệ", file=sys.stderr)
        return 2
    finally:
        if args.bootstrap_fd >= 3:
            try:
                os.close(args.bootstrap_fd)
            except OSError:
                pass
    # rclpy receives a normal argv including a program name, followed only by
    # the ROS arguments intended for the operator node.
    options = {} if args.ui_diagnostic is None else {
        'diagnostic_path': args.ui_diagnostic,
        'result_path': args.ui_diagnostic.with_name('operator_result.json')}
    return run(bootstrap, args=["fire_robot_integrated_operator", *ros_args], **options)


if __name__ == "__main__":
    raise SystemExit(main())
