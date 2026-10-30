"""App-owned scheduling of the unchanged R5 startup health checker."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys


def health_command(*args):
    return [sys.executable, "-B", "-m", "services.app_health_entry", *map(str, args)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--role", choices=("pi", "laptop"), required=True)
    # Keep the deployed R5 scheduler until loaded-Pi acceptance of sampled mode.
    parser.add_argument("--mode", choices=("r5", "sampled"), default="r5")
    a, ros_args = parser.parse_known_args(argv)
    from fire_robot_mission import startup_health as original
    if a.mode == "r5":
        return original.main(["--out", str(a.out), "--session", a.session, "--role", a.role, *ros_args])
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    stop = [False]
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    node = None
    try:
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        node = original.StartupHealthNode(a.session, a.role)
        def sample():
            data = node.sample()
            temp = a.out.with_suffix(".tmp")
            temp.write_text(json.dumps(data, allow_nan=False) + "\n")
            os.replace(temp, a.out)
        node.create_timer(.2, sample)
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
