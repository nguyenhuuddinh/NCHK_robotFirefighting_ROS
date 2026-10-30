"""App-only Nav2 launch owner: lifecycle teardown before ROS context shutdown.

Operator/mission STOP verification finishes before StackCleanup signals this
role. No motion/arm/pump publishers exist here. All descendants stay in the
runner-owned process group; the original group/child audits remain authoritative.
"""
import json
import os
import signal
import subprocess
import sys
import time

RPC_BUDGET = 12.
LAUNCH_INT_BUDGET = 8.
LAUNCH_TERM_BUDGET = 3.
OUTER_INT_BUDGET = RPC_BUDGET + LAUNCH_INT_BUDGET + LAUNCH_TERM_BUDGET + 6.
NAV_PREFIX = ['ros2', 'launch', '--noninteractive', 'fire_robot_navigation',
              'nav2_composed_shadow.launch.py']


def diagnostic(event, **fields):
    print('APP_NAV_SHUTDOWN ' + json.dumps(dict(event=event, **fields)), flush=True)


def shutdown_rpc():
    # Created only after the nav role receives STOP. A separate child bounds
    # native ROS calls even if one holds the Python GIL indefinitely.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .nav_teardown import drain_lifecycle
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fire_robot_desktop_nav_shutdown')
    deadline = time.monotonic() + RPC_BUDGET - 1.
    try:
        return drain_lifecycle(node, rclpy, deadline, diagnostic)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def supervise(argv, *, rpc_command=None, diagnostic_path=None):
    if argv[:len(NAV_PREFIX)] != NAV_PREFIX and rpc_command is None:
        raise ValueError('APP_NAV_COMMAND_INVALID')
    stopped, forced = [False], [False]
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    signal.signal(signal.SIGINT, lambda *_: stopped.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: forced.__setitem__(0, True))
    child = helper = performance = None
    stage, deadline = 'RUNNING', None
    try:
        if stopped[0] or forced[0]:
            return 0
        # No setsid here: wrapper, ros2 launch, container and RPC helper remain
        # in the one group owned by LocalProcesses/OwnedProcess.
        child = subprocess.Popen(argv, stdin=subprocess.DEVNULL)
        if diagnostic_path is not None:
            from .nav_performance import NavPerformance
            performance = NavPerformance(diagnostic_path, child.pid)
        while True:
            now = time.monotonic()
            if forced[0]:
                diagnostic('OUTER_TERMINATE')
                if child.poll() is None:
                    child.send_signal(signal.SIGTERM)
                return 2  # Original outer group audit must still account for descendants.
            rc = child.poll()
            if rc is not None:
                return rc if rc >= 0 else 128 - rc
            if performance is not None:
                performance.sample()
            if stopped[0] and stage == 'RUNNING':
                diagnostic('LIFECYCLE_BEGIN')
                helper = subprocess.Popen(rpc_command or [sys.executable, '-m',
                    'services.nav_session', '--rpc'], stdin=subprocess.DEVNULL)
                stage, deadline = 'RPC', now + RPC_BUDGET
            if stage == 'RPC' and (helper.poll() is not None or now >= deadline):
                if helper.poll() is None:
                    helper.kill()
                    helper.wait(timeout=1.)
                    diagnostic('LIFECYCLE_HELPER_TIMEOUT')
                else:
                    diagnostic('LIFECYCLE_HELPER_EXIT', rc=helper.returncode)
                child.send_signal(signal.SIGINT)
                diagnostic('LAUNCH_SIGINT')
                stage, deadline = 'INT', time.monotonic() + LAUNCH_INT_BUDGET + 1.
            elif stage == 'INT' and now >= deadline:
                diagnostic('LAUNCH_SIGTERM')
                child.send_signal(signal.SIGTERM)
                stage, deadline = 'TERM', now + LAUNCH_TERM_BUDGET
            elif stage == 'TERM' and now >= deadline:
                # The outer owner independently escalates against the whole
                # group. Do not claim clean or lose the still-live leader here.
                diagnostic('LAUNCH_STILL_LIVE')
                child.kill()
                child.wait(timeout=1.)
                return 2
            time.sleep(.03)
    finally:
        if performance is not None:
            performance.close()
        if helper is not None and helper.poll() is None:
            helper.kill()
            helper.wait(timeout=1.)
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    if sys.argv[1:] == ['--rpc']:
        return shutdown_rpc()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--diagnostics')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] != ['--']:
        raise ValueError('APP_NAV_COMMAND_INVALID')
    return supervise(args.command[1:], diagnostic_path=args.diagnostics)


if __name__ == '__main__':
    raise SystemExit(main())
