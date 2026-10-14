#!/usr/bin/env python3
"""Gracefully stop this robot's ROS processes on the laptop only.

No ROS graph-wide kill is attempted: graph nodes may belong to the Pi.
"""

import argparse
import os
from pathlib import Path
import signal

import psutil


ROOT = Path(__file__).resolve().parents[1]
LAPTOP_PACKAGES = frozenset({
    'fire_robot_navigation', 'fire_robot_perception', 'fire_robot_mission',
})
ROBOT_TOPICS = frozenset({'/cmd_vel_raw', '/cmd_vel', '/pump_cmd'})


def _within_workspace(cwd):
    """Return whether the process was started in this checkout."""
    return bool(cwd) and Path(cwd).resolve().is_relative_to(ROOT)


def classify(argv, cwd):
    """Classify only known laptop-side robot processes."""
    if not argv:
        return None
    for arg in argv:
        if Path(arg).name == 'fire_robot_session.py':
            script = Path(arg)
            if not script.is_absolute():
                script = Path(cwd or '/') / script
            if script.resolve() == ROOT / 'scripts/fire_robot_session.py':
                return 'session'

    ros2_at = next((i for i, arg in enumerate(argv)
                    if Path(arg).name == 'ros2'), None)
    if ros2_at is not None:
        command = argv[ros2_at + 1:]
        if len(command) >= 2 and command[0] == 'launch' and \
                command[1] in LAPTOP_PACKAGES:
            return 'launch'
        if len(command) >= 2 and command[0] == 'run' and \
                command[1] in LAPTOP_PACKAGES | {'teleop_twist_keyboard'}:
            return 'run'
        if command[:2] == ['topic', 'pub'] and any(
                arg in ROBOT_TOPICS or
                arg.startswith('/fire_mission/') and arg.endswith('_arm')
                for arg in command):
            return 'publisher'

    if not _within_workspace(cwd):
        return None
    executable = argv[0]
    if executable.startswith(str(ROOT / 'ros2_ws/install') + '/') and \
            '/lib/fire_robot_' in executable:
        return 'node'
    if executable.startswith('/opt/ros/humble/lib/nav2_') or \
            executable.startswith('/opt/ros/humble/lib/slam_toolbox/') or \
            executable.startswith(
                '/opt/ros/humble/lib/teleop_twist_keyboard/'):
        return 'node'
    if Path(executable).name in ('rviz2', 'rqt_image_view'):
        return 'viewer'
    if 'fire_robot_perception.yolo_observer' in argv or any(
            Path(arg).name == 'view_yolo_camera.py' for arg in argv):
        return 'viewer'
    return None


def collect():
    """Take a fresh snapshot of matching processes owned by this user."""
    found = []
    for process in psutil.process_iter(['uids', 'cmdline']):
        try:
            uids = process.info['uids']
            if process.pid == os.getpid() or uids is None or \
                    uids.real != os.getuid():
                continue
            argv = process.info['cmdline'] or []
            kind = classify(argv, process.cwd())
            if kind:
                found.append((kind, process))
        except (psutil.AccessDenied, psutil.NoSuchProcess,
                psutil.ZombieProcess, OSError):
            continue
    return found


def interrupt(kinds, timeout):
    """Send SIGINT to the currently matching processes of selected kinds."""
    targets = [(kind, proc) for kind, proc in collect() if kind in kinds]
    for kind, process in targets:
        try:
            print(f'SIGINT {process.pid} {kind}: '
                  f'{" ".join(process.cmdline())[:140]}', flush=True)
            process.send_signal(signal.SIGINT)
        except (psutil.AccessDenied, psutil.NoSuchProcess,
                psutil.ZombieProcess):
            pass
    if targets:
        psutil.wait_procs([proc for _, proc in targets], timeout=timeout)


def main():
    """Withdraw laptop publishers, then let launch parents clean children."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true',
                        help='List exact local targets without stopping them')
    args = parser.parse_args()
    targets = collect()
    if not targets:
        print('Không còn tiến trình ROS của robot trên laptop.')
        return 0
    if args.dry_run:
        for kind, process in targets:
            print(f'{process.pid} {kind}: '
                  f'{" ".join(process.cmdline())[:140]}')
        return 0

    print('Chỉ dừng tiến trình laptop; không SSH hoặc dừng node Pi.',
          flush=True)
    interrupt({'publisher'}, 2)
    interrupt({'session'}, 30)
    interrupt({'launch', 'run'}, 10)
    interrupt({'node', 'viewer'}, 5)
    remaining = collect()
    for kind, process in remaining:
        try:
            print(f'SIGTERM {process.pid} {kind}', flush=True)
            process.terminate()
        except (psutil.AccessDenied, psutil.NoSuchProcess,
                psutil.ZombieProcess):
            pass
    if remaining:
        psutil.wait_procs([proc for _, proc in remaining], timeout=3)
    remaining = collect()
    if remaining:
        print('Còn tiến trình cần kiểm tra thủ công: ' +
              ', '.join(str(proc.pid) for _, proc in remaining))
        return 1
    print('Đã dừng các tiến trình robot trên laptop. Node Pi vẫn chạy.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
