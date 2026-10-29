"""Offline process-scope tests for the laptop-only stop command."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parent))
from stop_laptop_nodes import ROOT, classify  # noqa: E402


def test_matches_robot_launch_and_session_only():
    """Match laptop robot owners without matching the Pi bringup launch."""
    cwd = str(ROOT)
    ros2 = '/opt/ros/humble/bin/ros2'
    assert classify(['/usr/bin/python3', ros2, 'launch',
                     'fire_robot_navigation', 'nav2_motion_shadow.launch.py'],
                    cwd) == 'launch'
    assert classify(['/usr/bin/python3',
                     str(ROOT / 'scripts/fire_robot_session.py'),
                     'demo'], cwd) == 'session'
    assert classify(['/usr/bin/python3', ros2, 'launch',
                     'fire_robot_bringup', 'robot.launch.py'], cwd) is None


def test_matches_robot_children_and_arm_but_not_daemon():
    """Cover orphaned Nav2 and arm publishers, leave unrelated tools."""
    cwd = str(ROOT)
    ros2 = '/opt/ros/humble/bin/ros2'
    assert classify(['/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator'],
                    cwd) == 'node'
    assert classify(['/usr/bin/python3', ros2, 'topic', 'pub', '--rate', '5',
                     '/fire_mission/live_nav_arm'], cwd) == 'publisher'
    assert classify(['/usr/bin/python3', '-c',
                     'from ros2cli.daemon.daemonize import main; main()'],
                    cwd) is None
    assert classify(['/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator'],
                    '/tmp/other_robot') is None
