"""Packaging and process-prefix plumbing; never starts ROS nodes or Pi."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys

import pytest

from services.candidate_build import build

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT/'apps/fire_robot_control'
SOURCE = ROOT/'releases/r5_20261029/ros'


@pytest.fixture
def candidate(tmp_path):
    folder = tmp_path/'candidate with spaces'
    build(SOURCE, APP, folder)
    return folder


def test_app_launch_loads_and_declares_parameters_without_starting_nodes(candidate):
    env = dict(os.environ, ROS_LOG_DIR=str(candidate.parent/'launch-logs'))
    result = subprocess.run(['ros2', 'launch', '--show-args',
        str(candidate/'desktop/generated/app_mission.launch.py')],
        capture_output=True, text=True, timeout=20, env=env)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'external_ir_preview' in result.stdout


@pytest.mark.parametrize('role', ['live_nav_stop_gateway', 'nav2_cancel_guard'])
def test_wrapper_installs_only_app_classes_and_preserves_argv(candidate, tmp_path, role):
    script = tmp_path/role
    script.write_text("import rclpy, sys\n"
        "from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore, CycleEndpoint\n"
        "from fire_robot_mission.nav2_cancel_guard import CycleEndpoint as GuardEndpoint\n"
        "assert not rclpy.ok()\n"
        "assert LiveNavStopCore.__name__ == 'AppNavCore'\n"
        "assert CycleEndpoint.__name__ == GuardEndpoint.__name__ == 'AppCycleEndpoint'\n"
        "assert sys.argv[1:] == ['--ros-args','-p','a:=value with spaces']\n"
        "print('APP_NODE_WRAPPER_NO_ROS_INIT_OK')\n")
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join((str(candidate/'desktop'), str(SOURCE/'ros2_ws/src/fire_robot_mission'), env.get('PYTHONPATH', '')))
    result = subprocess.run([sys.executable, '-B', '-m', 'services.app_mission_node', '--',
        str(script), '--ros-args', '-p', 'a:=value with spaces'],
        capture_output=True, text=True, env=env, timeout=10)
    assert result.returncode == 0 and 'NO_ROS_INIT_OK' in result.stdout, result.stdout+result.stderr


def test_r5_source_and_launch_remain_pinned(candidate):
    for line in (SOURCE/'operational_packet.sha256').read_text().splitlines():
        digest, rel = line.split('  ', 1)
        assert hashlib.sha256((SOURCE/rel).read_bytes()).hexdigest() == digest
        assert hashlib.sha256((candidate/rel).read_bytes()).hexdigest() == digest
    launch = (candidate/'desktop/generated/app_mission.launch.py').read_text()
    assert "if name in ('live_nav_stop_gateway','nav2_cancel_guard')" in launch


def test_wrapper_preserves_launch_child_names_for_cleanup_audit(candidate):
    code = '''import runpy, sys
from launch import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node
description = runpy.run_path(sys.argv[1])['generate_launch_description']()
context = LaunchContext()
count = 0
for action in description.entities:
    if isinstance(action, Node):
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable))
        if executable == 'ir_range_preview':
            continue
        label = perform_substitutions(context, action.process_description.name)
        assert label == executable, (label, executable)
        prefix = perform_substitutions(context, action.process_description.prefix)
        assert ('services.app_mission_node' in prefix) == (executable in ('live_nav_stop_gateway','nav2_cancel_guard'))
        count += 1
assert count == 9
print('LAUNCH_NAMES_AND_PREFIXES_OK_NO_NODE_EXECUTE')
'''
    env = dict(os.environ, ROS_LOG_DIR=str(candidate.parent/'launch-inspect-logs'))
    result = subprocess.run([sys.executable, '-B', '-c', code,
        str(candidate/'desktop/generated/app_mission.launch.py')],
        capture_output=True, text=True, env=env, timeout=10)
    assert result.returncode == 0 and 'NO_NODE_EXECUTE' in result.stdout, result.stdout+result.stderr
