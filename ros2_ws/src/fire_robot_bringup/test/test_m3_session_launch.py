"""M3 wrapper reuses static I/O with isolated command gate and snapshot Nav2."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from test_wireless_static_launch import _includes, _source_path, _text


ROOT = Path(__file__).resolve().parents[1]


def test_m3_profile_has_only_io_nav_and_minimal_supervisor_includes():
    path = ROOT / 'launch/robot_wireless_io_nav_static.launch.py'
    spec = importlib.util.spec_from_file_location('m3_io_base', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: f'/tmp/{name}'
    ld = module.generate_launch_description(mission_shadow=True)
    includes = _includes(ld.entities)
    by_file = {Path(_source_path(item)).name: dict(item.launch_arguments) for item in includes}
    assert set(by_file) == {'robot.launch.py', 'nav2_snapshot_shadow.launch.py', 'mission_session_shadow.launch.py'}
    robot = by_file['robot.launch.py']
    assert _text(robot['cmd_vel_input_topic']) == '/fire_robot_static/cmd_vel_raw'
    assert _text(robot['enable_diagnostics']) == _text(robot['use_dashboard']) == 'false'
    assert _text(by_file['nav2_snapshot_shadow.launch.py']['rviz']) == 'false'
    args = {item.name: item for item in ld.entities if isinstance(item, DeclareLaunchArgument)}
    assert args['initialize_shadow_budget'].default_value[0].text == 'false'
    assert 'shadow_budget_file' in args


def test_old_io_profile_keeps_original_nav2_and_no_mission():
    path = ROOT / 'launch/robot_wireless_io_nav_static.launch.py'
    spec = importlib.util.spec_from_file_location('old_io_base', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: f'/tmp/{name}'
    includes = _includes(module.generate_launch_description().entities)
    assert {Path(_source_path(item)).name for item in includes} == {'robot.launch.py', 'nav2_motion_shadow.launch.py'}
