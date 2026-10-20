"""Default launch regression plus snapshot-only shadow/lifecycle isolation."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location('m3_nav_launch', ROOT / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_snapshot_reuses_ten_node_schedule_with_only_bt_executable_replaced():
    module = load('nav2.launch.py')
    normal = module.generate_launch_description()
    shadow = module.generate_launch_description(snapshot=True)
    def schedule(ld):
        return [(timer.period, [node.node_executable for node in timer.actions])
                for timer in ld.entities if isinstance(timer, TimerAction)]
    old, new = schedule(normal), schedule(shadow)
    assert old[:5] == new[:5] and old[6:] == new[6:]
    assert old[5] == (40.0, ['bt_navigator', 'velocity_smoother'])
    assert new[5] == (40.0, ['bt_navigator_snapshot', 'velocity_smoother'])
    nodes = [node for timer in shadow.entities if isinstance(timer, TimerAction) for node in timer.actions]
    assert len(nodes) == 10 and all(isinstance(node, Node) for node in nodes)
    args = {item.name: item for item in shadow.entities if isinstance(item, DeclareLaunchArgument)}
    assert args['cmd_vel_output_topic'].default_value[0].text == '/fire_mission/nav2_cmd_vel_shadow'
    checks = [item for item in shadow.entities if isinstance(item, OpaqueFunction)]
    assert checks[0]._OpaqueFunction__function is module.snapshot_preflight


@pytest.mark.parametrize('output,clock', [('/cmd_vel_raw', 'false'),
                                         ('/cmd_vel', 'false'),
                                         ('/fire_mission/nav2_cmd_vel_shadow', 'true')])
def test_snapshot_preflight_rejects_real_outputs_and_sim_clock(monkeypatch, output, clock):
    module = load('nav2.launch.py')
    monkeypatch.setattr(module, 'preflight_checks', lambda context: None)
    context = LaunchContext()
    context.launch_configurations.update(cmd_vel_output_topic=output, use_sim_time=clock)
    with pytest.raises(RuntimeError):
        module.snapshot_preflight(context)


def test_snapshot_preflight_accepts_only_isolated_output_and_real_clock(monkeypatch):
    module = load('nav2.launch.py')
    checked = []
    monkeypatch.setattr(module, 'preflight_checks', lambda context: checked.append(True))
    context = LaunchContext()
    context.launch_configurations.update(cmd_vel_output_topic='/fire_mission/nav2_cmd_vel_shadow',
                                         use_sim_time='false')
    module.snapshot_preflight(context)
    assert checked == [True]


def test_cpp_compatibility_hook_has_no_new_publisher_or_cached_relay():
    header = (ROOT / 'include/fire_robot_navigation/action_status_refresh.hpp').read_text()
    source = (ROOT / 'src/bt_navigator_snapshot.cpp').read_text()
    assert 'server.*StatusRefreshAccess::method()' in header
    assert 'reinterpret_cast' not in header + source
    assert 'create_publisher' not in header + source
    assert 'create_subscription' not in header + source
    assert 'std::chrono::milliseconds(100)' in source
    assert 'pose_count != 1 || poses_count != 1' in source
    assert 'refresh_timer_.reset()' in source
    assert 'create_wall_timer' not in (ROOT / 'launch/nav2.launch.py').read_text()


def test_snapshot_wrapper_resolves_same_source_directory():
    module = load('nav2_snapshot_shadow.launch.py')
    ld = module.generate_launch_description()
    timers = [item for item in ld.entities if isinstance(item, TimerAction)]
    assert timers[5].actions[0].node_executable == 'bt_navigator_snapshot'
