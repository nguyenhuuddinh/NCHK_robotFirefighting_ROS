"""Offline C4 parity checks. No node, container, action client or hardware starts."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.get_package_share_directory
    module.get_package_share_directory = lambda package: (
        str(PACKAGE) if package == 'fire_robot_navigation' else original(package))
    return module


@pytest.fixture
def profiles(tmp_path):
    composed = load('c4_composed', 'nav2_composed_shadow.launch.py')
    baseline = load('c4_baseline', 'nav2.launch.py')
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\0')
    mapping = tmp_path / 'map.yaml'
    mapping.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    context = LaunchContext()
    context.launch_configurations.update(
        use_sim_time='false', autostart='true', rviz='false', map=str(mapping),
        params_file=str(PACKAGE / 'config' / 'nav2_params.yaml'),
        cmd_vel_output_topic=composed.SHADOW_CMD_VEL_TOPIC)
    return composed, baseline, context


def render(context, value):
    return value if isinstance(value, str) else perform_substitutions(context, value)


def evaluated(context, parameters):
    return [yaml.safe_load(item.read_text()) if isinstance(item, Path) else item
            for item in evaluate_parameters(context, parameters)]


def remaps(context, rules):
    return [(render(context, a), render(context, b)) for a, b in rules or []]


def test_exact_baseline_parameters_and_command_routes_for_all_ten_nodes(profiles):
    composed, baseline, context = profiles
    ordinary = [a for a in baseline.generate_launch_description().entities if isinstance(a, Node)]
    components = composed.build_components(str(PACKAGE))
    assert len(ordinary) == len(components) == 10
    for node, component in zip(ordinary, components):
        assert evaluated(context, node._Node__parameters) == evaluated(context, component.parameters)
        assert remaps(context, node._Node__remappings) == remaps(context, component.remappings)
    assert [render(context, c.node_name) for c in components] == [
        'map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'velocity_smoother',
        'lifecycle_manager_localization', 'lifecycle_manager_navigation']
    assert all(to != '/cmd_vel' and to != '/cmd_vel_raw'
               for c in components for _, to in remaps(context, c.remappings))


def test_one_isolated_container_no_extra_robot_nodes_and_exit_propagates(profiles):
    composed, _, context = profiles
    entities = composed.generate_launch_description().entities
    containers = [a for a in entities if isinstance(a, ComposableNodeContainer)]
    assert len(containers) == 1
    assert render(context, containers[0].node_executable) == 'fire_robot_component_container_isolated'
    assert len([a for a in entities if isinstance(a, Node)]) == 1
    handlers = [a for a in entities if isinstance(a, RegisterEventHandler)]
    assert len(handlers) == 1
    assert entities.index(handlers[0]) < entities.index(containers[0])
    handler = handlers[0].event_handler
    def exited(action):
        return ProcessExited(action=action, name='synthetic', cmd=[], cwd=None,
                             env=None, pid=123, returncode=1)
    event = exited(containers[0])
    assert handler.matches(event)
    assert not handler.matches(exited(ExecuteProcess(cmd=['unused'])))
    actions = list(handler.handle(event, context))
    assert len(actions) == 1 and isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)
    assert any(isinstance(a, OpaqueFunction) for a in entities)
    assert not any(isinstance(a, DeclareLaunchArgument) and a.name == 'cmd_vel_output_topic'
                   for a in entities)


@pytest.mark.parametrize('output', ['/cmd_vel_raw', '/cmd_vel', '/other', ''])
def test_output_override_is_rejected_before_plugin_checks(profiles, monkeypatch, output):
    composed, _, context = profiles
    context.launch_configurations['cmd_vel_output_topic'] = output
    monkeypatch.setattr(composed, 'verify_runtime_components',
                        lambda: pytest.fail('unsafe output reached component checks'))
    with pytest.raises(RuntimeError, match='rejects cmd_vel_output_topic'):
        composed.preflight_checks(context)


def test_baseline_map_validation_remains_active(profiles, monkeypatch):
    composed, _, context = profiles
    context.launch_configurations['map'] = 'relative.yaml'
    monkeypatch.setattr(composed, 'verify_runtime_components',
                        lambda: pytest.fail('invalid map reached plugin checks'))
    with pytest.raises(RuntimeError, match='must be an absolute path'):
        composed.preflight_checks(context)


def test_installed_humble_components_are_registered_without_loading_them(profiles):
    composed, _, context = profiles
    composed.preflight_checks(context)


def test_missing_component_fails_before_container_spawn(profiles, monkeypatch):
    composed, _, _ = profiles
    monkeypatch.setattr(composed, 'get_resource', lambda *_: ('wrong::Plugin;lib/missing.so', '/tmp'))
    with pytest.raises(RuntimeError, match='Component not registered'):
        composed.verify_runtime_components()
