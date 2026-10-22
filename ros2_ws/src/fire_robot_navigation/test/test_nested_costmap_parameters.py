"""Verify actual container argv carries the complete rewritten child-node YAML."""
from pathlib import Path
import importlib.util

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import ComposableNodeContainer
from launch.utilities import perform_substitutions

PACKAGE = Path(__file__).resolve().parents[1]


def render(tmp_path, sim_time, filename=None):
    path = filename or PACKAGE / 'launch/nav2_composed_shadow.launch.py'
    spec = importlib.util.spec_from_file_location('nested_costmap_candidate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.get_package_share_directory
    module.get_package_share_directory = lambda name: (
        str(PACKAGE) if name == 'fire_robot_navigation' else original(name))
    context = LaunchContext()
    context.launch_configurations.update(
        params_file=str(PACKAGE / 'config/nav2_params.yaml'),
        map=str(tmp_path / 'chosen_map.yaml'), use_sim_time=sim_time,
        autostart='true', rviz='false')
    container = next(x for x in module.generate_launch_description().entities
                     if isinstance(x, ComposableNodeContainer))
    # Resolve ROS launch's real argv without executing a process.
    container._perform_substitutions(context)
    # Node.execute normally supplies these aliases before ExecuteProcess runs.
    context.extend_locals({'ros_specific_arguments': {
        'name': '__node:=nav2_container', 'ns': '__ns:=/'}})
    argv = [perform_substitutions(context, part) for part in container.cmd]
    files = [Path(argv[i + 1]) for i, token in enumerate(argv) if token == '--params-file']
    return [yaml.safe_load(p.read_text()) for p in files]


@pytest.mark.parametrize('sim_time', ['true', 'false'])
def test_child_costmaps_receive_baseline_geometry_sensors_and_clock(tmp_path, sim_time):
    rendered = render(tmp_path, sim_time)
    assert len(rendered) == 1
    actual = rendered[0]
    baseline = yaml.safe_load((PACKAGE / 'config/nav2_params.yaml').read_text())
    for node in ('local_costmap', 'global_costmap'):
        expected = baseline[node][node]['ros__parameters'].copy()
        expected['use_sim_time'] = sim_time == 'true'
        # Includes full footprint, inflation, observation sources, plugin config.
        assert actual[node][node]['ros__parameters'] == expected
    assert actual['local_costmap']['local_costmap']['ros__parameters']['plugins'] == [
        'voxel_layer', 'inflation_layer']
    assert actual['map_server']['ros__parameters']['yaml_filename'] == str(tmp_path / 'chosen_map.yaml')
