"""Source contract for the laptop side of the wireless static profile."""

import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'laptop_wireless_static.launch.py'


def test_laptop_profile_only_starts_perception_and_optional_rviz():
    spec = importlib.util.spec_from_file_location('laptop_static', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: f'/tmp/{name}'
    entities = module.generate_launch_description().entities
    includes = [entity for entity in entities
                if isinstance(entity, IncludeLaunchDescription)]
    nodes = [entity for entity in entities if isinstance(entity, Node)]

    assert len(includes) == 1
    source = includes[0].launch_description_source
    location = perform_substitutions(
        LaunchContext(), source._LaunchDescriptionSource__location)
    assert location.endswith('/perception.launch.py')
    assert len(nodes) == 1
    assert nodes[0].node_package == 'rviz2'
    source = LAUNCH.read_text()
    assert 'nav2.launch.py' not in source
    assert 'mission_motion_arbiter_shadow.launch.py' not in source
