"""Source contract for the optional light laptop perception profile."""

import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions


LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'laptop_wireless_light.launch.py'


def test_laptop_light_disables_only_display_and_optional_stats():
    spec = importlib.util.spec_from_file_location('laptop_light', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: '/tmp/fire_robot_perception'
    includes = [item for item in module.generate_launch_description().entities
                if isinstance(item, IncludeLaunchDescription)]
    assert len(includes) == 1
    source = includes[0].launch_description_source
    path = perform_substitutions(
        LaunchContext(), source._LaunchDescriptionSource__location)
    assert path.endswith('/laptop_wireless_static.launch.py')
    args = dict(includes[0].launch_arguments)
    assert args['rviz'] == 'false'
    assert args['enable_diagnostics'] == 'false'
    assert args['publish_annotated'] == 'false'
    assert args['use_alignment_observer'] == 'false'
