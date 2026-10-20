"""M3 unarmed I/O/Nav2 and one session supervisor; no live mission owners."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    path = Path(__file__).with_name('robot_wireless_io_nav_static.launch.py')
    spec = importlib.util.spec_from_file_location('fire_robot_m3_io_base', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(mission_shadow=True)
