"""Humble source snapshots with existing Nav2 lifecycle and isolated outputs."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    # Reuse the exact ten-node schedule/config; do not copy a second Nav2 stack.
    path = Path(__file__).with_name('nav2.launch.py')
    spec = importlib.util.spec_from_file_location('fire_robot_nav2_snapshot_base', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(snapshot=True)
