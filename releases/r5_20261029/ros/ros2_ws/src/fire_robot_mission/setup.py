import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'fire_robot_mission'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml', 'README.md'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huudinh',
    maintainer_email='dinhnguyenhuu65@gmail.com',
    description=(
        'Fire mission supervision and opt-in Nav2 stop handoff.'),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'r5_startup_health = fire_robot_mission.startup_health:main',
            'fire_robot_operator = fire_robot_mission.operational_session:main',
            'fire_pulse_controller = fire_robot_mission.pulse_controller:main',
            'fire_mission_supervisor = '
            'fire_robot_mission.mission_supervisor:main',
            'mission_preview_diagnostics = '
            'fire_robot_mission.mission_diagnostics:main',
            'clock_sync_diagnostics = '
            'fire_robot_mission.clock_sync_diagnostics:main',
            'alignment_handoff_shadow = '
            'fire_robot_mission.alignment_handoff_shadow:main',
            'handoff_shadow_diagnostics = '
            'fire_robot_mission.handoff_shadow_diagnostics:main',
            'motion_arbiter_shadow = '
            'fire_robot_mission.motion_arbiter_shadow:main',
            'motion_arbiter_shadow_diagnostics = '
            'fire_robot_mission.motion_arbiter_shadow_diagnostics:main',
            'nav2_cancel_intent_shadow = '
            'fire_robot_mission.nav2_cancel_intent_shadow:main',
            'nav2_cancel_intent_diagnostics = '
            'fire_robot_mission.nav2_cancel_intent_diagnostics:main',
            'nav2_cancel_guard = '
            'fire_robot_mission.nav2_cancel_guard:main',
            'nav2_cancel_guard_diagnostics = '
            'fire_robot_mission.nav2_cancel_guard_diagnostics:main',
            'live_nav_stop_gateway = '
            'fire_robot_mission.live_nav_stop_gateway:main',
            'live_nav_stop_diagnostics = '
            'fire_robot_mission.live_nav_stop_diagnostics:main',
            'live_alignment_takeover = '
            'fire_robot_mission.live_alignment_takeover:main',
            'live_alignment_diagnostics = '
            'fire_robot_mission.live_alignment_diagnostics:main',
            'handoff_sequence_diagnostics = '
            'fire_robot_mission.handoff_sequence_diagnostics:main',
            'ir_range_preview = '
            'fire_robot_mission.ir_range_preview:main',
            'ir_approach_shadow = '
            'fire_robot_mission.ir_approach_shadow:main',
            'ir_approach_live = '
            'fire_robot_mission.ir_approach_live:main',
            'pump_demo_controller = '
            'fire_robot_mission.pump_demo_controller:main',
            'post_spray_review = '
            'fire_robot_mission.post_spray_review:main',
        ],
    },
)
