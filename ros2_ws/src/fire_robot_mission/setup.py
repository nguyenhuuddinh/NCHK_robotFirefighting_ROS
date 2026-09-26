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
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huudinh',
    maintainer_email='dinhnguyenhuu65@gmail.com',
    description='Observe-only Nav2 and YOLO mission supervision.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fire_mission_supervisor = '
            'fire_robot_mission.mission_supervisor:main',
            'mission_preview_diagnostics = '
            'fire_robot_mission.mission_diagnostics:main',
            'clock_sync_diagnostics = '
            'fire_robot_mission.clock_sync_diagnostics:main',
        ],
    },
)
