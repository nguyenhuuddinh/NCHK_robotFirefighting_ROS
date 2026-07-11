import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fire_robot_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huudinh',
    maintainer_email='dinhnguyenhuu65@gmail.com',
    description='Safety watchdog node — giám sát heartbeat /cmd_vel trên Pi',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_watchdog = fire_robot_safety.safety_watchdog:main',
        ],
    },
)
