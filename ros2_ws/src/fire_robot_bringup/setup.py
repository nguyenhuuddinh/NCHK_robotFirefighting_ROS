import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fire_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huudinh',
    maintainer_email='dinhnguyenhuu65@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_tf_broadcaster = fire_robot_bringup.odom_to_tf_broadcaster:main',
            'scan_qos_relay = fire_robot_bringup.scan_qos_relay:main',
            'serial_bridge_node = fire_robot_bringup.serial_bridge_node:main',
        ],
    },
)
