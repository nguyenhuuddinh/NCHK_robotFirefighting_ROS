"""R5.5 Gate2 supervised-live stack with fixed, non-argument overrides.

The dedicated entrypoint and token are the only supported caller. Starting the
launch file does not arm: every output remains behind the existing operator A,
BOOT, source, owner, UUID, freshness, stationary and budget gates.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config=Path(get_package_share_directory('fire_robot_mission'))/'config'
    baseline=str(config/'mission_params.yaml')
    operational=str(config/'operational_params.yaml')
    gate2=str(config/'operational_gate2.yaml')
    specs=[('fire_mission_supervisor',{}),('live_nav_stop_gateway',{}),
           ('nav2_cancel_intent_shadow',{}),('nav2_cancel_guard',{}),
           ('alignment_handoff_shadow',{}),('live_alignment_takeover',{}),
           ('ir_approach_live',{}),('fire_pulse_controller',{}),
           ('post_spray_review',{})]
    nodes=[Node(package='fire_robot_mission',executable=name,name=name,output='screen',
                parameters=[baseline,operational,gate2,{'use_sim_time':False},extra])
           for name,extra in specs]
    nodes.append(Node(package='fire_robot_mission',executable='ir_range_preview',
        name='ir_range_preview',output='screen',
        condition=UnlessCondition(LaunchConfiguration('external_ir_preview')),
        parameters=[str(config/'ir_range_preview.yaml'),gate2,{'use_sim_time':False}]))
    return LaunchDescription([DeclareLaunchArgument('external_ir_preview',default_value='false')]+nodes)
