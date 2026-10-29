"""All R5 consumers, observe-only. No live/protocol override is exposed."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config=Path(get_package_share_directory('fire_robot_mission'))/'config'
    baseline=str(config/'mission_params.yaml');operational=str(config/'operational_params.yaml')
    # Keep each adapter's established config and the operational false defaults.
    specs=[('fire_mission_supervisor',{}),('live_nav_stop_gateway',{}),
           ('nav2_cancel_intent_shadow',{}),('nav2_cancel_guard',{}),
           ('alignment_handoff_shadow',{'require_live_cancel_proof':True}),
           ('live_alignment_takeover',{}),('ir_approach_live',{}),
           ('fire_pulse_controller',{}),('post_spray_review',{})]
    nodes=[Node(package='fire_robot_mission',executable=name,name=name,output='screen',
                parameters=[baseline,operational,{'use_sim_time':False},extra]) for name,extra in specs]
    nodes.append(Node(package='fire_robot_mission',executable='ir_range_preview',
        name='ir_range_preview',output='screen',parameters=[str(config/'ir_range_preview.yaml'),{'use_sim_time':False}]))
    return LaunchDescription(nodes)
