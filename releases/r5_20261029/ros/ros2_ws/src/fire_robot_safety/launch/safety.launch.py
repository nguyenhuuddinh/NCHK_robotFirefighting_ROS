"""
safety.launch.py — Khởi chạy Safety Command Gate trên Pi.

Chạy trên: 🟢 PI
Chức năng:
    Subscribe /cmd_vel_raw (từ teleop/Nav2/dashboard)
    Publish   /cmd_vel     (tới micro-ROS Agent → ESP32)
    STOP nếu input mất > heartbeat_timeout_ms (default 1000ms).

[QA5 FIX] Đổi từ watchdog subscribe/publish cùng topic sang command gate
    với input/output tách biệt, tránh self-feedback.

Lưu ý: Default 1000ms khớp với firmware ESP32 watchdog (cũng 1000ms).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch Arguments ──
    input_topic_arg = DeclareLaunchArgument(
        'cmd_vel_input_topic',
        default_value='/cmd_vel_raw',
        description='Topic đầu vào command velocity'
    )

    output_topic_arg = DeclareLaunchArgument(
        'cmd_vel_output_topic',
        default_value='/cmd_vel',
        description='Topic đầu ra command velocity (tới ESP32)'
    )

    timeout_arg = DeclareLaunchArgument(
        'heartbeat_timeout_ms',
        default_value='1000',
        description='Thời gian tối đa (ms) không nhận input trước khi gửi STOP'
    )

    output_rate_arg = DeclareLaunchArgument(
        'cmd_vel_output_rate_hz',
        default_value='10.0',
        description='Tần suất publish output /cmd_vel (Hz)'
    )

    # ── Safety Command Gate Node ──
    safety_gate_node = Node(
        package='fire_robot_safety',
        executable='safety_watchdog',
        name='safety_watchdog',
        parameters=[{
            'cmd_vel_input_topic':
                LaunchConfiguration('cmd_vel_input_topic'),
            'cmd_vel_output_topic':
                LaunchConfiguration('cmd_vel_output_topic'),
            'heartbeat_timeout_ms':
                LaunchConfiguration('heartbeat_timeout_ms'),
            'cmd_vel_output_rate_hz':
                LaunchConfiguration('cmd_vel_output_rate_hz'),
        }],
        output='screen',
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        timeout_arg,
        output_rate_arg,
        safety_gate_node,
    ])
