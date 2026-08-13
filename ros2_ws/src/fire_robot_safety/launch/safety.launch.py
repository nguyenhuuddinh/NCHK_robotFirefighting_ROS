"""
safety.launch.py — Khởi chạy Safety Watchdog trên Pi.

Chạy trên: 🟢 PI
Chức năng: Giám sát heartbeat /cmd_vel, gửi STOP nếu Laptop mất kết nối > 1000ms.
Tham số: heartbeat_timeout_ms, check_period_ms (qua launch arguments, không hardcode).

Lưu ý: Default 1000ms khớp với firmware ESP32 watchdog (cũng 1000ms).
    teleop_twist_keyboard không publish liên tục khi không nhấn phím,
    500ms quá nhạy gây xe giật liên tục → SLAM scan matcher khó ổn định.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch Arguments (không hardcode) ──
    timeout_arg = DeclareLaunchArgument(
        'heartbeat_timeout_ms',
        default_value='1000',
        description='Thời gian tối đa (ms) không nhận /cmd_vel trước khi gửi STOP'
    )

    check_period_arg = DeclareLaunchArgument(
        'check_period_ms',
        default_value='100',
        description='Chu kỳ kiểm tra heartbeat (ms)'
    )

    # ── Safety Watchdog Node ──
    safety_watchdog_node = Node(
        package='fire_robot_safety',
        executable='safety_watchdog',
        name='safety_watchdog',
        parameters=[{
            'heartbeat_timeout_ms':
                LaunchConfiguration('heartbeat_timeout_ms'),
            'check_period_ms':
                LaunchConfiguration('check_period_ms'),
        }],
        output='screen',
    )

    return LaunchDescription([
        timeout_arg,
        check_period_arg,
        safety_watchdog_node,
    ])
