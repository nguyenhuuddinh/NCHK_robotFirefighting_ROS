"""
dashboard.launch.py — Khởi chạy rosbridge WebSocket server trên Pi.

Chạy trên: 🟢 PI
Chức năng: Mở WebSocket port 9090 để Web Dashboard (browser trên Laptop)
           kết nối và subscribe/publish ROS 2 topics qua roslibjs.
Cài đặt: sudo apt install ros-humble-rosbridge-server
Truy cập: ws://10.0.0.1:9090 (từ browser Laptop kết nối WiFi AP của Pi)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch Arguments (không hardcode) ──
    port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket port cho rosbridge server'
    )

    # ── Rosbridge WebSocket Server ──
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
        }],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        rosbridge_node,
    ])
