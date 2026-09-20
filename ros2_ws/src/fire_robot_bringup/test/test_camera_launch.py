"""Kiểm tra cấu hình camera mà không mở thiết bị hoặc chạy robot."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from rclpy.qos import ReliabilityPolicy
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_camera_launch_has_fast_disable_switch():
    """Camera bật mặc định nhưng có thể tắt mà không sửa source."""
    launch_path = PACKAGE_DIR / 'launch' / 'sensors.launch.py'
    spec = importlib.util.spec_from_file_location('sensors_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)

    actions = module.generate_launch_description().entities
    camera_arg = next(
        action for action in actions
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'use_camera'
    )
    camera_node = next(
        action for action in actions
        if isinstance(action, Node) and action.node_package == 'usb_cam'
    )
    relay_node = next(
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == 'camera_qos_relay'
    )

    assert camera_arg.default_value[0].text == 'true'
    assert camera_node.node_executable == 'usb_cam_node_exe'
    context = LaunchContext()
    context.launch_configurations['use_camera'] = 'true'
    assert camera_node.condition.evaluate(context)
    assert relay_node.condition.evaluate(context)
    context.launch_configurations['use_camera'] = 'false'
    assert not camera_node.condition.evaluate(context)
    assert not relay_node.condition.evaluate(context)

    remappings = [
        (perform_substitutions(context, source),
         perform_substitutions(context, target))
        for source, target in camera_node._Node__remappings
    ]
    assert ('image_raw', '/camera_raw_local') in remappings
    assert ('image_raw/compressed', '/image_raw/compressed_local') in remappings


def test_robot_launch_forwards_camera_switch():
    """Công tắc hoạt động từ master launch trên Pi."""
    launch_path = PACKAGE_DIR / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)

    actions = module.generate_launch_description().entities
    assert any(
        isinstance(action, DeclareLaunchArgument)
        and action.name == 'use_camera'
        and action.default_value[0].text == 'true'
        for action in actions
    )
    assert any(
        isinstance(action, IncludeLaunchDescription)
        and any(name == 'use_camera' for name, _ in action.launch_arguments)
        for action in actions
    )


def test_camera_starts_with_bounded_local_jpeg_stream():
    """Raw chỉ ở topic nội bộ; giữ FPS và ngân sách JPEG qua WiFi."""
    params = yaml.safe_load((PACKAGE_DIR / 'config' / 'pi_params.yaml').read_text())
    camera = params['usb_cam']['ros__parameters']

    assert camera['video_device'] == '/dev/video0'
    assert (camera['image_width'], camera['image_height']) == (640, 480)
    assert camera['framerate'] == 10.0
    assert camera['pixel_format'] == 'yuyv2rgb'
    assert camera['frame_id'] == 'camera_frame'
    assert params['camera_qos_relay']['ros__parameters']['max_bytes_per_sec'] == 750000


def test_camera_wifi_qos_keeps_only_newest_frame():
    """Không retransmit hay giữ backlog ảnh JPEG qua WiFi."""
    from fire_robot_bringup.camera_qos_relay import camera_qos_profiles

    input_qos, output_qos = camera_qos_profiles()
    assert input_qos.reliability == ReliabilityPolicy.RELIABLE
    assert input_qos.depth == 1
    assert output_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert output_qos.depth == 1


def test_camera_relay_keeps_original_stamp_and_jpeg():
    """Relay không đổi message, kể cả timestamp của frame."""
    from fire_robot_bringup.camera_qos_relay import ByteRateLimiter, CameraQosRelay
    from sensor_msgs.msg import CompressedImage

    class Publisher:
        last_message = None

        def publish(self, msg):
            self.last_message = msg

    class FakeNode:
        _pub = Publisher()
        _budget = ByteRateLimiter(750000)
        _dropped_frames = 0
        _published_frames = 0

    msg = CompressedImage()
    msg.header.stamp.sec = 123
    msg.header.frame_id = 'camera_frame'
    msg.format = 'jpeg'
    msg.data = [1, 2, 3]

    CameraQosRelay._relay(FakeNode(), msg)
    assert FakeNode._pub.last_message is msg


def test_camera_budget_drops_excess_frames_without_backlog():
    """Băng thông ảnh bị giới hạn, thời gian mới chỉ hồi lại token."""
    from fire_robot_bringup.camera_qos_relay import ByteRateLimiter

    budget = ByteRateLimiter(100)
    assert budget.allow(80, now=0.0)
    assert not budget.allow(30, now=0.0)
    assert budget.allow(30, now=0.3)
    assert not budget.allow(90, now=0.3)
    assert budget.allow(90, now=1.0)
