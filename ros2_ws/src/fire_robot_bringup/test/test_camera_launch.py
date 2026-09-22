"""Kiểm tra cấu hình camera mà không mở thiết bị hoặc chạy robot."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_camera_launch_selects_native_mjpeg_or_usb_fallback():
    """Camera có backend MJPEG mặc định, fallback, và công tắc tắt nhanh."""
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
    backend_arg = next(
        action for action in actions
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_backend'
    )
    fps_arg = next(
        action for action in actions
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_fps'
    )
    budget_arg = next(
        action for action in actions
        if isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_max_bytes_per_sec'
    )
    native_node = next(
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == 'native_mjpeg_camera'
    )
    usb_node = next(
        action for action in actions
        if isinstance(action, Node) and action.node_package == 'usb_cam'
    )
    relay_node = next(
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == 'camera_qos_relay'
    )

    assert camera_arg.default_value[0].text == 'true'
    assert backend_arg.default_value[0].text == 'native_mjpeg'
    assert fps_arg.default_value[0].text == '15'
    assert budget_arg.default_value[0].text == '750000'
    assert usb_node.node_executable == 'usb_cam_node_exe'
    context = LaunchContext()
    context.launch_configurations['use_camera'] = 'true'
    context.launch_configurations['camera_backend'] = 'native_mjpeg'
    assert native_node.condition.evaluate(context)
    assert not usb_node.condition.evaluate(context)
    assert not relay_node.condition.evaluate(context)

    context.launch_configurations['camera_backend'] = 'usb_cam'
    assert not native_node.condition.evaluate(context)
    assert usb_node.condition.evaluate(context)
    assert relay_node.condition.evaluate(context)

    context.launch_configurations['use_camera'] = 'false'
    assert not native_node.condition.evaluate(context)
    assert not usb_node.condition.evaluate(context)
    assert not relay_node.condition.evaluate(context)

    remappings = [
        (perform_substitutions(context, source),
         perform_substitutions(context, target))
        for source, target in usb_node._Node__remappings
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
        isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_backend'
        and action.default_value[0].text == 'native_mjpeg'
        for action in actions
    )
    assert any(
        isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_fps'
        and action.default_value[0].text == '15'
        for action in actions
    )
    assert any(
        isinstance(action, DeclareLaunchArgument)
        and action.name == 'camera_max_bytes_per_sec'
        and action.default_value[0].text == '750000'
        for action in actions
    )
    assert any(
        isinstance(action, IncludeLaunchDescription)
        and {name for name, _ in action.launch_arguments}
        >= {
            'use_camera',
            'camera_backend',
            'camera_fps',
            'camera_max_bytes_per_sec',
        }
        for action in actions
    )


def test_native_mjpeg_starts_with_bounded_wifi_stream():
    """MJPEG native dùng cấu hình bảo thủ và ngân sách WiFi hữu hạn."""
    params = yaml.safe_load((PACKAGE_DIR / 'config' / 'pi_params.yaml').read_text())
    camera = params['native_mjpeg_camera']['ros__parameters']

    assert camera['video_device'] == '/dev/video0'
    assert (camera['image_width'], camera['image_height']) == (640, 480)
    assert camera['framerate'] == 15
    assert camera['frame_id'] == 'camera_frame'
    assert camera['output_topic'] == '/image_raw/compressed'
    assert camera['max_bytes_per_sec'] == 750000
    assert camera['disable_dynamic_framerate'] is True
    assert camera['frame_timeout_s'] == 2.0


def test_native_mjpeg_parser_handles_chunk_boundaries_and_prefix():
    """Parser lấy đúng JPEG dù USB chia frame qua nhiều lần đọc."""
    from fire_robot_bringup.native_mjpeg_camera import JpegStreamParser

    parser = JpegStreamParser()
    assert parser.feed(b'noise\xff') == []
    frames = parser.feed(b'\xd8first\xff\xd9\xff\xd8sec')
    assert frames == [b'\xff\xd8first\xff\xd9']
    assert parser.feed(b'ond\xff\xd9') == [b'\xff\xd8second\xff\xd9']
    assert parser.invalid_prefix_bytes == 5


def test_native_mjpeg_qos_keeps_only_newest_frame():
    """Ảnh native không reliable-retry hoặc tích backlog qua WiFi."""
    from fire_robot_bringup.native_mjpeg_camera import camera_output_qos

    qos = camera_output_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 1


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
