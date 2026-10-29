"""Kiểm tra cấu hình camera mà không mở thiết bị hoặc chạy robot."""

import importlib.util
from pathlib import Path
import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
        if isinstance(action, OpaqueFunction)
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


@pytest.mark.parametrize('mode', ['ASYNCHRONOUS', 'SYNCHRONOUS'])
def test_native_camera_dds_mode_is_scoped_and_forwarded(mode):
    from launch.utilities import normalize_to_list_of_substitutions

    def resolve(context, value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def load(name):
        spec = importlib.util.spec_from_file_location(name + '_dds', PACKAGE_DIR / 'launch' / name)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.get_package_share_directory = lambda _: str(PACKAGE_DIR)
        return module.generate_launch_description().entities

    context = LaunchContext()
    context.launch_configurations.update(native_camera_publication_mode=mode, native_camera_transport='system')
    sensors = load('sensors.launch.py')
    declaration = next(a for a in sensors if isinstance(a, DeclareLaunchArgument)
                       and a.name == 'native_camera_publication_mode')
    assert resolve(context, declaration.default_value) == 'ASYNCHRONOUS'
    assert set(declaration.choices) == {'ASYNCHRONOUS', 'SYNCHRONOUS'}
    native = next(a for a in sensors if isinstance(a, OpaqueFunction)).execute(context)[0]
    env = {perform_substitutions(context, key): perform_substitutions(context, value)
           for key, value in native.additional_env}
    assert env == {'RMW_FASTRTPS_PUBLICATION_MODE': mode}
    assert all(not a.additional_env for a in sensors if isinstance(a, Node) and a is not native)
    robot = load('robot.launch.py')
    forwarded = next(dict(a.launch_arguments) for a in robot
                     if isinstance(a, IncludeLaunchDescription)
                     and 'native_camera_publication_mode' in dict(a.launch_arguments))
    assert resolve(context, forwarded['native_camera_publication_mode']) == mode


def test_native_camera_publish_metrics_preserve_frame_and_reset(monkeypatch):
    from types import SimpleNamespace
    import queue
    from builtin_interfaces.msg import Time
    from fire_robot_bringup import native_mjpeg_camera as camera

    now = [1_000_000_000]
    monkeypatch.setattr(camera.time, 'monotonic_ns', lambda: now[0])

    class Publisher:
        def __init__(self, duration):
            self.duration = duration
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)
            now[0] += self.duration

    pending = queue.Queue(maxsize=1)
    stamp_ns = 1_234_567_890
    jpeg = b'\xff\xd8payload\xff\xd9'
    pending.put((stamp_ns, jpeg))
    node = SimpleNamespace(
        frame_queue=pending, byte_budget=SimpleNamespace(allow=lambda *_: True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        image_pub=Publisher(7_000_000), info_pub=Publisher(2_000_000),
        frame_id='camera_frame', width=640, height=480, published=0, published_bytes=0,
        budget_drops=0, restart_count=0, image_publish_max_ms=0.0, publish_pair_max_ms=0.0)
    camera.NativeMjpegCamera._publish_latest(node)
    msg = node.image_pub.messages[0]
    assert msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec == stamp_ns
    assert bytes(msg.data) == jpeg
    assert node.info_pub.messages[0].header.stamp == msg.header.stamp
    assert node.image_publish_max_ms == 7.0 and node.publish_pair_max_ms == 9.0
    assert node.published == 1 and node.published_bytes == len(jpeg)
    logs = []
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    node.reader = SimpleNamespace(frames=1, queue_drops=0)
    node.captured_at_last_stats = node.queue_drops_at_last_stats = 0
    camera.NativeMjpegCamera._log_stats(node)
    assert 'image_publish_max_ms=7.000' in logs[0]
    assert 'publish_pair_max_ms=9.000' in logs[0]
    assert node.image_publish_max_ms == node.publish_pair_max_ms == 0.0


def test_native_camera_budget_drop_does_not_send_or_accumulate_metrics():
    from types import SimpleNamespace
    import queue
    from fire_robot_bringup import native_mjpeg_camera as camera

    pending = queue.Queue(maxsize=1)
    pending.put((123, b'jpeg'))
    node = SimpleNamespace(frame_queue=pending,
        byte_budget=SimpleNamespace(allow=lambda *_: False), budget_drops=0,
        image_publish_max_ms=5.0, publish_pair_max_ms=6.0)
    # No clock/publishers are provided: the drop path must not touch them.
    camera.NativeMjpegCamera._publish_latest(node)
    assert node.budget_drops == 1 and pending.empty()
    assert (node.image_publish_max_ms, node.publish_pair_max_ms) == (5.0, 6.0)
