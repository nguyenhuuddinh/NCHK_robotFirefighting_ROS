"""Print bounded, read-only Pi camera and telemetry diagnostics."""

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
import yaml


TOPICS = (
    ('/image_raw/compressed_local', CompressedImage, True),
    ('/image_raw/compressed', CompressedImage, True),
    ('/scan', LaserScan, False),
    ('/odom', Odometry, False),
    ('/imu/data', Imu, False),
)


class TopicStats:
    """Keep counts and timing only; never retain image payloads."""

    def __init__(self, image=False):
        self.image = image
        self.count = 0
        self.total_bytes = 0
        self.valid_jpegs = 0
        self.total_age_ms = 0.0
        self.max_age_ms = 0.0
        self.age_samples = 0
        self.first = None
        self.last = None
        self.max_gap = 0.0
        self.frame_id = '-'
        self.stamp = '-'

    def record(self, msg, now, wall_time=None):
        """Add one sample without storing the message."""
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        if self.first is None:
            self.first = now
        self.last = now
        self.count += 1
        if self.image:
            self.total_bytes += len(msg.data)
            if (len(msg.data) >= 4 and msg.data[0] == 0xff
                    and msg.data[1] == 0xd8 and msg.data[-2] == 0xff
                    and msg.data[-1] == 0xd9):
                self.valid_jpegs += 1
        self.frame_id = msg.header.frame_id or '-'
        stamp = msg.header.stamp
        self.stamp = f'{stamp.sec}.{stamp.nanosec:09d}'
        if self.image and (stamp.sec or stamp.nanosec):
            if wall_time is None:
                wall_time = time.time()
            age_ms = (wall_time - stamp.sec - stamp.nanosec / 1e9) * 1000.0
            self.total_age_ms += age_ms
            self.max_age_ms = max(self.max_age_ms, age_ms)
            self.age_samples += 1

    def summary(self, elapsed):
        """Produce one short line even when no publisher exists."""
        span = self.last - self.first if self.count > 1 else 0.0
        rate = (self.count - 1) / span if span > 0 else 0.0
        parts = [
            f'frames={self.count}',
            f'rx_hz={rate:.2f}',
            f'max_gap_s={self.max_gap:.3f}',
        ]
        if self.image:
            average = self.total_bytes / self.count if self.count else 0.0
            average_age = (
                self.total_age_ms / self.age_samples if self.age_samples else 0.0)
            parts.extend((
                f'avg_jpeg_bytes={average:.0f}',
                f'payload_Bps={self.total_bytes / elapsed:.0f}',
                f'valid_jpeg={self.valid_jpegs}/{self.count}',
                f'wall_age_ms_avg={average_age:.1f}',
                f'wall_age_ms_max={self.max_age_ms:.1f}',
            ))
        parts.extend((f'frame_id={self.frame_id}', f'last_stamp={self.stamp}'))
        return ' '.join(parts)


def _run(command, timeout=3):
    """Return one-line command output or an explicit unavailable marker."""
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f'unavailable ({exc})'
    return result.stdout.strip() or result.stderr.strip() or 'unavailable'


def _print_config():
    """Show the exact installed camera settings used by launch."""
    try:
        share = Path(get_package_share_directory('fire_robot_bringup'))
        config_path = share / 'config' / 'pi_params.yaml'
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        camera = config['native_mjpeg_camera']['ros__parameters']
        fallback = config['usb_cam']['ros__parameters']
        relay = config['camera_qos_relay']['ros__parameters']
        print(
            f'CONFIG file={config_path.resolve()} backend_default=native_mjpeg '
            f'device={camera["video_device"]} '
            f'yaml_size={camera["image_width"]}x{camera["image_height"]} '
            f'yaml_fps_default={camera["framerate"]} '
            'pixel_format=MJPG_passthrough '
            f'yaml_max_bytes_per_sec={camera["max_bytes_per_sec"]} '
            f'fallback={fallback["pixel_format"]} '
            f'fallback_max_bytes_per_sec={relay["max_bytes_per_sec"]}'
        )
        device = Path(camera['video_device'])
        print(f'DEVICE exists={device.exists()} readable={os.access(device, os.R_OK)}')
    except (KeyError, OSError, ValueError) as exc:
        print(f'CONFIG error={exc}')


def _print_recent_log(label, paths, limit=8):
    """Print only the newest matching ROS log and its last few useful lines."""
    files = [path for path in paths if path.is_file()]
    if not files:
        print(f'{label} none')
        return
    latest = max(files, key=lambda path: path.stat().st_mtime)
    print(f'{label} file={latest}')
    try:
        lines = latest.read_text(errors='replace').splitlines()
    except OSError as exc:
        print(f'{label} read_error={exc}')
        return
    if label == 'LAUNCH_LOG':
        lines = [
            line for line in lines
            if ('native_mjpeg_camera' in line or 'usb_cam' in line
                or 'camera_qos_relay' in line)
        ]
    for line in lines[-limit:]:
        print(f'  {line[:240]}')


def _print_environment():
    """Show settings that can isolate the Pi from the laptop."""
    print(f'TIME_UTC {datetime.now(timezone.utc).isoformat()}')
    print(
        f'ENV ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<default>")} '
        f'ROS_LOCALHOST_ONLY={os.environ.get("ROS_LOCALHOST_ONLY", "<unset>")}'
    )
    print(f'USB_CAM_PACKAGE {_run(["dpkg-query", "-W", "ros-humble-usb-cam"])}')
    print(f'V4L2_UTILS_PACKAGE {_run(["dpkg-query", "-W", "v4l-utils"])}')
    _print_config()


def _network_bytes(interface):
    """Read interface counters without invoking privileged networking tools."""
    stats_dir = Path('/sys/class/net') / interface / 'statistics'
    try:
        rx_bytes = int((stats_dir / 'rx_bytes').read_text().strip())
        tx_bytes = int((stats_dir / 'tx_bytes').read_text().strip())
    except (OSError, ValueError) as exc:
        raise RuntimeError(f'cannot read interface {interface}: {exc}') from exc
    return rx_bytes, tx_bytes


def _print_network_start(interface, peer_ip):
    """Verify the selected peer route before the bounded topic sample."""
    print(f'NETWORK_ROUTE {_run(["ip", "route", "get", peer_ip])}')
    ping = _run(['ping', '-c', '3', '-W', '1', peer_ip], timeout=5)
    print(f'NETWORK_PING {" | ".join(ping.splitlines()[-2:])}')
    return _network_bytes(interface)


def _print_network_end(interface, before, elapsed):
    """Show actual traffic observed by one interface during diagnostics."""
    try:
        after = _network_bytes(interface)
    except RuntimeError as exc:
        print(f'NETWORK_TRAFFIC error={exc}')
        return
    rx_delta = max(0, after[0] - before[0])
    tx_delta = max(0, after[1] - before[1])
    print(
        f'NETWORK_TRAFFIC interface={interface} elapsed_s={elapsed:.1f} '
        f'rx_bytes={rx_delta} tx_bytes={tx_delta} '
        f'rx_Bps={rx_delta / elapsed:.0f} tx_Bps={tx_delta / elapsed:.0f}'
    )


def _print_graph(node, stats, elapsed):
    """Print publisher QoS and bounded receive counts for relevant topics."""
    names = set(node.get_node_names())
    print(
        f'NODES native_mjpeg_camera={"native_mjpeg_camera" in names} '
        f'usb_cam={"usb_cam" in names} '
        f'camera_qos_relay={"camera_qos_relay" in names}'
    )
    for topic, _, _ in TOPICS:
        publishers = node.get_publishers_info_by_topic(topic)
        endpoints = ','.join(
            f'{entry.node_name}:{entry.qos_profile.reliability.name}'
            for entry in publishers
        ) or '-'
        print(
            f'TOPIC {topic} publishers={len(publishers)} [{endpoints}] '
            f'{stats[topic].summary(elapsed)}'
        )


def main(args=None):
    """Sample Pi topics for a fixed interval and exit without hardware writes."""
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--network-interface')
    parser.add_argument('--peer-ip')
    options = parser.parse_args(args)
    if not 1.0 <= options.seconds <= 60.0:
        parser.error('--seconds must be from 1 to 60')
    if bool(options.network_interface) != bool(options.peer_ip):
        parser.error('--network-interface and --peer-ip must be used together')

    print('=== CAMERA DIAGNOSTICS START ===', flush=True)
    _print_environment()
    network_before = None
    if options.network_interface:
        try:
            network_before = _print_network_start(
                options.network_interface, options.peer_ip)
        except RuntimeError as exc:
            print(f'NETWORK_START error={exc}')
    rclpy.init()
    node = rclpy.create_node('camera_diagnostics')
    stats = {}
    subscriptions = []
    try:
        for topic, message_type, image in TOPICS:
            stats[topic] = TopicStats(image=image)
            subscriptions.append(node.create_subscription(
                message_type, topic,
                lambda msg, name=topic: stats[name].record(msg, time.monotonic()),
                qos_profile_sensor_data,
            ))
        start = time.monotonic()
        deadline = start + options.seconds
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
        elapsed = time.monotonic() - start
        _print_graph(node, stats, elapsed)
        if network_before is not None:
            _print_network_end(options.network_interface, network_before, elapsed)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    log_dir = Path(os.environ.get('ROS_LOG_DIR', Path.home() / '.ros/log'))
    _print_recent_log(
        'NATIVE_MJPEG_LOG', log_dir.glob('native_mjpeg_camera_*.log'))
    _print_recent_log('USB_CAM_LOG', log_dir.glob('usb_cam_node_exe_*.log'))
    _print_recent_log('LAUNCH_LOG', log_dir.glob('*/launch.log'))
    print('=== CAMERA DIAGNOSTICS END ===')


if __name__ == '__main__':
    main()
