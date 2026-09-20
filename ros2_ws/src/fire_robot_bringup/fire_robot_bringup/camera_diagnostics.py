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
        self.first = None
        self.last = None
        self.max_gap = 0.0
        self.frame_id = '-'
        self.stamp = '-'

    def record(self, msg, now):
        """Add one sample without storing the message."""
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        if self.first is None:
            self.first = now
        self.last = now
        self.count += 1
        if self.image:
            self.total_bytes += len(msg.data)
        self.frame_id = msg.header.frame_id or '-'
        stamp = msg.header.stamp
        self.stamp = f'{stamp.sec}.{stamp.nanosec:09d}'

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
            parts.extend((
                f'avg_jpeg_bytes={average:.0f}',
                f'payload_Bps={self.total_bytes / elapsed:.0f}',
            ))
        parts.extend((f'frame_id={self.frame_id}', f'last_stamp={self.stamp}'))
        return ' '.join(parts)


def _run(command):
    """Return one-line command output or an explicit unavailable marker."""
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f'unavailable ({exc})'
    return result.stdout.strip() or result.stderr.strip() or 'unavailable'


def _print_config():
    """Show the exact installed camera settings used by launch."""
    try:
        share = Path(get_package_share_directory('fire_robot_bringup'))
        config_path = share / 'config' / 'pi_params.yaml'
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        camera = config['usb_cam']['ros__parameters']
        relay = config['camera_qos_relay']['ros__parameters']
        print(
            f'CONFIG file={config_path.resolve()} '
            f'device={camera["video_device"]} '
            f'size={camera["image_width"]}x{camera["image_height"]} '
            f'fps={camera["framerate"]} '
            f'pixel_format={camera["pixel_format"]} '
            f'max_bytes_per_sec={relay["max_bytes_per_sec"]}'
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
            if 'usb_cam' in line or 'camera_qos_relay' in line
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
    _print_config()


def _print_graph(node, stats, elapsed):
    """Print publisher QoS and bounded receive counts for relevant topics."""
    names = set(node.get_node_names())
    print(
        f'NODES usb_cam={"usb_cam" in names} '
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
    options = parser.parse_args(args)
    if not 1.0 <= options.seconds <= 60.0:
        parser.error('--seconds must be from 1 to 60')

    print('=== CAMERA DIAGNOSTICS START ===', flush=True)
    _print_environment()
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
    finally:
        node.destroy_node()
        rclpy.shutdown()

    log_dir = Path(os.environ.get('ROS_LOG_DIR', Path.home() / '.ros/log'))
    _print_recent_log('USB_CAM_LOG', log_dir.glob('usb_cam_node_exe_*.log'))
    _print_recent_log('LAUNCH_LOG', log_dir.glob('*/launch.log'))
    print('=== CAMERA DIAGNOSTICS END ===')


if __name__ == '__main__':
    main()
