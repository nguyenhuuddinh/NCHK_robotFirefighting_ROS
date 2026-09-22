"""Publish a V4L2 camera's native MJPEG stream without decode/re-encode."""

import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage

from .camera_qos_relay import ByteRateLimiter


SOI = b'\xff\xd8'
EOI = b'\xff\xd9'


def camera_output_qos():
    """Keep only the newest image and never retransmit stale frames over WiFi."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class JpegStreamParser:
    """Split a concatenated V4L2 MJPEG byte stream into JPEG frames."""

    def __init__(self, max_frame_bytes=2_000_000):
        self.buffer = bytearray()
        self.max_frame_bytes = max_frame_bytes
        self.invalid_prefix_bytes = 0
        self.oversize_frames = 0

    def feed(self, chunk):
        """Return every complete JPEG found in *chunk*."""
        self.buffer.extend(chunk)
        frames = []
        while True:
            start = self.buffer.find(SOI)
            if start < 0:
                if len(self.buffer) > 2:
                    self.invalid_prefix_bytes += len(self.buffer) - 2
                    del self.buffer[:-2]
                break
            if start:
                self.invalid_prefix_bytes += start
                del self.buffer[:start]
            end = self.buffer.find(EOI, 2)
            if end < 0:
                if len(self.buffer) > self.max_frame_bytes:
                    self.oversize_frames += 1
                    self.buffer.clear()
                break
            frames.append(bytes(self.buffer[:end + 2]))
            del self.buffer[:end + 2]
        return frames


class MjpegReader(threading.Thread):
    """Read JPEGs from v4l2-ctl and retain at most one pending frame."""

    def __init__(self, stream, output_queue, stop_event, notify):
        super().__init__(name='native-mjpeg-reader', daemon=True)
        self.stream = stream
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.notify = notify
        self.parser = JpegStreamParser()
        self.frames = 0
        self.queue_drops = 0
        self.error = ''
        self.last_frame_time = time.monotonic()

    def _offer_latest(self, frame):
        captured_ns = time.time_ns()
        try:
            self.output_queue.put_nowait((captured_ns, frame))
        except queue.Full:
            try:
                self.output_queue.get_nowait()
                self.queue_drops += 1
            except queue.Empty:
                pass
            self.output_queue.put_nowait((captured_ns, frame))
        self.notify()

    def run(self):
        try:
            while not self.stop_event.is_set():
                chunk = self.stream.read(65536)
                if not chunk:
                    break
                for frame in self.parser.feed(chunk):
                    self.frames += 1
                    self.last_frame_time = time.monotonic()
                    self._offer_latest(frame)
        except Exception as exc:  # The health timer reports and restarts capture.
            self.error = f'{type(exc).__name__}: {exc}'


class BoundedPipeReader(threading.Thread):
    """Drain a subprocess pipe continuously while retaining a bounded tail."""

    def __init__(self, stream, max_bytes=16384):
        super().__init__(name='native-mjpeg-stderr', daemon=True)
        self.stream = stream
        self.max_bytes = max_bytes
        self.buffer = bytearray()

    def run(self):
        try:
            while True:
                chunk = self.stream.read(4096)
                if not chunk:
                    break
                self.buffer.extend(chunk)
                if len(self.buffer) > self.max_bytes:
                    del self.buffer[:-self.max_bytes]
        except (OSError, ValueError):
            pass

    def text(self):
        return bytes(self.buffer).decode(errors='replace').strip()


class NativeMjpegCamera(Node):
    """Capture hardware MJPEG and publish current frames directly to ROS 2."""

    def __init__(self):
        super().__init__('native_mjpeg_camera')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 10)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('output_topic', '/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('max_bytes_per_sec', 750000)
        self.declare_parameter('disable_dynamic_framerate', True)
        self.declare_parameter('restart_delay_s', 2.0)
        self.declare_parameter('frame_timeout_s', 2.0)
        self.declare_parameter('stats_period_s', 10.0)

        self.device = str(self.get_parameter('video_device').value)
        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        self.fps = int(self.get_parameter('framerate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self.restart_delay = float(self.get_parameter('restart_delay_s').value)
        self.frame_timeout = float(self.get_parameter('frame_timeout_s').value)
        self.disable_dynamic_framerate = bool(
            self.get_parameter('disable_dynamic_framerate').value)

        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise ValueError('Camera width, height, and framerate must be positive')
        if self.restart_delay < 0.1:
            raise ValueError('restart_delay_s must be at least 0.1')
        if self.frame_timeout < 0.5:
            raise ValueError('frame_timeout_s must be at least 0.5')

        qos = camera_output_qos()
        self.image_pub = self.create_publisher(
            CompressedImage,
            self.output_topic,
            qos,
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            qos,
        )
        self.byte_budget = ByteRateLimiter(
            int(self.get_parameter('max_bytes_per_sec').value))
        self.frame_queue = queue.Queue(maxsize=1)
        self.frame_guard = self.create_guard_condition(self._publish_latest)

        self.process = None
        self.reader = None
        self.stderr_reader = None
        self.stop_event = None
        self.stopping = False
        self.last_start_attempt = 0.0
        self.original_dynamic_framerate = None
        self.captured_at_last_stats = 0
        self.queue_drops_at_last_stats = 0
        self.published = 0
        self.published_bytes = 0
        self.budget_drops = 0
        self.restart_count = 0

        stats_period = float(self.get_parameter('stats_period_s').value)
        if stats_period <= 0:
            raise ValueError('stats_period_s must be positive')
        self.create_timer(1.0, self._check_capture)
        self.create_timer(stats_period, self._log_stats)
        self._start_capture()

    def _notify_frame(self):
        if not self.stopping:
            self.frame_guard.trigger()

    def _v4l2_control(self, value=None):
        control = 'exposure_dynamic_framerate'
        command = ['v4l2-ctl', '-d', self.device]
        if value is None:
            command.extend(['-C', control])
        else:
            command.extend(['-c', f'{control}={value}'])
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=3, check=False)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        if value is not None:
            return value
        match = re.search(r':\s*(-?\d+)\s*$', result.stdout)
        if not match:
            raise RuntimeError(f'Cannot parse {control}: {result.stdout.strip()}')
        return int(match.group(1))

    def _configure_exposure(self):
        if not self.disable_dynamic_framerate:
            return
        try:
            if self.original_dynamic_framerate is None:
                self.original_dynamic_framerate = self._v4l2_control()
            self._v4l2_control(0)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warning(
                f'Cannot disable exposure_dynamic_framerate: {exc}')

    def _start_capture(self):
        self.last_start_attempt = time.monotonic()
        if not Path(self.device).exists():
            self.get_logger().error(f'Camera device does not exist: {self.device}')
            return
        self._configure_exposure()
        command = [
            'v4l2-ctl',
            '-d', self.device,
            f'--set-fmt-video=width={self.width},height={self.height},pixelformat=MJPG',
            f'--set-parm={self.fps}',
            '--stream-mmap=4',
            '--stream-poll',
            '--stream-to=-',
        ]
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=0,
            )
        except OSError as exc:
            self.process = None
            self.get_logger().error(f'Cannot start v4l2-ctl: {exc}')
            return
        while True:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        self.captured_at_last_stats = 0
        self.queue_drops_at_last_stats = 0
        self.stop_event = threading.Event()
        self.reader = MjpegReader(
            self.process.stdout,
            self.frame_queue,
            self.stop_event,
            self._notify_frame,
        )
        self.stderr_reader = BoundedPipeReader(self.process.stderr)
        self.stderr_reader.start()
        self.reader.start()
        self.get_logger().info(
            f'Native MJPEG {self.device} {self.width}x{self.height}@{self.fps} '
            f'-> {self.output_topic} (BEST_EFFORT depth=1)')

    def _publish_latest(self):
        captured_ns = None
        frame = None
        while True:
            try:
                captured_ns, frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break
        if frame is None:
            return
        if not self.byte_budget.allow(len(frame), time.monotonic()):
            self.budget_drops += 1
            return

        stamp = self.get_clock().now().to_msg()
        if captured_ns is not None:
            stamp.sec, stamp.nanosec = divmod(captured_ns, 1_000_000_000)
        image = CompressedImage()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.format = 'jpeg'
        image.data = frame
        self.image_pub.publish(image)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        self.info_pub.publish(info)
        self.published += 1
        self.published_bytes += len(frame)

    def _capture_error(self):
        if self.reader and self.reader.error:
            return self.reader.error
        if self.stderr_reader:
            # v4l2-ctl prints a carriage-return FPS progress meter. Collapse it
            # so one restart cannot flood launch logs with thousands of chars.
            return ' '.join(self.stderr_reader.text().split())[-500:]
        return ''

    def _stop_process(self):
        if self.stop_event:
            self.stop_event.set()
        process = self.process
        if process and process.poll() is None:
            for sig, timeout in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0)):
                try:
                    os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)
        if self.reader:
            self.reader.join(timeout=2.0)
        if self.stderr_reader:
            self.stderr_reader.join(timeout=2.0)
        self.process = None
        self.reader = None
        self.stderr_reader = None
        self.stop_event = None

    def _check_capture(self):
        if self.stopping:
            return
        if self.process is not None and self.process.poll() is None:
            if self.reader is None:
                return
            frame_age = time.monotonic() - self.reader.last_frame_time
            if frame_age <= self.frame_timeout:
                return
            self.get_logger().error(
                f'MJPEG frame timeout: no frame for {frame_age:.1f}s; restarting')
            self._stop_process()
            self.restart_count += 1
        if self.process is not None:
            code = self.process.returncode
            error = self._capture_error()
            self.get_logger().error(
                f'MJPEG capture stopped rc={code}: {error or "no stderr"}')
            self._stop_process()
            self.restart_count += 1
        if time.monotonic() - self.last_start_attempt >= self.restart_delay:
            self.get_logger().warning('Retrying MJPEG camera capture')
            self._start_capture()

    def _log_stats(self):
        captured = self.reader.frames if self.reader else self.captured_at_last_stats
        queue_drops = (
            self.reader.queue_drops if self.reader else self.queue_drops_at_last_stats)
        self.get_logger().info(
            f'[NativeMjpegCamera] captured={captured - self.captured_at_last_stats} '
            f'published={self.published} bytes={self.published_bytes} '
            f'queue_drop={queue_drops - self.queue_drops_at_last_stats} '
            f'budget_drop={self.budget_drops} restarts={self.restart_count}')
        self.captured_at_last_stats = captured
        self.queue_drops_at_last_stats = queue_drops
        self.published = 0
        self.published_bytes = 0
        self.budget_drops = 0
        self.restart_count = 0

    def stop_capture(self):
        """Stop only the camera subprocess and restore its runtime control."""
        self.stopping = True
        self._stop_process()
        if self.original_dynamic_framerate is not None:
            try:
                self._v4l2_control(self.original_dynamic_framerate)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.get_logger().warning(
                    f'Cannot restore exposure_dynamic_framerate: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NativeMjpegCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_capture()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
