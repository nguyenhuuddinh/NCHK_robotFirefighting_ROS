"""
Observe-only YOLO inference for compressed camera images.

This laptop node publishes visualization and JSON telemetry only. It owns no
motion or actuator publisher, so detections cannot directly move the robot or
start the pump.
"""

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def sensor_qos():
    """Return a no-backlog profile for camera and perception telemetry."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def normalized_center(box, image_width, image_height):
    """Return box center in pixels and normalized image coordinates."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    x1, y1, x2, y2 = box
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    normalized_x = (2.0 * center_x / image_width) - 1.0
    normalized_y = (2.0 * center_y / image_height) - 1.0
    return center_x, center_y, normalized_x, normalized_y


def normalized_to_pixel(center, image_width, image_height):
    """Map a normalized image point in [-1, 1] back to pixel coordinates."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    normalized_x, normalized_y = center
    pixel_x = (float(normalized_x) + 1.0) * image_width / 2.0
    pixel_y = (float(normalized_y) + 1.0) * image_height / 2.0
    pixel_x = max(0.0, min(float(image_width - 1), pixel_x))
    pixel_y = max(0.0, min(float(image_height - 1), pixel_y))
    return pixel_x, pixel_y


def aim_observation(center, deadband_x, deadband_y):
    """Classify a target center without creating a motion command."""
    deadband = [round(float(deadband_x), 4), round(float(deadband_y), 4)]
    if center is None:
        return {
            'state': 'NO_TARGET',
            'inside_deadband': False,
            'error_normalized': None,
            'deadband_normalized': deadband,
        }

    error_x = float(center[0])
    error_y = float(center[1])
    horizontal = ''
    vertical = ''
    if error_x < -deadband_x:
        horizontal = 'LEFT'
    elif error_x > deadband_x:
        horizontal = 'RIGHT'
    if error_y < -deadband_y:
        vertical = 'UP'
    elif error_y > deadband_y:
        vertical = 'DOWN'

    state = '+'.join(item for item in (horizontal, vertical) if item)
    inside_deadband = not state
    if inside_deadband:
        state = 'CENTERED'
    return {
        'state': state,
        'inside_deadband': inside_deadband,
        'error_normalized': [round(error_x, 4), round(error_y, 4)],
        'deadband_normalized': deadband,
    }


@dataclass(frozen=True)
class Detection:
    """A single model detection expressed in source-image coordinates."""

    class_id: int
    class_name: str
    confidence: float
    box: tuple
    center: tuple
    center_normalized: tuple

    def as_dict(self):
        """Convert to a compact JSON-safe dictionary."""
        return {
            'class_id': self.class_id,
            'class_name': self.class_name,
            'confidence': round(self.confidence, 4),
            'bbox_xyxy_px': [round(value, 1) for value in self.box],
            'center_px': [round(value, 1) for value in self.center],
            'center_normalized': [
                round(value, 4) for value in self.center_normalized
            ],
        }


class TargetCenterSmoother:
    """Smooth target coordinates without inventing a target after a miss."""

    VALID_METHODS = ('median', 'ema')

    def __init__(self, method, window_frames, ema_alpha):
        method = str(method).casefold()
        if method not in self.VALID_METHODS:
            raise ValueError(
                f'center_filter must be one of {self.VALID_METHODS}'
            )
        if window_frames <= 0:
            raise ValueError('window_frames must be positive')
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError('ema_alpha must be in (0, 1]')
        self.method = method
        self.window_frames = window_frames
        self.ema_alpha = float(ema_alpha)
        self._median_history = deque(maxlen=window_frames)
        self._ema_center = None

    def reset(self):
        """Discard coordinates so reacquisition starts at the new target."""
        self._median_history.clear()
        self._ema_center = None

    def update(self, center):
        """Return a filtered center, or no center when this frame is a miss."""
        if center is None:
            self.reset()
            return None

        point = (float(center[0]), float(center[1]))
        if self.method == 'median':
            self._median_history.append(point)
            filtered = [
                statistics.median(item[axis]
                                  for item in self._median_history)
                for axis in (0, 1)
            ]
        else:
            if self._ema_center is None:
                self._ema_center = list(point)
            else:
                alpha = self.ema_alpha
                self._ema_center = [
                    alpha * point[axis]
                    + (1.0 - alpha) * self._ema_center[axis]
                    for axis in (0, 1)
                ]
            filtered = self._ema_center

        return [round(value, 4) for value in filtered]

    def description(self):
        """Return JSON-safe filter metadata for diagnostics."""
        metadata = {
            'method': self.method,
            'reset_on_miss': True,
        }
        if self.method == 'ema':
            metadata['alpha'] = round(self.ema_alpha, 4)
        else:
            metadata['window_frames'] = self.window_frames
        return metadata


class TemporalFireGate:
    """Require repeated current fire evidence without holding stale boxes."""

    def __init__(self, window_frames, minimum_detections):
        if window_frames <= 0:
            raise ValueError('window_frames must be positive')
        if not 1 <= minimum_detections <= window_frames:
            raise ValueError(
                'minimum_detections must be within the confirmation window'
            )
        self.window_frames = window_frames
        self.minimum_detections = minimum_detections
        self._history = deque(maxlen=window_frames)

    def reset(self):
        """Discard all old evidence after a camera or inference failure."""
        self._history.clear()

    def update(self, current_detection):
        """Record one frame and return the current confirmation state."""
        self._history.append(current_detection)
        valid = [item for item in self._history if item is not None]
        confirmed = (
            current_detection is not None
            and len(valid) >= self.minimum_detections
        )
        return {
            'confirmed': confirmed,
            'hits': len(valid),
            'samples': len(self._history),
            'window_frames': self.window_frames,
            'minimum_detections': self.minimum_detections,
        }


class YoloObserver(Node):
    """Run YOLO on current compressed frames and publish observations."""

    def __init__(self):
        super().__init__('yolo_observer')
        self._declare_parameters()
        self._read_and_validate_parameters()

        self._detections_pub = self.create_publisher(
            String, self.detections_topic, sensor_qos())
        self._annotated_pub = self.create_publisher(
            CompressedImage, self.annotated_topic, sensor_qos())
        self._image_sub = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._on_image,
            sensor_qos(),
        )

        self._fire_gate = TemporalFireGate(
            self.confirmation_window,
            self.confirmation_minimum,
        )
        self._center_smoother = TargetCenterSmoother(
            self.center_filter,
            self.confirmation_window,
            self.ema_alpha,
        )
        self._clahe = None
        if self.use_clahe:
            grid = (self.clahe_grid_size, self.clahe_grid_size)
            self._clahe = cv2.createCLAHE(
                clipLimit=self.clahe_clip_limit,
                tileGridSize=grid,
            )

        self._received = 0
        self._processed = 0
        self._decode_errors = 0
        self._inference_errors = 0
        self._period_received = 0
        self._period_processed = 0
        self._period_decode_errors = 0
        self._period_inference_errors = 0
        self._period_processing_ms = []
        self._last_input_time = time.monotonic()
        self._last_annotation_time = 0.0
        self._camera_stale = False
        self._started_at = time.monotonic()

        self._load_model()
        # Model import and GPU warm-up may take several seconds. Camera health
        # starts after that initialization, not before it.
        self._last_input_time = time.monotonic()
        self.create_timer(0.5, self._check_input_health)
        self.create_timer(self.stats_period, self._log_stats)

        self.get_logger().warning(
            '[YOLO] OBSERVE-ONLY: publishes detections and annotated images; '
            'no motion or actuator commands are created by this node'
        )
        self.get_logger().info(
            f'[YOLO] {self.input_topic} -> {self.detections_topic}, '
            f'{self.annotated_topic} (BEST_EFFORT depth=1)'
        )

    def _declare_parameters(self):
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', '0')
        self.declare_parameter('quantize', 16)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 50)
        self.declare_parameter('warmup_runs', 2)
        self.declare_parameter('input_topic', '/image_raw/compressed')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter(
            'annotated_topic', '/yolo/image_annotated/compressed')
        self.declare_parameter('use_clahe', False)
        self.declare_parameter('clahe_clip_limit', 1.0)
        self.declare_parameter('clahe_tile_grid_size', 8)
        self.declare_parameter('fire_class_name', 'fire')
        self.declare_parameter('confirmation_window_frames', 5)
        self.declare_parameter('confirmation_min_detections', 3)
        self.declare_parameter('center_filter', 'ema')
        self.declare_parameter('ema_alpha', 0.60)
        self.declare_parameter('aim_deadband_x', 0.10)
        self.declare_parameter('aim_deadband_y', 0.10)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_max_fps', 10.0)
        self.declare_parameter('annotated_jpeg_quality', 80)
        self.declare_parameter('input_timeout_s', 1.0)
        self.declare_parameter('stats_period_s', 5.0)

    def _read_and_validate_parameters(self):
        self.model_path = Path(
            str(self.get_parameter('model_path').value)).expanduser()
        self.device = str(self.get_parameter('device').value)
        self.quantize = int(self.get_parameter('quantize').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.confidence = float(
            self.get_parameter('confidence_threshold').value)
        self.iou = float(self.get_parameter('iou_threshold').value)
        self.max_detections = int(
            self.get_parameter('max_detections').value)
        self.warmup_runs = int(self.get_parameter('warmup_runs').value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.detections_topic = str(
            self.get_parameter('detections_topic').value)
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value)
        self.use_clahe = bool(self.get_parameter('use_clahe').value)
        self.clahe_clip_limit = float(
            self.get_parameter('clahe_clip_limit').value)
        self.clahe_grid_size = int(
            self.get_parameter('clahe_tile_grid_size').value)
        self.fire_class_name = str(
            self.get_parameter('fire_class_name').value).casefold()
        self.confirmation_window = int(
            self.get_parameter('confirmation_window_frames').value)
        self.confirmation_minimum = int(
            self.get_parameter('confirmation_min_detections').value)
        self.center_filter = str(
            self.get_parameter('center_filter').value).casefold()
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.aim_deadband_x = float(
            self.get_parameter('aim_deadband_x').value)
        self.aim_deadband_y = float(
            self.get_parameter('aim_deadband_y').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value)
        self.annotated_max_fps = float(
            self.get_parameter('annotated_max_fps').value)
        self.annotated_quality = int(
            self.get_parameter('annotated_jpeg_quality').value)
        self.input_timeout = float(
            self.get_parameter('input_timeout_s').value)
        self.stats_period = float(
            self.get_parameter('stats_period_s').value)

        if not self.model_path.is_file():
            raise ValueError(f'model_path is not a file: {self.model_path}')
        if self.quantize not in (16, 32):
            raise ValueError('quantize must be 16 (FP16) or 32 (FP32)')
        if self.image_size <= 0 or self.max_detections <= 0:
            raise ValueError('image_size and max_detections must be positive')
        if self.warmup_runs < 0:
            raise ValueError('warmup_runs cannot be negative')
        if not 0.0 < self.confidence <= 1.0:
            raise ValueError('confidence_threshold must be in (0, 1]')
        if not 0.0 < self.iou <= 1.0:
            raise ValueError('iou_threshold must be in (0, 1]')
        if self.clahe_clip_limit <= 0 or self.clahe_grid_size <= 0:
            raise ValueError('CLAHE parameters must be positive')
        TemporalFireGate(
            self.confirmation_window,
            self.confirmation_minimum,
        )
        TargetCenterSmoother(
            self.center_filter,
            self.confirmation_window,
            self.ema_alpha,
        )
        if not 0.0 <= self.aim_deadband_x < 1.0 or not \
                0.0 <= self.aim_deadband_y < 1.0:
            raise ValueError('aim deadbands must be in [0, 1)')
        if self.annotated_max_fps <= 0:
            raise ValueError('annotated_max_fps must be positive')
        if not 1 <= self.annotated_quality <= 100:
            raise ValueError('annotated_jpeg_quality must be in [1, 100]')
        if self.input_timeout <= 0 or self.stats_period <= 0:
            raise ValueError('timeout and stats period must be positive')
        if not all((self.input_topic, self.detections_topic,
                    self.annotated_topic, self.fire_class_name)):
            raise ValueError('topic names and fire_class_name cannot be empty')

    def _load_model(self):
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'YOLO dependencies are missing. Activate the '
                'fire_robot_yolo venv before building and launching.'
            ) from exc

        gpu_requested = self.device.casefold() not in ('cpu', '')
        if gpu_requested and not torch.cuda.is_available():
            raise RuntimeError(
                f'GPU device {self.device!r} requested but CUDA is unavailable'
            )

        self._torch = torch
        self._model = YOLO(str(self.model_path))
        names = self._model.names
        if isinstance(names, dict):
            self._class_names = {
                int(class_id): str(name) for class_id, name in names.items()
            }
        else:
            self._class_names = {
                class_id: str(name) for class_id, name in enumerate(names)
            }
        matching_ids = [
            class_id for class_id, name in self._class_names.items()
            if name.casefold() == self.fire_class_name
        ]
        if len(matching_ids) != 1:
            raise RuntimeError(
                f'Expected one {self.fire_class_name!r} class, got '
                f'{self._class_names}'
            )
        self._fire_class_id = matching_ids[0]

        if self.quantize == 16 and not gpu_requested:
            raise RuntimeError('FP16 inference requires a CUDA GPU device')

        if self.warmup_runs:
            warmup = np.zeros(
                (self.image_size, self.image_size, 3), dtype=np.uint8)
            with self._torch.inference_mode():
                for _ in range(self.warmup_runs):
                    self._model.predict(warmup, **self._predict_arguments())
            self._synchronize_gpu()

        device_name = 'CPU'
        if gpu_requested:
            device_name = torch.cuda.get_device_name(0)
        precision = 'FP16' if self.quantize == 16 else 'FP32'
        self.get_logger().info(
            f'[YOLO] loaded {self.model_path} on {device_name} ({precision}); '
            f'classes={self._class_names}, conf={self.confidence:.2f}, '
            f'center_filter={self.center_filter}, '
            f'ema_alpha={self.ema_alpha:.2f}'
        )

    def _reset_fire_tracking(self):
        """Clear both confirmation evidence and filtered coordinates."""
        self._fire_gate.reset()
        self._center_smoother.reset()

    def _predict_arguments(self):
        arguments = {
            'imgsz': self.image_size,
            'conf': self.confidence,
            'iou': self.iou,
            'max_det': self.max_detections,
            'device': self.device,
            'verbose': False,
            'save': False,
        }
        if self.quantize == 16:
            arguments['quantize'] = 16
        return arguments

    def _synchronize_gpu(self):
        if self.device.casefold() not in ('cpu', ''):
            self._torch.cuda.synchronize()

    def _preprocess(self, frame):
        if self._clahe is None:
            return frame
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lightness, channel_a, channel_b = cv2.split(lab)
        adjusted = self._clahe.apply(lightness)
        merged = cv2.merge((adjusted, channel_a, channel_b))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

    def _extract_detections(self, result, width, height):
        detections = []
        if result.boxes is None:
            return detections
        boxes = result.boxes.xyxy.detach().cpu().tolist()
        classes = result.boxes.cls.detach().cpu().tolist()
        confidences = result.boxes.conf.detach().cpu().tolist()
        for raw_box, raw_class, raw_confidence in zip(
                boxes, classes, confidences):
            x1, y1, x2, y2 = raw_box
            box = (
                max(0.0, min(float(width), float(x1))),
                max(0.0, min(float(height), float(y1))),
                max(0.0, min(float(width), float(x2))),
                max(0.0, min(float(height), float(y2))),
            )
            center_x, center_y, normalized_x, normalized_y = (
                normalized_center(box, width, height)
            )
            class_id = int(raw_class)
            detections.append(Detection(
                class_id=class_id,
                class_name=self._class_names.get(
                    class_id, f'unknown_{class_id}'),
                confidence=float(raw_confidence),
                box=box,
                center=(center_x, center_y),
                center_normalized=(normalized_x, normalized_y),
            ))
        return detections

    def _primary_fire(self, detections):
        candidates = [
            detection for detection in detections
            if detection.class_id == self._fire_class_id
        ]
        return max(candidates, key=lambda item: item.confidence, default=None)

    def _source_age_ms(self, msg):
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return None
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0

    def _publish_observation(
            self, msg, state, detections=None, fire_status=None,
            processing_ms=None, inference_ms=None, source_age_ms=None,
            image_shape=None, error=None):
        detections = detections or []
        payload = {
            'schema_version': 1,
            'observe_only': True,
            'state': state,
            'frame_index': self._received,
            'source': {
                'stamp': {
                    'sec': int(msg.header.stamp.sec),
                    'nanosec': int(msg.header.stamp.nanosec),
                },
                'frame_id': msg.header.frame_id,
                'age_ms': (
                    None if source_age_ms is None
                    else round(source_age_ms, 2)
                ),
            },
            'image': None,
            'processing_ms': (
                None if processing_ms is None else round(processing_ms, 2)
            ),
            'inference_ms': (
                None if inference_ms is None else round(inference_ms, 2)
            ),
            'detections': [item.as_dict() for item in detections],
            'fire': fire_status,
        }
        if image_shape is not None:
            payload['image'] = {
                'width': int(image_shape[1]),
                'height': int(image_shape[0]),
            }
        if error:
            payload['error'] = error
        output = String()
        output.data = json.dumps(
            payload, separators=(',', ':'), ensure_ascii=True)
        self._detections_pub.publish(output)

    def _annotation_due(self, now):
        if not self.publish_annotated:
            return False
        if self._annotated_pub.get_subscription_count() <= 0:
            return False
        minimum_period = 1.0 / self.annotated_max_fps
        return now - self._last_annotation_time >= minimum_period

    def _draw_annotation(
            self, frame, detections, fire_status,
            source_age_ms=None, processing_ms=None):
        annotated = frame.copy()
        height, width = annotated.shape[:2]
        center_px = (width // 2, height // 2)
        deadband_min = normalized_to_pixel(
            (-self.aim_deadband_x, -self.aim_deadband_y), width, height)
        deadband_max = normalized_to_pixel(
            (self.aim_deadband_x, self.aim_deadband_y), width, height)
        cv2.rectangle(
            annotated,
            tuple(int(value) for value in deadband_min),
            tuple(int(value) for value in deadband_max),
            (255, 255, 0),
            1,
        )
        cv2.drawMarker(
            annotated,
            center_px,
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=1,
        )
        colors = {
            'person': (0, 220, 0),
            'fire': (0, 0, 255),
            'smoke': (0, 165, 255),
        }
        for detection in detections:
            color = colors.get(detection.class_name.casefold(), (255, 0, 0))
            x1, y1, x2, y2 = [int(value) for value in detection.box]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f'{detection.class_name} {detection.confidence:.2f}'
            label_y = y1 - 8
            if label_y < 62:
                label_y = min(height - 5, y1 + 20)
            cv2.putText(
                annotated,
                label,
                (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
            if detection.class_id == self._fire_class_id:
                center = tuple(int(value) for value in detection.center)
                cv2.drawMarker(
                    annotated,
                    center,
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=18,
                    thickness=2,
                )

        smoothed_center = fire_status.get('smoothed_center_normalized')
        if smoothed_center is not None:
            smoothed_px = normalized_to_pixel(
                smoothed_center, width, height)
            cv2.drawMarker(
                annotated,
                tuple(int(value) for value in smoothed_px),
                (0, 255, 255),
                markerType=cv2.MARKER_DIAMOND,
                markerSize=22,
                thickness=2,
            )

        aim = fire_status.get('aim', {})
        error = aim.get('error_normalized')
        error_text = '(-,-)'
        if error is not None:
            error_text = f'({error[0]:+.2f},{error[1]:+.2f})'
        candidate = fire_status.get('candidate')
        confidence_text = '-'
        if candidate is not None:
            confidence_text = f'{candidate["confidence"]:.2f}'
        age_text = '-' if source_age_ms is None else f'{source_age_ms:.0f}'
        processing_text = (
            '-' if processing_ms is None else f'{processing_ms:.0f}')
        status = (
            'OBSERVE ONLY | fire '
            f'{fire_status["hits"]}/{fire_status["window_frames"]} | '
            f'confirmed={fire_status["confirmed"]} | '
            f'filter={fire_status["center_filter"]["method"].upper()}'
        )
        aim_status = (
            f'aim={aim.get("state", "NO_TARGET")} err={error_text} '
            f'conf={confidence_text} age={age_text}ms '
            f'proc={processing_text}ms'
        )
        cv2.rectangle(annotated, (0, 0), (width, 54),
                      (20, 20, 20), -1)
        cv2.putText(
            annotated,
            status,
            (8, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            annotated,
            aim_status,
            (8, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
        )
        return annotated

    def _publish_annotation(
            self, source_msg, frame, detections, fire_status,
            source_age_ms, processing_ms):
        annotated = self._draw_annotation(
            frame,
            detections,
            fire_status,
            source_age_ms=source_age_ms,
            processing_ms=processing_ms,
        )
        encoded, jpeg = cv2.imencode(
            '.jpg',
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, self.annotated_quality],
        )
        if not encoded:
            self.get_logger().error('[YOLO] failed to encode annotated JPEG')
            return
        output = CompressedImage()
        output.header = source_msg.header
        output.format = 'jpeg'
        output.data = jpeg.tobytes()
        self._annotated_pub.publish(output)

    def _on_image(self, msg):
        callback_start = time.perf_counter()
        now = time.monotonic()
        self._last_input_time = now
        self._received += 1
        self._period_received += 1
        if self._camera_stale:
            self._camera_stale = False
            self.get_logger().info('[YOLO] camera stream resumed')

        compressed = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(compressed, cv2.IMREAD_COLOR)
        if frame is None:
            self._decode_errors += 1
            self._period_decode_errors += 1
            self._reset_fire_tracking()
            self._publish_observation(
                msg,
                state='decode_error',
                error='cv2.imdecode returned no image',
            )
            return

        try:
            model_input = self._preprocess(frame)
            self._synchronize_gpu()
            inference_start = time.perf_counter()
            with self._torch.inference_mode():
                result = self._model.predict(
                    model_input, **self._predict_arguments())[0]
            self._synchronize_gpu()
            inference_ms = (
                time.perf_counter() - inference_start) * 1000.0
            detections = self._extract_detections(
                result, frame.shape[1], frame.shape[0])
        except Exception as exc:
            self._inference_errors += 1
            self._period_inference_errors += 1
            self._reset_fire_tracking()
            self.get_logger().error(
                f'[YOLO] inference failed: {type(exc).__name__}: {exc}')
            self._publish_observation(
                msg,
                state='inference_error',
                image_shape=frame.shape,
                error=f'{type(exc).__name__}: {exc}',
            )
            return

        primary_fire = self._primary_fire(detections)
        fire_status = self._fire_gate.update(primary_fire)
        candidate_center = (
            None if primary_fire is None
            else primary_fire.center_normalized
        )
        filtered_center = self._center_smoother.update(candidate_center)
        fire_status['smoothed_center_normalized'] = (
            filtered_center if fire_status['confirmed'] else None
        )
        fire_status['center_filter'] = self._center_smoother.description()
        fire_status['detected_now'] = primary_fire is not None
        fire_status['candidate'] = (
            None if primary_fire is None else primary_fire.as_dict()
        )
        fire_status['aim'] = aim_observation(
            fire_status['smoothed_center_normalized'],
            self.aim_deadband_x,
            self.aim_deadband_y,
        )

        processing_ms = (time.perf_counter() - callback_start) * 1000.0
        source_age_ms = self._source_age_ms(msg)
        self._processed += 1
        self._period_processed += 1
        self._period_processing_ms.append(processing_ms)
        self._publish_observation(
            msg,
            state='ok',
            detections=detections,
            fire_status=fire_status,
            processing_ms=processing_ms,
            inference_ms=inference_ms,
            source_age_ms=source_age_ms,
            image_shape=frame.shape,
        )

        if self._annotation_due(now):
            self._publish_annotation(
                msg,
                frame,
                detections,
                fire_status,
                source_age_ms,
                processing_ms,
            )
            self._last_annotation_time = now

    def _check_input_health(self):
        age = time.monotonic() - self._last_input_time
        if age <= self.input_timeout or self._camera_stale:
            return
        self._camera_stale = True
        self._reset_fire_tracking()
        self.get_logger().warning(
            f'[YOLO] no camera frame for {age:.2f}s; temporal fire '
            'evidence cleared'
        )
        empty = CompressedImage()
        self._publish_observation(
            empty,
            state='camera_stale',
            error=f'no input frame for {age:.2f}s',
        )

    def _log_stats(self):
        elapsed = max(0.001, self.stats_period)
        processing_mean = 0.0
        processing_max = 0.0
        if self._period_processing_ms:
            processing_mean = statistics.mean(self._period_processing_ms)
            processing_max = max(self._period_processing_ms)
        self.get_logger().info(
            f'[YOLO] {elapsed:.0f}s: rx={self._period_received} '
            f'({self._period_received / elapsed:.2f} FPS), '
            f'processed={self._period_processed} '
            f'({self._period_processed / elapsed:.2f} FPS), '
            f'processing_ms mean={processing_mean:.1f} '
            f'max={processing_max:.1f}, '
            f'decode_errors={self._period_decode_errors}, '
            f'inference_errors={self._period_inference_errors}'
        )
        self._period_received = 0
        self._period_processed = 0
        self._period_decode_errors = 0
        self._period_inference_errors = 0
        self._period_processing_ms.clear()


def main(args=None):
    """Run the observer until ROS shutdown or Ctrl-C."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloObserver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
