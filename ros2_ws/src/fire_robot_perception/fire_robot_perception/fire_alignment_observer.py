"""Preview fire-target yaw alignment without publishing motion commands."""

import json
import math
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
from std_msgs.msg import String


def telemetry_qos():
    """Return a no-backlog profile for perception telemetry."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def extract_target_error(payload, max_source_age_ms):
    """Validate observer JSON and return a trackable horizontal error."""
    if payload.get('observe_only') is not True:
        return 'UNSAFE_INPUT', None, 'input is not marked observe-only'
    state = str(payload.get('state', 'missing'))
    if state != 'ok':
        return f'INPUT_{state.upper()}', None, ''

    fire = payload.get('fire') or {}
    if not fire.get('detected_now') or not fire.get('confirmed'):
        return 'NO_TARGET', None, ''
    center = fire.get('smoothed_center_normalized')
    if not isinstance(center, (list, tuple)) or len(center) != 2:
        return 'INVALID_TARGET', None, 'missing smoothed center'
    if not all(isinstance(value, (int, float)) for value in center):
        return 'INVALID_TARGET', None, 'non-numeric smoothed center'
    error_x = float(center[0])
    if not math.isfinite(error_x) or not -1.0 <= error_x <= 1.0:
        return 'INVALID_TARGET', None, 'horizontal error outside [-1, 1]'

    source_age = (payload.get('source') or {}).get('age_ms')
    warning = ''
    if isinstance(source_age, (int, float)):
        source_age = float(source_age)
        if not math.isfinite(source_age):
            return 'INVALID_TARGET', None, 'non-finite source age'
        if source_age >= 0.0 and source_age > max_source_age_ms:
            return 'STALE_SOURCE', None, 'source image is too old'
        if source_age < 0.0:
            warning = 'negative source age; check Pi/laptop clock sync'
    return 'TRACKABLE', error_x, warning


class AlignmentPreviewController:
    """Generate a bounded, slew-limited yaw preview from image error."""

    def __init__(
            self, deadband_enter, deadband_exit, proportional_gain,
            max_angular_speed, max_angular_acceleration, max_step_dt=0.25):
        if not 0.0 <= deadband_enter < deadband_exit < 1.0:
            raise ValueError(
                'deadbands must satisfy 0 <= enter < exit < 1'
            )
        if proportional_gain <= 0.0:
            raise ValueError('proportional_gain must be positive')
        if max_angular_speed <= 0.0 or max_angular_acceleration <= 0.0:
            raise ValueError('angular limits must be positive')
        if max_step_dt <= 0.0:
            raise ValueError('max_step_dt must be positive')
        self.deadband_enter = float(deadband_enter)
        self.deadband_exit = float(deadband_exit)
        self.proportional_gain = float(proportional_gain)
        self.max_angular_speed = float(max_angular_speed)
        self.max_angular_acceleration = float(max_angular_acceleration)
        self.max_step_dt = float(max_step_dt)
        self.reset()

    def reset(self):
        """Clear controller state after target loss or invalid telemetry."""
        self._centered = False
        self._last_output = 0.0
        self._last_update = None

    def _zero_preview(self, now):
        self._last_output = 0.0
        self._last_update = now
        return {
            'state': 'CENTERED',
            'direction': 'STOP',
            'target_angular_z_rad_s': 0.0,
            'proposed_angular_z_rad_s': 0.0,
            'slew_limited': False,
            'saturated': False,
        }

    def update(self, error_x, now=None):
        """Return one preview step; positive yaw is a left turn."""
        error_x = float(error_x)
        if not math.isfinite(error_x) or not -1.0 <= error_x <= 1.0:
            raise ValueError('error_x must be finite and within [-1, 1]')
        now = time.monotonic() if now is None else float(now)
        magnitude = abs(error_x)

        if self._centered:
            if magnitude <= self.deadband_exit:
                preview = self._zero_preview(now)
                preview['error_x_normalized'] = round(error_x, 4)
                return preview
            self._centered = False
        elif magnitude <= self.deadband_enter:
            self._centered = True
            preview = self._zero_preview(now)
            preview['error_x_normalized'] = round(error_x, 4)
            return preview

        effective_error = max(0.0, magnitude - self.deadband_enter)
        raw_target = -math.copysign(
            self.proportional_gain * effective_error,
            error_x,
        )
        target = _clamp(
            raw_target,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        saturated = not math.isclose(target, raw_target, abs_tol=1e-9)

        if self._last_update is None:
            output = 0.0
        else:
            dt = _clamp(now - self._last_update, 0.0, self.max_step_dt)
            max_delta = self.max_angular_acceleration * dt
            delta = _clamp(
                target - self._last_output,
                -max_delta,
                max_delta,
            )
            output = self._last_output + delta
        slew_limited = not math.isclose(output, target, abs_tol=1e-9)
        self._last_output = output
        self._last_update = now

        direction = 'RIGHT' if error_x > 0.0 else 'LEFT'
        return {
            'state': f'ALIGNING_{direction}',
            'direction': direction,
            'error_x_normalized': round(error_x, 4),
            'target_angular_z_rad_s': round(target, 4),
            'proposed_angular_z_rad_s': round(output, 4),
            'slew_limited': slew_limited,
            'saturated': saturated,
        }

    def inactive_preview(self, state):
        """Reset immediately and return a zero preview for unsafe input."""
        self.reset()
        return {
            'state': state,
            'direction': 'STOP',
            'error_x_normalized': None,
            'target_angular_z_rad_s': 0.0,
            'proposed_angular_z_rad_s': 0.0,
            'slew_limited': False,
            'saturated': False,
        }


class FireAlignmentObserver(Node):
    """Convert fire target telemetry into action-free yaw previews."""

    def __init__(self):
        super().__init__('fire_alignment_observer')
        self._declare_parameters()
        self._read_parameters()
        self._controller = AlignmentPreviewController(
            self.deadband_enter,
            self.deadband_exit,
            self.proportional_gain,
            self.max_angular_speed,
            self.max_angular_acceleration,
        )
        self._publisher = self.create_publisher(
            String, self.preview_topic, telemetry_qos())
        self.create_subscription(
            String, self.input_topic, self._on_observation, telemetry_qos())
        self._last_input = time.monotonic()
        self._input_stale = False
        self.create_timer(0.1, self._check_input_health)
        self.get_logger().warning(
            '[ALIGNMENT] OBSERVE-ONLY: yaw values are telemetry previews; '
            'this node owns no motion or actuator publisher'
        )
        self.get_logger().info(
            f'[ALIGNMENT] {self.input_topic} -> {self.preview_topic}; '
            f'deadband={self.deadband_enter:.2f}/{self.deadband_exit:.2f}, '
            f'kp={self.proportional_gain:.2f}, '
            f'max_speed={self.max_angular_speed:.2f} rad/s, '
            f'max_accel={self.max_angular_acceleration:.2f} rad/s^2'
        )

    def _declare_parameters(self):
        self.declare_parameter('input_topic', '/yolo/detections')
        self.declare_parameter(
            'preview_topic', '/yolo/alignment_preview')
        self.declare_parameter('deadband_enter_x', 0.10)
        self.declare_parameter('deadband_exit_x', 0.13)
        self.declare_parameter('proportional_gain', 1.0)
        self.declare_parameter('max_angular_speed_rad_s', 0.50)
        self.declare_parameter('max_angular_acceleration_rad_s2', 0.80)
        self.declare_parameter('max_source_age_ms', 300.0)
        self.declare_parameter('input_timeout_s', 0.50)

    def _read_parameters(self):
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.preview_topic = str(self.get_parameter('preview_topic').value)
        self.deadband_enter = float(
            self.get_parameter('deadband_enter_x').value)
        self.deadband_exit = float(
            self.get_parameter('deadband_exit_x').value)
        self.proportional_gain = float(
            self.get_parameter('proportional_gain').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed_rad_s').value)
        self.max_angular_acceleration = float(
            self.get_parameter('max_angular_acceleration_rad_s2').value)
        self.max_source_age = float(
            self.get_parameter('max_source_age_ms').value)
        self.input_timeout = float(
            self.get_parameter('input_timeout_s').value)
        if not self.input_topic or not self.preview_topic:
            raise ValueError('alignment topic names cannot be empty')
        if self.max_source_age <= 0.0 or self.input_timeout <= 0.0:
            raise ValueError('alignment freshness limits must be positive')

    def _publish(self, source, preview, warning='', error=''):
        output = {
            'schema_version': 1,
            'observe_only': True,
            'state': preview['state'],
            'input_frame_index': source.get('frame_index'),
            'source': source.get('source'),
            'preview': preview,
            'limits': {
                'deadband_enter_x': self.deadband_enter,
                'deadband_exit_x': self.deadband_exit,
                'proportional_gain': self.proportional_gain,
                'max_angular_speed_rad_s': self.max_angular_speed,
                'max_angular_acceleration_rad_s2': (
                    self.max_angular_acceleration
                ),
            },
            'safety': {
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
            },
        }
        if warning:
            output['warning'] = warning
        if error:
            output['error'] = error
        message = String()
        message.data = json.dumps(
            output, separators=(',', ':'), ensure_ascii=True)
        self._publisher.publish(message)

    def _on_observation(self, message):
        self._last_input = time.monotonic()
        if self._input_stale:
            self._input_stale = False
            self.get_logger().info('[ALIGNMENT] target telemetry resumed')
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('JSON root is not an object')
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            preview = self._controller.inactive_preview('INVALID_JSON')
            self._publish({}, preview, error=str(exc))
            return

        state, error_x, warning = extract_target_error(
            payload, self.max_source_age)
        if state == 'TRACKABLE':
            preview = self._controller.update(error_x)
        else:
            preview = self._controller.inactive_preview(state)
        self._publish(payload, preview, warning=warning)

    def _check_input_health(self):
        age = time.monotonic() - self._last_input
        if age <= self.input_timeout or self._input_stale:
            return
        self._input_stale = True
        preview = self._controller.inactive_preview('INPUT_STALE')
        self._publish({}, preview, error=f'no input for {age:.2f}s')
        self.get_logger().warning(
            f'[ALIGNMENT] no target telemetry for {age:.2f}s; preview zeroed'
        )


def main(args=None):
    """Run the alignment observer until ROS shutdown or Ctrl-C."""
    rclpy.init(args=args)
    node = None
    try:
        node = FireAlignmentObserver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
