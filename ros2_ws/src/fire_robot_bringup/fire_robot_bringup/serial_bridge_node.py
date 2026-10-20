"""Bridge ROS 2 and raw serial V2."""
import json
import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool
import threading
import time
from uuid import uuid4

from fire_robot_bringup.serial_transport_core import SerialTransportCore
from fire_robot_bringup.serial_process import SerialProcessClient


DEFAULT_SERIAL_PORT = (
    '/dev/serial/by-id/'
    'usb-Espressif_Systems_Freenove_ESP32-S3_WROOM_N8R2_'
    '_8MB_Flash___2MB_PSRAM__28372F1F73C8-if00'
)


class UsbRecoveryCapture:
    """Bounded USB frame logging, independent of periodic timing statistics."""

    MAX_FRAMES = 128

    def __init__(self, duration_s):
        if (isinstance(duration_s, bool) or not isinstance(duration_s, int)
                or not 0 <= duration_s <= 180):
            raise ValueError('usb_recovery_capture_s must be an integer 0-180')
        self.duration_s = duration_s
        self.deadline = (time.monotonic() + duration_s
                         if duration_s else None)
        self.count = 0
        self.finished = False

    def finish_if_due(self):
        if self.deadline is None or self.finished:
            return None
        reason = ('frame_limit' if self.count >= self.MAX_FRAMES else
                  'duration_elapsed' if time.monotonic() >= self.deadline else
                  None)
        if reason is None:
            return None
        self.finished = True
        return {'reason': reason, 'frames': self.count}

    def record(self, frame, epoch, frame_mono):
        if (self.deadline is None or self.finished
                or self.count >= self.MAX_FRAMES
                or time.monotonic() >= self.deadline):
            return None
        self.count += 1
        return {
            'epoch': epoch,
            'pi_wall_ns': time.time_ns(),
            'receive_mono_ns': int(frame_mono * 1_000_000_000),
            'usb': frame,
        }


class SerialBridgeNode(SerialTransportCore, Node):
    """Bridge Node between ROS 2 topics and raw serial."""

    def __init__(self, serial_cls=None):
        """Initialize SerialBridgeNode."""
        super().__init__('serial_bridge_node')
        # ACT provenance is control telemetry, independent of diagnostic logs.
        # Connection epochs alone can repeat after this node restarts.
        self._actuator_source_instance = uuid4().hex

        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('first_state_timeout_s', 8.0)
        self.declare_parameter('pump_command_timeout_s', 0.7)
        self.declare_parameter('enable_diagnostics', True)
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value
        if not isinstance(self.enable_diagnostics, bool):
            raise ValueError('enable_diagnostics must be a boolean')
        self.declare_parameter('usb_recovery_capture_s', 0)
        capture = UsbRecoveryCapture(
            self.get_parameter('usb_recovery_capture_s').value)
        self._usb_capture = capture if capture.deadline is not None else None
        self.declare_parameter('usb_fault_trace_events', 0)
        fault_trace_events = self.get_parameter('usb_fault_trace_events').value
        if (isinstance(fault_trace_events, bool)
                or not isinstance(fault_trace_events, int)
                or not 0 <= fault_trace_events <= 16):
            raise ValueError('usb_fault_trace_events must be an integer 0-16')

        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('serial_baudrate').value
        self.first_state_timeout_s = float(
            self.get_parameter('first_state_timeout_s').value)
        if (not math.isfinite(self.first_state_timeout_s)
                or self.first_state_timeout_s <= 0.0):
            self.get_logger().warning(
                'Invalid first_state_timeout_s; falling back to 8.0 s')
            self.first_state_timeout_s = 8.0
        self.pump_command_timeout_s = float(
            self.get_parameter('pump_command_timeout_s').value)
        if (not math.isfinite(self.pump_command_timeout_s)
                or not 0.5 <= self.pump_command_timeout_s <= 1.0):
            raise ValueError('pump_command_timeout_s must be 0.5-1.0 s')

        self.serial_cls = serial_cls
        if self.serial_cls is None:
            import serial
            self.serial_cls = serial.Serial

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', best_effort_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', best_effort_qos)
        self.env_pub = self.create_publisher(String, '/env_status', best_effort_qos)
        self.ir_pub = self.create_publisher(
            String, '/fire_sensors/ir_raw', best_effort_qos)
        self.actuator_pub = self.create_publisher(
            String, '/fire_sensors/actuator_state', best_effort_qos)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, best_effort_qos)
        self.create_subscription(Point, '/fire_target', self.fire_target_callback, reliable_qos)
        self.create_subscription(Bool, '/pump_cmd', self.pump_cmd_callback, reliable_qos)

        self.init_transport(self.port, self.baudrate, self.first_state_timeout_s,
                            self.pump_command_timeout_s, self.serial_cls,
                            diagnostics_enabled=self.enable_diagnostics,
                            fault_trace_events=fault_trace_events)
        self.transport_process = None
        self._ipc_seen = {}
        self._ipc_fault_logged = False
        self.log_timer = (self.create_timer(10.0, self.log_timer_callback)
                          if self.enable_diagnostics else None)
        self.telemetry_timer = self.create_timer(0.05, self.telemetry_publish_callback)

        # Explicit serial dependency injection is for legacy unit tests only.
        # Normal runtime always uses the isolated worker and never opens USB here.
        if serial_cls is not None:
            self.worker_thread = threading.Thread(target=self.serial_worker, daemon=True)
            self.worker_thread.start()
        else:
            try:
                self.transport_process = SerialProcessClient({
                    'port': self.port, 'baudrate': self.baudrate,
                    'first_state_timeout_s': self.first_state_timeout_s,
                    'pump_command_timeout_s': self.pump_command_timeout_s,
                    'enable_diagnostics': self.enable_diagnostics,
                    'usb_fault_trace_events': fault_trace_events})
                self.transport_process.heartbeat()
            except BaseException:
                if self.transport_process is not None:
                    self.transport_process.close()
                self.telemetry_timer.cancel()
                if self.log_timer is not None:
                    self.log_timer.cancel()
                Node.destroy_node(self)
                raise
        if self._usb_capture is not None:
            self.get_logger().info(
                'esp_usb_capture_start duration_s=%d max_frames=%d' % (
                    self._usb_capture.duration_s,
                    self._usb_capture.MAX_FRAMES))


    def _poll_transport(self):
        process = self.transport_process
        process.heartbeat()
        status = process.poll()
        now = time.monotonic()
        valid = (status is not None and not process.fault
                 and 0 <= now - status['mono'] <= 0.2)
        with self.state_lock:
            self.session_ready = bool(valid and status['ready'] and not status['fault'])
            self.telemetry_healthy = bool(self.session_ready and status['healthy'])
            if valid:
                self.session_epoch = status['epoch']
                self._session_state = status['state']
                for name, item in status['slots'].items():
                    if item is not None:
                        key = (item[1], item[2])
                        if self._ipc_seen.get(name) != key:
                            setattr(self, name, tuple(item))
                            self._ipc_seen[name] = key
            if not self.session_ready:
                for name in ('latest_state', 'latest_env', 'latest_ir',
                             'latest_actuator', 'latest_usb_diagnostic'):
                    setattr(self, name, None)
        fault = process.fault or (status or {}).get('fault', '')
        if fault and not self._ipc_fault_logged:
            self.get_logger().error('Serial IPC fault latched; restart session required: ' + fault)
            self._ipc_fault_logged = True

    def cmd_vel_callback(self, msg):
        if getattr(self, 'transport_process', None) is None:
            return SerialTransportCore.cmd_vel_callback(self, msg)
        self.transport_process.command('CMD', [msg.linear.x, msg.angular.z], self.session_epoch)

    def pump_cmd_callback(self, msg):
        if getattr(self, 'transport_process', None) is None:
            return SerialTransportCore.pump_cmd_callback(self, msg)
        self.transport_process.command('PUMP', [int(msg.data)], self.session_epoch)

    def fire_target_callback(self, msg):
        if getattr(self, 'transport_process', None) is None:
            return SerialTransportCore.fire_target_callback(self, msg)
        self.transport_process.command('FIRE', [msg.x, msg.y], self.session_epoch)

    def _record_publish_result(self, *, published=0, failed=0, dropped=0,
                               state_age=None, env_age=None):
        """Update windowed publish diagnostics atomically."""
        if not getattr(self, 'enable_diagnostics', True):
            return
        with self.state_lock:
            self.telemetry_publish_count += published
            self.telemetry_publish_fail_count += failed
            self.telemetry_drop_count += dropped
            if state_age is not None:
                self.state_publish_max_age = max(
                    self.state_publish_max_age, state_age)
            if env_age is not None:
                self.env_publish_max_age = max(
                    self.env_publish_max_age, env_age)


    def telemetry_publish_callback(self):
        """Publish telemetry from latest slots."""
        if getattr(self, 'transport_process', None) is not None:
            self._poll_transport()
        capture = getattr(self, '_usb_capture', None)
        if capture is not None:
            ended = capture.finish_if_due()
            if ended is not None:
                self.get_logger().info('esp_usb_capture_end ' + json.dumps(
                    ended, sort_keys=True, separators=(',', ':')))
        if getattr(self, 'enable_diagnostics', True):
            self.odom_timing.mark('telemetry_timer')
        with self.publish_lock:
            if getattr(self, 'node_destroyed', False):
                return

        with self.state_lock:
            state_item = self.latest_state
            self.latest_state = None
            env_item = self.latest_env
            self.latest_env = None
            ir_item = self.latest_ir
            self.latest_ir = None
            actuator_item = self.latest_actuator
            self.latest_actuator = None
            usb_item = getattr(self, 'latest_usb_diagnostic', None)
            self.latest_usb_diagnostic = None

        if state_item:
            frame, epoch, frame_mono = state_item
            age = time.monotonic() - frame_mono

            with self.state_lock:
                valid_odom = (self.running and self.session_ready and
                              self.telemetry_healthy and self.session_epoch == epoch)

            if valid_odom and age < 0.2:
                try:
                    now = self.get_clock().now().to_msg()
                    if getattr(self, 'transport_process', None) is not None:
                        stamp_ns = now.sec * 1000000000 + now.nanosec - int(age * 1e9)
                        now.sec, now.nanosec = divmod(max(0, stamp_ns), 1000000000)
                    odom = Odometry()
                    odom.header.stamp = now
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x = frame['x']
                    odom.pose.pose.position.y = frame['y']
                    odom.pose.pose.position.z = 0.0

                    cy = math.cos(frame['yaw'] * 0.5)
                    sy = math.sin(frame['yaw'] * 0.5)
                    odom.pose.pose.orientation.x = 0.0
                    odom.pose.pose.orientation.y = 0.0
                    odom.pose.pose.orientation.z = sy
                    odom.pose.pose.orientation.w = cy

                    odom.twist.twist.linear.x = frame['vx']
                    odom.twist.twist.angular.z = frame['wz']

                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            if getattr(self, 'enable_diagnostics', True):
                                publish_started = time.monotonic()
                                self.odom_timing.mark(
                                    'odom_publish_start', publish_started)
                                try:
                                    self.odom_pub.publish(odom)
                                finally:
                                    self.odom_timing.observe(
                                        'odom_publish_call',
                                        (time.monotonic() - publish_started)
                                        * 1000.0)
                                self.odom_timing.mark('odom_publish_success')
                            else:
                                self.odom_pub.publish(odom)
                    self._record_publish_result(
                        published=1, state_age=age)
                except Exception as e:
                    self.get_logger().warn(f"Odom publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

            with self.state_lock:
                valid_imu = (self.running and self.session_ready and
                             self.telemetry_healthy and self.session_epoch == epoch)

            age = time.monotonic() - frame_mono
            if valid_imu and age < 0.2:
                try:
                    # Reuse 'now', but don't overwrite it since they should match
                    imu = Imu()
                    imu.header.stamp = now
                    imu.header.frame_id = 'imu_frame'
                    imu.orientation.x = 0.0
                    imu.orientation.y = 0.0
                    imu.orientation.z = sy
                    imu.orientation.w = cy
                    imu.angular_velocity.z = frame['gyro_z']
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.imu_pub.publish(imu)
                    self._record_publish_result(published=1)
                except Exception as e:
                    self.get_logger().warn(f"Imu publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

        if env_item:
            frame, epoch, frame_mono = env_item
            age = time.monotonic() - frame_mono

            with self.state_lock:
                valid_env = (self.running and self.session_ready and
                             self.telemetry_healthy and self.session_epoch == epoch)

            if valid_env and age < 1.0:
                try:
                    msg = String()
                    if frame['valid'] == 0:
                        msg.data = json.dumps({"status": "WROOM_OFFLINE"})
                    else:
                        fire_str = f"{frame['fire_flags']:03d}"
                        msg.data = json.dumps({
                            "fire": fire_str,
                            "gas": frame['gas_ppm'],
                            "temp": frame['temp_c'],
                            "batt": frame['batt_v']
                        })
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.env_pub.publish(msg)
                    self._record_publish_result(
                        published=1, env_age=age)
                except Exception as e:
                    self.get_logger().warn(f"Env publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

        if ir_item:
            frame, epoch, frame_mono = ir_item
            age = time.monotonic() - frame_mono
            with self.state_lock:
                valid_session = (self.running and self.session_ready and
                                 self.telemetry_healthy and
                                 self.session_epoch == epoch)
            if valid_session and age < 0.3:
                try:
                    valid_sample = frame['valid'] == 1
                    msg = String()
                    msg.data = json.dumps({
                        'schema_version': 1,
                        'valid': valid_sample,
                        'left': frame['left'] if valid_sample else None,
                        'center': frame['center'] if valid_sample else None,
                        'right': frame['right'] if valid_sample else None,
                        'sample_age_ms': frame['sample_age_ms'],
                        'receive_age_ms': round(age * 1000, 1),
                    })
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.ir_pub.publish(msg)
                    self._record_publish_result(published=1)
                except Exception as e:
                    self.get_logger().warn(f'IR publish exception: {e}')
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

        if actuator_item:
            frame, epoch, frame_mono = actuator_item
            # Read ROS time first, then charge the full monotonic receive age.
            # The stamp marks Pi receipt of ACT, not ESP wall time or pump ACK.
            actuator_ros_ns = self.get_clock().now().nanoseconds
            age = time.monotonic() - frame_mono
            with self.state_lock:
                valid_session = (self.running and self.session_ready and
                                 self.telemetry_healthy and
                                 self.session_epoch == epoch)
            if valid_session and 0.0 <= age < 0.5:
                try:
                    stamp_ns = actuator_ros_ns - math.ceil(age * 1e9)
                    if stamp_ns <= 0:
                        raise ValueError('ACT source clock is not ready')
                    stamp_sec, stamp_nanosec = divmod(stamp_ns, 1000000000)
                    phase = ('IDLE', 'SWEEPING', 'RETURNING_HOME', 'LOCKED')
                    msg = String()
                    msg.data = json.dumps({
                        'schema_version': 1,
                        'mode': 'COMMAND_TELEMETRY_ONLY',
                        'source': {
                            'stamp': {'sec': stamp_sec,
                                      'nanosec': stamp_nanosec},
                            'bridge_instance_id': self._actuator_source_instance,
                            'serial_epoch': epoch,
                            'frame_seq': frame['seq'],
                        },
                        'esp_ms': frame['esp_ms'],
                        'pump_pwm_commanded_on': bool(frame['pump_pwm']),
                        'pump_pwm_ready': bool(frame['pump_ready']),
                        'servo_phase': phase[frame['servo_phase']],
                        'servo_pwm_commanded_on': bool(frame['servo_pwm']),
                        'servo_commanded_angle_deg': (
                            frame['servo_angle_deg'] if frame['servo_pwm']
                            else None),
                        'home_command_completed_count': frame['home_count'],
                        'receive_age_ms': round(age * 1000, 1),
                        'physical_pump_or_nozzle_verified': False,
                    })
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.actuator_pub.publish(msg)
                    self._record_publish_result(published=1)
                except Exception as e:
                    self.get_logger().warn(
                        f'Actuator telemetry publish exception: {e}')
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

        if usb_item:
            frame, epoch, frame_mono = usb_item
            with self.state_lock:
                valid_diagnostic = (self.running and self.session_ready and
                                    self.session_epoch == epoch)
            if valid_diagnostic:
                if capture is not None:
                    event = capture.record(frame, epoch, frame_mono)
                    if event is not None:
                        self.get_logger().info(
                            'esp_usb_capture ' + json.dumps(
                                event, sort_keys=True, separators=(',', ':')))
                if getattr(self, 'enable_diagnostics', True):
                    self.get_logger().info(
                        'esp_usb_diag epoch=%d receive_age_ms=%.1f %s' % (
                            epoch, max(0.0, time.monotonic() - frame_mono) * 1000.0,
                            json.dumps(frame, sort_keys=True)))


    def destroy_node(self):
        if getattr(self, 'node_destroyed', False):
            return True
        process = getattr(self, 'transport_process', None)
        if process is not None:
            self.telemetry_timer.cancel()
            if self.log_timer is not None:
                self.log_timer.cancel()
            self.session_ready = self.telemetry_healthy = False
            process.close()
        elif not self.shutdown_transport():
            return False
        try:
            ret = Node.destroy_node(self)
        except Exception:
            return False
        self.node_destroyed = ret is None or ret is True
        return self.node_destroyed


def main(args=None):
    """Run main entry point."""
    node = None
    shutdown_requested = [False]

    def graceful_signal_handler(signum, frame):
        if not shutdown_requested[0]:
            shutdown_requested[0] = True
            raise KeyboardInterrupt()

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)
    primary_exc = None
    cleanup_errors = []

    try:
        signal.signal(signal.SIGTERM, graceful_signal_handler)
        signal.signal(signal.SIGINT, graceful_signal_handler)

        try:
            rclpy.init(
                args=args,
                signal_handler_options=SignalHandlerOptions.NO)
            node = SerialBridgeNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            shutdown_requested[0] = True
        except BaseException as e:
            primary_exc = e
        finally:
            shutdown_requested[0] = True

            if node is not None:
                try:
                    deadline = time.monotonic() + 3.7
                    while True:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        if node.destroy_node():
                            break
                        time.sleep(min(0.1, remaining))

                    if not getattr(node, 'node_destroyed', False):
                        node.destroy_node()
                except BaseException as e:
                    cleanup_errors.append(e)

            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException as e:
                cleanup_errors.append(e)
    except KeyboardInterrupt:
        shutdown_requested[0] = True
    except BaseException as e:
        if primary_exc is None:
            primary_exc = e
    finally:
        try:
            signal.signal(signal.SIGTERM, original_sigterm)
        except BaseException as e:
            cleanup_errors.append(e)

        try:
            signal.signal(signal.SIGINT, original_sigint)
        except BaseException as e:
            cleanup_errors.append(e)

        if primary_exc is not None:
            raise primary_exc
        if cleanup_errors:
            raise cleanup_errors[0]


if __name__ == '__main__':
    main()
