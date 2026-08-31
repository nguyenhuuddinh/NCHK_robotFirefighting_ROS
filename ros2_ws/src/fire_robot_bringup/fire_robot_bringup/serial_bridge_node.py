"""Bridge ROS 2 and raw serial V2."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool
import threading
import time

from fire_robot_bringup.serial_protocol import SerialProtocolV2


class SerialBridgeNode(Node):
    """Bridge Node between ROS 2 topics and raw serial."""

    def __init__(self, serial_cls=None):
        """Initialize SerialBridgeNode."""
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)

        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('serial_baudrate').value

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

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, best_effort_qos)
        self.create_subscription(Point, '/fire_target', self.fire_target_callback, reliable_qos)
        self.create_subscription(Bool, '/pump_cmd', self.pump_cmd_callback, reliable_qos)

        self.protocol = SerialProtocolV2()
        self.ser = None

        self.state_lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self.node_destroyed = False
        self.stop_request = False

        self.session_ready = False
        self.session_epoch = 0
        self.telemetry_healthy = False
        self.state_stale_timeout_ms = 500

        self.pending_cmd = None
        self.pending_pump = None
        self.pending_fire = None

        self.pump_state = 0
        self.cmd_version = 0
        self.pump_version = 0
        self.fire_version = 0
        self.pump_written_version = 0
        self.last_pump_time = 0.0
        self.session_started_mono = 0.0
        self.last_state_time_mono = 0.0
        self.last_env_time_mono = 0.0

        self.reconnect_backoff = 0.0
        self.next_reconnect_mono = 0.0
        self.latest_state = None
        self.latest_env = None

        self.tx_fail_count = 0
        self.tx_partial_count = 0
        self.reconnect_count = 0
        self.cmd_drop_count = 0
        self.telemetry_drop_count = 0
        self.telemetry_publish_count = 0
        self.telemetry_publish_fail_count = 0
        self.bootstrap_fail_count = 0
        self.read_fail_count = 0
        self.open_attempt_count = 0
        self.open_success_count = 0
        self.open_fail_count = 0
        self.last_failure_reason = 'none'

        self.state_publish_max_age = 0.0
        self.env_publish_max_age = 0.0

        self.log_timer = self.create_timer(10.0, self.log_timer_callback)
        self.telemetry_timer = self.create_timer(0.05, self.telemetry_publish_callback)

        self.running = True
        self.worker_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.worker_thread.start()

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and
                     (msg.linear.x != 0.0 or msg.angular.z != 0.0)):
                self.cmd_drop_count += 1
                return
            self.cmd_version += 1
            self.pending_cmd = ((msg.linear.x, msg.angular.z),
                                self.session_epoch, self.cmd_version)

    def fire_target_callback(self, msg):
        """Handle /fire_target."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    not getattr(self, 'telemetry_healthy', False):
                self.cmd_drop_count += 1
                return
            self.fire_version += 1
            self.pending_fire = ((msg.x, msg.y), self.session_epoch, self.fire_version)

    def pump_cmd_callback(self, msg):
        """Handle /pump_cmd."""
        pump_req = 1 if msg.data else 0
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and pump_req != 0):
                self.cmd_drop_count += 1
                return
            self.pump_version += 1
            self.pending_pump = (pump_req, self.session_epoch, self.pump_version)

    def close_serial(self):
        """Close serial port safely."""
        with self.state_lock:
            self.session_ready = False
            self.telemetry_healthy = False
            if not getattr(self, 'stop_request', False):
                self.session_epoch += 1
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None

        if self.ser:
            try:
                with getattr(self, 'tx_lock', threading.Lock()):
                    self.ser.close()
            except Exception as e:
                self.get_logger().warn(f"Serial close error: {e}")
            finally:
                self.ser = None

    def _write_frame(self, frame: bytes) -> bool:
        """Write frame directly, handle errors. Return True if success."""
        if not frame or not self.ser:
            return False
        try:
            written = self.ser.write(frame)
            if written < len(frame):
                self.tx_partial_count += 1
                raise Exception("Partial write")
            return True
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self.tx_fail_count += 1
            self.last_failure_reason = f"tx:{type(e).__name__}:{str(e)}"
            self.close_serial()
            return False

    def _pending_snapshot(self):
        """Take a snapshot of pending TX items without clearing."""
        snapshot = []
        now_mono = time.monotonic()
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    not getattr(self, 'telemetry_healthy', False):
                return snapshot

            c_cmd = getattr(self, 'pending_cmd', None)
            if c_cmd:
                args, ep, ver = c_cmd
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('CMD', args, ep, ver, c_cmd))

            c_pump = getattr(self, 'pending_pump', None)
            if c_pump:
                req, ep, ver = c_pump
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('PUMP', req, ep, ver, c_pump))
            elif now_mono - getattr(self, 'last_pump_time', 0.0) >= 0.5:
                req = getattr(self, 'pump_state', 0)
                ver = getattr(self, 'pump_written_version', 0)
                ep = getattr(self, 'session_epoch', -1)
                snapshot.append(('PUMP_REFRESH', req, ep, ver, None))

            c_fire = getattr(self, 'pending_fire', None)
            if c_fire:
                args, ep, ver = c_fire
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('FIRE', args, ep, ver, c_fire))

        return snapshot

    def _write_pending_item(self, item, current_ser):
        """Re-validate and write a single pending item."""
        itype, data, ep, ver, original_tuple = item
        frame = None

        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    ep != getattr(self, 'session_epoch', -1) or \
                    getattr(self, 'stop_request', False) or \
                    self.ser is not current_ser or getattr(self.ser, 'closed', True):
                return True

            if not getattr(self, 'telemetry_healthy', False):
                if itype == 'CMD' and data != (0.0, 0.0):
                    return True
                if itype == 'PUMP' and data != 0:
                    return True
                if itype == 'FIRE' or itype == 'PUMP_REFRESH':
                    return True

            if itype == 'CMD':
                current_item = getattr(self, 'pending_cmd', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        self.cmd_drop_count += 1
                        return True
                frame = self.protocol.generate_cmd(data[0], data[1])
            elif itype == 'PUMP':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not original_tuple and current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        self.cmd_drop_count += 1
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'PUMP_REFRESH':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'FIRE':
                current_item = getattr(self, 'pending_fire', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_fire(data[0], data[1])

        if not frame:
            return True

        if getattr(self, 'stop_request', False):
            return True

        try:
            written = current_ser.write(frame)
            if written < len(frame):
                self.tx_partial_count += 1
                return False
        except Exception as e:
            self.tx_fail_count += 1
            self.last_failure_reason = f"tx:{type(e).__name__}:{str(e)}"
            return False

        with self.state_lock:
            if itype == 'CMD':
                if getattr(self, 'pending_cmd', None) is original_tuple:
                    self.pending_cmd = None
            elif itype == 'PUMP':
                if getattr(self, 'pending_pump', None) is original_tuple:
                    self.pending_pump = None
                self.pump_state = data
                self.pump_written_version = ver
                self.last_pump_time = time.monotonic()
            elif itype == 'PUMP_REFRESH':
                self.last_pump_time = time.monotonic()
            elif itype == 'FIRE':
                if getattr(self, 'pending_fire', None) is original_tuple:
                    self.pending_fire = None

        return True

    def serial_worker(self):
        """Worker thread for reading and writing."""
        import time
        try:
            while getattr(self, 'running', False):
                if getattr(self, 'stop_request', False):
                    break
                now = time.monotonic()
                if now < getattr(self, 'next_reconnect_mono', 0.0):
                    time.sleep(0.01)
                    continue

                budget = 10
                read_failed = False
                while budget > 0 and getattr(self, 'running', False):
                    try:
                        waiting = getattr(self.ser, 'in_waiting', 0) if self.ser else 0
                        if waiting == 0:
                            if self.ser:
                                chunk = self.ser.read(1)
                                if chunk:
                                    waiting = getattr(self.ser, 'in_waiting', 0)
                                    if waiting > 0:
                                        chunk += self.ser.read(min(waiting, 1023))
                                    for frame in self.protocol.parse_chunk(chunk):
                                        self.handle_frame(frame)
                            break
                        if self.ser:
                            chunk = self.ser.read(min(waiting, 1024))
                            if chunk:
                                for frame in self.protocol.parse_chunk(chunk):
                                    self.handle_frame(frame)
                    except Exception as e:
                        err_str = str(e)
                        self.get_logger().warn(f"Serial read error: {err_str}")
                        self.last_failure_reason = f"read:{type(e).__name__}:{err_str}"
                        self.read_fail_count += 1
                        self.close_serial()
                        self.reconnect_backoff = min(
                            1.0, max(0.1, getattr(self, 'reconnect_backoff', 0.0) * 1.5))
                        self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff
                        read_failed = True
                        break
                    budget -= 1

                if read_failed:
                    continue

                if getattr(self, 'ser', None) is None or getattr(self.ser, 'closed', True):
                    self.close_serial()
                    self.open_attempt_count += 1
                    try:
                        self.ser = self.serial_cls(
                            self.port, self.baudrate, timeout=0.01, write_timeout=0.1)
                        self.open_success_count += 1
                        self.get_logger().info(f"Opened serial port {self.port}")
                        self.reconnect_count += 1
                        self.protocol.reset_parser()
                    except Exception as e:
                        err_str = str(e)
                        self.open_fail_count += 1
                        self.last_failure_reason = f"open:{type(e).__name__}:{err_str}"
                        self.get_logger().warn(f"Failed to open port {self.port}: {err_str}")
                        self.close_serial()
                        self.reconnect_backoff = min(
                            1.0, max(0.1, getattr(self, 'reconnect_backoff', 0.0) * 1.5))
                        self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff
                        continue

                    try:
                        current_ser = self.ser
                        if current_ser is None:
                            raise Exception("Serial port closed immediately after open")
                        with getattr(self, 'tx_lock', __import__('threading').Lock()):
                            with self.state_lock:
                                if getattr(self, 'stop_request', False) or \
                                        self.ser is not current_ser:
                                    raise Exception("Stopped before bootstrap")
                                cmd = self.protocol.generate_cmd(0.0, 0.0)
                                pump = self.protocol.generate_pump(0)

                            if getattr(self, 'stop_request', False):
                                raise Exception("Stopped before write")
                            w1 = current_ser.write(cmd)
                            if w1 < len(cmd):
                                self.tx_partial_count += 1
                                raise Exception("Partial write during bootstrap")

                            with self.state_lock:
                                if getattr(self, 'stop_request', False) or \
                                        self.ser is not current_ser:
                                    raise Exception("Stopped before pump")

                            w2 = current_ser.write(pump)
                            if w2 < len(pump):
                                self.tx_partial_count += 1
                                raise Exception("Partial write during bootstrap")

                            with self.state_lock:
                                if getattr(self, 'stop_request', False) or \
                                        self.ser is not current_ser:
                                    raise Exception("Stopped after write")
                                self.session_ready = True
                                self.telemetry_healthy = False
                                import time
                                self.session_started_mono = time.monotonic()
                                self.last_state_time_mono = 0.0
                                self.last_pump_time = time.monotonic()
                                self.pump_state = 0
                    except Exception as e:
                        err_str = str(e)
                        self.get_logger().warn(f"Failed to bootstrap port {self.port}: {err_str}")
                        self.bootstrap_fail_count += 1
                        self.last_failure_reason = f"boot:{type(e).__name__}:{err_str}"
                        self.close_serial()
                        self.reconnect_backoff = min(
                            1.0, max(0.1, getattr(self, 'reconnect_backoff', 0.0) * 1.5))
                        self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff
                        continue

                need_close = False
                with self.state_lock:
                    stale = False
                    if self.session_ready:
                        import time
                        if getattr(self, 'last_state_time_mono', 0.0) > 0.0:
                            age = time.monotonic() - self.last_state_time_mono
                        else:
                            age = time.monotonic() - getattr(self, 'session_started_mono', 0.0)
                        if age > (self.state_stale_timeout_ms / 1000.0):
                            stale = True
                    if stale:
                        self.telemetry_healthy = False
                        need_close = True

                if need_close:
                    self.close_serial()
                    continue

                snapshot = self._pending_snapshot()
                if not snapshot:
                    continue

                tx_acquired = self.tx_lock.acquire(timeout=0.01)
                if tx_acquired:
                    tx_fail = False
                    try:
                        current_ser = self.ser
                        for item in snapshot:
                            if not self._write_pending_item(item, current_ser):
                                tx_fail = True
                                break
                    finally:
                        self.tx_lock.release()

                    if tx_fail:
                        self.close_serial()

        except Exception as e:
            self.get_logger().error(f"Worker exception: {e}")
        finally:
            try:
                if getattr(self, 'ser', None) and not getattr(self.ser, 'closed', True):
                    tx_lock = getattr(self, 'tx_lock', None)
                    if tx_lock and tx_lock.acquire(timeout=0.1):
                        try:
                            cmd = self.protocol.generate_cmd(0.0, 0.0)
                            pump = self.protocol.generate_pump(0)
                            if cmd:
                                self.ser.write(cmd)
                            if pump:
                                self.ser.write(pump)
                        finally:
                            tx_lock.release()
            except Exception:
                pass
            with self.state_lock:
                self.session_ready = False
            self.close_serial()

    def handle_frame(self, frame):
        """Handle parsed telemetry frame."""
        now_mono = time.monotonic()
        with self.state_lock:
            if not self.session_ready:
                return
            epoch = self.session_epoch

            if frame['type'] == 'STATE':
                self.last_state_time_mono = now_mono
                self.telemetry_healthy = True
                self.reconnect_backoff = 0.0
                self.next_reconnect_mono = 0.0
                self.latest_state = (frame, epoch, now_mono)
            elif frame['type'] == 'ENV':
                self.last_env_time_mono = now_mono
                self.latest_env = (frame, epoch, now_mono)

    def telemetry_publish_callback(self):
        """Publish telemetry from latest slots."""
        with self.publish_lock:
            if getattr(self, 'node_destroyed', False):
                return

        with self.state_lock:
            state_item = self.latest_state
            self.latest_state = None
            env_item = self.latest_env
            self.latest_env = None

        if state_item:
            frame, epoch, frame_mono = state_item
            age = time.monotonic() - frame_mono

            with self.state_lock:
                valid_odom = (self.running and self.session_ready and
                              self.telemetry_healthy and self.session_epoch == epoch)

            if valid_odom and age < 0.2:
                try:
                    now = self.get_clock().now().to_msg()
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
                            self.odom_pub.publish(odom)
                    self.telemetry_publish_count += 1
                    self.state_publish_max_age = max(self.state_publish_max_age, age)
                except Exception as e:
                    self.get_logger().warn(f"Odom publish exception: {e}")
                    self.telemetry_publish_fail_count += 1
            else:
                self.telemetry_drop_count += 1

            with self.state_lock:
                valid_imu = (self.running and self.session_ready and
                             self.telemetry_healthy and self.session_epoch == epoch)

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
                    self.telemetry_publish_count += 1
                except Exception as e:
                    self.get_logger().warn(f"Imu publish exception: {e}")
                    self.telemetry_publish_fail_count += 1
            else:
                self.telemetry_drop_count += 1

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
                    self.telemetry_publish_count += 1
                    self.env_publish_max_age = max(self.env_publish_max_age, age)
                except Exception as e:
                    self.get_logger().warn(f"Env publish exception: {e}")
                    self.telemetry_publish_fail_count += 1
            else:
                self.telemetry_drop_count += 1

    def log_timer_callback(self):
        """Log diagnostics."""
        if not getattr(self, 'session_ready', False):
            age = "never"
            with getattr(self, 'state_lock', __import__('threading').Lock()):
                if getattr(self, 'last_state_time_mono', 0.0) > 0.0:
                    age_val = time.monotonic() - self.last_state_time_mono
                    age = f"{age_val:.1f}s"
                epoch = getattr(self, 'session_epoch', 0)
                recon = getattr(self, 'reconnect_count', 0)
                backoff = getattr(self, 'reconnect_backoff', 0.0)
                last_fail = getattr(self, 'last_failure_reason', 'none')
                tx_f = getattr(self, 'tx_fail_count', 0)
                boot_f = getattr(self, 'bootstrap_fail_count', 0)
                read_f = getattr(self, 'read_fail_count', 0)

                open_attempt = getattr(self, 'open_attempt_count', 0)
                open_succ = getattr(self, 'open_success_count', 0)
                open_fail = getattr(self, 'open_fail_count', 0)

            self.get_logger().info(
                f"Session offline | epoch={epoch} recon={recon} backoff={backoff:.2f}s "
                f"age={age} tx_fail={tx_f} rx_fail={read_f} boot_fail={boot_f} "
                f"open_attempt={open_attempt} open_succ={open_succ} open_fail={open_fail} "
                f"last_err={last_fail}"
            )
            return

        with getattr(self, 'state_lock', threading.Lock()):
            drop = getattr(self, 'cmd_drop_count', 0)
            self.cmd_drop_count = 0
            t_drop = getattr(self, 'telemetry_drop_count', 0)
            self.telemetry_drop_count = 0
            t_pub = getattr(self, 'telemetry_publish_count', 0)
            self.telemetry_publish_count = 0
            t_fail = getattr(self, 'telemetry_publish_fail_count', 0)
            self.telemetry_publish_fail_count = 0
            tx_p = getattr(self, 'tx_partial_count', 0)
            self.tx_partial_count = 0
            tx_f = getattr(self, 'tx_fail_count', 0)
            self.tx_fail_count = 0
            recon = getattr(self, 'reconnect_count', 0)
            self.reconnect_count = 0
            s_max = getattr(self, 'state_publish_max_age', 0.0)
            self.state_publish_max_age = 0.0
            e_max = getattr(self, 'env_publish_max_age', 0.0)
            self.env_publish_max_age = 0.0

        self.get_logger().info(
            f"Serial telemetry: {t_pub} pub, {t_drop} drop, {t_fail} fail. "
            f"Age max: state={s_max:.3f}s, env={e_max:.3f}s. "
            f"CMD drop: {drop}. "
            f"TX: {tx_f} fail, {tx_p} partial. Recon: {recon}"
        )

    def destroy_node(self):
        """Clean up resources, cancel timers, and safely terminate worker threads."""
        if getattr(self, 'node_destroyed', False):
            return True

        import time
        with getattr(self, 'state_lock', __import__('threading').Lock()):
            if not getattr(self, '_shutdown_started', False):
                self._shutdown_started = True
                self._shutdown_deadline_mono = time.monotonic() + 3.5
                self.stop_request = True
                self.running = False
                self.session_ready = False
                self.telemetry_healthy = False
                self.session_epoch += 1
                self.pending_cmd = None
                self.pending_pump = None
                self.pending_fire = None
                self.latest_state = None
                self.latest_env = None
                self.session_started_mono = 0.0
                self.last_state_time_mono = 0.0

                if getattr(self, 'telemetry_timer', None):
                    try:
                        self.telemetry_timer.cancel()
                    except Exception:
                        pass
                if getattr(self, 'log_timer', None):
                    try:
                        self.log_timer.cancel()
                    except Exception:
                        pass

        deadline = getattr(self, '_shutdown_deadline_mono', time.monotonic())

        if getattr(self, 'worker_thread', None):
            if self.ser and hasattr(self.ser, 'cancel_read'):
                try:
                    self.ser.cancel_read()
                except Exception:
                    pass
            if self.ser and hasattr(self.ser, 'cancel_write'):
                try:
                    self.ser.cancel_write()
                except Exception:
                    pass

            timeout = max(0.0, deadline - time.monotonic())
            self.worker_thread.join(timeout=timeout)
            if self.worker_thread.is_alive():
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (worker thread).")
                return False

        tl = getattr(self, 'tx_lock', None)
        if tl:
            timeout = max(0.0, deadline - time.monotonic())
            if not tl.acquire(timeout=timeout):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (tx_lock).")
                return False
            tl.release()

        pl = getattr(self, 'publish_lock', None)
        if pl:
            timeout = max(0.0, deadline - time.monotonic())
            if not pl.acquire(timeout=timeout):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (publish_lock).")
                return False
            pl.release()

        ret = super().destroy_node()
        self.node_destroyed = True
        return True if ret is None else ret


def main(args=None):
    """Run main entry point."""
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        import time as tm
        deadline = tm.monotonic() + 3.7
        while True:
            remaining = deadline - tm.monotonic()
            if remaining <= 0:
                break
            if node.destroy_node():
                break
            tm.sleep(min(0.1, remaining))

        # Final nonblocking cleanup check within absolute deadline
        if not getattr(node, 'node_destroyed', False):
            # Force non-blocking check by setting deadline to now
            node._shutdown_deadline_mono = tm.monotonic()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
