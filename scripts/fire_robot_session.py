#!/usr/bin/env python3
"""
Laptop entry point for a supervised mapping or fire-demo session.

This file only orchestrates existing ROS launches. All motor and pump commands
still pass through their independent owner, proof, and watchdog nodes.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time

import yaml

from fire_robot_pi_session import PiSession, install_subreaper, cleanup_descendants


ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / 'ros2_ws/src/fire_robot_navigation/maps'
SELECTED_MAP = ROOT / 'scripts/selected_map.txt'
PI_HOST = os.environ.get('FIRE_PI_SSH_HOST', 'pi@10.42.0.185')
PI_WORKSPACE = 'NCHK_robotFirefighting_ROS/ros2_ws'
ARM_TOPICS = (
    '/fire_mission/live_nav_arm',
    '/fire_mission/nav_cancel_arm',
    '/fire_mission/alignment_arm_shadow',
    '/fire_mission/live_alignment_arm',
    '/fire_mission/ir_approach_arm',
)
STATE_TOPICS = (
    '/fire_mission/live_nav_stop_state',
    '/fire_mission/nav2_cancel_guard_state',
    '/fire_mission/state',
    '/fire_mission/live_alignment_state',
    '/fire_mission/ir_range_preview',
    '/fire_mission/ir_approach_live_state',
    '/fire_mission/pump_demo_state',
    '/fire_mission/post_spray_review',
    '/fire_sensors/actuator_state',
)
EXCLUSIVE_NODES = (
    '/slam_toolbox', '/bt_navigator', '/rviz', '/yolo_observer',
    '/fire_mission_supervisor', '/motion_arbiter_shadow',
    '/nav2_cancel_guard', '/live_nav_stop_gateway',
    '/live_alignment_takeover', '/ir_range_preview',
    '/ir_approach_live', '/pump_demo_controller',
    '/post_spray_review', '/fire_annotated_viewer',
)


class SessionError(RuntimeError):
    """Represent a failed preflight, launch or proof check."""

    pass


def run(args, *, timeout=8, check=True):
    """Run one short, non-interactive command with bounded wait time."""
    try:
        completed = subprocess.run(
            args, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        raise SessionError('Quá hạn: ' + shlex.join(args)) from exc
    if check and completed.returncode:
        raise SessionError(
            f'Lệnh lỗi ({completed.returncode}): {shlex.join(args)}\n'
            + completed.stdout[-1200:])
    return completed


def nodes():
    """Read the current ROS graph without the cached daemon."""
    output = run(['ros2', 'node', 'list', '--no-daemon',
                  '--spin-time', '1.0'], timeout=7).stdout
    return output.splitlines()


def nodes_present(*required):
    """Evaluate one coherent ROS graph snapshot for a readiness check."""
    current = set(nodes())
    return all(name in current for name in required)


def lifecycle_active(node):
    """Accept only ROS lifecycle active (state 3), never inactive (state 2)."""
    output = run(['ros2', 'lifecycle', 'get', node], timeout=7).stdout.strip()
    return re.fullmatch(r'active\s+\[3\]', output) is not None


def publisher_count(topic):
    """Count all publishers of a topic, treating an unknown topic as zero."""
    result = run(['ros2', 'topic', 'info', topic],
                 timeout=6, check=False)
    output = result.stdout.strip()
    if result.returncode and output == f"Unknown topic '{topic}'":
        return 0
    if result.returncode:
        raise SessionError(
            f'Không kiểm tra được publisher của {topic} '
            f'(mã {result.returncode}): {output}')
    match = re.search(r'Publisher count:\s*(\d+)', output)
    if match is None:
        raise SessionError(f'Không đọc được publisher count: {topic}\n{output}')
    return int(match.group(1))


def no_fire_goal_ready(gateway):
    """Require clear proof before reporting goal 2 as ready."""
    return (isinstance(gateway, dict) and
            gateway.get('state') == 'RELEASED_STOPPED' and
            gateway.get('stop_reason') == 'NAV_GOAL_TERMINAL_NO_FIRE' and
            gateway.get('next_goal_clear_ready') is True and
            bool(gateway.get('cycle_goal_uuid')))


def state_event_details(payload):
    """Keep stop evidence in session logs; gateway UUID lives in cycle_goal_uuid."""
    details = {
        'state': payload.get('state') if 'state' in payload else payload.get('zone'),
        'goal_uuid': payload.get('cycle_goal_uuid') or payload.get('goal_uuid'),
        'stop_reason': payload.get('stop_reason'),
    }
    for key in ('blockers', 'rearm_blockers', 'first_stop'):
        if key in payload:
            details[key] = payload[key]
    health = payload.get('health')
    if isinstance(health, dict):
        details['health_status'] = health.get('status')
        details['health_blockers'] = health.get('blockers')
    return details


def ir_demo_event_details(uuid, detail, gateway, guard, ir):
    """Diagnostic fallback only; never substitute UUIDs for cancel/arm decisions."""
    return {
        'goal_uuid': uuid or gateway.get('cycle_goal_uuid'),
        'detail': detail,
        'gateway': state_event_details(gateway),
        'guard': state_event_details(guard),
        'ir': state_event_details(ir),
    }


class TelemetryMonitor:
    """Cache fresh ROS state continuously so short transitions are not missed."""

    def __init__(self, on_change=None):
        """Subscribe before actuation and spin a private read-only node."""
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.rclpy.init()
        self.node = self.rclpy.create_node('fire_demo_session_monitor')
        self.lock = threading.Lock()
        self.cache = {}
        self.on_change = on_change
        self.stop_event = threading.Event()
        for topic in STATE_TOPICS:
            self.node.create_subscription(
                String, topic,
                lambda msg, name=topic: self._on_message(name, msg),
                qos_profile_sensor_data)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _on_message(self, topic, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            with self.lock:
                previous = self.cache.get(topic)
                self.cache[topic] = (payload, time.monotonic())
            field = 'state' if 'state' in payload else 'zone'
            if self.on_change is not None and payload.get(field) != (
                    previous[0].get(field) if previous else None):
                self.on_change('STATE_CHANGE', topic=topic,
                               **state_event_details(payload))

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            while not self.stop_event.is_set() and self.rclpy.ok():
                self.rclpy.spin_once(self.node, timeout_sec=0.05)
        except ExternalShutdownException:
            # SIGINT can shut down the context before Session.close joins us.
            self.stop_event.set()

    def latest(self, topic, max_age=0.6):
        """Return only fresh JSON from the exact requested topic."""
        with self.lock:
            item = self.cache.get(topic)
        if item is None or time.monotonic() - item[1] > max_age:
            return None
        return item[0]

    def close(self):
        """Stop subscriptions after the owned launch processes have stopped."""
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def wait_until(label, predicate, session=None, timeout=50):
    """Wait for a launch readiness condition while watching child health."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        if session is not None:
            session.assert_alive()
        try:
            if predicate():
                print(f'[OK] {label}', flush=True)
                return
            last_error = None
        except SessionError as exc:
            last_error = str(exc)
        time.sleep(0.6)
    detail = f'; kiểm tra gần nhất: {last_error}' if last_error else ''
    raise SessionError(f'Không sẵn sàng sau {timeout}s: {label}{detail}')


def startup_state_gaps(monitor, align_only, ir_only):
    """Use fresh reports from each owned node, not multiple graph snapshots."""
    required = {
        '/fire_mission/live_nav_stop_state': 'WAITING_FOR_INPUT',
        '/fire_mission/nav2_cancel_guard_state': 'WAITING_FOR_ADMISSION',
        '/fire_mission/live_alignment_state': 'WAITING_FOR_ADMISSION',
    }
    if not align_only:
        required['/fire_mission/ir_approach_live_state'] = \
            'WAITING_FOR_ADMISSION'
        if not ir_only:
            required['/fire_mission/post_spray_review'] = \
                'WAITING_FOR_PULSE'
    gaps = []
    for topic, expected_state in required.items():
        report = monitor.latest(topic)
        if not isinstance(report, dict):
            gaps.append(f'{topic}: thiếu state mới (<=0,6s)')
        elif report.get('schema_version') != 1:
            gaps.append(f'{topic}: schema_version khác 1')
        elif report.get('state') != expected_state:
            gaps.append(f'{topic}: {report.get("state")!r}, '
                        f'cần {expected_state}')
        elif topic != '/fire_mission/post_spray_review' and \
                report.get('enabled') is not True:
            gaps.append(f'{topic}: chưa enabled')
    return gaps


def wait_for_startup_states(session, align_only, ir_only, timeout=50):
    """Require fresh initial telemetry and child liveness before preflight."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        session.assert_alive()
        gaps = startup_state_gaps(session.monitor, align_only, ir_only)
        if not gaps:
            print('[OK] cổng dừng/hủy/căn đã phát state ban đầu',
                  flush=True)
            return
        time.sleep(0.6)
    gaps = startup_state_gaps(session.monitor, align_only, ir_only)
    raise SessionError(
        f'Không sẵn sàng sau {timeout}s: cổng dừng/hủy/căn; '
        + '; '.join(gaps or ['state thay đổi trong lúc kiểm tra']))


class StableSafeWindow:
    """A transient IDLE sample never grants an operator arm."""

    def __init__(self, hold_s=1.0):
        self.hold_s = float(hold_s)
        self.since = None

    def observe(self, safe, now):
        if not safe:
            self.since = None
            return False
        if self.since is None:
            self.since = float(now)
        return float(now) - self.since >= self.hold_s


def safe_initial_state(gateway, guard, mission, raw_publishers,
                       pump_publishers):
    """Check a fresh snapshot; the caller must also require a stable window."""
    return (gateway.get('state') == 'WAITING_FOR_INPUT' and
            gateway.get('fire_latched') is False and
            gateway.get('cmd_vel_fresh_zero') is True and
            gateway.get('odom_stationary') is True and
            guard.get('state') == 'WAITING_FOR_ADMISSION' and
            mission.get('state') == 'IDLE' and
            (mission.get('health') or {}).get('status') == 'OK' and
            not raw_publishers and not pump_publishers)


def startup_preflight_stable(monitor, window, now=None):
    """Read fresh safety state after graph checks; reset on query failure."""
    topics = ('/fire_mission/live_nav_stop_state',
              '/fire_mission/nav2_cancel_guard_state',
              '/fire_mission/state')
    before = tuple(monitor.latest(topic) or {} for topic in topics)
    if not safe_initial_state(*before, 0, 0):
        return window.observe(False, time.monotonic() if now is None else now)
    try:
        raw_publishers = publisher_count('/cmd_vel_raw')
        pump_publishers = publisher_count('/pump_cmd')
    except SessionError:
        window.observe(False, time.monotonic())
        raise
    after = tuple(monitor.latest(topic) or {} for topic in topics)
    safe = safe_initial_state(*after, raw_publishers, pump_publishers)
    return window.observe(safe, time.monotonic() if now is None else now)


def pre_spray_clear_waiting(mission, gateway, handled_goals):
    """Recognize missing fire on the canceled goal without declaring it out."""
    uuid = gateway.get('cycle_goal_uuid')
    perception = mission.get('perception') or {}
    return (isinstance(uuid, str) and len(uuid) == 32 and
            uuid not in handled_goals and
            mission.get('state') == 'TARGET_LOST' and
            perception.get('detected_now') is False and
            perception.get('confirmed') is False and
            gateway.get('fire_latched') is True and
            gateway.get('stop_reason') == 'FIRE_ACQUIRED')


def target_recovery_event(waiting_uuid, mission, gateway, handled_goals):
    """Report target loss/reacquisition; neither event withdraws an arm."""
    uuid = gateway.get('cycle_goal_uuid')
    if pre_spray_clear_waiting(mission, gateway, handled_goals):
        return uuid, ('TARGET_REACQUIRE_WAITING'
                      if waiting_uuid != uuid else None)
    if waiting_uuid != uuid:
        return None, None
    perception = mission.get('perception') or {}
    if (mission.get('state') in (
            'FIRE_TRACKING', 'CENTERED_HOLD', 'ALIGNMENT_READY') and
            perception.get('detected_now') is True and
            perception.get('confirmed') is True):
        return None, 'TARGET_REACQUIRED'
    if ((mission.get('lifecycle') or {}).get(
            'pre_spray_clear_goal_uuid') == uuid and
            mission.get('state') == 'IDLE'):
        return None, None
    return waiting_uuid, None


def no_external_owners():
    """Reject any prior owner that would duplicate the planned session."""
    current = nodes()
    duplicates = [name for name in EXCLUSIVE_NODES if name in current]
    if duplicates:
        raise SessionError('Node cũ còn chạy, Ctrl-C đúng terminal cũ trước: '
                           + ', '.join(duplicates))
    for topic in ('/cmd_vel_raw', '/pump_cmd', *ARM_TOPICS,
                  '/fire_mission/pump_demo_arm'):
        if publisher_count(topic) != 0:
            raise SessionError(f'{topic} đã có publisher; không mở trùng')


def pi_ready():
    """Check the minimum live Pi bridge, watchdog and LiDAR nodes."""
    current = nodes()
    return all(name in current for name in (
        '/serial_bridge_node', '/safety_watchdog',
        '/scan_qos_relay', '/camsense_x1_node'))


def detached_shell_command(command, log_path):
    """Detach setup and launch together without retaining SSH output pipes."""
    return (f'nohup bash -c {shlex.quote(command)} '
            f'</dev/null >{shlex.quote(str(log_path))} 2>&1 &')


def ensure_pi():
    """Reuse an existing Pi stack or start exactly one over configured SSH."""
    if pi_ready():
        print('[OK] Pi robot.launch.py đã chạy; không mở trùng.', flush=True)
        return
    # LAN DDS discovery can transiently lose a Pi endpoint while its launch
    # is healthy. Allow it to reappear before deciding whether SSH is needed.
    try:
        wait_until('Pi DDS rediscovery', pi_ready, timeout=8)
        return
    except SessionError:
        pass
    if os.environ.get('FIRE_AUTO_START_PI', '1') != '1':
        raise SessionError('Pi chưa chạy; cần robot.launch.py trên Pi')
    print(f'[INFO] Kiểm tra Pi qua SSH {PI_HOST}...', flush=True)
    remote_env = (
        f'cd ~/{PI_WORKSPACE} && '
        'source /opt/ros/humble/setup.bash && '
        'source install/setup.bash && '
        'export ROS_DOMAIN_ID=0 && unset ROS_LOCALHOST_ONLY')
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', PI_HOST]
    remote_nodes = run(ssh + ['bash -lc ' + shlex.quote(
        remote_env + ' && ros2 node list --no-daemon --spin-time 1.0')],
        timeout=12).stdout.splitlines()
    if any(name in remote_nodes for name in (
            '/serial_bridge_node', '/safety_watchdog',
            '/scan_qos_relay', '/camsense_x1_node')):
        raise SessionError(
            'Pi đã chạy một phần nhưng laptop chưa thấy đủ qua ROS LAN. '
            'Kiểm tra mạng/DDS hoặc launch Pi cũ; không mở Pi lần hai.')
    remote_launch = (
        remote_env + ' && exec ros2 launch fire_robot_bringup '
        'robot.launch.py use_sim_time:=false use_camera:=true '
        'camera_backend:=native_mjpeg camera_fps:=15 '
        'camera_max_bytes_per_sec:=750000')
    remote_start = detached_shell_command(
        remote_launch, '/tmp/fire_robot_pi_auto.log')
    run(ssh + ['bash -lc ' + shlex.quote(remote_start)], timeout=12)
    print('[INFO] Đã yêu cầu Pi khởi động. Log: Pi:/tmp/fire_robot_pi_auto.log',
          flush=True)
    wait_until('Pi: bridge + watchdog + LiDAR', pi_ready, timeout=60)


def wait_sensor(topic, message_type, *, timeout=10):
    """Demand an actual sensor message rather than just a graph endpoint."""
    result = run([
        'ros2', 'topic', 'echo', topic, message_type, '--once',
        '--qos-reliability', 'best_effort',
    ], timeout=timeout, check=False)
    if result.returncode or not result.stdout.strip():
        raise SessionError(f'Không có dữ liệu thật trên {topic}')
    print(f'[OK] {topic} có dữ liệu', flush=True)


def pi_sensor_preflight(camera, ir=True):
    """Confirm that navigation inputs and optional camera are flowing."""
    sensors = [
            ('/scan', 'sensor_msgs/msg/LaserScan'),
            ('/odom', 'nav_msgs/msg/Odometry'),
            ('/imu/data', 'sensor_msgs/msg/Imu')]
    if ir:
        sensors.append(('/fire_sensors/ir_raw', 'std_msgs/msg/String'))
    for topic, msg in sensors:
        wait_sensor(topic, msg)
    if camera:
        wait_sensor('/image_raw/compressed',
                    'sensor_msgs/msg/CompressedImage')


class Session:
    """Own only subprocesses opened by this launcher and log events."""

    def __init__(self, manage_pi=False):
        """Create a unique log directory and empty process registry."""
        self.manage_pi = manage_pi
        self.pi = None
        if manage_pi:
            install_subreaper()
        self.log_dir = Path(tempfile.mkdtemp(prefix='fire_robot_session_'))
        self.processes = []
        self.arms = []
        self.pump = None
        self.optional = set()
        self.monitor = None
        self.events = self.log_dir / 'events.jsonl'
        self.event_lock = threading.Lock()
        print(f'[LOG] {self.log_dir}', flush=True)

    def event(self, kind, **details):
        """Append one dashboard-friendly event to the session JSONL log."""
        record = {'time_unix': time.time(), 'event': kind, **details}
        with self.event_lock:
            with self.events.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        print(f'[EVENT] {kind} {details}', flush=True)

    def start(self, label, cmd, *, arm=False, optional=False):
        """Start one process group and register it for scoped cleanup."""
        output = (self.log_dir / f'{label}.log').open('w', encoding='utf-8')
        try:
            process = subprocess.Popen(
                cmd, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True)
        finally:
            output.close()
        pair = (label, process)
        self.processes.append(pair)
        if arm:
            self.arms.append(pair)
        if optional:
            self.optional.add(label)
        self.event('PROCESS_STARTED', label=label, pid=process.pid)
        time.sleep(0.3)
        if not optional:
            self.assert_alive()
        return process

    def launch(self, label, package, launch_file, *args):
        """Start a ROS launch file under this session."""
        return self.start(label, ['ros2', 'launch', package, launch_file, *args])

    def assert_alive(self):
        """Fail closed when any mandatory launch or arm publisher exits."""
        if self.pi is not None:
            try:
                self.pi.assert_alive()
            except RuntimeError as exc:
                raise SessionError(str(exc)) from exc
        for label, process in self.processes:
            if label in self.optional:
                continue
            if process.poll() is not None:
                raise SessionError(
                    f'{label} thoát (code {process.returncode}); '
                    f'xem {self.log_dir / (label + ".log")}')

    def stop(self, process):
        """Gracefully interrupt an owned group, then terminate if needed."""
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print(f'[WARN] PID {process.pid} chưa thoát; kiểm tra thủ công',
                      flush=True)

    def remove(self, process):
        """Stop and unregister one owned process."""
        self.stop(process)
        if process.poll() is None:
            raise SessionError(
                f'PID {process.pid} chưa thoát; giữ quyền quản lý tiến trình')
        self.processes = [pair for pair in self.processes
                          if pair[1] is not process]
        self.arms = [pair for pair in self.arms if pair[1] is not process]

    def disarm(self):
        """Withdraw every owned heartbeat while leaving read-only UI open."""
        for _, process in reversed(tuple(self.arms)):
            self.remove(process)
        self.event('ARMS_WITHDRAWN')

    def start_pi(self):
        """Sync before launch; register ownership before any remote operation."""
        self.pi = PiSession(PI_HOST, self.log_dir, self.event)
        try:
            self.pi.start()
        except RuntimeError as exc:
            raise SessionError(str(exc)) from exc
        wait_until('Pi: bridge + watchdog + LiDAR', pi_ready,
                   session=self, timeout=60)

    def close(self):
        """Withdraw arms, stop laptop, then stop the owned Pi stack."""
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        errors = []
        try:
            print('[STOP] Đang dừng các launch của phiên; chờ xác nhận cleanup.', flush=True)
            # Pi watchdog remains available while laptop publishers withdraw.
            for process in ([p for _, p in reversed(self.arms)] +
                            ([self.pump] if self.pump is not None else []) +
                            [p for _, p in reversed(self.processes)]):
                try:
                    self.stop(process)
                except (OSError, subprocess.TimeoutExpired) as exc:
                    errors.append(str(exc))
            if self.manage_pi:
                ssh = self.pi.process if self.pi else None
                remaining = cleanup_descendants(exclude_roots=(ssh.pid,) if ssh else ())
                if remaining:
                    errors.append(f'Laptop còn PID {remaining}')
            if self.monitor is not None:
                try:
                    self.monitor.close()
                except Exception as exc:
                    errors.append('Monitor close: '+str(exc))
        finally:
            try:
                if self.pi is not None and not self.pi.close():
                    errors.append('Chưa xác nhận cleanup Pi; xem log Pi/SSH')
                if self.manage_pi:
                    remaining = cleanup_descendants()
                    if remaining:
                        errors.append(f'Laptop còn PID {remaining}')
                self.event('SESSION_CLEANUP_INCOMPLETE' if errors else 'SESSION_STOPPED',
                           errors=errors)
            finally:
                signal.signal(signal.SIGINT, previous_sigint)
        if errors:
            raise SessionError('; '.join(errors))


def arm_once_false(topic):
    """Provide the explicit false edge required before an IR or pump arm."""
    run([
        'ros2', 'topic', 'pub', '--rate', '5', '--times', '3',
        '--wait-matching-subscriptions', '1',
        '--qos-reliability', 'reliable', '--qos-durability', 'volatile',
        topic, 'std_msgs/msg/Bool', '{data: false}',
    ], timeout=12)


def arm_true(session, topic, label):
    """Keep exactly one volatile arm heartbeat alive in this session."""
    session.start(label, [
        'ros2', 'topic', 'pub', '--rate', '5', '--print', '1000',
        '--qos-reliability', 'reliable', '--qos-durability', 'volatile',
        topic, 'std_msgs/msg/Bool', '{data: true}',
    ], arm=True)
    wait_until(label + ' publisher', lambda: publisher_count(topic) == 1,
               session, timeout=12)


def ir_arm_stage_ready(report, *, armed):
    """Require the IR subscriber's fresh receipt, not only a ROS publisher."""
    if not isinstance(report, dict) or report.get('schema_version') != 1:
        return False
    arm = report.get('arm')
    return (isinstance(arm, dict) and
            arm.get('false_seen') is True and
            arm.get('cycle_seen') is armed and
            arm.get('heartbeat_true') is armed and
            arm.get('receive_fresh') is True and
            arm.get('publisher_count') == 1)


def ir_arm_missing_after_handoff(ir, guard, gateway, alignment):
    """Detect a persistent lost arm only after the canceled goal is safe."""
    uuid = guard.get('goal_uuid')
    return (isinstance(uuid, str) and
            re.fullmatch(r'[0-9a-fA-F]{32}', uuid) is not None and
            ir.get('state') == 'WAITING_FOR_ADMISSION' and
            'OPERATOR_ARM_MISSING_OR_STALE' in (ir.get('blockers') or []) and
            guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
            guard.get('blockers') == [] and
            gateway.get('state') == 'HANDOFF_READY' and
            gateway.get('cycle_goal_uuid') == uuid and
            alignment.get('state') == 'COMPLETE' and
            alignment.get('goal_uuid') == uuid and
            alignment.get('stop_reason') == 'TARGET_CENTERED')


def arm_ir_with_ack(session):
    """Hold each IR arm edge until the live node acknowledges receiving it."""
    topic = '/fire_mission/ir_approach_arm'
    false_publisher = session.start('ir_arm_false', [
        'ros2', 'topic', 'pub', '--rate', '5', '--print', '1000',
        '--qos-reliability', 'reliable', '--qos-durability', 'volatile',
        topic, 'std_msgs/msg/Bool', '{data: false}',
    ], arm=True)
    try:
        wait_until('IR arm false đã tới node',
                   lambda: ir_arm_stage_ready(session.monitor.latest(
                       '/fire_mission/ir_approach_live_state'), armed=False),
                   session, timeout=8)
    finally:
        session.remove(false_publisher)
    wait_until('IR arm false publisher đã nhả',
               lambda: publisher_count(topic) == 0, session, timeout=8)
    arm_true(session, topic, 'arm_4')
    wait_until('IR arm true đã tới node',
               lambda: ir_arm_stage_ready(session.monitor.latest(
                   '/fire_mission/ir_approach_live_state'), armed=True),
               session, timeout=8)
    session.event('IR_ARM_ACKNOWLEDGED')


def arm_pump_with_ack(session):
    """Require the pump subscriber to acknowledge both arm edges."""
    topic = '/fire_mission/pump_demo_arm'
    state_topic = '/fire_mission/pump_demo_state'
    false_publisher = session.start('pump_arm_false', [
        'ros2', 'topic', 'pub', '--rate', '5', '--print', '1000',
        '--qos-reliability', 'reliable', '--qos-durability', 'volatile',
        topic, 'std_msgs/msg/Bool', '{data: false}',
    ], arm=True)
    try:
        wait_until('pump arm false đã tới node',
                   lambda: ir_arm_stage_ready(
                       session.monitor.latest(state_topic), armed=False),
                   session, timeout=8)
    finally:
        session.remove(false_publisher)
    wait_until('pump arm false publisher đã nhả',
               lambda: publisher_count(topic) == 0, session, timeout=8)
    arm_true(session, topic, 'pump_arm')
    wait_until('pump arm true đã tới node',
               lambda: ir_arm_stage_ready(
                   session.monitor.latest(state_topic), armed=True),
               session, timeout=8)
    session.event('PUMP_ARM_ACKNOWLEDGED')


def save_map(name):
    """Save a unique PGM/YAML pair from the active SLAM map."""
    if '/slam_toolbox' not in nodes():
        raise SessionError('SLAM chưa chạy; mở start_mapping.sh trước')
    if '/bt_navigator' in nodes():
        raise SessionError('Nav2 đang chạy; không lưu nhầm map của Nav2')
    if name is None:
        name = time.strftime('fire_room_%Y%m%d_%H%M%S')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
        raise SessionError('Tên map chỉ dùng chữ, số, _ và -')
    prefix = MAPS / name
    if prefix.with_suffix('.yaml').exists() or \
            prefix.with_suffix('.pgm').exists():
        raise SessionError(f'Map đã tồn tại, không ghi đè: {prefix}')
    run([
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', str(prefix), '--ros-args',
        '-p', 'save_map_timeout:=10.0',
        '-p', 'map_subscribe_transient_local:=true',
    ], timeout=25)
    if not prefix.with_suffix('.yaml').is_file() or \
            not prefix.with_suffix('.pgm').is_file():
        raise SessionError('map_saver_cli trả về nhưng thiếu .yaml hoặc .pgm')
    SELECTED_MAP.write_text(name + '.yaml\n', encoding='utf-8')
    print(f'[SAVED] {prefix.with_suffix(".yaml")}', flush=True)
    print('Sau khi Ctrl-C phiên quét, chạy:', flush=True)
    print('  ./scripts/start_demo.sh', flush=True)


def mapping(args):
    """Launch SLAM and hand keyboard control to the local operator."""
    if args.action == 'save':
        save_map(args.name)
        return
    no_external_owners()
    ensure_pi()
    pi_sensor_preflight(camera=False)
    session = Session()
    try:
        session.launch('slam', 'fire_robot_navigation', 'slam.launch.py',
                       'use_sim_time:=false', 'rviz:=true')
        wait_until('SLAM node và map publisher',
                   lambda: '/slam_toolbox' in nodes() and
                   publisher_count('/map') == 1,
                   session, timeout=60)
        print('\n[READY] Dùng phím teleop trong terminal này để quét map.',
              flush=True)
        print('Tốc độ ban đầu: 0,08 m/s, xoay 0,5 rad/s; '
              'đừng dùng phím tăng tốc khi chưa kiểm tra khoảng trống.',
              flush=True)
        print('Mở terminal khác để lưu (không Ctrl-C SLAM trước):', flush=True)
        print('  ./scripts/start_mapping.sh save [ten_map]', flush=True)
        print('Ctrl-C tại đây để tắt teleop + SLAM sau khi lưu.', flush=True)
        # Keep keyboard in this foreground terminal. A held key is not a
        # heartbeat; the independent Pi watchdog handles command timeout.
        command = [
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '-r', 'cmd_vel:=/cmd_vel_raw',
            '-p', 'speed:=0.08', '-p', 'turn:=0.5',
        ]
        if publisher_count('/cmd_vel_raw') != 0:
            raise SessionError('/cmd_vel_raw bị chiếm trước khi mở teleop')
        result = subprocess.run(command, check=False)
        if result.returncode:
            raise SessionError(f'Teleop thoát với mã {result.returncode}')
    finally:
        session.close()


def selected_map():
    """Read the explicitly selected map name without allowing path escape."""
    name = SELECTED_MAP.read_text(encoding='utf-8').strip()
    if (not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\.yaml', name) or
            Path(name).name != name):
        raise SessionError('selected_map.txt sai định dạng; cần tên .yaml')
    return MAPS / name


def validate_map(path):
    """Require an existing YAML map and its referenced image."""
    chosen = Path(path).expanduser().resolve(strict=True)
    if not chosen.is_file() or chosen.suffix != '.yaml':
        raise SessionError('Cần file map .yaml, không phải .pgm')
    with chosen.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise SessionError('Map YAML thiếu image')
    image = Path(data['image'])
    if not image.is_absolute():
        image = chosen.parent / image
    if not image.is_file():
        raise SessionError(f'Ảnh map không tồn tại: {image}')
    return chosen


def pump_admission(ir, guard, gateway, alignment, mission, preview):
    """Script-level sequencing; PumpDemoCore independently rechecks safety."""
    uuid = guard.get('goal_uuid')
    ir_ready = (
        ir.get('goal_uuid') == uuid and
        ir.get('state') == 'COMPLETE' and
        (ir.get('stop_reason') == 'SPRAY_RANGE_REACHED' if
         ir.get('continuous_approach') else
         ir.get('stop_reason') in (
             'SPRAY_RANGE_REACHED', 'STEP_DISTANCE_REACHED',
             'STEP_PROGRESS_TIME_LIMIT')) and
        ir.get('step_count', 0) >= 1 and
        ir.get('publisher_owned') is False)
    # If already in range after centering, no IR motion is needed.
    direct_ready = (
        ir.get('state') == 'WAITING_FOR_ADMISSION' and
        ir.get('publisher_owned') is False and
        ir.get('step_count') == 0)
    return bool(
        uuid and re.fullmatch(r'[0-9a-fA-F]{32}', uuid) and
        (ir_ready or direct_ready) and
        preview.get('zone') == 'SPRAY_RANGE' and
        preview.get('ir_band') == 'SPRAY_RANGE' and
        preview.get('blockers') == [] and
        alignment.get('state') == 'COMPLETE' and
        alignment.get('stop_reason') == 'TARGET_CENTERED' and
        alignment.get('goal_uuid') == uuid and
        alignment.get('publisher_owned') is False and
        guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
        guard.get('goal_status') == 5 and
        guard.get('raw_cmd_publishers') == 0 and
        guard.get('blockers') == [] and
        gateway.get('state') == 'HANDOFF_READY' and
        gateway.get('cycle_goal_uuid') == uuid and
        gateway.get('handoff_ready') is True and
        gateway.get('cmd_vel_fresh_zero') is True and
        gateway.get('odom_stationary') is True and
        mission.get('state') in ('ALIGNMENT_READY', 'CENTERED_HOLD') and
        (mission.get('perception') or {}).get('alignment_state') == 'CENTERED' and
        (mission.get('robot') or {}).get('stationary') is True and
        publisher_count('/cmd_vel_raw') == 0 and
        publisher_count('/pump_cmd') == 0)


def wait_actuator_off(session):
    """Wait for fresh ESP32 proof that pump and servo have stopped."""
    def off():
        state = session.monitor.latest('/fire_sensors/actuator_state') or {}
        return (state.get('pump_pwm_commanded_on') is False and
                state.get('servo_phase') == 'IDLE' and
                state.get('servo_pwm_commanded_on') is False and
                state.get('receive_age_ms', 9999) < 350)
    wait_until('ESP32: bơm OFF và servo về home', off, session, timeout=15)


def reviewable_pump_result(state, saw_firing, uuid):
    """Only a completed pulse or an expected sensor stop may reach review."""
    if not saw_firing or state.get('goal_uuid') != uuid or \
            state.get('pump_on') is not False:
        return False
    return (state.get('state') == 'COMPLETE' and
            state.get('stop_reason') == 'PULSE_TIME_LIMIT') or (
                state.get('state') == 'ABORTED' and
                state.get('stop_reason') in (
                    'IR_NOT_CONFIRMED_SPRAY_RANGE',
                    'MISSION_NOT_SAFE_AND_CENTERED'))


def pump_pulse(session, uuid, *, respray=False):
    """Open one bounded pump owner after the IR-to-pump handoff."""
    if publisher_count('/cmd_vel_raw') or publisher_count('/pump_cmd'):
        raise SessionError('Không thể bắt đầu bơm khi còn motion/pump owner')
    session.pump = session.launch(
        'pump', 'fire_robot_mission', 'pump_demo.launch.py',
        'enable_pump_demo:=true', 'allow_provisional_ir:=true', 'pulse_s:=5.0')
    wait_until('pump_demo_controller',
               lambda: '/pump_demo_controller' in nodes() and
               publisher_count('/pump_cmd') == 1,
               session, timeout=20)
    if respray:
        def review_still_ready():
            review = session.monitor.latest('/fire_mission/post_spray_review') or {}
            return (review.get('goal_uuid') == uuid and
                    review.get('state') == 'RESPRAY_READY_SHADOW' and
                    review.get('blockers') == [])
        wait_until('hậu kiểm vẫn cho phép phun lại',
                   review_still_ready, session, timeout=15)
    pump_arm = None
    try:
        arm_pump_with_ack(session)
        pump_arm = session.arms[-1][1]
        deadline = time.monotonic() + 20
        saw_firing = False
        while time.monotonic() < deadline:
            session.assert_alive()
            state = session.monitor.latest('/fire_mission/pump_demo_state') or {}
            if state.get('goal_uuid') == uuid and state.get('state') == 'FIRING':
                saw_firing = True
            if state.get('goal_uuid') == uuid and state.get('state') in (
                    'COMPLETE', 'ABORTED'):
                early_reason = state.get('stop_reason')
                early_reviewable = (state.get('state') == 'ABORTED' and
                                    early_reason in (
                                        'IR_NOT_CONFIRMED_SPRAY_RANGE',
                                        'MISSION_NOT_SAFE_AND_CENTERED') and
                                    state.get('pump_on') is False)
                if not reviewable_pump_result(state, saw_firing, uuid):
                    raise SessionError(f'Bơm dừng lỗi: {state}')
                session.event(
                    'PUMP_PULSE_EARLY_STOP' if early_reviewable else
                    'PUMP_PULSE_COMPLETE', goal_uuid=uuid,
                    stop_reason=early_reason)
                return
            time.sleep(0.4)
        raise SessionError('Bơm không hoàn tất trong 20s; xem pump.log')
    finally:
        if pump_arm is not None:
            session.remove(pump_arm)
        # Keep the pump controller alive, publishing OFF, until post-spray
        # review and ESP32 home confirmation. Session cleanup stops it on error.


def review_pulse(session, uuid):
    """Wait for a stable post-spray assessment of this goal."""
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        session.assert_alive()
        review = session.monitor.latest('/fire_mission/post_spray_review') or {}
        if review.get('goal_uuid') == uuid and review.get('blockers') == []:
            phase = review.get('state')
            if phase in ('NO_FIRE_EVIDENCE', 'RESPRAY_READY_SHADOW',
                         'REALIGN_REQUIRED', 'APPROACH_REQUIRED',
                         'RETRY_LIMIT_REACHED'):
                return review
        time.sleep(0.5)
    raise SessionError('Hậu kiểm bơm chưa xác nhận trong 25s; giữ xe dừng')


def alignment_demo_outcome(alignment, guard, gateway, raw_publishers):
    """Classify a finished alignment without terminating the UI session."""
    if gateway.get('state') == 'RELEASED_STOPPED' and \
            gateway.get('stop_reason') not in ('', 'FIRE_ACQUIRED'):
        return 'FAILED', 'Gateway: ' + str(gateway.get('stop_reason'))
    if guard.get('state') == 'FAILED' and guard.get('goal_uuid'):
        return 'FAILED', 'Cancel guard: ' + str(guard.get('blockers'))
    uuid = guard.get('goal_uuid')
    if alignment.get('state') != 'COMPLETE' or not uuid or \
            alignment.get('goal_uuid') != uuid:
        return None, None
    if alignment.get('stop_reason') != 'TARGET_CENTERED':
        return 'FAILED', 'Alignment: ' + str(alignment.get('stop_reason'))
    if guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and \
            gateway.get('state') == 'HANDOFF_READY' and \
            gateway.get('cycle_goal_uuid') == uuid and \
            raw_publishers == 0:
        return 'COMPLETE', uuid
    return None, None


def ir_demo_outcome(ir, guard, gateway, preview, raw_publishers):
    """Classify a pump-free IR trial only after the raw owner is released."""
    uuid = guard.get('goal_uuid')
    if ir.get('state') == 'ABORTED' and ir.get('goal_uuid') == uuid:
        return 'FAILED', str(ir.get('stop_reason'))
    if ir.get('state') != 'COMPLETE' or not uuid or \
            ir.get('goal_uuid') != uuid:
        return None, None
    if ir.get('stop_reason') in (
            'REALIGN_REQUIRED', 'APPROACH_DISTANCE_LIMIT'):
        return None, None
    if ir.get('stop_reason') != 'SPRAY_RANGE_REACHED':
        return 'FAILED', str(ir.get('stop_reason'))
    if (ir.get('publisher_owned') is False and raw_publishers == 0 and
            preview.get('zone') == 'SPRAY_RANGE' and
            preview.get('blockers') == [] and
            guard.get('state') == 'CANCEL_CONFIRMED_STOPPED' and
            gateway.get('state') == 'HANDOFF_READY' and
            gateway.get('cycle_goal_uuid') == uuid):
        return 'COMPLETE', uuid
    return None, None


def ir_failure_evidence(ir, preview, mission):
    """Preserve the cause of a stopped IR attempt for safe recovery work."""
    reason = ir.get('stop_reason')
    if reason in ('APPROACH_DISTANCE_LIMIT', 'TOTAL_TIME_LIMIT'):
        category = 'APPROACH_BUDGET'
    elif isinstance(reason, str) and reason.startswith('SCAN_'):
        category = 'FRONT_CLEARANCE_OR_SCAN'
    elif reason in ('IR_BAND_INVALID_OR_STALE', 'IR_BAND_HOLD',
                    'IR_DIRECTION_CHANGED', 'SPRAY_NOT_CONFIRMED'):
        category = 'IR_OR_TARGET_EVIDENCE'
    elif reason == 'EXACT_HANDOFF_PROOF_INVALID':
        category = 'TARGET_OR_HANDOFF_PROOF'
    elif reason in ('ODOM_MISSING_OR_INVALID', 'ODOM_PROGRESS_LOST',
                    'ODOM_MOTION_INVALID_OR_OVERSPEED',
                    'SAFETY_COMMAND_CONFLICT', 'CMD_VEL_NOT_ZERO'):
        category = 'MOTION_FEEDBACK_OR_SAFETY'
    else:
        category = 'OTHER_SAFETY_STOP'
    return {
        'goal_uuid': ir.get('goal_uuid'),
        'reason': reason,
        'category': category,
        'stop_trigger_blockers': ir.get('stop_trigger_blockers', []),
        'ir_band': ir.get('ir_band'),
        'preview_zone': preview.get('zone'),
        'preview_blockers': preview.get('blockers'),
        'mission_state': mission.get('state'),
        'perception': mission.get('perception'),
        'recovery': 'STOPPED_OPERATOR_REVIEW_REQUIRED',
    }


def ir_stop_proof_overdue(state, stopping_since, now, timeout_s=4.0):
    """Escalate a zero-command stop with no stationary/release proof."""
    return (state == 'STOPPING' and stopping_since is not None and
            now - stopping_since >= timeout_s)


def ir_telemetry_overdue(ir, last_seen_at, now, timeout_s=1.0):
    """Reject a stale monitor feed before further motion or pumping."""
    return ir is None and now - last_seen_at >= timeout_s


def demo(args):
    """Start a supervised alignment-only or full live demo session."""
    if not args.live:
        raise SessionError('Demo chuyển động/phun cần --live rõ ràng')
    map_path = validate_map(args.map or selected_map())
    no_external_owners()
    session = Session(manage_pi=True)
    phase = 'pi_start'
    try:
        session.start_pi()
        phase = 'pi_sensor_preflight'
        pi_sensor_preflight(camera=True, ir=not args.align_only)
        session.monitor = TelemetryMonitor(on_change=session.event)
        phase = 'nav2_startup'
        session.launch('nav2', 'fire_robot_navigation',
                       'nav2_motion_shadow.launch.py',
                       'use_sim_time:=false', f'map:={map_path}',
                       'rviz:=true')
        print('\n[SETUP] Chờ bản đồ hiện trong RViz rồi đặt 2D Pose Estimate. '
              'Nav2 cần pose để hoàn tất khởi động; chưa đặt goal cho tới '
              'khi terminal báo [READY].', flush=True)
        wait_until('Nav2 active + map_server',
                   lambda: ('/bt_navigator' in nodes() and
                            lifecycle_active('/bt_navigator') and
                            lifecycle_active('/map_server')),
                   session, timeout=180)
        phase = 'perception_startup'
        session.launch('yolo', 'fire_robot_perception',
                       'perception.launch.py', 'use_sim_time:=false')
        wait_until('YOLO + alignment observer',
                   lambda: nodes_present('/yolo_observer',
                                         '/fire_alignment_observer') and
                   publisher_count('/yolo/detections') == 1,
                   session, timeout=150)
        if os.environ.get('DISPLAY'):
            session.start(
                'camera_view',
                ['/home/huudinh/.venvs/fire_robot_yolo/bin/python',
                 str(ROOT / 'scripts/view_yolo_camera.py')],
                optional=True)
        phase = 'mission_startup'
        session.launch('mission', 'fire_robot_mission',
                       'mission_motion_arbiter_shadow.launch.py',
                       'require_live_cancel_proof:=true')
        wait_until('mission + cancel intent',
                   lambda: nodes_present('/fire_mission_supervisor',
                                         '/nav2_cancel_intent_shadow',
                                         '/alignment_handoff_shadow'),
                   session)
        if not args.align_only:
            session.launch('ir_preview', 'fire_robot_mission',
                           'ir_range_preview.launch.py')
            wait_until('IR preview',
                       lambda: '/ir_range_preview' in nodes() and
                       session.monitor.latest(
                           '/fire_mission/ir_range_preview') is not None,
                       session)
        session.launch('gateway', 'fire_robot_mission',
                       'mission_live_nav_stop.launch.py',
                       'enable_live_output:=true')
        session.launch('guard', 'fire_robot_mission',
                       'nav2_cancel_guard.launch.py',
                       'enable_real_cancel:=true')
        session.launch('alignment', 'fire_robot_mission',
                       'mission_live_alignment.launch.py',
                       'enable_live_alignment:=true')
        if not args.align_only:
            session.launch('ir_live', 'fire_robot_mission',
                           'ir_approach_live.launch.py',
                           'enable_live_ir_jog:=true',
                           'allow_provisional_ir:=true',
                           'integrated_handoff:=true',
                           'continuous_approach:=true',
                           'speed_m_s:=0.08',
                           'max_total_approach_m:=0.0',
                           'enable_reverse_jog:=false')
            if not args.ir_only:
                session.launch('review', 'fire_robot_mission',
                               'post_spray_review.launch.py')
        wait_for_startup_states(session, args.align_only, args.ir_only)
        wait_until('trạng thái ban đầu của gateway, guard, mission',
                   lambda: all(session.monitor.latest(topic) is not None
                               for topic in (
                                   '/fire_mission/live_nav_stop_state',
                                   '/fire_mission/nav2_cancel_guard_state',
                                   '/fire_mission/state')),
                   session, timeout=20)
        if not args.align_only:
            wait_until('trạng thái ban đầu của IR live',
                       lambda: session.monitor.latest(
                           '/fire_mission/ir_approach_live_state') is not None,
                       session, timeout=20)
        phase = 'startup_admission'
        safe_window = StableSafeWindow()

        try:
            wait_until('mission/gateway/guard an toàn ổn định trước khi arm',
                       lambda: startup_preflight_stable(
                           session.monitor, safe_window), session, timeout=50)
        except SessionError as exc:
            if not str(exc).startswith('Không sẵn sàng sau '):
                raise
            latest = session.monitor.latest
            gateway = latest('/fire_mission/live_nav_stop_state') or {}
            guard = latest('/fire_mission/nav2_cancel_guard_state') or {}
            mission = latest('/fire_mission/state') or {}
            health = mission.get('health') or {}
            detail = {
                'gateway': gateway.get('state'),
                'gateway_blockers': gateway.get('blockers'),
                'guard': guard.get('state'),
                'guard_blockers': guard.get('blockers'),
                'mission': mission.get('state'),
                'mission_health': health.get('status') if isinstance(
                    health, dict) else None,
                'mission_health_blockers': health.get('blockers') if
                isinstance(health, dict) else None,
            }
            raise SessionError(f'{exc}; state cuối: '
                               f'{json.dumps(detail, ensure_ascii=False)}') \
                from exc
        arm_topics = ARM_TOPICS[:4] if args.align_only else ARM_TOPICS
        for index, topic in enumerate(arm_topics):
            if topic == '/fire_mission/ir_approach_arm':
                arm_ir_with_ack(session)
            else:
                arm_true(session, topic, f'arm_{index}')
        session.event('READY_FOR_RVIZ_GOAL', map=str(map_path),
                      pump_auto=not (args.align_only or args.ir_only),
                      align_only=args.align_only)
        phase = 'live_demo'
        if args.align_only:
            print('\n[READY] Chỉ Nav2 → YOLO dừng/hủy → xoay căn tâm. '
                  'IR tiến và bơm KHÔNG được mở. Đặt 2D Pose Estimate nếu '
                  'AMCL chưa định vị, rồi đặt goal ngắn; chỉ đưa lửa vào '
                  'camera sau khi xe thật sự chạy. Ctrl-C để thu hồi arm.',
                  flush=True)
        elif args.ir_only:
            print('\n[READY] Nav2 → YOLO dừng/hủy → căn tâm → IR tiến '
                  '0,08 m/s theo các chặng tối đa 30 cm; YOLO có lửa nhưng '
                  'IR còn ở nền thì tiếp cận như vùng xa. '
                  'Lệch tâm thì dừng và căn lại. KHÔNG mở node bơm. '
                  'Đặt goal ngắn và chỉ đưa lửa vào khi xe đang chạy; '
                  'Ctrl-C để thu hồi arm.', flush=True)
        else:
            print('\n[READY] Nếu AMCL chưa định vị, đặt 2D Pose Estimate '
                  'trong RViz trước. Sau đó đặt goal ngắn khi khu vực an toàn. '
                  'Xe có thể tự chạy, xoay, tiến IR 0,08 m/s theo chặng '
                  '30 cm và phun khi đã dừng, IR xác nhận '
                  'vùng phun. '
                  'Ctrl-C ở terminal này để thu hồi arm.', flush=True)
        handled = set()
        normal_goals_announced = set()
        pre_spray_clears_announced = set()
        realign_events = set()
        alignment_completed_at = None
        alignment_terminal = False
        ir_terminal = False
        ir_completed_at = None
        ir_stopping_since = None
        ir_last_seen_at = time.monotonic()
        ir_arm_missing_since = None
        target_wait_uuid = None
        last_cycle_uuid = None
        while True:
            session.assert_alive()
            latest = session.monitor.latest
            guard = latest('/fire_mission/nav2_cancel_guard_state') or {}
            gateway = latest('/fire_mission/live_nav_stop_state') or {}
            alignment = latest('/fire_mission/live_alignment_state') or {}
            cycle_uuid = gateway.get('cycle_goal_uuid')
            if cycle_uuid is not None and cycle_uuid != last_cycle_uuid:
                last_cycle_uuid = cycle_uuid
                alignment_completed_at = None
                ir_completed_at = None
                ir_stopping_since = None
                ir_arm_missing_since = None
                ir_last_seen_at = time.monotonic()
            uuid = guard.get('goal_uuid')
            mission = latest('/fire_mission/state') or {}
            lifecycle = mission.get('lifecycle') or {}
            pre_spray_uuid = lifecycle.get('pre_spray_clear_goal_uuid')
            target_wait_uuid, recovery_event = target_recovery_event(
                target_wait_uuid, mission, gateway, handled)
            if recovery_event is not None:
                session.event(recovery_event,
                              goal_uuid=gateway.get('cycle_goal_uuid'),
                              alignment_state=alignment.get('state'))
            if (mission.get('state') == 'IDLE' and pre_spray_uuid and
                    pre_spray_uuid == gateway.get('cycle_goal_uuid')):
                if pre_spray_uuid not in pre_spray_clears_announced:
                    pre_spray_clears_announced.add(pre_spray_uuid)
                    session.event('PRE_SPRAY_FIRE_CLEARED',
                                  goal_uuid=pre_spray_uuid)
                    print('[READY] Lửa đã tắt trước khi phun; đang nhả '
                          'quyền chuyển động và bơm vẫn OFF. Chờ xe '
                          'đứng yên rồi có thể đặt goal mới.',
                          flush=True)
                time.sleep(0.3)
                continue
            if (no_fire_goal_ready(gateway) and
                    gateway['cycle_goal_uuid'] not in normal_goals_announced):
                normal_goals_announced.add(gateway['cycle_goal_uuid'])
                session.event('NAV_GOAL_NO_FIRE_CLEARED',
                              goal_uuid=gateway['cycle_goal_uuid'])
                print('[READY] Goal Nav2 đã kết thúc không có lửa; '
                      'đặt goal mới sau khi xe đứng yên và bơm OFF.',
                      flush=True)
            if args.align_only:
                if not alignment_terminal:
                    raw_publishers = (
                        publisher_count('/cmd_vel_raw') if
                        alignment.get('state') == 'COMPLETE' else None)
                    outcome, detail = alignment_demo_outcome(
                        alignment, guard, gateway, raw_publishers)
                    if outcome is None and \
                            alignment.get('state') == 'COMPLETE' and \
                            alignment.get('stop_reason') == 'TARGET_CENTERED':
                        if alignment_completed_at is None:
                            alignment_completed_at = time.monotonic()
                        elif time.monotonic() - alignment_completed_at > 5.0:
                            outcome, detail = ('FAILED',
                                               'Thiếu proof dừng sau căn tâm')
                    if outcome is not None:
                        alignment_terminal = True
                        session.disarm()
                        session.event('ALIGNMENT_DEMO_' + outcome,
                                      goal_uuid=uuid, detail=detail)
                        if outcome == 'COMPLETE':
                            print('[OK] Xe đã dừng và căn tâm. Không tiến IR/'
                                  'phun. Terminal vẫn mở tới khi b Ctrl-C.',
                                  flush=True)
                        else:
                            print('[WARN] Chưa căn tâm: ' + str(detail) +
                                  '. Đã thu hồi arm; terminal và RViz vẫn '
                                  'mở tới khi b Ctrl-C.', flush=True)
                time.sleep(0.3)
                continue
            ir_report = latest('/fire_mission/ir_approach_live_state')
            if ir_report is not None:
                ir_last_seen_at = time.monotonic()
            ir = ir_report or {}
            preview = latest('/fire_mission/ir_range_preview') or {}
            if (ir.get('state') == 'COMPLETE' and
                    ir.get('stop_reason') == 'REALIGN_REQUIRED' and
                    ir.get('publisher_owned') is False and
                    ir.get('goal_uuid') == uuid and uuid):
                key = (uuid, ir.get('step_count'))
                if key not in realign_events:
                    realign_events.add(key)
                    session.event(
                        'IR_REALIGN_HANDOFF', goal_uuid=uuid,
                        step_count=ir.get('step_count'),
                        total_progress_m=ir.get('total_progress_m'),
                        stop_trigger_blockers=ir.get(
                            'stop_trigger_blockers'))
            if not ir_terminal and uuid not in handled and \
                    ir_telemetry_overdue(ir_report, ir_last_seen_at,
                                         time.monotonic()):
                ir_terminal = True
                session.disarm()
                session.event('IR_TELEMETRY_LOST', goal_uuid=uuid,
                              recovery='STOPPED_OPERATOR_REVIEW_REQUIRED')
                print('[WARN] Mất telemetry IR live; đã thu hồi arm. '
                      'KHÔNG tiến/phun cho tới khi kiểm tra lại mạng và xe.',
                      flush=True)
            if ir.get('state') == 'STOPPING':
                if ir_stopping_since is None:
                    ir_stopping_since = time.monotonic()
            else:
                ir_stopping_since = None
            if not ir_terminal and ir_stop_proof_overdue(
                    ir.get('state'), ir_stopping_since,
                    time.monotonic()):
                ir_terminal = True
                session.disarm()
                session.event(
                    'IR_STOP_PROOF_TIMEOUT', goal_uuid=ir.get('goal_uuid'),
                    stop_reason=ir.get('stop_reason'),
                    raw_owner_reported=ir.get('publisher_owned'))
                print('[WARN] IR đã yêu cầu dừng nhưng chưa có proof đứng '
                      'yên/nhả quyền trong 4 s. Đã thu hồi arm; kiểm tra '
                      'xe và nguồn động cơ. KHÔNG phun.', flush=True)
            if not ir_terminal and uuid not in handled and \
                    ir_arm_missing_after_handoff(
                    ir, guard, gateway, alignment):
                if ir_arm_missing_since is None:
                    ir_arm_missing_since = time.monotonic()
                elif time.monotonic() - ir_arm_missing_since >= 2.0:
                    ir_terminal = True
                    session.disarm()
                    session.event('IR_ARM_LOST_AFTER_HANDOFF',
                                  goal_uuid=uuid,
                                  blockers=ir.get('blockers'))
                    print('[WARN] IR mất quyền arm sau khi căn tâm; đã thu '
                          'hồi các arm. Xe phải đứng yên, KHÔNG tiến/phun. '
                          'Ctrl-C rồi kiểm tra IR arm.', flush=True)
            else:
                ir_arm_missing_since = None
            if ir_terminal:
                time.sleep(0.3)
                continue
            if args.ir_only:
                if not ir_terminal:
                    raw_publishers = (
                        publisher_count('/cmd_vel_raw') if
                        ir.get('state') == 'COMPLETE' else None)
                    outcome, detail = ir_demo_outcome(
                        ir, guard, gateway, preview, raw_publishers)
                    if outcome is None and guard.get('state') == 'FAILED' and \
                            uuid:
                        outcome, detail = 'FAILED', str(guard.get('blockers'))
                    if outcome is None and gateway.get('state') == \
                            'RELEASED_STOPPED' and gateway.get(
                                'stop_reason') not in (
                                    '', 'FIRE_ACQUIRED',
                                    'NAV_GOAL_TERMINAL_NO_FIRE'):
                        outcome, detail = ('FAILED', 'Gateway: ' +
                                           str(gateway.get('stop_reason')))
                    if outcome is None and alignment.get('state') == \
                            'COMPLETE' and alignment.get('stop_reason') != \
                            'TARGET_CENTERED':
                        outcome, detail = ('FAILED', 'Alignment: ' +
                                           str(alignment.get('stop_reason')))
                    if outcome is None and ir.get('state') == 'COMPLETE' and \
                            ir.get('stop_reason') not in (
                                'REALIGN_REQUIRED',
                                'APPROACH_DISTANCE_LIMIT'):
                        if ir_completed_at is None:
                            ir_completed_at = time.monotonic()
                        elif time.monotonic() - ir_completed_at > 5.0:
                            outcome, detail = 'FAILED', 'Thiếu proof IR dừng'
                    if outcome is not None:
                        ir_terminal = True
                        session.disarm()
                        session.event('IR_DEMO_' + outcome,
                                      **ir_demo_event_details(
                                          uuid, detail, gateway, guard, ir))
                        print(('[OK] IR đã dừng trong vùng phun; bơm chưa '
                               'bật.' if outcome == 'COMPLETE' else
                               '[WARN] IR dừng lỗi: ' + str(detail)) +
                              ' Terminal/RViz giữ mở tới khi b Ctrl-C.',
                              flush=True)
                time.sleep(0.3)
                continue
            if not ir_terminal and \
                    ir.get('state') == 'ABORTED' and ir.get('goal_uuid'):
                ir_terminal = True
                session.disarm()
                details = ir_failure_evidence(ir, preview, mission)
                session.event('IR_APPROACH_FAILED', **details)
                print('[WARN] IR dừng lỗi: ' + str(details['reason']) +
                      '. Đã thu hồi arm; KHÔNG phun. RViz/log vẫn mở để '
                      'kiểm tra. Ctrl-C để đóng phiên.', flush=True)
            if ir_terminal:
                time.sleep(0.3)
                continue
            if guard.get('state') == 'FAILED' and uuid:
                ir_terminal = True
                session.disarm()
                session.event('CANCEL_GUARD_FAILED', goal_uuid=uuid,
                              blockers=guard.get('blockers'))
                print('[WARN] Cancel guard dừng lỗi; đã thu hồi arm. '
                      'KHÔNG tiến/phun; xem log và bag.', flush=True)
                time.sleep(0.3)
                continue
            if (alignment.get('state') == 'COMPLETE' and
                    alignment.get('stop_reason') != 'TARGET_CENTERED' and
                    uuid):
                ir_terminal = True
                session.disarm()
                session.event('ALIGNMENT_FAILED', goal_uuid=uuid,
                              reason=alignment.get('stop_reason'))
                print('[WARN] Căn tâm dừng lỗi; đã thu hồi arm. '
                      'KHÔNG tiến/phun; xem log và bag.', flush=True)
                time.sleep(0.3)
                continue
            if uuid not in handled and pump_admission(
                    ir, guard, gateway, alignment, mission, preview):
                handled.add(uuid)
                session.event('PUMP_ADMISSION', goal_uuid=uuid)
                pulses = 0
                while pulses < 3:
                    pump_pulse(session, uuid, respray=pulses > 0)
                    pulses += 1
                    wait_actuator_off(session)
                    review = review_pulse(session, uuid)
                    session.event('POST_SPRAY_REVIEW', goal_uuid=uuid,
                                  state=review['state'], pulse_index=pulses)
                    if review['state'] == 'NO_FIRE_EVIDENCE':
                        # Give the gateway time to latch review proof before
                        # removing the pump-state publisher.
                        time.sleep(0.5)
                        session.remove(session.pump)
                        session.pump = None
                        wait_until('pump publisher released',
                                   lambda: publisher_count('/pump_cmd') == 0,
                                   session, timeout=10)
                        print('[READY] Hết lửa; có thể đặt goal mới sau khi '
                              'RViz/AMCL ổn định.', flush=True)
                        break
                    if review['state'] == 'RESPRAY_READY_SHADOW' and pulses < 3:
                        session.remove(session.pump)
                        session.pump = None
                        wait_until('pump publisher released',
                                   lambda: publisher_count('/pump_cmd') == 0,
                                   session, timeout=10)
                        session.event('RESPRAY_AUTHORIZED', goal_uuid=uuid,
                                      next_pulse=pulses + 1)
                        continue
                    if review['state'] in (
                            'REALIGN_REQUIRED', 'APPROACH_REQUIRED') and \
                            pulses < 3:
                        # Never transfer motion ownership while a pump
                        # publisher exists, even when its command is OFF.
                        session.remove(session.pump)
                        session.pump = None
                        wait_until('pump publisher released',
                                   lambda: publisher_count('/pump_cmd') == 0,
                                   session, timeout=10)
                        session.event('POST_SPRAY_RECOVERY_STARTED',
                                      goal_uuid=uuid,
                                      recovery=review['state'],
                                      pulse_index=pulses)

                        deadline = time.monotonic() + 90
                        while time.monotonic() < deadline:
                            session.assert_alive()
                            latest = session.monitor.latest
                            current_guard = latest(
                                '/fire_mission/nav2_cancel_guard_state') or {}
                            current_ir = latest(
                                '/fire_mission/ir_approach_live_state') or {}
                            if current_guard.get('state') == 'FAILED' or \
                                    current_ir.get('state') == 'ABORTED':
                                raise SessionError(
                                    'Phục hồi sau phun thất bại: guard=' +
                                    str(current_guard.get('blockers')) +
                                    ', IR=' + str(current_ir.get(
                                        'stop_reason')))
                            if pump_admission(
                                    current_ir, current_guard,
                                    latest(
                                        '/fire_mission/live_nav_stop_state') or {},
                                    latest(
                                        '/fire_mission/live_alignment_state') or {},
                                    latest('/fire_mission/state') or {},
                                    latest(
                                        '/fire_mission/ir_range_preview') or {}):
                                break
                            time.sleep(0.3)
                        else:
                            raise SessionError(
                                'Không thể căn/tiến lại tới vùng phun '
                                'trong 90 s; giữ xe dừng và bơm OFF')
                        session.event('POST_SPRAY_RECOVERY_READY',
                                      goal_uuid=uuid,
                                      next_pulse=pulses + 1)
                        continue
                    raise SessionError(
                        f'Hậu kiểm cần xử lý thủ công: {review["state"]}; '
                        'không tự xoay/tiến/phun tiếp')
            time.sleep(0.3)
    except KeyboardInterrupt:
        session.event('SESSION_INTERRUPTED', phase=phase)
        raise
    except Exception as exc:
        session.event('SESSION_FAILED', phase=phase,
                      error_type=type(exc).__name__, error=str(exc))
        raise
    finally:
        session.close()


def parse_args():
    """Parse the mapping or explicit-live demo command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    mapping_parser = modes.add_parser('mapping')
    mapping_parser.add_argument('action', nargs='?', choices=('save',))
    mapping_parser.add_argument('name', nargs='?')
    demo_parser = modes.add_parser('demo')
    demo_parser.add_argument('--map',
                             help='Optional saved map YAML; defaults to '
                                  'scripts/selected_map.txt')
    demo_parser.add_argument('--live', action='store_true',
                             help='Explicitly authorize live motion; pump '
                                  'only in the full demo mode')
    trial = demo_parser.add_mutually_exclusive_group()
    trial.add_argument('--align-only', action='store_true',
                       help='Live Nav2, fire stop/cancel and body '
                            'alignment only; never start IR jog/pump')
    trial.add_argument('--ir-only', action='store_true',
                       help='Live Nav2, fire stop/cancel, alignment and '
                            'continuous IR approach; never start pump')
    return parser.parse_args()


def main():
    """Run the requested session and report any fail-closed condition."""
    args = parse_args()
    try:
        if args.mode == 'mapping':
            mapping(args)
        else:
            demo(args)
    except KeyboardInterrupt:
        print('\n[STOP] Thu hồi arm và đóng launch của phiên này.',
              flush=True)
    except (SessionError, OSError, yaml.YAMLError) as exc:
        print(f'[ERROR] {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
