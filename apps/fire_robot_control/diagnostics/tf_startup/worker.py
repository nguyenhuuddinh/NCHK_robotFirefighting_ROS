"""Confined ROS fixture/observer workers; never starts a robot runner."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from common import Trace, require_isolation, source_age_ms, stamp_ns, verify_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role', choices=['fixture', 'health', 'fast', 'readers'], required=True)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--sampling', choices=['callback', 'timer'], default='callback')
    p.add_argument('--delay-ms', type=float, default=0.)
    a = p.parse_args()
    require_isolation()
    if not 3 <= a.duration <= 45 or not 0 <= a.delay_ms <= 30:
        raise ValueError('DIAGNOSTIC_BOUNDS')
    verify_source(a.source)
    sys.dont_write_bytecode = True
    for package in ('fire_robot_mission', 'fire_robot_bringup'):
        sys.path.insert(0, str(a.source / 'ros2_ws/src' / package))
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256
    from fire_robot_mission.startup_health import StartupHealthNode
    from fire_robot_bringup.odom_to_tf_broadcaster import OdomToTfBroadcaster
    # Confirm import origins; never silently use another installed node version.
    import fire_robot_mission.startup_health as health_module
    import fire_robot_bringup.odom_to_tf_broadcaster as tf_module
    for module, package, name in ((health_module, 'fire_robot_mission', 'startup_health.py'),
                                  (tf_module, 'fire_robot_bringup', 'odom_to_tf_broadcaster.py')):
        expected = a.source / 'ros2_ws/src' / package / package / name
        if Path(module.__file__).resolve() != expected.resolve():
            raise ValueError('DIAGNOSTIC_IMPORT_NOT_PINNED')
    stop = [False]
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    rclpy.init(args=['--ros-args', '-p', 'tf_stamp_offset_ms:=-100'],
                signal_handler_options=SignalHandlerOptions.NO)
    trace = Trace()
    trace.add('start', sampling=a.sampling, injected_delay_ms=a.delay_ms)
    nodes, executor = [], SingleThreadedExecutor()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        if a.role == 'fixture':
            serial = Node('serial_bridge_node'); safety = Node('safety_watchdog'); scan = Node('scan_qos_relay')
            broadcaster = OdomToTfBroadcaster()
            nodes = [serial, safety, scan, broadcaster]
            safety.declare_parameter('cmd_vel_input_topic', '/cmd_vel_raw')
            safety.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
            odom_pub = serial.create_publisher(Odometry, '/odom', qos)
            identity_pub = serial.create_publisher(String, '/fire_sensors/serial_session_identity', qos)
            act_pub = serial.create_publisher(String, '/fire_sensors/actuator_state', qos)
            cmd_pub = safety.create_publisher(Twist, '/cmd_vel', qos)
            scan_pub = scan.create_publisher(LaserScan, '/scan', qos)
            instance = str(uuid.uuid4())
            def odom():
                msg = Odometry(); msg.header.stamp = serial.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_link'; msg.pose.pose.orientation.w = 1.
                odom_pub.publish(msg)
            def sensors():
                ticks = (time.monotonic_ns() // 10**6) % 2**32
                identity_pub.publish(String(data=json.dumps(dict(schema_version=1, frame_protocol=2,
                    source='SERIAL_V2_BOOT_WITH_LIVE_STATE', valid=True, session_ready=True,
                    telemetry_healthy=True, state_age_ms=0., state_esp_ms=ticks,
                    bridge_instance_id=instance, connection_epoch=1, elf_sha256=FIRMWARE_ELF_SHA256))))
                act_pub.publish(String(data=json.dumps(dict(schema_version=1, mode='COMMAND_TELEMETRY_ONLY',
                    pump_pwm_commanded_on=False, pump_pwm_ready=True, servo_phase='IDLE',
                    servo_pwm_commanded_on=False, home_command_completed_count=0, receive_age_ms=0., esp_ms=ticks))))
            def laser():
                msg = LaserScan(); msg.header.stamp = scan.get_clock().now().to_msg()
                msg.header.frame_id = 'laser_frame'; msg.ranges = [1.] * 32; scan_pub.publish(msg)
            serial.create_timer(.1, odom); serial.create_timer(.2, sensors)
            safety.create_timer(.1, lambda: cmd_pub.publish(Twist())); scan.create_timer(.1, laser)
            original = broadcaster._tf_broadcaster.sendTransform
            def send(transform):
                trace.add('tf_send', stamp_ns=stamp_ns(transform),
                    source_age_ms=source_age_ms(stamp_ns(transform), time.time_ns()))
                return original(transform)
            broadcaster._tf_broadcaster.sendTransform = send
        elif a.role == 'health':
            health = StartupHealthNode('diagnostic-fixture', 'pi'); nodes = [health]
            original = health.tf.set_transform
            def receive(transform, authority):
                if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                    trace.add('tf_callback', stamp_ns=stamp_ns(transform),
                        source_age_ms=source_age_ms(stamp_ns(transform), time.time_ns()))
                return original(transform, authority)
            health.tf.set_transform = receive
        else:
            node = Node('diagnostic_fast_listener' if a.role == 'fast' else 'diagnostic_ui_reader_model'); nodes = [node]
            def receive(message):
                if a.role == 'fast':
                    for transform in message.transforms:
                        if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                            trace.add('tf_callback', stamp_ns=stamp_ns(transform),
                                source_age_ms=source_age_ms(stamp_ns(transform), time.time_ns()))
            node.create_subscription(TFMessage, '/tf', receive, 100)
            if a.role == 'readers':
                node.create_subscription(LaserScan, '/scan', lambda _: None, qos)
                node.create_subscription(String, '/fire_sensors/actuator_state', lambda _: None, qos)
        if a.role != 'health':
            for node in nodes:
                executor.add_node(node)
        (a.output / (a.role + '.ready')).write_text(json.dumps(dict(scope='SYNTHETIC_ONLY', pid=os.getpid())) + '\n')
        deadline = time.monotonic() + a.duration
        next_sample = 0.
        while not stop[0] and time.monotonic() < deadline:
            before = time.monotonic_ns()
            if a.role == 'health':
                # Match R5 main exactly, including add/remove on the global
                # executor in rclpy.spin_once. An always-attached executor
                # would change the very scheduling under investigation.
                rclpy.spin_once(health, timeout_sec=.05)
            else:
                executor.spin_once(timeout_sec=.05)
            spin_ms = (time.monotonic_ns() - before) / 1e6
            if a.role == 'health' and (a.sampling == 'callback' or time.monotonic() >= next_sample):
                before = time.monotonic_ns(); row = health.sample()
                if a.delay_ms:
                    time.sleep(a.delay_ms / 1000.)
                sample_ms = (time.monotonic_ns() - before) / 1e6
                before = time.monotonic_ns()
                target = a.output / 'observer_health.json'; temp = target.with_suffix('.tmp')
                temp.write_text(json.dumps(row, allow_nan=False) + '\n'); os.replace(temp, target)
                write_ms = (time.monotonic_ns() - before) / 1e6
                trace.add('sample', spin_cost_ms=spin_ms, sample_cost_ms=sample_ms, write_cost_ms=write_ms,
                    io=row['checks']['io'], blockers=row['io_diagnostics']['blockers'],
                    tf_source_age_ms=row['io_diagnostics']['detail']['odom_tf'].get('source_age_ms'))
                next_sample = time.monotonic() + .2
    finally:
        executor.shutdown(timeout_sec=2.)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        trace.save(a.output, a.role)
        verify_source(a.source)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
