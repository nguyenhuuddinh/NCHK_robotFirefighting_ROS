"""Verify real keyboard teleop output, isolated ROS localhost only; no robot."""
import os
import pty
import signal
import subprocess
import time


def test_mapping_teleop_command_values_and_keyboard_stop():
    assert os.environ.get('ROS_DOMAIN_ID') == '211'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(args=['mapping_speed_qa'], context=context)
    node = rclpy.create_node('mapping_speed_qa', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    messages = []
    node.create_subscription(Twist, '/cmd_vel_raw', lambda msg: messages.append((msg.linear.x, msg.angular.z)), 10)
    master, slave = pty.openpty()
    proc = subprocess.Popen(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
        '--ros-args', '-r', 'cmd_vel:=/cmd_vel_raw', '-p', 'speed:=0.1', '-p', 'turn:=0.8'],
        stdin=slave, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    os.close(slave)
    def send_and_check(key, expected):
        messages.clear()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            assert proc.poll() is None, 'teleop exited during isolated QA'
            os.write(master, key.encode())
            for _ in range(4):
                executor.spin_once(timeout_sec=.05)
                if any(abs(x-expected[0]) < 1e-6 and abs(z-expected[1]) < 1e-6 for x,z in messages):
                    return
        raise AssertionError(f'key {key!r} did not produce {expected}; received {messages[-10:]}')
    try:
        send_and_check('i', (.1, 0.))
        send_and_check('j', (0., .8))
        send_and_check('l', (0., -.8))
        send_and_check('k', (0., 0.))
        send_and_check(',', (-.1, 0.))
        send_and_check(' ', (0., 0.))
    finally:
        os.close(master)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
