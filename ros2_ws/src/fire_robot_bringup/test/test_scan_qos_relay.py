import subprocess
import time
import os
import signal
import sys
import pytest
import rclpy
from fire_robot_bringup.scan_qos_relay import main

def test_scan_qos_relay_subprocess_sigint():
    """
    Test real SIGINT on scan_qos_relay process.
    It must exit 0 and have no traceback.
    This simulates ROS 2 signal handler shutting down the context before Python handles it.
    """
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'

    # Run the script as a module
    proc = subprocess.Popen(
        [sys.executable, '-m', 'fire_robot_bringup.scan_qos_relay'],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # Wait for node to initialize
    time.sleep(1.0)

    # Send SIGINT (same as Ctrl+C)
    proc.send_signal(signal.SIGINT)

    # Wait for it to exit
    stdout, stderr = proc.communicate(timeout=5.0)

    assert proc.returncode == 0, f"Process exited with {proc.returncode}\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}"
    assert "Traceback" not in stderr, f"Traceback found in stderr:\n{stderr}"


def test_scan_qos_relay_main_keyboard_interrupt(monkeypatch):
    """
    Test main() directly with KeyboardInterrupt when context is STILL valid.
    """
    import fire_robot_bringup.scan_qos_relay as sqr

    # Mock spin to raise KeyboardInterrupt
    def mock_spin(node):
        raise KeyboardInterrupt()

    monkeypatch.setattr(sqr.rclpy, 'spin', mock_spin)

    # Ensure rclpy context is clean
    if rclpy.ok():
        rclpy.shutdown()

    # Call main; it should init, raise KeyboardInterrupt in spin, then shutdown without error
    try:
        main()
    except Exception as e:
        pytest.fail(f"main() raised an exception: {e}")

    # Ensure it successfully shutdown
    assert not rclpy.ok(), "Context should be shutdown"
