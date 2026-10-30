"""Validate ROS argument plumbing in an isolated local ROS domain.

This creates only a dummy node without hardware or control publishers. It does
not start the operator, connect to Pi, or test the robot runtime.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

from services.operator_launch import launch_candidate_operator
from services.operator_runner import OperatorBootstrap


ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
CANDIDATE = ROOT / "releases/r5_20261029/ros"


def test_generated_arguments_reach_a_real_isolated_rclpy_node(tmp_path):
    session, runtime = tmp_path / "session", tmp_path / "runtime"
    session.mkdir(mode=0o700)
    runtime.mkdir(mode=0o700)
    boot = OperatorBootstrap("8518a120-a012-4db3-bbf3-f73f67aed61e", "b" * 32,
        "c" * 32, "a" * 64, runtime / "control.sock")
    env = dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(tmp_path))
    captured = {}
    def fake_popen(argv, **kwargs):
        captured["ros_args"] = argv[argv.index("--ros-args"):]
        return None
    launch_candidate_operator(boot, app_dir=APP, candidate_root=CANDIDATE,
        session_dir=session, runtime_dir=runtime, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, popen=fake_popen)
    source = """
import json, sys, rclpy
rclpy.init(args=['qa_operator_args', *sys.argv[1:]])
node = rclpy.create_node('fire_robot_operator')
try:
    defaults = dict(show_gui=True, enable_live_intents=False, allow_close_stationary=False,
        automatic_mode=False, automatic_start_file='', automatic_runner_session='',
        automatic_packet='', ir_confirm_samples=5, unbounded_approach=False)
    values = {name: node.declare_parameter(name, default).value for name, default in defaults.items()}
    print(json.dumps(values))
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
    result = subprocess.run([sys.executable, "-c", source, *captured["ros_args"]],
                            cwd=str(APP), env=env, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    values = json.loads(result.stdout.strip())
    assert values == dict(show_gui=False, enable_live_intents=True, allow_close_stationary=True,
        automatic_mode=True, automatic_start_file=str(session / "automatic_start.json"),
        automatic_runner_session=boot.session, automatic_packet=boot.packet,
        ir_confirm_samples=3, unbounded_approach=True)
