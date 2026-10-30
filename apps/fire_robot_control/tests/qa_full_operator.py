"""Socket -> real R5 operator node with the existing simulated six-node bus.

Run with pytest and ROS 2 Humble Python packages. There is no ROS init, DDS,
Pi, actuator or actual hardware here. This is not physical STOP acceptance.
"""
import json
import os
from pathlib import Path
import socket
import sys
import pytest
import rclpy

from services.frontend_lease import FrontendLease
from services.operator_adapter import OperatorAdapter
from services.operator_channel import OperatorChannel
from services.operator_runner import OperatorBootstrap, run_integrated


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission"
sys.path.insert(0, str(PKG / "test/r5_full_adapter"))
sys.path.insert(0, str(PKG))
from harness import BE, Robot


@pytest.fixture(autouse=True)
def simulated_camera_and_no_ros_context(monkeypatch):
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    assert not rclpy.ok()
    monkeypatch.setattr(rclpy, "init", lambda *args, **kwargs: pytest.fail("real ROS context forbidden"))
    original_init, original_inputs = Robot.__init__, Robot.inputs
    ok, encoded = cv2.imencode(".jpg", np.zeros((4, 4, 3), dtype=np.uint8))
    assert ok
    def init(self, *args, **kwargs):
        original_init(self, *args, **kwargs)
        self.camera_pub = self.bus.publisher("yolo_observer", CompressedImage,
                                             "/yolo/image_annotated/compressed", BE)
    def inputs(self):
        original_inputs(self)
        frame = CompressedImage(format="jpeg", data=encoded.tobytes())
        stamp = round(self.bus.now * 1e9)
        frame.header.stamp.sec, frame.header.stamp.nanosec = stamp // 10**9, stamp % 10**9
        frame.header.frame_id = "camera_frame"
        self.camera_pub.publish(frame)
    monkeypatch.setattr(Robot, "__init__", init)
    monkeypatch.setattr(Robot, "inputs", inputs)
    yield
    assert not rclpy.ok()


def msg(event, seq, now):
    return json.dumps(dict(version=1, session="s", generation="g", nonce="n",
        seq=seq, sent_mono=now, event=event)).encode() + b"\n"


class OriginalOperator:
    def __init__(self, node):
        self.node = node
        self.original = node.tick
    def tick(self, key): return self.original(key)
    def request_shutdown(self, reason): return self.node.request_shutdown(reason)


def test_expired_initial_frontend_still_advances_real_operator_stop(monkeypatch, tmp_path):
    robot = Robot(monkeypatch, goal=False)
    lease = FrontendLease("s", "g", "n", os.getuid(), robot.bus.now - 4.)
    with OperatorChannel(Path(tmp_path) / "control.sock", lease) as channel:
        adapter = OperatorAdapter(OriginalOperator(robot.op), channel)
        robot.op.tick = lambda _key=None: adapter.tick(robot.bus.now)
        report = robot.step()
        assert report['closed'] and not report['shutdown']['done']
        report = robot.until(lambda r: r.get('shutdown', {}).get('done'), limit=200)
        assert report['shutdown']['done']
        assert lease.reason == 'FRONTEND_LEASE_EXPIRED'
        assert not any(p.topic in ('/cmd_vel_raw', '/pump_cmd') for p in robot.bus.pubs)


@pytest.mark.parametrize("phase", ["standby", "nav", "turn", "IR", "WAIT_P", "pulse", "review"])
def test_socket_q_reaches_actual_operator_and_waits_for_stop_proof(monkeypatch, tmp_path, phase):
    robot = Robot(monkeypatch, goal=phase == "nav")
    robot.arm()
    if phase == "nav":
        robot.until(lambda _: robot.by_name["live_nav_stop_gateway"].core.ever_forwarded)
    elif phase in ("turn", "IR", "WAIT_P", "pulse", "review"):
        robot.fire = True
        robot.step("F")
        robot.until(lambda r: r["phase"] == "WAIT_T")
        robot.step("T")
        robot.until(lambda _: abs(robot.yaw) > .025)
        if phase != "turn":
            robot.centered = True
            robot.until(lambda r: r["phase"] == "WAIT_IR_I")
            robot.step("I")
            robot.until(lambda _: robot.x > .025)
            if phase != "IR":
                robot.right = 950
                robot.until(lambda r: r["phase"] == "WAIT_P")
                if phase in ("pulse", "review"):
                    robot.step("P")
                    robot.until(lambda _: robot.on_at is not None)
                    if phase == "review":
                        robot.until(lambda _: robot.home == 1)
    lease = FrontendLease("s", "g", "n", os.getuid(), robot.bus.now)
    with OperatorChannel(Path(tmp_path) / "control.sock", lease) as channel:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(channel.path))
            adapter = OperatorAdapter(OriginalOperator(robot.op), channel)
            robot.op.tick = lambda _key=None: adapter.tick(robot.bus.now)
            # Same batch intentionally contains X and Q; Q must win.
            sent = round(robot.bus.now + .05, 8)
            client.sendall(msg("HEARTBEAT", 1, sent) + msg("X", 2, sent) + msg("Q", 3, sent))
            report = robot.step()
            assert report["closed"] and not report["shutdown"]["done"]
            report = robot.until(lambda r: r.get("shutdown", {}).get("done"), limit=100)
            assert report["shutdown"]["confirmed"], robot.diagnostic(report)
            assert robot.op.intent_executor.closed
            assert not any(p.topic in ("/cmd_vel_raw", "/pump_cmd") for p in robot.bus.pubs)
            if phase in ("pulse", "review"):
                assert robot.home == 1 and report["shutdown"]["home_required"]


def test_socket_x_cancels_active_goal_but_keeps_session_open(monkeypatch, tmp_path):
    robot = Robot(monkeypatch, goal=True)
    robot.arm()
    robot.until(lambda _: robot.by_name["live_nav_stop_gateway"].core.ever_forwarded)
    lease = FrontendLease("s", "g", "n", os.getuid(), robot.bus.now)
    with OperatorChannel(Path(tmp_path) / "control.sock", lease) as channel:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(channel.path))
            adapter = OperatorAdapter(OriginalOperator(robot.op), channel)
            seq = [2]
            sent = round(robot.bus.now + .05, 8)
            client.sendall(msg("HEARTBEAT", 1, sent) + msg("X", 2, sent))
            def tick(_key=None):
                seq[0] += 1
                client.sendall(msg("HEARTBEAT", seq[0], robot.bus.now))
                return adapter.tick(robot.bus.now)
            robot.op.tick = tick
            report = robot.step()
            assert not report["closed"]
            robot.until(lambda _: bool(robot.bus.cancelled), limit=100)
            assert not robot.op.core.closed


@pytest.mark.parametrize('runtime_fault', [None, 'RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'])
def test_candidate_runner_drains_actual_operator_with_simulated_bus(monkeypatch, tmp_path, runtime_fault):
    robot = Robot(monkeypatch, goal=True)
    robot.arm()
    robot.until(lambda _: robot.by_name["live_nav_stop_gateway"].core.ever_forwarded)
    session = "8518a120-a012-4db3-bbf3-f73f67aed61e"
    generation, nonce, packet = "b" * 32, "c" * 32, "a" * 64
    # Only the runner's parameter binding is synthetic; node/STOP code is R5.
    robot.bus.overrides["fire_robot_operator"].update(
        automatic_mode=True, automatic_runner_session=session, automatic_packet=packet)
    robot.op.destroy_node = lambda: None
    if runtime_fault:
        robot.op.core.stop_fault(runtime_fault)

    class SimRos:
        active = False
        def init(self, **_): self.active = True
        def ok(self): return self.active
        def shutdown(self): self.active = False
        def spin_once(self, node, timeout_sec):
            bus = robot.bus
            bus.advance()
            robot.inputs()
            bus.flush()
            for participant in robot.nodes:
                for callback in participant.test_timers: callback()
                bus.flush()

    ros = SimRos()
    clients = []
    def channel_factory(path, lease):
        channel = OperatorChannel(path, lease)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        clients.append(client)
        client.connect(str(path))
        stamp = robot.bus.now
        def event(name, seq):
            return json.dumps(dict(version=1, session=session, generation=generation,
                nonce=nonce, seq=seq, sent_mono=stamp, event=name)).encode() + b"\n"
        client.sendall(event("HEARTBEAT", 1) + event("Q", 2))
        return channel

    bootstrap = OperatorBootstrap(session, generation, nonce, packet, Path(tmp_path) / "control.sock")
    try:
        result = run_integrated(bootstrap, ros=ros, node_factory=lambda: robot.op,
                                channel_factory=channel_factory, clock=lambda: robot.bus.now,
                                result_path=tmp_path/'operator_result.json')
    finally:
        for client in clients: client.close()
    assert result == 0
    terminal = json.loads((tmp_path/'operator_result.json').read_text())
    assert terminal['runtime_fault'] == runtime_fault
    assert terminal['runtime_status'] == ('FAILED' if runtime_fault else 'SUCCEEDED')
    assert robot.op.stop_verification.confirmed
    assert not any(p.topic in ("/cmd_vel_raw", "/pump_cmd") for p in robot.bus.pubs)
    assert not ros.active
