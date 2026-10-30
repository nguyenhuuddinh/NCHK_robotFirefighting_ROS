"""App operator + fake R5 node bus: Q during a latched runtime recovery fault.

No ROS context, DDS, Pi or physical actuator. Fresh STOP telemetry is separately
tested from mission fault and from the order/repetition of local UI commands.
"""
import json
import os
from pathlib import Path
import socket

import pytest

from qa_full_operator import simulated_camera_and_no_ros_context, OriginalOperator, msg
from qa_app_operator_timing import app_class
import harness
from services.frontend_lease import FrontendLease
from services.operator_adapter import OperatorAdapter
from services.operator_channel import OperatorChannel


@pytest.mark.parametrize('fault', ['RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST', 'BOOT_OR_SERIAL_EPOCH_CHANGED'])
@pytest.mark.parametrize('sequence', ['Q', 'QQQ', 'XQ', 'QXQ'])
@pytest.mark.parametrize('telemetry', ['fresh', 'late', 'absent'])
@pytest.mark.parametrize('reverse', [False, True])
def test_recovery_stop_preserves_fault_and_requires_current_telemetry(
        monkeypatch, tmp_path, app_class, fault, sequence, telemetry, reverse):
    robot = harness.Robot(monkeypatch, goal=True, reverse=reverse)
    robot.arm()
    robot.until(lambda _: robot.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    robot.op.core.stop_fault(fault)
    at = robot.bus.now
    emit = robot.emit
    def delayed(topic, value):
        if (topic in ('/cmd_vel', '/odom', '/fire_sensors/actuator_state')
                and telemetry != 'fresh' and (telemetry == 'absent' or robot.bus.now-at < 1.)):
            return
        emit(topic, value)
    monkeypatch.setattr(robot, 'emit', delayed)
    lease = FrontendLease('s', 'g', 'n', os.getuid(), at)
    with OperatorChannel(tmp_path/'control.sock', lease) as channel:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(channel.path))
            adapter = OperatorAdapter(OriginalOperator(robot.op), channel)
            robot.op.tick = lambda _key=None: adapter.tick(robot.bus.now)
            sent = round(robot.bus.now+.05, 8)
            client.sendall(msg('HEARTBEAT', 1, sent))
            if sequence == 'XQ':
                client.sendall(msg('X', 2, sent))
                first = robot.step()
                assert not first['closed']
                client.sendall(msg('Q', 3, round(robot.bus.now+.05, 8)))
            else:
                client.sendall(b''.join(msg(key, index+2, sent) for index,key in enumerate(sequence)))
            report = robot.step()
            started = robot.op.stop_verification.start
            assert started is not None and report['closed']
            for _ in range(100):
                if report['shutdown']['done']:
                    break
                report = robot.step()
                assert robot.op.stop_verification.start == started, 'Repeated Q extended the STOP deadline'
            stop = report['shutdown']
            assert stop['done'] and stop['cause'] == 'Q', robot.diagnostic(report)
            assert stop['confirmed'] is (telemetry != 'absent'), robot.diagnostic(report)
            assert robot.op.core.fault == fault
            assert robot.op.intent_executor.closed
            remaining = [dict(node=p.info.node_name,topic=p.topic) for p in robot.bus.pubs
                         if p.topic in ('/cmd_vel_raw', '/pump_cmd')]
            for when, owner, topic, command in robot.bus.sent:
                if when <= started:
                    continue
                if topic == '/cmd_vel_raw':
                    assert all(v == 0. for v in (command.linear.x, command.linear.y, command.linear.z,
                        command.angular.x, command.angular.y, command.angular.z)), (owner,command)
                if topic == '/pump_cmd':
                    assert command.data is False
            if telemetry != 'absent':
                assert remaining == []
            else:
                # R5 intentionally retains its zero-only owner until stationary
                # odom/cmd telemetry returns. Missing proof is not a release.
                assert remaining == [dict(node='live_nav_stop_gateway',topic='/cmd_vel_raw')]
                assert 'WAIT_RAW_RELEASE' in stop['blockers']
            assert stop['motion_authorized'] is False and stop['pulse_replayed'] is False
            if telemetry == 'late':
                assert stop['elapsed_s'] >= 1.
            if telemetry == 'absent':
                assert stop['reason'] == 'STOP_UNCONFIRMED' and stop['blockers']
                assert stop['elapsed_s'] >= 4.
            result = dict(fault=fault, sequence=sequence, telemetry=telemetry, reverse=reverse,
                          stop=stop, fault_retained=True, remaining_control_publishers=remaining,
                          post_q_outputs_only_zero_or_off=True,
                          scope='FAKE_BUS_NO_ROBOT_NOT_PHYSICAL_STOP_ACCEPTANCE')
            out = os.environ.get('FIRE_ROBOT_QA_RECOVERY_INPUTS')
            if out:
                name = fault.replace(':', '_')+'_'+sequence+'_'+telemetry+'_'+str(reverse)+'.json'
                (Path(out)/name).write_text(json.dumps(result,indent=2)+'\n')
