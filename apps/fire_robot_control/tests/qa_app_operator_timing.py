"""Run the app-generated operator against the actual R5 simulated node bus."""
from pathlib import Path
import sys
import types
import pytest

from services.app_operator_source import patch_operator
from qa_full_operator import simulated_camera_and_no_ros_context  # autouse fixture
import harness

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT/'releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/operational_session.py'


@pytest.fixture
def app_class(monkeypatch):
    from services.app_mission_node import install
    install(monkeypatch.setattr)
    module = types.ModuleType('fire_robot_mission._qa_app_operator')
    module.__package__ = 'fire_robot_mission'
    exec(compile(patch_operator(SOURCE.read_text()), str(SOURCE)+'[APP_COPY]', 'exec'), module.__dict__)
    monkeypatch.setattr(harness, 'OperationalSessionNode', module.OperationalSessionNode)
    return module.OperationalSessionNode


@pytest.fixture
def robot(monkeypatch, app_class):
    return harness.Robot(monkeypatch)


def test_q_begins_stop_before_any_normal_graph_scan(robot, monkeypatch):
    robot.step()
    monkeypatch.setattr(robot.op.graph, 'check', lambda *_: pytest.fail('Normal graph scan before Q'))
    result = robot.op.tick('Q')
    assert result['closed']
    assert result['shutdown']['requested']
    assert result['shutdown']['cause'] == 'Q'


def test_graph_delay_holds_authority_and_recovers_without_closing_session(robot, monkeypatch):
    robot.arm()
    original = robot.op.graph.check
    def delayed(*args):
        value = original(*args)
        robot.bus.now += .5
        return value
    monkeypatch.setattr(robot.op.graph, 'check', delayed)
    result = robot.step()
    assert result['input_hold'] and not result['closed'] and result['fault'] is None
    assert not result['live_intents_active']
    assert not any(result['execution']['plan']['arms'].values())
    monkeypatch.setattr(robot.op.graph, 'check', original)
    result = robot.until(lambda r: not r['input_hold'], limit=150)
    assert result['armed'] and not result['closed'] and result['fault'] is None
    assert robot.on_at is None


def test_arm_cannot_use_data_that_aged_during_graph_query(robot, monkeypatch):
    robot.until(lambda r: r['startup_readiness']['ready'], limit=200)
    original = robot.op.graph.check
    def delayed(*args):
        value = original(*args)
        robot.bus.now += .5
        return value
    monkeypatch.setattr(robot.op.graph, 'check', delayed)
    result = robot.step('A')
    assert not result['armed'] and not result['startup_readiness']['ready']
    assert not result['live_intents_active']


def test_late_ownership_queries_emit_only_false_heartbeats(robot, monkeypatch):
    from fire_robot_mission.operational_intents import ARM_TOPICS
    robot.arm()
    original = robot.op.handback.step
    def delayed(*args, **kwargs):
        value = original(*args, **kwargs)
        robot.bus.now += .5
        return value
    monkeypatch.setattr(robot.op.handback, 'step', delayed)
    before = len(robot.bus.sent)
    result = robot.step()
    assert result['execution']['stop_only'] and not result['live_intents_active']
    assert result['input_hold'] and not result['closed']
    for _, owner, topic, message in robot.bus.sent[before:]:
        if owner == 'fire_robot_operator' and topic in ARM_TOPICS:
            assert message.data is False


def test_stop_freshness_clock_is_resampled_after_source_queries(robot, monkeypatch):
    robot.step()
    robot.op.request_shutdown('Q')
    original = robot.op._stop_source
    once = [False]
    def delayed(*args):
        if not once[0]:
            robot.bus.now += .5
            once[0] = True
        return original(*args)
    monkeypatch.setattr(robot.op, '_stop_source', delayed)
    observed = []
    original_step = robot.op.stop_verification.step
    def capture(now, *args, **kwargs):
        observed.append(now)
        return original_step(now, *args, **kwargs)
    monkeypatch.setattr(robot.op.stop_verification, 'step', capture)
    before = robot.bus.now
    result = robot.op.stop_tick(before)
    assert observed == [pytest.approx(before+.5)]
    assert not result['shutdown']['confirmed']


@pytest.mark.parametrize('phase', ['standby', 'nav', 'turn', 'IR', 'WAIT_P', 'pulse', 'review'])
def test_app_copy_retains_existing_stop_behavior_in_each_stage(monkeypatch, tmp_path, app_class, phase):
    from qa_full_operator import test_socket_q_reaches_actual_operator_and_waits_for_stop_proof
    test_socket_q_reaches_actual_operator_and_waits_for_stop_proof(monkeypatch, tmp_path, phase)


def test_app_copy_retains_existing_x_cancel(monkeypatch, tmp_path, app_class):
    from qa_full_operator import test_socket_x_cancels_active_goal_but_keeps_session_open
    test_socket_x_cancels_active_goal_but_keeps_session_open(monkeypatch, tmp_path)
