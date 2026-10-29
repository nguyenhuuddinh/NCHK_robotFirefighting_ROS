"""Actual live operator: ARM proof, key receipt and recovery after a wait."""
import json

from test_operator_live_delta import operator_fixture
from fire_robot_mission.operational_intents import ARM_TOPICS
from fire_robot_mission.operational_view import status_lines


def test_missing_safety_has_visible_current_blocker_and_exact_A_raw_receipt(monkeypatch):
    a = operator_fixture(monkeypatch)
    logs = []
    a.node.get_logger = lambda: type('Logger', (), {
        'info': lambda _, message: logs.append(message),
    })()
    report = a.tick('A', safety=False)
    proof = report['startup_readiness']
    assert not report['armed'] and not proof['ready']
    assert 'WAIT_SAFETY_PARAMETERS' in proof['adapter_blockers']
    assert not any(a.sent[topic][-1] for topic in ARM_TOPICS)
    text = '\n'.join(status_lines(report))
    assert 'ARM CHO' in text and 'WAIT_SAFETY_PARAMETERS' in text
    keys = [json.loads(line.split(' ', 1)[1]) for line in logs if line.startswith('OPERATOR_KEY ')]
    assert len(keys) == 1 and keys[0]['feedback']['key'] == 'A'
    assert not keys[0]['feedback']['accepted']
    assert keys[0]['feedback']['reason'] == 'WAIT_SAFETY_PARAMETERS'
    assert any(line.startswith('OPERATOR_ARM_PROOF ') for line in logs)


def test_early_A_is_discarded_ready_is_explicit_then_fresh_A_arms(monkeypatch):
    a = operator_fixture(monkeypatch)
    report = a.tick('A')
    assert not report['operator_feedback']['accepted'] and not report['armed']
    for _ in range(35):
        report = a.tick()
    assert report['startup_readiness']['ready'] and not report['armed']
    assert 'ARM SAN SANG: NHAN A' in '\n'.join(status_lines(report))
    assert not any(a.sent[topic][-1] for topic in ARM_TOPICS)
    report = a.tick('A')
    assert report['armed'] and report['operator_feedback']['reason'] == 'ARM_ACCEPTED'
    assert a.sent[ARM_TOPICS[0]][-1] is True


def test_lost_and_recovered_proof_requires_new_full_three_second_hold(monkeypatch):
    a = operator_fixture(monkeypatch)
    for _ in range(35):
        report = a.tick()
    assert report['startup_readiness']['ready']
    assert not a.tick(safety=False)['startup_readiness']['ready']
    for _ in range(20):
        report = a.tick()
    assert not report['startup_readiness']['ready']
    assert not a.tick('A')['armed']
    for _ in range(15):
        report = a.tick()
    assert report['startup_readiness']['ready']
    assert a.tick('A')['armed']
