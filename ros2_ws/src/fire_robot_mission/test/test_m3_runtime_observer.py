"""QA observer must not pass a no-session or stale/source-invalid run."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


PATH = Path(__file__).resolve().parents[4] / 'scripts/qa/m3_shadow_observer.py'
spec = importlib.util.spec_from_file_location('m3_qa_observer', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(module.Observer, 'create_subscription', lambda *args: SimpleNamespace())
    monkeypatch.setattr(module.Observer, 'create_timer', lambda *args: None)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)
    monkeypatch.setattr(module.Observer, 'get_clock', lambda self:
                        SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    return module.Observer()


def payload(seq, phase='IDLE', session=None, closed=0, ready=False):
    return SimpleNamespace(data=json.dumps({
        'observe_only': True,
        'lifecycle': {
            'fire_session_observer': {'source': {'sequence': seq, 'instance_id': 'ab' * 16},
                'actuation_authorized': False, 'phase': phase, 'session_id': session,
                'close_generation': 1 if phase == 'STOPPING' else None,
                'close_reason': 'TARGET_ABSENT_CONFIRMED' if phase == 'STOPPING' else None},
            'shadow_session_runtime': {'actuation_authorized': False, 'closed_count': closed,
                                      'budget': {'planning_only': True}},
            'global_stop_collection': {'stop_evidence_ready': ready, 'session_id': session,
                                      'close_generation': 1 if phase == 'STOPPING' else None}}}))


def test_no_messages_and_no_fire_sessions_cannot_pass(node):
    assert node.receipt()['verdict'] == 'INCOMPLETE_OR_BLOCKED'


def test_current_topic_counts_alone_do_not_prove_two_session_closures(node):
    node.observe('mission', payload(1), {'source_timestamp': 99_999_999_999})
    for stats in node.stats.values():
        stats.update(count=30, last_mono=10.0)
    node.graph_samples = 30
    result = node.receipt()
    assert not result['checks']['two_fire_sessions_with_postclose_proof']
    assert result['verdict'] == 'INCOMPLETE_OR_BLOCKED'


def test_requires_two_distinct_active_stop_ready_sessions_and_idle(node):
    stamp, seq = 99_999_999_000, 0
    for session in ('fire:a:1', 'fire:a:2'):
        for phase in ('ACTIVE', 'STOPPING'):
            stamp, seq = stamp + 1, seq + 1
            node.observe('mission', payload(seq, phase, session, ready=phase == 'STOPPING'),
                         {'source_timestamp': stamp})
    node.observe('mission', payload(seq + 1, closed=2), {'source_timestamp': stamp + 1})
    for stats in node.stats.values():
        stats.update(count=30, last_mono=10.0)
    node.graph_samples = 30
    assert node.receipt()['verdict'] == 'PASS_M3_STATIC_SESSION_RUNTIME'
    node.error('UNEXPECTED_COMMAND_PUBLISHER:/pump_cmd')
    assert node.receipt()['verdict'] == 'INCOMPLETE_OR_BLOCKED'


@pytest.mark.parametrize('metadata', [{}, {'source_timestamp': True}, {'source_timestamp': 0}])
def test_missing_original_dds_metadata_has_no_receipt_time_fallback(node, metadata):
    node.observe('pose', SimpleNamespace(), metadata)
    assert node.stats['pose']['count'] == 0
    assert 'DDS_STAMP_MISSING:pose' in node.errors


def test_repeated_source_sequence_cannot_look_like_healthy_new_heartbeat(node):
    node.observe('mission', payload(1), {'source_timestamp': 99_999_999_000})
    node.observe('mission', payload(1), {'source_timestamp': 99_999_999_001})
    assert 'SESSION_SOURCE_SEQUENCE_INVALID' in node.errors
