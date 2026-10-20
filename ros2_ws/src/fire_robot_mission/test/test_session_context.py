"""Supervisor source admission and immutable owner binding without ROS init."""

from dataclasses import FrozenInstanceError

import pytest

from fire_robot_mission.session_context import SessionContextGuard


ORIGIN = 100_000_000_000
INSTANCE = 'cd' * 16
GOAL = 'a' * 32


def report(stamp=ORIGIN - 10_000_000, seq=1, *, instance=INSTANCE,
           session=1, goal=GOAL, phase='ACTIVE', generation=None):
    sec, ns = divmod(stamp, 1_000_000_000)
    return {
        'schema_version': 1, 'observe_only': True,
        'lifecycle': {'fire_session_observer': {
            'mode': 'OBSERVE_ONLY', 'actuation_authorized': False,
            'phase': phase,
            'session_id': None if phase == 'IDLE' else f'fire:{instance}:{session}',
            'nav_goal_uuid': goal,
            'close_reason': 'OPERATOR_CANCEL' if phase == 'STOPPING' else None,
            'close_generation': generation,
            'source_blocker': None,
            'source': {'instance_id': instance, 'sequence': seq,
                       'stamp': {'sec': sec, 'nanosec': ns}},
        }},
    }


def admitted():
    guard = SessionContextGuard()
    guard.update(report(), ORIGIN, 1.0, source_ok=True)
    return guard


def test_binding_uses_admitted_snapshot_and_never_changes_with_payload_mutation():
    guard = SessionContextGuard()
    payload = report()
    guard.update(payload, ORIGIN, 1.0, source_ok=True)
    payload['lifecycle']['fire_session_observer']['session_id'] = 'changed'
    binding = guard.bind(GOAL, 1.01)
    assert binding.session_id == f'fire:{INSTANCE}:1'
    assert binding.nav_goal_uuid == GOAL
    assert binding.source_sequence == 1
    assert guard.blocker_for(binding, 1.01) is None
    with pytest.raises(FrozenInstanceError):
        binding.session_id = 'changed'


@pytest.mark.parametrize('offset,reason', [
    (-351_000_000, 'SESSION_SOURCE_STALE'),
    (21_000_000, 'SESSION_SOURCE_IN_FUTURE'),
])
def test_source_age_cannot_be_replaced_by_fresh_local_receipt(offset, reason):
    guard = SessionContextGuard()
    guard.update(report(ORIGIN + offset), ORIGIN, 1.0, source_ok=True)
    assert guard.bind(GOAL, 1.0) is None
    assert guard.reason == reason


def test_source_age_and_monotonic_cache_both_count():
    guard = SessionContextGuard()
    guard.update(report(ORIGIN - 300_000_000), ORIGIN, 1.0, source_ok=True)
    assert guard.bind(GOAL, 1.01) is not None
    assert guard.bind(GOAL, 1.06) is None
    assert guard.reason == 'SESSION_SOURCE_STALE'


@pytest.mark.parametrize('stamp,seq,reason', [
    (ORIGIN - 10_000_000, 2, 'SESSION_STAMP_NON_INCREASING'),
    (ORIGIN + 10_000_000, 1, 'SESSION_SEQUENCE_NON_INCREASING'),
    (ORIGIN + 10_000_000, 0, 'SESSION_SOURCE_INVALID'),
    (ORIGIN + 10_000_000, True, 'SESSION_SOURCE_INVALID'),
])
def test_duplicate_reorder_or_invalid_sequence_clears_cached_context(stamp, seq, reason):
    guard = admitted()
    guard.update(report(stamp, seq), ORIGIN + 20_000_000, 1.02, source_ok=True)
    assert guard.current(1.02) is None
    assert guard.reason == reason


@pytest.mark.parametrize('change', ['schema', 'observer_off', 'authority', 'source',
                                     'namespace', 'session_zero', 'goal', 'generation'])
def test_malformed_or_legacy_reports_cannot_authorize_a_binding(change):
    payload = report()
    observer = payload['lifecycle']['fire_session_observer']
    if change == 'schema':
        payload['schema_version'] = True
    elif change == 'observer_off':
        payload['lifecycle'].clear()
    elif change == 'authority':
        observer['actuation_authorized'] = True
    elif change == 'source':
        observer.pop('source')
    elif change == 'namespace':
        observer['session_id'] = 'fire:' + 'ef' * 16 + ':1'
    elif change == 'session_zero':
        observer['session_id'] = f'fire:{INSTANCE}:01'
    elif change == 'goal':
        observer['nav_goal_uuid'] = '0' * 32
    else:
        observer['close_generation'] = 1  # ACTIVE must not carry a close request.
    guard = admitted()
    guard.update(payload, ORIGIN + 10_000_000, 1.01, source_ok=True)
    assert guard.bind(GOAL, 1.01) is None


def test_wrong_endpoint_or_clock_rollback_revokes_cached_context():
    guard = admitted()
    binding = guard.bind(GOAL, 1.0)
    guard.update(report(ORIGIN, 2), ORIGIN + 10_000_000, 1.01, source_ok=False)
    assert guard.blocker_for(binding, 1.01) == 'SESSION_PUBLISHER_INVALID'
    guard.update(report(ORIGIN, 2), ORIGIN - 1, 1.02, source_ok=True)
    assert guard.blocker_for(binding, 1.02) == 'SESSION_CLOCK_BACKWARD'
    # Replayed sequence remains rejected even after the clock is corrected.
    guard.update(report(ORIGIN + 20_000_000, 1), ORIGIN + 30_000_000, 1.03, source_ok=True)
    assert guard.blocker_for(binding, 1.03) == 'SESSION_SEQUENCE_NON_INCREASING'


@pytest.mark.parametrize('changed', [{'session': 2}, {'goal': 'b' * 32},
                                   {'instance': 'ef' * 16}])
def test_new_source_or_session_cannot_rebind_an_existing_owner(changed):
    guard = admitted()
    binding = guard.bind(GOAL, 1.0)
    guard.update(report(ORIGIN + 10_000_000, 2, **changed),
                 ORIGIN + 20_000_000, 1.02, source_ok=True)
    assert guard.blocker_for(binding, 1.02) == 'SESSION_BINDING_CHANGED'
    assert binding.session_id == f'fire:{INSTANCE}:1'


@pytest.mark.parametrize('phase,goal,generation', [
    ('IDLE', None, None), ('ACTIVE', None, None), ('STOPPING', GOAL, 1),
])
def test_idle_no_goal_and_closing_are_not_goal_only_pump_admission(phase, goal, generation):
    guard = SessionContextGuard()
    guard.update(report(phase=phase, goal=goal, generation=generation), ORIGIN, 1.0, source_ok=True)
    assert guard.sample is not None  # Valid telemetry is not actuation admission.
    assert guard.bind(GOAL, 1.0) is None


def test_bound_close_is_a_stop_request_and_not_a_release_receipt():
    guard = admitted()
    binding = guard.bind(GOAL, 1.0)
    guard.update(report(ORIGIN + 10_000_000, 2, phase='STOPPING', generation=1),
                 ORIGIN + 20_000_000, 1.02, source_ok=True)
    assert guard.blocker_for(binding, 1.02) == 'SESSION_CLOSE_REQUESTED'
    assert not hasattr(guard, 'receipt')


@pytest.mark.parametrize('timeout', [True, 0, -1, 0.351, float('nan')])
def test_session_guard_cannot_raise_the_existing_350ms_limit(timeout):
    with pytest.raises(ValueError):
        SessionContextGuard(timeout_s=timeout)
