"""Pre-fire navigation identity and one-way fire correlation; no command authority."""

from copy import deepcopy
from dataclasses import FrozenInstanceError, asdict, replace

import pytest

from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import (
    NavigationBinding, SessionBinding, SessionContextGuard,
)
from test_session_context import GOAL, INSTANCE, ORIGIN, report


ZERO = (0.0,) * 6


def idle_context():
    guard = SessionContextGuard()
    payload = report(phase='IDLE', goal=None)
    guard.update(payload, ORIGIN, 1.0, source_ok=True)
    return guard, payload


def navigation_lease():
    guard, _ = idle_context()
    proof = MotionReleaseProof(owner='live_nav_stop_gateway')
    proof.created(guard.bind_navigation(GOAL, 1.0), 1.0)
    return guard, proof


def fire_context(guard, navigation, *, now=1.2, seq=2, session=7,
                 instance=INSTANCE, goal=GOAL, phase='ACTIVE', generation=None):
    elapsed = round((now - 1.0) * 1e9)
    payload = report(ORIGIN + elapsed - 10_000_000, seq,
                     phase=phase, generation=generation,
                     session=session, goal=goal, instance=instance)
    guard.update(payload, ORIGIN + elapsed, now, source_ok=True)
    return guard.bind_fire_for_navigation(navigation, now), guard.current(now)


def test_idle_observer_binds_real_navigation_goal_without_a_fake_fire_session():
    guard, payload = idle_context()
    navigation = guard.bind_navigation(GOAL, 1.0)
    assert isinstance(navigation, NavigationBinding)
    assert navigation.nav_goal_uuid == GOAL
    assert navigation.source_instance_id == INSTANCE
    assert not hasattr(navigation, 'session_id')
    assert guard.current(1.0)['nav_goal_uuid'] is None
    assert guard.current(1.0)['session_id'] is None
    payload['lifecycle']['fire_session_observer']['source']['instance_id'] = 'ef' * 16
    assert navigation.source_instance_id == INSTANCE
    with pytest.raises(FrozenInstanceError):
        navigation.nav_goal_uuid = 'ef' * 16


@pytest.mark.parametrize('goal', [None, True, '0' * 32, 'not-a-goal'])
def test_navigation_binding_never_fabricates_a_missing_or_invalid_goal(goal):
    guard, _ = idle_context()
    assert guard.bind_navigation(goal, 1.0) is None


@pytest.mark.parametrize('phase', ['ACTIVE', 'STOPPING'])
def test_fire_or_close_context_cannot_admit_a_new_navigation_lease(phase):
    guard = SessionContextGuard()
    guard.update(report(phase=phase, generation=1 if phase == 'STOPPING' else None),
                 ORIGIN, 1.0, source_ok=True)
    assert guard.bind_navigation(GOAL, 1.0) is None


@pytest.mark.parametrize('now', [0.99, 1.36])
def test_future_or_expired_cache_cannot_bind_navigation(now):
    guard, _ = idle_context()
    assert guard.bind_navigation(GOAL, now) is None


@pytest.mark.parametrize('changes', [
    {'nav_goal_uuid': None}, {'source_instance_id': '0' * 32},
    {'source_sequence': True}, {'source_sequence': 0},
    {'source_stamp_ns': False}, {'source_stamp_ns': -1},
])
def test_navigation_identity_shape_is_validated_before_owner_creation(changes):
    guard, _ = idle_context()
    values = asdict(guard.bind_navigation(GOAL, 1.0))
    values.update(changes)
    with pytest.raises(ValueError):
        NavigationBinding(**values)


def test_first_fire_association_keeps_navigation_token_and_resets_pre_fire_zero():
    guard, proof = navigation_lease()
    original, token = proof.binding, proof.owner_token
    proof.twist_published(ZERO, 1.0)
    binding, _ = fire_context(guard, original)
    assert proof.associate_fire(binding, 1.2)
    assert proof.binding is original and proof.owner_token == token
    assert proof.created_at == 1.0
    assert proof.fire_binding == binding
    assert proof.fire_associated_at == 1.2
    assert proof.zero_published_at is None and proof.stop_latched
    assert not proof.can_release(1.51, odom_at=1.5, cmd_at=1.5)
    proof.twist_published(ZERO, 1.21)
    newer, _ = fire_context(guard, original, now=1.3, seq=3)
    assert proof.associate_fire(newer, 1.3)
    assert proof.fire_binding is binding  # First source snapshot remains frozen.
    assert proof.fire_associated_at == 1.2 and proof.zero_published_at == 1.21
    proof.released(checked_at=1.72, released_at=1.73, odom_at=1.7, cmd_at=1.7,
                   stationary_since=1.21)
    event = deepcopy(proof.receipt)
    assert event['session_id'] == f'fire:{INSTANCE}:7'
    assert event['close_generation'] is None
    _, closing = fire_context(guard, original, now=1.8, seq=4,
                              phase='STOPPING', generation=1)
    assert not proof.accept_close(closing, 1.8)
    assert proof.receipt == event


@pytest.mark.parametrize('change', ['goal', 'instance', 'session'])
def test_fresh_other_context_cannot_rebind_an_existing_navigation_owner(change):
    guard, proof = navigation_lease()
    first, _ = fire_context(guard, proof.binding)
    assert proof.associate_fire(first, 1.2)
    changes = ({'goal': 'ef' * 16} if change == 'goal' else
               {'instance': 'ef' * 16} if change == 'instance' else {'session': 8})
    candidate, observer = fire_context(guard, proof.binding, now=1.3, seq=3, **changes)
    if change != 'session':
        assert candidate is None
    # Also check the proof boundary independently of the context factory.
    source = observer['source']
    candidate = SessionBinding(observer['session_id'], observer['nav_goal_uuid'],
                               source['instance_id'], source['sequence'],
                               source['stamp']['sec'] * 1_000_000_000 + source['stamp']['nanosec'])
    assert not proof.associate_fire(candidate, 1.3)
    assert proof.fire_binding is first


@pytest.mark.parametrize('field', ['source_sequence', 'source_stamp_ns'])
def test_fire_metadata_must_follow_the_navigation_admission_sample(field):
    guard, proof = navigation_lease()
    binding, _ = fire_context(guard, proof.binding)
    same_as_acquire = replace(binding, **{field: getattr(proof.binding, field)})
    assert not proof.associate_fire(same_as_acquire, 1.2)
    assert proof.fire_binding is None


@pytest.mark.parametrize('changes', [
    {'session_id': None}, {'session_id': f'fire:{INSTANCE}:0'},
    {'session_id': f'fire:{INSTANCE}:07'}, {'session_id': f'fire:{INSTANCE}:'},
    {'source_sequence': True}, {'source_sequence': '2'},
    {'source_stamp_ns': None}, {'source_stamp_ns': 100.5},
])
def test_malformed_typed_fire_binding_cannot_correlate_a_navigation_owner(changes):
    guard, proof = navigation_lease()
    binding, _ = fire_context(guard, proof.binding)
    assert not proof.associate_fire(replace(binding, **changes), 1.2)
    assert proof.fire_binding is None


@pytest.mark.parametrize('axis', range(6))
def test_any_nonzero_twist_axis_invalidates_the_zero_marker(axis):
    _, proof = navigation_lease()
    proof.twist_published(ZERO, 1.0)
    values = list(ZERO)
    values[axis] = 0.05
    proof.twist_published(values, 1.2)
    assert proof.zero_published_at is None
    assert not proof.can_release(1.8, odom_at=1.79, cmd_at=1.79)
    proof.twist_published(ZERO, 1.8)
    assert not proof.can_release(1.9, odom_at=1.89, cmd_at=1.89)


def test_gateway_cannot_record_only_angular_zero_while_ignoring_linear_velocity():
    _, proof = navigation_lease()
    proof.twist_published(ZERO, 1.0)
    with pytest.raises(RuntimeError, match='NAVIGATION_REQUIRES_FULL_TWIST_EVIDENCE'):
        proof.command_published(0.0, 1.2)
    assert proof.zero_published_at is None


@pytest.mark.parametrize('values', [
    None, (), (0.0,) * 5, (0.0,) * 5 + (True,),
    (0.0,) * 5 + (float('nan'),), (0.0,) * 5 + (float('inf'),),
])
def test_invalid_full_twist_cannot_leave_an_old_zero_proof(values):
    _, proof = navigation_lease()
    proof.twist_published(ZERO, 1.0)
    with pytest.raises(RuntimeError, match='MOTION_PUBLISH_EVIDENCE_INVALID'):
        proof.twist_published(values, 1.2)
    assert proof.zero_published_at is None


@pytest.mark.parametrize('now', [True, 0.99, None, float('nan')])
def test_invalid_publish_time_invalidates_previous_zero_evidence(now):
    _, proof = navigation_lease()
    proof.twist_published(ZERO, 1.0)
    with pytest.raises(RuntimeError, match='MOTION_PUBLISH_EVIDENCE_INVALID'):
        proof.twist_published(ZERO, now)
    assert proof.zero_published_at is None


@pytest.mark.parametrize('event', ['fire', 'close'])
def test_publish_time_cannot_predate_the_latest_stop_event(event):
    guard, proof = navigation_lease()
    binding, observer = fire_context(guard, proof.binding,
                                     phase='STOPPING' if event == 'close' else 'ACTIVE',
                                     generation=1 if event == 'close' else None)
    assert proof.associate_fire(binding, 1.2)
    if event == 'close':
        assert proof.accept_close(observer, 1.3)
    with pytest.raises(RuntimeError, match='MOTION_PUBLISH_EVIDENCE_INVALID'):
        proof.twist_published(ZERO, 1.29 if event == 'close' else 1.19)
    assert proof.zero_published_at is None
    proof.twist_published(ZERO, 1.31)
    assert proof.can_release(1.82, odom_at=1.8, cmd_at=1.8)


def test_normal_navigation_release_has_no_fire_session_and_cannot_be_retagged():
    guard, proof = navigation_lease()
    proof.twist_published(ZERO, 1.0)
    assert proof.can_release(1.51, odom_at=1.5, cmd_at=1.5)
    # Simulate caller confirmation of actual destroy=True; helper has no ROS API.
    proof.released(checked_at=1.51, released_at=1.52, odom_at=1.5, cmd_at=1.5,
                   stationary_since=1.0)
    event = deepcopy(proof.receipt)
    assert event['owner'] == 'live_nav_stop_gateway'
    assert event['session_id'] is event['session_binding'] is None
    assert event['close_generation'] is event['fire_associated_mono'] is None
    assert event['navigation_binding']['nav_goal_uuid'] == GOAL
    assert event['physical_motion_verified'] is False
    late, _ = fire_context(guard, proof.binding, now=1.6, seq=2)
    assert not proof.associate_fire(late, 1.6)
    _, closing = fire_context(guard, proof.binding, now=1.7, seq=3,
                              phase='STOPPING', generation=1)
    assert not proof.accept_close(closing, 1.7)
    assert proof.receipt == event
    with pytest.raises(ValueError):
        proof.created(proof.binding, 1.8)
    assert proof.receipt == event
    snapshot = proof.snapshot(owned=False, blocker=None)
    snapshot['receipt']['navigation_binding']['nav_goal_uuid'] = 'ef' * 16
    assert proof.receipt == event


def test_close_receipt_keeps_both_identities_and_first_generation_after_new_hold():
    guard, proof = navigation_lease()
    nav, token = proof.binding, proof.owner_token
    active, _ = fire_context(guard, nav)
    assert proof.associate_fire(active, 1.2)
    proof.twist_published(ZERO, 1.21)
    _, closing = fire_context(guard, nav, now=1.4, seq=3,
                              phase='STOPPING', generation=1)
    assert proof.accept_close(closing, 1.4)
    close_source = deepcopy(proof.close_source)
    assert proof.zero_published_at is None
    proof.twist_published(ZERO, 1.41)
    _, duplicate = fire_context(guard, nav, now=1.5, seq=4,
                                phase='STOPPING', generation=1)
    assert proof.accept_close(duplicate, 1.5)
    assert proof.close_received_at == 1.4 and proof.zero_published_at == 1.41
    changed = deepcopy(duplicate)
    changed['close_generation'] = 2
    assert not proof.accept_close(changed, 1.6)
    assert proof.close_source == close_source
    assert not proof.can_release(1.90, odom_at=1.89, cmd_at=1.89)
    proof.released(checked_at=1.92, released_at=1.93, odom_at=1.91, cmd_at=1.91,
                   stationary_since=1.41)
    event = proof.receipt
    assert event['owner_token'] == token
    assert event['session_id'] == f'fire:{INSTANCE}:7'
    assert event['session_binding'] == asdict(active)
    assert event['navigation_binding'] == asdict(nav)
    assert event['close_generation'] == 1
    assert event['close_request_source'] == close_source


def test_first_observed_close_can_correlate_only_for_stop_not_new_navigation():
    guard, proof = navigation_lease()
    closing_binding, observer = fire_context(guard, proof.binding,
                                             phase='STOPPING', generation=1)
    assert guard.bind_navigation(GOAL, 1.2) is None
    assert proof.associate_fire(closing_binding, 1.2)
    assert proof.accept_close(observer, 1.2)
    assert proof.stop_latched and proof.receipt is None


def test_no_owner_cannot_associate_fire_or_fabricate_a_release_receipt():
    guard, _ = idle_context()
    navigation = guard.bind_navigation(GOAL, 1.0)
    proof = MotionReleaseProof(owner='live_nav_stop_gateway')
    fire, observer = fire_context(guard, navigation,
                                  phase='STOPPING', generation=1)
    assert not proof.associate_fire(fire, 1.2)
    assert not proof.accept_close(observer, 1.2)
    assert proof.owner_token is proof.receipt is None
    with pytest.raises(RuntimeError):
        proof.released(checked_at=1.8, released_at=1.8, odom_at=1.7, cmd_at=1.7,
                       stationary_since=1.2)


def test_navigation_and_fire_bindings_cannot_be_interchanged_at_owner_creation():
    guard, _ = idle_context()
    navigation = guard.bind_navigation(GOAL, 1.0)
    fire, _ = fire_context(guard, navigation)
    with pytest.raises(ValueError):
        MotionReleaseProof().created(navigation, 1.2)
    with pytest.raises(ValueError):
        MotionReleaseProof(owner='live_nav_stop_gateway').created(fire, 1.2)
