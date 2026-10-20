"""Motion release events need real post-zero evidence, not a re-dated snapshot."""

from copy import deepcopy

import pytest

from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import SessionContextGuard
from test_session_context import GOAL, ORIGIN, report


def lease():
    guard = SessionContextGuard()
    guard.update(report(), ORIGIN, 1.0, source_ok=True)
    proof = MotionReleaseProof()
    proof.created(guard.bind(GOAL, 1.0), 1.0)
    return proof


def close(proof, generation=1, now=1.2):
    observer = report(ORIGIN + 190_000_000, 2, phase='STOPPING',
                      generation=generation)['lifecycle']['fire_session_observer']
    return proof.accept_close(observer, now)


def test_receipt_is_created_once_after_zero_hold_with_new_fresh_evidence():
    proof = lease()
    proof.command_published(0.0, 1.0)
    assert proof.can_release(1.51, odom_at=1.5, cmd_at=1.5)
    proof.released(checked_at=1.51, released_at=1.52, odom_at=1.5, cmd_at=1.5,
                   stationary_since=1.0)
    event = deepcopy(proof.receipt)
    assert event['session_id'] == proof.binding.session_id
    assert event['close_generation'] is None  # Normal release without close request.
    assert event['physical_motion_verified'] is False
    assert proof.snapshot(owned=False, blocker=None)['receipt'] == event
    assert not close(proof, now=2.0)
    assert proof.receipt == event
    with pytest.raises(RuntimeError, match='MOTION_RELEASE_EVIDENCE_INVALID'):
        proof.released(checked_at=2.0, released_at=2.0, odom_at=1.9, cmd_at=1.9,
                       stationary_since=1.0)


@pytest.mark.parametrize('now,odom_at,cmd_at', [
    (1.49, 1.48, 1.48),  # Insufficient zero hold.
    (1.51, 1.0, 1.5), (1.51, 1.5, 1.0),  # Pre-zero data.
    (1.51, 1.1, 1.5), (1.51, 1.5, 1.1),  # Stale source/cache evidence.
    (1.51, 1.6, 1.5), (1.51, 1.5, 1.6),  # Future local evidence.
])
def test_early_stale_pre_zero_or_future_data_cannot_form_a_receipt(now, odom_at, cmd_at):
    proof = lease()
    proof.command_published(0.0, 1.0)
    assert not proof.can_release(now, odom_at=odom_at, cmd_at=cmd_at)
    assert proof.receipt is None


def test_nonzero_command_cancels_the_old_zero_marker():
    proof = lease()
    proof.command_published(0.0, 1.0)
    proof.command_published(0.1, 1.2)
    assert proof.zero_published_at is None
    assert not proof.can_release(1.8, odom_at=1.79, cmd_at=1.79)
    proof.command_published(0.0, 1.8)
    assert not proof.can_release(1.9, odom_at=1.89, cmd_at=1.89)


def test_close_resets_old_zero_proof_once_and_freezes_first_request_source():
    proof = lease()
    proof.command_published(0.0, 1.0)
    assert close(proof)
    first = deepcopy(proof.close_source)
    assert proof.zero_published_at is None
    proof.command_published(0.0, 1.21)
    assert close(proof, now=1.3)
    assert proof.zero_published_at == 1.21
    assert proof.close_received_at == 1.2 and proof.close_source == first
    assert not close(proof, generation=2, now=1.4)
    assert proof.close_generation == 1
    assert proof.can_release(1.72, odom_at=1.7, cmd_at=1.7)
    proof.released(checked_at=1.72, released_at=1.73, odom_at=1.7, cmd_at=1.7,
                   stationary_since=1.21)
    assert proof.receipt['close_generation'] == 1
    assert proof.receipt['close_request_source'] == first


@pytest.mark.parametrize('changed', ['session', 'goal', 'instance', 'generation'])
def test_wrong_close_context_cannot_bind_to_the_owner(changed):
    proof = lease()
    observer = report(phase='STOPPING', generation=1)['lifecycle']['fire_session_observer']
    if changed == 'session':
        observer['session_id'] += '0'
    elif changed == 'goal':
        observer['nav_goal_uuid'] = 'b' * 32
    elif changed == 'instance':
        observer['source']['instance_id'] = 'ef' * 16
    else:
        observer['close_generation'] = True
    assert not proof.accept_close(observer, 1.2)
    assert proof.close_generation is None


@pytest.mark.parametrize('kwargs', [
    {'timeout_s': 0.351}, {'timeout_s': True}, {'timeout_s': 0},
    {'zero_hold_s': 0.49}, {'zero_hold_s': float('nan')},
])
def test_limits_cannot_weaken_freshness_or_zero_hold(kwargs):
    with pytest.raises(ValueError):
        MotionReleaseProof(**kwargs)
