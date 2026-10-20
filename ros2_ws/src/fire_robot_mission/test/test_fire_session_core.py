"""Replay independent fire sessions without ROS, timers or hardware."""

from dataclasses import replace

import pytest

from fire_robot_mission.fire_session_core import (
    FireSessionCore, SessionCleanupEvidence,
)


GOAL_A = 'ab' * 16
GOAL_B = 'cd' * 16


def stopped(identity, at=1.1, generation=1):
    return SessionCleanupEvidence(
        identity.session_id, identity.nav_goal_uuid, at,
        nav_inactive=True, nav_terminal_status=(
            5 if identity.nav_goal_uuid is not None else None),
        motion_owner_released=True, pump_owner_released=True,
        pump_off=True, servo_idle=True, stationary=True, cmd_zero=True,
        close_generation=generation)


def test_fire_without_nav_goal_has_real_independent_identity():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    assert identity.nav_goal_uuid is None
    assert identity.session_id.startswith('fire:')
    assert identity.session_id not in (GOAL_A, GOAL_B)
    assert core.phase == 'ACTIVE'
    assert core.begin() is identity  # Same acquire event is idempotent.


@pytest.mark.parametrize('goal', [None, GOAL_A])
def test_target_absence_closes_without_pumping_and_allows_new_fire(goal):
    core = FireSessionCore(proof_timeout_s=0.35)
    old = core.begin(goal)
    assert core.request_close(old.session_id, 'TARGET_ABSENT_CONFIRMED', 1.0)
    assert core.begin() is None
    assert core.blocker == 'SESSION_CLEANUP_PENDING'
    assert core.finish_close(stopped(old), 1.2)
    assert core.phase == 'IDLE'
    new = core.begin()  # New fire before any new Nav2 goal.
    assert new.session_id != old.session_id
    assert new.nav_goal_uuid is None
    assert core.last_closed == (old, 'TARGET_ABSENT_CONFIRMED')


@pytest.mark.parametrize('status', [4, 5, 6])
def test_terminal_goal_allows_session_cleanup_but_does_not_issue_nav_action(status):
    core = FireSessionCore(proof_timeout_s=0.35)
    old = core.begin(GOAL_A)
    core.request_close(old.session_id, 'OPERATOR_CANCEL', 1.0)
    proof = replace(stopped(old), nav_terminal_status=status)
    assert core.finish_close(proof, 1.2)
    new = core.begin(GOAL_B)
    assert new.nav_goal_uuid == GOAL_B
    assert new.session_id != old.session_id


def test_session_binding_cannot_be_replaced_by_another_active_goal():
    core = FireSessionCore(proof_timeout_s=0.35)
    current = core.begin()
    assert core.begin(GOAL_A) is None
    assert core.blocker == 'SESSION_GOAL_BINDING_CHANGED'
    assert core.identity is current
    assert core.identity.nav_goal_uuid is None


def test_late_closure_or_cleanup_from_old_session_cannot_change_new_one():
    core = FireSessionCore(proof_timeout_s=0.35)
    old = core.begin(GOAL_A)
    core.request_close(old.session_id, 'TARGET_ABSENT_CONFIRMED', 1.0)
    old_proof = stopped(old)
    assert core.finish_close(old_proof, 1.2)
    new = core.begin()
    assert not core.request_close(old.session_id, 'OPERATOR_CANCEL', 1.3)
    assert core.identity is new and core.phase == 'ACTIVE'
    core.request_close(new.session_id, 'OPERATOR_CANCEL', 1.4)
    assert not core.finish_close(replace(old_proof, observed_at=1.5), 1.5)
    assert core.identity is new and core.phase == 'STOPPING'


@pytest.mark.parametrize('field,blocker', [
    ('nav_inactive', 'NAVIGATION_STILL_ACTIVE'),
    ('motion_owner_released', 'MOTION_OWNER_PRESENT'),
    ('pump_owner_released', 'PUMP_OWNER_PRESENT'),
    ('pump_off', 'PUMP_NOT_OFF'), ('servo_idle', 'SERVO_NOT_IDLE'),
    ('stationary', 'ROBOT_NOT_STATIONARY'), ('cmd_zero', 'CMD_VEL_NOT_ZERO'),
])
def test_each_missing_stop_or_release_proof_keeps_session_stopping(field, blocker):
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    core.request_close(identity.session_id, 'TARGET_ABSENT_CONFIRMED', 1.0)
    assert not core.finish_close(replace(stopped(identity), **{field: False}), 1.2)
    assert core.phase == 'STOPPING' and core.blocker == blocker
    assert core.begin() is None


@pytest.mark.parametrize('observed_at,now', [
    (0.9, 1.2), (1.1, 1.5), (1.3, 1.2), (float('nan'), 1.2),
])
def test_stale_future_or_pre_stop_evidence_cannot_finish(observed_at, now):
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    core.request_close(identity.session_id, 'OPERATOR_CANCEL', 1.0)
    assert not core.finish_close(stopped(identity, observed_at), now)
    assert core.blocker == 'CLEANUP_EVIDENCE_STALE_OR_EARLY'


def test_post_spray_closure_still_requires_review_and_does_not_reset_pump_core():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin(GOAL_A)
    core.request_close(identity.session_id, 'POST_SPRAY_CLEAR', 1.0)
    proof = stopped(identity)
    assert not core.finish_close(proof, 1.2)
    assert core.blocker == 'POST_SPRAY_REVIEW_MISSING'
    assert core.finish_close(replace(proof, post_spray_clear=True), 1.2)
    # Lifecycle has no pump quota, output, or access to existing pump objects.
    assert not hasattr(core, 'pump') and not hasattr(core, 'pulse_index')


def test_duplicate_close_keeps_first_reason_and_does_not_extend_proof_window():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    core.request_close(identity.session_id, 'SAFETY_ABORT', 1.0)
    core.request_close(identity.session_id, 'OPERATOR_CANCEL', 1.1)
    assert core.close_reason == 'SAFETY_ABORT'
    assert core.close_requested_at == 1.0
    assert not core.finish_close(stopped(identity), 1.6)


def test_no_goal_does_not_accept_a_fabricated_nav_cancel_proof():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    core.request_close(identity.session_id, 'OPERATOR_CANCEL', 1.0)
    assert not core.finish_close(replace(stopped(identity), nav_terminal_status=5), 1.2)
    assert core.blocker == 'UNEXPECTED_NAV_GOAL_PROOF'


def test_cleanup_cannot_use_a_different_goal_uuid():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin(GOAL_A)
    core.request_close(identity.session_id, 'OPERATOR_CANCEL', 1.0)
    assert not core.finish_close(replace(stopped(identity), nav_goal_uuid=GOAL_B), 1.2)
    assert core.phase == 'STOPPING' and core.blocker == 'SESSION_ID_MISMATCH'


@pytest.mark.parametrize('status', [None, 2, 3])
def test_bound_goal_requires_terminal_status_to_close(status):
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin(GOAL_A)
    core.request_close(identity.session_id, 'NAVIGATION_ABORTED', 1.0)
    assert not core.finish_close(replace(stopped(identity), nav_terminal_status=status), 1.2)
    assert core.blocker == 'NAV_GOAL_TERMINAL_PROOF_MISSING'


def test_link_or_camera_loss_is_not_implicitly_confirmed_target_absence():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    assert not core.request_close(identity.session_id, 'CAMERA_STALE', 1.0)
    assert core.close_reason is None
    assert core.request_close(identity.session_id, 'SAFETY_ABORT', 1.1)
    assert core.close_reason == 'SAFETY_ABORT'


def test_new_core_instance_does_not_reuse_previous_session_namespace():
    first = FireSessionCore(proof_timeout_s=0.35).begin()
    second = FireSessionCore(proof_timeout_s=0.35).begin()
    assert first.session_id != second.session_id
