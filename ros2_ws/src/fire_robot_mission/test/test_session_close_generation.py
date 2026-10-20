"""Close-event correlation with no ROS initialization or command outputs."""

from dataclasses import FrozenInstanceError, replace

import pytest

from fire_robot_mission.fire_session_core import FireSessionCore
from fire_robot_mission.mission_supervisor import MissionSupervisorCore
from test_fire_session_core import GOAL_A, stopped
from test_mission_fire_session_observer import acquire, feed, observer


def closing():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin(GOAL_A)
    assert core.request_close(identity.session_id, 'SAFETY_ABORT', 1.0)
    return core, identity


def test_close_event_is_immutable_and_duplicate_requests_preserve_it():
    core, identity = closing()
    request = core.close_request
    assert request.session_id == identity.session_id
    assert request.nav_goal_uuid == GOAL_A
    assert request.generation == 1
    assert request.reason == 'SAFETY_ABORT'
    assert request.requested_at == 1.0
    assert core.request_close(identity.session_id, 'OPERATOR_CANCEL', 1.2)
    assert core.close_request is request
    with pytest.raises(FrozenInstanceError):
        request.generation = 2


@pytest.mark.parametrize('generation', [None, True, False, 0, -1, 2, 1.0, '1'])
def test_matching_session_goal_and_fresh_flags_cannot_replace_close_generation(generation):
    core, identity = closing()
    evidence = replace(stopped(identity), close_generation=generation)
    assert not core.finish_close(evidence, 1.2)
    assert core.blocker == 'CLOSE_GENERATION_MISMATCH'
    assert core.phase == 'STOPPING'
    assert core.identity is identity


def test_old_generation_cannot_be_relabelled_to_a_new_session_even_with_new_time():
    core, old = closing()
    old_receipt = stopped(old)
    assert core.finish_close(old_receipt, 1.2)
    assert core.close_request is None
    new = core.begin(GOAL_A)  # Same real goal binding, distinct fire identity.
    assert core.close_request is None
    core.request_close(new.session_id, 'TARGET_ABSENT_CONFIRMED', 2.0)
    assert core.close_request.generation == 2
    relabelled = replace(old_receipt, session_id=new.session_id, observed_at=2.1)
    assert not core.finish_close(relabelled, 2.2)
    assert core.blocker == 'CLOSE_GENERATION_MISMATCH'
    assert core.finish_close(stopped(new, 2.1, generation=2), 2.2)
    # The core still has no pump counters or actuation authority.
    assert not hasattr(core, 'pulse_index') and not hasattr(core, 'pump')


def test_invalid_requests_do_not_advance_the_generation():
    core = FireSessionCore(proof_timeout_s=0.35)
    identity = core.begin()
    assert not core.request_close('old-session', 'SAFETY_ABORT', 1.0)
    assert not core.request_close(identity.session_id, 'CAMERA_STALE', 1.0)
    assert not core.request_close(identity.session_id, 'SAFETY_ABORT', float('nan'))
    assert core.close_request is None
    assert core.request_close(identity.session_id, 'SAFETY_ABORT', 1.0)
    assert core.close_request.generation == 1
    assert core.close_request.nav_goal_uuid is None


@pytest.mark.parametrize('goal', [None, GOAL_A])
def test_supervisor_exposes_only_the_current_close_generation_and_stays_stopping(goal):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    active = acquire(core, goal)
    assert active['close_generation'] is None
    identity = core._fire_session.identity
    core._fire_session.request_close(identity.session_id, 'OPERATOR_CANCEL', 0.12)
    request = core._fire_session.close_request
    for now in (0.13, 0.2):
        core.update_goal_statuses([] if goal is None else [(goal, 5)], now)
        feed(core, now, present=False)
        state = observer(core, now)
        assert state['phase'] == 'STOPPING'
        assert state['close_generation'] == request.generation
        assert state['session_id'] == request.session_id
        assert state['nav_goal_uuid'] == request.nav_goal_uuid
        assert state['actuation_authorized'] is False
        assert state['blocker'] == 'VERIFIED_SESSION_CLEANUP_NOT_CONNECTED'
        assert core._fire_session.close_request is request
        assert core._fire_session.last_closed is None


def test_observer_disabled_does_not_add_a_close_generation_contract():
    core = MissionSupervisorCore()
    result = core.evaluate(0.0)
    assert core._fire_session is None
    assert 'fire_session_observer' not in result['lifecycle']
