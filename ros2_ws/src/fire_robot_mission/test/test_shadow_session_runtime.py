"""Shadow coordinator, independent lifecycle proof and durable event writes."""

from dataclasses import replace
import pytest

from fire_robot_mission.fire_session_core import FireSessionCore
from fire_robot_mission.global_stop_evidence import GlobalStopReadiness
from fire_robot_mission.mission_supervisor import MissionSupervisorCore
from fire_robot_mission.runtime_manifest import runtime_manifest
from fire_robot_mission.shadow_budget import ShadowBudgetStore
from fire_robot_mission.shadow_session_runtime import ShadowSessionRuntime
from test_global_stop_evidence import Case, GOAL
from test_mission_fire_session_observer import feed


@pytest.fixture
def runtime(tmp_path):
    store = ShadowBudgetStore(str(tmp_path / 'budget.json'), initialize=True)
    result = ShadowSessionRuntime(store, runtime_manifest('io_nav_static_no_owners_v1'))
    yield result
    store.close()


def mission(runtime):
    core = MissionSupervisorCore(enable_fire_session_observer=True, fire_acquire_hold_s=.1)
    core._session_runtime = runtime
    return core


def stopping_case(runtime):
    case = Case(goal=GOAL)
    result = case.ready()
    core = mission(runtime)
    core._fire_session = case.core
    runtime.store.reserve_session(case.identity.session_id, GOAL)
    return case, core, result


class Retirement:
    def __init__(self, request):
        self.request, self.called = request, False
    def retire_after_close(self, core, evidence, now):
        assert core.can_retire_collectors(self.request, evidence, now)
        self.called = True


def test_successful_typed_close_clears_old_inputs_keeps_identity_namespace_and_reservation(runtime):
    case, core, result = stopping_case(runtime)
    feed(core, 10.61)
    core.update_goal_statuses([(GOAL, 5)], 10.61)
    namespace = core._fire_session.source_instance_id
    owners = Retirement(case.core.close_request)
    assert runtime.finish_after_publish(core, result, owners, case.stop, now=10.61, clock=lambda: 10.61)
    assert owners.called and case.stop.request is None
    assert core._fire_session.phase == 'IDLE' and core._detection is None
    assert core._goal_status_records == [] and core._alignment is None
    assert core._fire_session.source_instance_id == namespace
    assert core._fire_session.last_closed_evidence is result.evidence
    assert runtime.store.state['used']['sessions'] == 1
    assert runtime.store.state['pending'] is None
    assert runtime.closed_count == 1


@pytest.mark.parametrize('field,value', [
    ('nav_inactive', False), ('cmd_zero', False), ('pump_off', False),
    ('motion_owner_released', False), ('servo_idle', False),
    ('close_generation', 99), ('nav_goal_uuid', None), ('observed_at', 9),
])
def test_wrong_proof_never_clears_inputs_or_persists_completion(runtime, field, value):
    case, core, result = stopping_case(runtime)
    feed(core, 10.61)
    bad = GlobalStopReadiness(True, (), replace(result.evidence, **{field: value}))
    owners = Retirement(case.core.close_request)
    assert not runtime.finish_after_publish(core, bad, owners, case.stop, now=10.61, clock=lambda: 10.61)
    assert not owners.called and core._fire_session.phase == 'STOPPING'
    assert core._detection is not None
    assert runtime.store.state['pending']['completion_marked'] is False


def test_commit_delay_cannot_rehabilitate_expired_proof(runtime):
    case, core, result = stopping_case(runtime)
    owners = Retirement(case.core.close_request)
    assert not runtime.finish_after_publish(core, result, owners, case.stop, now=10.61, clock=lambda: 11.1)
    assert core._fire_session.phase == 'STOPPING' and not owners.called
    assert runtime.store.state['pending']['completion_marked'] is True


def test_ros_recheck_failure_after_commit_does_not_retire(runtime):
    case, core, result = stopping_case(runtime)
    owners = Retirement(case.core.close_request)
    assert not runtime.finish_after_publish(
        core, result, owners, case.stop, now=10.61, clock=lambda: 10.61,
        recheck=lambda: GlobalStopReadiness(False, ('GLOBAL_CLOCK_DISCONTINUITY',), None))
    assert core._fire_session.phase == 'STOPPING' and not owners.called
    assert runtime.store.state['pending']['completion_marked'] is True


def test_disk_failure_propagates_before_retirement_or_idle(runtime, monkeypatch):
    case, core, result = stopping_case(runtime)
    owners = Retirement(case.core.close_request)
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr(runtime.store, 'mark_completion', fail)
    with pytest.raises(OSError):
        runtime.finish_after_publish(core, result, owners, case.stop, now=10.61, clock=lambda: 10.61)
    assert core._fire_session.phase == 'STOPPING' and not owners.called


def test_operator_recovery_supersedes_generation_preserves_quota_and_needs_new_proof(runtime):
    core = FireSessionCore(proof_timeout_s=.35)
    identity = runtime.try_begin(core, GOAL, 10)
    core.request_close(identity.session_id, 'OPERATOR_CANCEL', 10)
    old = core.close_request
    runtime.recovery_latch = 'SOURCE_CHANGED'
    request = runtime.operator_request(core, 10.1)
    assert request.generation == old.generation + 1
    assert core.last_superseded_request is old
    assert runtime.store.state['used']['sessions'] == 1
    assert runtime.store.state['recoveries'] == 1
    assert runtime.recovery_latch is None and core.phase == 'STOPPING'
    assert runtime.blocker == 'NEW_POST_CLOSE_PROOF_REQUIRED'
    assert not core.renew_close_for_recovery(10.2)


def test_restart_pending_never_starts_automatically_and_uses_new_recovery_identity(runtime):
    old = FireSessionCore(proof_timeout_s=.35)
    identity = runtime.try_begin(old, GOAL, 10)
    restart = ShadowSessionRuntime(runtime.store, runtime.manifest)
    core = FireSessionCore(proof_timeout_s=.35)
    assert restart.try_begin(core, None, 11) is None
    assert core.identity is None
    request = restart.operator_request(core, 11)
    assert request.session_id != identity.session_id and request.nav_goal_uuid == GOAL
    assert runtime.store.state['pending']['origin_session_id'] == identity.session_id
    assert runtime.store.state['used']['sessions'] == 1


@pytest.mark.parametrize('value', [None, True, 0, 'false'])
def test_required_owner_must_explicitly_declare_live_disabled(runtime, value):
    one = ShadowSessionRuntime(runtime.store, runtime_manifest('all_four_external_v1'))
    owner = one.manifest.required[0]
    one.observe_owner(owner, {'live_enabled': value, 'source': {'instance_id': 'ab' * 16}}, 10)
    assert one.owner_blocker(10).startswith('SHADOW_OWNER_LIVE_OR_UNDECLARED')
    assert one.try_begin(FireSessionCore(proof_timeout_s=.35), GOAL, 10) is None


def test_owner_process_change_is_sticky_but_regular_refresh_is_not(runtime):
    one = ShadowSessionRuntime(runtime.store, runtime_manifest('all_four_external_v1'))
    for owner in one.owners:
        one.observe_owner(owner, {'live_enabled': False, 'source': {'instance_id': 'ab' * 16}}, 10)
    assert one.owner_blocker(10.1) is None
    assert one.owner_blocker(10.4) == 'SHADOW_OWNER_REPORT_NOT_CURRENT'
    owner = one.manifest.required[0]
    one.observe_owner(owner, {'live_enabled': False, 'source': {'instance_id': 'cd' * 16}}, 10.5)
    one.observe_owner(owner, {'live_enabled': False, 'source': {'instance_id': 'ab' * 16}}, 10.6)
    assert one.recovery_latch == 'SHADOW_OWNER_PROCESS_CHANGED:' + owner


@pytest.mark.parametrize('goal', [GOAL, None])
def test_fire_absence_closes_after_distinct_fresh_frames_with_or_without_goal(runtime, goal):
    core = mission(runtime)
    for now in (10, 10.11):
        feed(core, now)
        core.update_goal_statuses([] if goal is None else [(goal, 2)], now)
        core.evaluate(now)
    assert core._fire_session.phase == 'ACTIVE'
    for now in (10.2, 10.4, 10.6, 10.8, 11, 11.2, 11.4, 11.6, 11.71):
        feed(core, now, present=False)
        core.update_goal_statuses([] if goal is None else [(goal, 2)], now)
        core.evaluate(now)
    assert core._fire_session.close_reason == 'TARGET_ABSENT_CONFIRMED'
    assert core._fire_session.identity.nav_goal_uuid == goal
    assert runtime.store.state['pending'] is not None  # no automatic refund/close


def test_absence_flicker_stale_and_repeated_frames_restart_hold(runtime):
    assert not runtime.observe_absence(absent=True, event_at=10, now=10)
    assert not runtime.observe_absence(absent=True, event_at=10, now=10.3)
    assert runtime.absence_count == 1
    assert not runtime.observe_absence(absent=True, event_at=10, now=10.4)
    assert runtime.absence_since is None
    for n in range(8):
        runtime.observe_absence(absent=True, event_at=11 + n * .2, now=11 + n * .2)
    assert not runtime.observe_absence(absent=False, event_at=12.5, now=12.5)
    assert not runtime.observe_absence(absent=True, event_at=12.6, now=12.6)
    assert runtime.absence_count == 1


def test_exhausted_encounter_blocks_new_goal_without_reset(runtime):
    core = mission(runtime)
    for n in range(1, 4):
        identity = runtime.try_begin(core._fire_session, GOAL, 10)
        # Ledger-only reservation test: a separate core models a new process.
        runtime.store.mark_completion(identity.session_id)
        runtime.store.finish_completion(identity.session_id)
        core._fire_session = FireSessionCore(proof_timeout_s=.35)
    assert runtime.try_begin(core._fire_session, None, 20) is None
    assert runtime.blocker == 'SHADOW_ENCOUNTER_BUDGET_EXHAUSTED'


def test_post_spray_review_is_not_synthesized_by_global_stop(runtime):
    case, core, result = stopping_case(runtime)
    old = core._fire_session.close_request
    core._fire_session.close_request = replace(old, reason='POST_SPRAY_CLEAR')
    core._fire_session.close_reason = 'POST_SPRAY_CLEAR'
    owners = Retirement(core._fire_session.close_request)
    assert not runtime.finish_after_publish(core, result, owners, case.stop, now=10.61, clock=lambda: 10.61)
    assert core._fire_session.blocker is None  # precheck does not pretend a close happened
    assert runtime.blocker == 'POST_SPRAY_REVIEW_MISSING'


@pytest.mark.parametrize('now', [True, float('nan'), float('inf'), -1])
def test_invalid_operator_time_does_not_mutate_idle_core(runtime, now):
    idle = FireSessionCore(proof_timeout_s=.35)
    with pytest.raises(ValueError, match='time'):
        runtime.operator_request(idle, now)
    assert idle.identity is None
