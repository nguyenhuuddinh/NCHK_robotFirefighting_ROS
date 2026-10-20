"""Pure transport/owner-close contract; no ROS init, publisher or cleanup effect."""

from copy import deepcopy
from dataclasses import FrozenInstanceError, replace

import pytest

from fire_robot_mission.close_owner_evidence import (
    CLOSE_OWNERS, CloseOwnerEvidenceCollector, OwnerCloseReporter,
)
from fire_robot_mission.fire_session_core import (
    FireSessionCore, SessionCleanupEvidence, SessionCloseRequest,
)
from fire_robot_mission.motion_release import MotionReleaseProof
from fire_robot_mission.session_context import NavigationBinding, SessionContextGuard
from test_session_context import ORIGIN, report


GOAL = 'ab' * 16


def ros(now):
    return ORIGIN + round(now * 1e9)


class Round:
    def __init__(self, *, goal=GOAL, begin=True):
        self.core = FireSessionCore(proof_timeout_s=0.35)
        self.identity = self.core.begin(goal)
        self.collector = CloseOwnerEvidenceCollector()
        self.contexts = {owner: SessionContextGuard() for owner in CLOSE_OWNERS}
        self.reporters = {owner: OwnerCloseReporter(owner) for owner in CLOSE_OWNERS}
        self.sequences = dict.fromkeys(CLOSE_OWNERS, 0)
        if begin:
            self.begin()

    def begin(self):
        assert self.core.request_close(self.identity.session_id, 'OPERATOR_CANCEL', 10.0)
        assert self.collector.begin_close(self.core.close_request,
                                         announced_ros_ns=ros(10.0), announced_at=10.0)

    def payload(self, owner, now=10.1, *, owned=False, phase=None, old_ms=5):
        phase = phase or self.core.phase
        self.sequences[owner] += 1
        payload = report(
            ros(now) - 10_000_000, self.sequences[owner],
            instance=self.core.source_instance_id,
            session=int(self.identity.session_id.rsplit(':', 1)[1]),
            goal=self.identity.nav_goal_uuid if phase != 'IDLE' else None,
            phase=phase,
            generation=self.core.close_request.generation if phase == 'STOPPING' else None)
        self.contexts[owner].update(payload, ros(now), now, source_ok=True)
        return self.reporters[owner].report(
            self.contexts[owner], publisher_owned=owned,
            stamp_ns=ros(now) - round(old_ms * 1e6), now=now)

    def observe(self, owner, payload=None, now=10.1, *, source_ok=True, **kwargs):
        if payload is None:
            payload = self.payload(owner, now, **kwargs)
        return self.collector.observe(owner, payload, ros_now_ns=ros(now),
                                      received_at=now, source_ok=source_ok)

    def all(self, now=10.1):
        for owner in CLOSE_OWNERS:
            assert self.observe(owner, now=now)
        return self.assess(now + 0.001)

    def assess(self, now=10.101):
        return self.collector.assess(ros_now_ns=ros(now), now=now)


@pytest.mark.parametrize('goal', [GOAL, None])
def test_all_four_current_owner_acks_correlate_to_real_close_without_cleanup_authority(goal):
    case = Round(goal=goal)
    result = case.all()
    request = case.core.close_request
    assert result.ready is True and result.blockers == ()
    assert (result.session_id, result.nav_goal_uuid, result.close_generation) == (
        request.session_id, goal, request.generation)
    assert result.observed_at == pytest.approx(10.095)
    assert not isinstance(result, SessionCleanupEvidence)
    assert not hasattr(result, 'pump_off') and not hasattr(result, 'nav_inactive')
    assert case.core.phase == 'STOPPING' and case.core.close_request is request
    assert case.core.last_closed is None
    with pytest.raises(FrozenInstanceError):
        result.ready = False


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_missing_or_legacy_zero_count_is_not_an_owner_ack(owner):
    case = Round()
    for other in CLOSE_OWNERS:
        if other != owner:
            assert case.observe(other)
    result = case.assess()
    assert result.ready is False
    assert owner + ':OWNER_REPORT_NOT_RECEIVED' in result.blockers
    legacy = {'publisher_owned': False, 'released': True, 'raw_publishers': 0,
              'pump_publishers': 0, 'command': {'linear_x_m_s': 0}}
    assert not case.observe(owner, legacy, now=10.11)
    assert not case.assess(10.12).ready


@pytest.mark.parametrize('owner', CLOSE_OWNERS)
def test_owned_report_blocks_then_current_no_owner_can_recover_without_faking_destroy(owner):
    case = Round()
    assert case.all().ready
    assert case.observe(owner, now=10.12, owned=True)
    assert owner + ':OWNER_STILL_OWNED' in case.assess(10.121).blockers
    payload = case.payload(owner, 10.13, owned=False)
    assert payload['close_ack']['kind'] == 'NO_OWNER_AT_CLOSE'
    assert 'receipt' not in payload
    assert case.observe(owner, payload, now=10.13)
    assert case.assess(10.131).ready


def test_reporter_has_no_ack_in_idle_active_owned_or_expired_context():
    case = Round(begin=False)
    owner = CLOSE_OWNERS[0]
    assert case.payload(owner, 9.8, phase='IDLE')['close_ack'] is None
    assert case.payload(owner, 9.9, phase='ACTIVE')['close_ack'] is None
    case.begin()
    payload = case.payload(owner, 10.1, owned=True)
    assert payload['close_ack'] is None
    # Source/cache age is not refreshed by creating another outgoing report.
    payload = case.reporters[owner].report(case.contexts[owner], publisher_owned=False,
                                          stamp_ns=ros(10.5), now=10.5)
    assert payload['source'] is not None and payload['close_ack'] is None


def test_release_before_close_uses_a_new_no_owner_observation_and_preserves_old_receipt():
    case = Round()
    lease = MotionReleaseProof(owner='live_nav_stop_gateway')
    lease.created(NavigationBinding(GOAL, case.core.source_instance_id, 1, ros(8.8)), 8.9)
    lease.twist_published((0.0,) * 6, 9.0)
    lease.released(checked_at=9.6, released_at=9.61, odom_at=9.59,
                   cmd_at=9.59, stationary_since=9.0)
    receipt = deepcopy(lease.receipt)
    assert receipt['session_id'] is receipt['close_generation'] is None
    assert case.all().ready
    assert lease.receipt == receipt
    assert lease.receipt['session_id'] is lease.receipt['close_generation'] is None
    assert case.core.phase == 'STOPPING'


@pytest.mark.parametrize('field,value', [
    ('schema_version', True), ('schema_version', 2), ('mode', 'legacy'),
    ('owner', 'foreign'), ('actuation_authorized', True),
    ('publisher_owned', 0), ('publisher_owned', None),
    ('source_blocker', 'CLOCK_INVALID'),
])
def test_bad_envelope_invalidates_a_previously_ready_cache(field, value):
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    payload[field] = value
    assert not case.observe(owner, payload, now=10.12)
    assert not case.assess(10.121).ready


@pytest.mark.parametrize('path,value', [
    (('instance_id',), '0' * 32), (('instance_id',), 'AB' * 16),
    (('sequence',), True), (('sequence',), 0),
    (('stamp', 'sec'), True), (('stamp', 'sec'), -1),
    (('stamp', 'nanosec'), True), (('stamp', 'nanosec'), 1_000_000_000),
])
def test_source_metadata_must_have_canonical_typed_fields(path, value):
    case = Round()
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner)
    target = payload['source']
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    assert not case.observe(owner, payload)
    assert owner + ':OWNER_SOURCE_INVALID' in case.assess().blockers


@pytest.mark.parametrize('change', ['kind', 'session', 'namespace', 'goal', 'generation',
                                    'reason', 'supervisor_source', 'owned'])
def test_malformed_close_ack_is_not_a_release_or_no_owner_proof(change):
    case = Round()
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner)
    ack = payload['close_ack']
    if change == 'kind':
        ack['kind'] = 'RELEASED'
    elif change == 'session':
        ack['session_id'] = f'fire:{case.core.source_instance_id}:01'
    elif change == 'namespace':
        ack['session_id'] = 'fire:' + 'ef' * 16 + ':1'
    elif change == 'goal':
        ack['nav_goal_uuid'] = '0' * 32
    elif change == 'generation':
        ack['close_generation'] = True
    elif change == 'reason':
        ack['close_reason'] = 'CAMERA_STALE'
    elif change == 'supervisor_source':
        ack['supervisor_source'].pop('sequence')
    else:
        payload['publisher_owned'] = True
    assert not case.observe(owner, payload)
    assert owner + ':OWNER_CLOSE_ACK_INVALID' in case.assess().blockers


@pytest.mark.parametrize('field,value', [
    ('nav_goal_uuid', 'ef' * 16), ('nav_goal_uuid', None),
    ('close_generation', 2), ('close_reason', 'SAFETY_ABORT'),
])
def test_valid_but_wrong_close_context_cannot_join_the_current_round(field, value):
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    payload['close_ack'][field] = value
    assert case.observe(owner, payload, now=10.12)
    assert owner + ':OWNER_CLOSE_CONTEXT_MISMATCH' in case.assess(10.121).blockers


def test_other_valid_session_and_supervisor_cannot_be_relabelled_as_this_close():
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    ack = payload['close_ack']
    ack['session_id'] = 'fire:' + 'ef' * 16 + ':1'
    ack['supervisor_source']['instance_id'] = 'ef' * 16
    assert case.observe(owner, payload, now=10.12)
    assert owner + ':OWNER_CLOSE_CONTEXT_MISMATCH' in case.assess(10.121).blockers


@pytest.mark.parametrize('offset,reason', [
    (-351_000_000, 'OWNER_SOURCE_STALE'), (21_000_000, 'OWNER_SOURCE_IN_FUTURE'),
])
def test_old_or_future_source_is_not_renewed_by_a_current_receipt(offset, reason):
    case = Round()
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner)
    sec, ns = divmod(ros(10.1) + offset, 1_000_000_000)
    payload['source']['stamp'] = {'sec': sec, 'nanosec': ns}
    assert not case.observe(owner, payload)
    assert owner + ':' + reason in case.assess().blockers


def test_source_age_and_local_cache_age_add_up_after_admission():
    case = Round()
    owner = CLOSE_OWNERS[0]
    for other in CLOSE_OWNERS[1:]:
        assert case.observe(other, now=10.4)
    payload = case.payload(owner, 10.4, old_ms=300)
    assert case.observe(owner, payload, now=10.4)
    assert case.assess(10.41).ready
    assert owner + ':OWNER_SOURCE_STALE' in case.assess(10.46).blockers


@pytest.mark.parametrize('change,reason', [
    ('stamp', 'OWNER_STAMP_NON_INCREASING'),
    ('sequence', 'OWNER_SEQUENCE_NON_INCREASING'),
])
def test_replayed_or_reordered_owner_report_clears_current_proof(change, reason):
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    if change == 'stamp':
        payload['source']['stamp'] = {'sec': 110, 'nanosec': 95_000_000}
    else:
        payload['source']['sequence'] = 1
    assert not case.observe(owner, payload, now=10.12)
    assert owner + ':' + reason in case.assess(10.121).blockers


def test_inner_supervisor_age_cannot_be_hidden_by_a_new_outer_owner_stamp():
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    sec, ns = divmod(ros(9.7), 1_000_000_000)
    payload['close_ack']['supervisor_source']['stamp'] = {'sec': sec, 'nanosec': ns}
    assert not case.observe(owner, payload, now=10.12)
    assert owner + ':OWNER_SUPERVISOR_SOURCE_STALE_OR_FUTURE' in case.assess(10.121).blockers


def test_inner_source_age_keeps_aging_if_ros_clock_is_frozen_after_receipt():
    case = Round()
    for owner in CLOSE_OWNERS[1:]:
        assert case.observe(owner, now=10.4)
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.4)
    sec, ns = divmod(ros(10.1), 1_000_000_000)
    payload['close_ack']['supervisor_source']['stamp'] = {'sec': sec, 'nanosec': ns}
    assert case.observe(owner, payload, now=10.4)
    result = case.collector.assess(ros_now_ns=ros(10.4), now=10.41)
    assert result.ready
    result = case.collector.assess(ros_now_ns=ros(10.4), now=10.46)
    assert owner + ':OWNER_SUPERVISOR_SOURCE_STALE_OR_FUTURE' in result.blockers


@pytest.mark.parametrize('now', [10.01, 10.02, 10.025])
def test_owner_snapshot_before_close_skew_margin_does_not_satisfy_barrier(now):
    case = Round()
    owner = CLOSE_OWNERS[0]
    assert case.observe(owner, now=now)
    # Header is 5ms before receipt; stamp <= announced +20ms must reject.
    assert owner + ':OWNER_OBSERVATION_NOT_AFTER_CLOSE' in case.assess(now + 0.001).blockers


def test_close_duplicate_does_not_re_date_barrier_clear_cache_or_mutate_request():
    case = Round()
    assert case.all().ready
    request = case.core.close_request
    assert case.collector.begin_close(request, announced_ros_ns=ros(10.11), announced_at=10.11)
    assert case.collector.request is request
    assert case.collector.announced_ros_ns == ros(10) and case.collector.announced_at == 10
    assert case.assess(10.12).ready


@pytest.mark.parametrize('change', [{'generation': 2}, {'reason': 'SAFETY_ABORT'},
                                    {'requested_at': 10.1}])
def test_changed_pending_close_cannot_rebaseline_a_ready_round(change):
    case = Round()
    assert case.all().ready
    request = case.core.close_request
    assert not case.collector.begin_close(replace(request, **change),
                                          announced_ros_ns=ros(10.2), announced_at=10.2)
    assert case.collector.request is request
    assert case.assess(10.21).blockers == ('OWNER_CLOSE_ROUND_CHANGED',)
    assert not case.collector.begin_close(request, announced_ros_ns=ros(10.3), announced_at=10.3)


@pytest.mark.parametrize('baseline', [False, True])
def test_owner_process_restart_during_close_is_latched_even_if_old_source_returns(baseline):
    case = Round(begin=not baseline)
    owner = CLOSE_OWNERS[0]
    if baseline:
        assert case.observe(owner, now=9.9, phase='ACTIVE', owned=True)
        case.begin()
    else:
        assert case.all().ready
    original = case.reporters[owner]
    case.reporters[owner] = OwnerCloseReporter(owner)
    assert not case.observe(owner, now=10.12)
    case.reporters[owner] = original
    assert not case.observe(owner, now=10.13)
    assert case.assess(10.14).blockers == ('OWNER_PROCESS_CHANGED:' + owner,)


@pytest.mark.parametrize('clock', ['ros', 'monotonic'])
def test_clock_rollback_revokes_all_cached_proofs_and_cannot_auto_resume(clock):
    case = Round()
    assert case.all().ready
    result = case.collector.assess(
        ros_now_ns=ros(10.09 if clock == 'ros' else 10.12),
        now=10.12 if clock == 'ros' else 10.09)
    assert result.blockers == ('OWNER_COLLECTOR_CLOCK_BACKWARD',)
    for owner in CLOSE_OWNERS:
        assert not case.observe(owner, now=10.13)
    assert not case.assess(10.14).ready


def test_ros_clock_forward_jump_also_expires_a_cached_proof():
    case = Round()
    assert case.all().ready
    result = case.collector.assess(ros_now_ns=ros(20), now=10.12)
    assert not result.ready
    assert all('OWNER_SOURCE_STALE' in blocker for blocker in result.blockers)


def test_wrong_endpoint_revokes_cache_then_fresh_same_process_can_be_observed_again():
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    assert not case.observe(owner, payload, now=10.12, source_ok=False)
    assert owner + ':OWNER_PUBLISHER_INVALID' in case.assess(10.121).blockers
    assert case.observe(owner, now=10.13)
    assert case.assess(10.131).ready


def test_mutating_input_and_returned_reports_cannot_relabel_cached_observations():
    case = Round()
    assert case.all().ready
    owner = CLOSE_OWNERS[0]
    payload = case.payload(owner, 10.12)
    assert case.observe(owner, payload, now=10.12)
    payload['close_ack']['session_id'] = 'changed'
    payload['close_ack']['supervisor_source']['instance_id'] = 'changed'
    payload['source']['instance_id'] = 'changed'
    assert case.assess(10.121).ready
    # Producer report contains a copy, not the guard's mutable source object.
    assert case.contexts[owner].current(10.121)['source']['instance_id'] == case.core.source_instance_id


@pytest.mark.parametrize('stamp', [None, True, 0, -1])
def test_reporter_invalid_clock_has_no_usable_source_or_ack(stamp):
    case = Round()
    owner = CLOSE_OWNERS[0]
    reporter = case.reporters[owner]
    payload = reporter.report(case.contexts[owner], publisher_owned=False, stamp_ns=stamp, now=10.1)
    assert payload['source'] is payload['close_ack'] is None
    assert payload['source_blocker'] == 'OWNER_REPORT_CLOCK_INVALID'
    assert reporter.sequence == 0


def test_reporter_does_not_fabricate_stamp_progress_or_change_instance_on_bad_clock():
    case = Round()
    owner = CLOSE_OWNERS[0]
    first = case.payload(owner)
    reporter = case.reporters[owner]
    for stamp in (ros(10.095), ros(10.09)):
        payload = reporter.report(case.contexts[owner], publisher_owned=False,
                                  stamp_ns=stamp, now=10.12)
        assert payload['source'] is payload['close_ack'] is None
        assert payload['source_blocker'] == 'OWNER_REPORT_STAMP_NON_INCREASING'
    assert reporter.instance_id == first['source']['instance_id'] and reporter.sequence == 1


@pytest.mark.parametrize('bad_ros,bad_mono', [(True, 10.2), (None, 10.2),
                                           (ros(10.2), float('nan')), (ros(10.2), -1)])
def test_invalid_collector_clock_cannot_keep_a_previous_ready_result(bad_ros, bad_mono):
    case = Round()
    assert case.all().ready
    result = case.collector.assess(ros_now_ns=bad_ros, now=bad_mono)
    assert result.ready is False and result.observed_at is None
    assert result.blockers == ('OWNER_COLLECTOR_CLOCK_INVALID',)
    assert not case.assess(10.3).ready


@pytest.mark.parametrize('kwargs', [{'timeout_s': 0.36}, {'timeout_s': True},
                                   {'timeout_s': float('nan')}, {'future_skew_ms': 21},
                                   {'future_skew_ms': True}, {'future_skew_ms': -1}])
def test_configuration_cannot_relax_source_age_or_clock_skew(kwargs):
    with pytest.raises(ValueError):
        CloseOwnerEvidenceCollector(**kwargs)


@pytest.mark.parametrize('owner', ['foreign', None, True])
def test_unknown_owner_is_not_an_extra_slot_or_supported_reporter(owner):
    with pytest.raises(ValueError):
        OwnerCloseReporter(owner)
    with pytest.raises(ValueError):
        CloseOwnerEvidenceCollector().observe(owner, {}, ros_now_ns=ros(10),
                                              received_at=10, source_ok=True)


def test_collector_needs_a_real_valid_close_request_and_post_request_announcement():
    collector = CloseOwnerEvidenceCollector()
    assert collector.assess(ros_now_ns=ros(9), now=9).blockers == ('OWNER_CLOSE_ROUND_NOT_STARTED',)
    assert not collector.begin_close(None, announced_ros_ns=ros(10), announced_at=10)
    request = SessionCloseRequest('fire:' + 'cd' * 16 + ':1', GOAL, 1, 'OPERATOR_CANCEL', 10)
    assert not collector.begin_close(request, announced_ros_ns=ros(9.9), announced_at=9.9)
    assert collector.request is None
    assert collector.begin_close(request, announced_ros_ns=ros(10), announced_at=10)


@pytest.mark.parametrize('change', [{'session_id': 'not-a-session'},
                                    {'nav_goal_uuid': '0' * 32}, {'generation': True},
                                    {'generation': 0}, {'reason': 'UNKNOWN'},
                                    {'requested_at': float('nan')}])
def test_malformed_close_request_never_starts_a_round(change):
    collector = CloseOwnerEvidenceCollector()
    request = SessionCloseRequest('fire:' + 'cd' * 16 + ':1', GOAL, 1, 'OPERATOR_CANCEL', 10)
    assert not collector.begin_close(replace(request, **change),
                                     announced_ros_ns=ros(10), announced_at=10)
    assert collector.request is None
    assert not collector.assess(ros_now_ns=ros(10.1), now=10.1).ready


@pytest.mark.parametrize('ownership', [None, 0, 1, 'false'])
def test_reporter_requires_actual_boolean_ownership_not_truthy_or_legacy_values(ownership):
    reporter = OwnerCloseReporter(CLOSE_OWNERS[0])
    with pytest.raises(ValueError):
        reporter.report(None, publisher_owned=ownership, stamp_ns=ros(10), now=10)
