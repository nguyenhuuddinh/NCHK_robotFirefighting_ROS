"""Coherent global stop proof, simulated source/ROS/monotonic time only."""

from dataclasses import FrozenInstanceError, replace

import pytest

from fire_robot_mission.close_owner_evidence import CloseOwnersReadiness
from fire_robot_mission.fire_session_core import FireSessionCore, SessionCleanupEvidence
from fire_robot_mission.global_stop_evidence import GlobalStopEvidence, STOP_SOURCES


GOAL, OTHER = 'ab' * 16, 'cd' * 16
ORIGIN = 100_000_000_000
ZERO = (0.0,) * 6
SOURCES = {role: bytes([index + 1]) * 24 for index, role in enumerate(STOP_SOURCES)}


def ros(now):
    return ORIGIN + round(now * 1e9)


def actuator(now, seq, *, epoch=1, pump=False, servo=False, phase='IDLE', age_ms=0):
    stamp = ros(now) - round(age_ms * 1e6)
    sec, ns = divmod(stamp, 1_000_000_000)
    return {'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
            'physical_pump_or_nozzle_verified': False,
            'pump_pwm_commanded_on': pump, 'servo_pwm_commanded_on': servo,
            'servo_phase': phase, 'esp_ms': 10000 + seq, 'receive_age_ms': age_ms,
            'source': {'bridge_instance_id': 'ef' * 16, 'serial_epoch': epoch,
                       'frame_seq': seq, 'stamp': {'sec': sec, 'nanosec': ns}}}


class Case:
    def __init__(self, *, goal=GOAL, reason='OPERATOR_CANCEL', begin=True):
        self.core = FireSessionCore(proof_timeout_s=0.35)
        self.identity = self.core.begin(goal)
        self.core.request_close(self.identity.session_id, reason, 10.0)
        self.stop = GlobalStopEvidence()
        self.seq, self.now = 0, 10.0
        if begin:
            self.begin()

    def begin(self):
        assert self.stop.begin_close(self.core.close_request, announced_ros_ns=ros(10.0), announced_at=10.0)

    def owners(self, now=None):
        now = self.now if now is None else now
        req = self.core.close_request
        return CloseOwnersReadiness(req.session_id, req.nav_goal_uuid, req.generation,
                                    True, now - 0.005, ())

    def graph(self, *, now=None, raw=0, pump=0, sources=None, lag=0.0):
        now = self.now if now is None else now
        self.stop.observe_graph(raw_count=raw, pump_count=pump,
                                sources=dict(SOURCES) if sources is None else sources,
                                started_ros_ns=ros(now - lag), started_at=now - lag,
                                ros_ns=ros(now), now=now)

    def feed(self, now, *, odom=ZERO, cmd=ZERO, statuses=None, act=None, graph=True):
        self.now, self.seq = now, self.seq + 1
        for role, values in [('odom', odom), ('cmd', cmd)]:
            self.stop.observe_motion(role, values, stamp_ns=ros(now), source_id=SOURCES[role],
                                     ros_ns=ros(now), now=now)
        self.stop.observe_status(
            ([] if self.identity.nav_goal_uuid is None else [(self.identity.nav_goal_uuid, 5)])
            if statuses is None else statuses,
            stamp_ns=ros(now), source_id=SOURCES['nav_status'], ros_ns=ros(now), now=now)
        self.stop.observe_actuator(actuator(now, self.seq) if act is None else act,
                                   source_id=SOURCES['actuator'], ros_ns=ros(now), now=now)
        if graph:
            self.graph()

    def ready(self):
        for now in (10.1, 10.2, 10.3, 10.4, 10.5, 10.61):
            self.feed(now)
        return self.assess()

    def assess(self, now=None, owners=None):
        now = self.now if now is None else now
        return self.stop.assess(self.owners(now) if owners is None else owners,
                                ros_ns=ros(now), now=now)


@pytest.mark.parametrize('goal,status', [(GOAL, 4), (GOAL, 5), (GOAL, 6), (None, None)])
def test_full_coherent_stop_snapshot_returns_typed_evidence_without_closing(goal, status):
    case = Case(goal=goal)
    assert case.ready().ready
    case.feed(10.7, statuses=[] if goal is None else [(GOAL, status)])
    result = case.assess()
    assert result.ready and result.blockers == ()
    evidence = result.evidence
    assert isinstance(evidence, SessionCleanupEvidence)
    assert (evidence.session_id, evidence.nav_goal_uuid, evidence.close_generation) == (
        case.identity.session_id, goal, case.core.close_request.generation)
    assert evidence.nav_terminal_status == status
    assert evidence.observed_at == pytest.approx(10.695)
    assert all(getattr(evidence, field) is True for field in (
        'nav_inactive', 'motion_owner_released', 'pump_owner_released',
        'pump_off', 'servo_idle', 'stationary', 'cmd_zero'))
    assert case.core.phase == 'STOPPING' and case.core.last_closed is None
    with pytest.raises(FrozenInstanceError):
        evidence.cmd_zero = False


@pytest.mark.parametrize('role', ['odom', 'cmd'])
@pytest.mark.parametrize('axis', range(6))
def test_each_axis_motion_or_nonzero_command_resets_hold(role, axis):
    case = Case()
    assert case.ready().ready
    values = list(ZERO)
    values[axis] = 0.2 if role == 'odom' else 0.000001
    case.feed(10.7, **{role: values})
    assert not case.assess().ready and case.stop._hold_since is None
    for now in (10.8, 10.9, 11.0, 11.1, 11.2):
        case.feed(now)
    assert not case.assess().ready
    case.feed(11.31)
    assert case.assess().ready


@pytest.mark.parametrize('role', ['odom', 'cmd'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, None, '0'])
def test_invalid_motion_axis_invalidates_previous_good_sample(role, value):
    case = Case()
    assert case.ready().ready
    axes = list(ZERO)
    axes[2] = value
    case.feed(10.7, **{role: axes})
    assert case.stop.samples[role] is None
    assert not case.assess().ready


@pytest.mark.parametrize('role', ['odom', 'cmd', 'actuator', 'nav_status'])
def test_old_preclose_data_cannot_be_redated_at_receipt(role):
    case = Case()
    assert case.ready().ready
    now = 10.7
    if role in ('odom', 'cmd'):
        case.stop.observe_motion(role, ZERO, stamp_ns=ros(9.9), source_id=SOURCES[role], ros_ns=ros(now), now=now)
    elif role == 'actuator':
        case.stop.observe_actuator(actuator(now, 100, age_ms=800), source_id=SOURCES[role], ros_ns=ros(now), now=now)
    else:
        case.stop.observe_status([(GOAL, 5)], stamp_ns=ros(9.9), source_id=SOURCES[role], ros_ns=ros(now), now=now)
    case.now = now
    assert case.stop.samples[role] is None and not case.assess().ready


def test_same_zero_samples_cannot_manufacture_hold_by_timer_ticks_or_cover_outage():
    case = Case()
    case.feed(10.1)
    assert not case.assess(10.3).ready
    assert not case.assess(10.7).ready
    # A gap resets the hold even when data then resumes as zeros.
    case.feed(10.8)
    assert not case.assess().ready
    assert case.stop._hold_since == pytest.approx(10.8)


@pytest.mark.parametrize('role', ['odom', 'cmd', 'nav_status'])
@pytest.mark.parametrize('stamp', [None, 0, True, -1])
def test_missing_or_invalid_source_stamp_is_not_replaced_by_receive_time(role, stamp):
    case = Case()
    case.feed(10.1)
    if role == 'nav_status':
        case.stop.observe_status([(GOAL, 5)], stamp_ns=stamp, source_id=SOURCES[role], ros_ns=ros(10.2), now=10.2)
    else:
        case.stop.observe_motion(role, ZERO, stamp_ns=stamp, source_id=SOURCES[role], ros_ns=ros(10.2), now=10.2)
    assert case.stop.samples[role] is None


@pytest.mark.parametrize('role', ['odom', 'cmd', 'actuator', 'nav_status'])
def test_duplicate_stamp_clears_current_proof(role):
    case = Case()
    assert case.ready().ready
    if role in ('odom', 'cmd'):
        case.stop.observe_motion(role, ZERO, stamp_ns=ros(case.now), source_id=SOURCES[role], ros_ns=ros(case.now), now=case.now)
    elif role == 'actuator':
        case.stop.observe_actuator(actuator(case.now, case.seq), source_id=SOURCES[role], ros_ns=ros(case.now), now=case.now)
    else:
        case.stop.observe_status([(GOAL, 5)], stamp_ns=ros(case.now), source_id=SOURCES[role], ros_ns=ros(case.now), now=case.now)
    assert case.stop.samples[role] is None and not case.assess().ready


@pytest.mark.parametrize('role', STOP_SOURCES)
def test_every_source_process_pin_blocks_midround_replacement(role):
    case = Case()
    assert case.ready().ready
    sources = dict(SOURCES)
    sources[role] = b'\xff' * 24
    case.now = 10.7
    case.graph(sources=sources)
    assert case.stop.fault == 'GLOBAL_SOURCE_CHANGED:' + role
    case.feed(10.8)
    assert not case.assess().ready  # Original source cannot silently recover the round.


@pytest.mark.parametrize('raw,pump', [(1, 0), (0, 1), (2, 2), (True, 0), (-1, 0)])
def test_global_graph_requires_strict_zero_counts_and_loses_old_snapshot(raw, pump):
    case = Case()
    assert case.ready().ready
    case.now = 10.7
    case.graph(raw=raw, pump=pump)
    assert case.stop.graph is None and not case.assess().ready


@pytest.mark.parametrize('lag', [0.351, 1.0])
def test_slow_graph_work_is_not_stamped_as_a_new_snapshot(lag):
    case = Case()
    assert case.ready().ready
    case.now = 11.0
    case.graph(lag=lag)
    assert case.stop.graph is None and not case.assess().ready


def test_missing_source_does_not_mean_role_is_safely_unlaunched():
    case = Case()
    assert case.ready().ready
    sources = dict(SOURCES)
    sources['pump_demo_controller'] = None
    case.now = 10.7
    case.graph(sources=sources)
    assert not case.assess().ready


@pytest.mark.parametrize('statuses', [
    [(OTHER, 5)], [(GOAL, 5), (OTHER, 2)], [(GOAL, 3)], [(GOAL, 0)],
    [(GOAL, 5), (GOAL, 5)], [('00' * 16, 5)], [(GOAL, True)], None,
])
def test_wrong_or_ambiguous_navigation_is_not_exact_terminal_inactivity(statuses):
    case = Case()
    assert case.ready().ready
    case.stop.observe_status(statuses, stamp_ns=ros(10.7), source_id=SOURCES['nav_status'], ros_ns=ros(10.7), now=10.7)
    case.now = 10.7
    assert not case.assess().ready


def test_no_goal_still_requires_a_fresh_inactive_status_array():
    case = Case(goal=None)
    assert case.ready().ready
    case.feed(10.7, statuses=[(OTHER, 1)])
    assert not case.assess().ready


@pytest.mark.parametrize('pump,servo,phase', [(True, False, 'IDLE'), (False, True, 'IDLE'), (False, False, 'RETURNING_HOME')])
def test_command_telemetry_must_be_off_and_idle(pump, servo, phase):
    case = Case()
    assert case.ready().ready
    case.feed(10.7, act=actuator(10.7, 100, pump=pump, servo=servo, phase=phase))
    assert 'GLOBAL_ACTUATOR_NOT_OFF_IDLE' in case.assess().blockers


def test_serial_epoch_change_latches_without_resetting_any_owner_or_quota():
    case = Case()
    assert case.ready().ready
    case.feed(10.7, act=actuator(10.7, 100, epoch=2))
    assert case.stop.fault == 'GLOBAL_ACTUATOR_CONTEXT_CHANGED'
    assert not case.assess().ready
    assert case.core.phase == 'STOPPING'


def test_reported_actuator_age_counts_toward_postclose_barrier_and_cache_expiry():
    case = Case()
    case.feed(10.1, act=actuator(10.1, 1, age_ms=200))
    assert case.stop._fresh('actuator', ros(10.1), 10.1) == 'NOT_AFTER_CLOSE'
    case.feed(10.4, act=actuator(10.4, 2, age_ms=300))
    assert case.stop.samples['actuator'].effective_at == pytest.approx(10.1)
    assert case.stop._fresh('actuator', ros(10.46), 10.46) == 'SOURCE_STALE_OR_FUTURE'


def test_mutating_received_payload_does_not_change_retained_stop_state():
    case = Case()
    assert case.ready().ready
    payload = actuator(10.7, 100, pump=True)
    case.stop.observe_actuator(payload, source_id=SOURCES['actuator'], ros_ns=ros(10.7), now=10.7)
    payload['pump_pwm_commanded_on'] = False
    assert case.stop.samples['actuator'].value[0] is True
    assert not case.assess(10.7).ready


@pytest.mark.parametrize('change', [
    {'ready': False}, {'session_id': 'fire:' + 'ef' * 16 + ':1'},
    {'nav_goal_uuid': OTHER}, {'close_generation': 2}, {'observed_at': 9.9},
    {'observed_at': 10.1}, {'blockers': ('OWNER_MISSING',)},
])
def test_owner_readiness_must_match_same_close_and_still_be_fresh(change):
    case = Case()
    assert case.ready().ready
    result = case.assess(owners=replace(case.owners(), **change))
    assert 'GLOBAL_OWNER_ACKS_NOT_CURRENT' in result.blockers and result.evidence is None


def test_close_round_and_barrier_cannot_be_redated_or_replaced():
    case = Case()
    request = case.stop.request
    assert case.stop.begin_close(request, announced_ros_ns=ros(100), announced_at=100)
    assert (case.stop.barrier_ros_ns, case.stop.barrier_at) == (ros(10), 10)
    assert not case.stop.begin_close(replace(request, generation=2), announced_ros_ns=ros(10.1), announced_at=10.1)
    assert case.stop.request is request and case.stop.fault == 'GLOBAL_CLOSE_ROUND_CHANGED'


@pytest.mark.parametrize('ros_ns,now', [(ros(10.0), 10.7), (ros(20), 10.7), (ros(10.7), 9.0), (0, 10.7), (ros(10.7), None)])
def test_clock_faults_are_sticky_and_never_emit_evidence(ros_ns, now):
    case = Case()
    assert case.ready().ready
    result = case.stop.assess(case.owners(), ros_ns=ros_ns, now=now)
    assert not result.ready and result.evidence is None and case.stop.fault is not None
    assert not case.assess(20).ready


def test_preclose_samples_and_future_skew_cannot_cross_the_close_barrier():
    case = Case(begin=False)
    case.feed(9.99, graph=False)
    case.begin()
    # A sample stamped just 10ms ahead may have been produced before close.
    case.stop.observe_motion('cmd', ZERO, stamp_ns=ros(10.01), source_id=SOURCES['cmd'], ros_ns=ros(10.0), now=10.0)
    assert case.stop._fresh('cmd', ros(10.0), 10.0) == 'NOT_AFTER_CLOSE'
    assert not case.assess(10.0).ready


def test_post_spray_stop_evidence_does_not_invent_review_clear_proof():
    case = Case(reason='POST_SPRAY_CLEAR')
    result = case.ready()
    assert result.ready and result.evidence.post_spray_clear is False
    assert not case.core.finish_close(result.evidence, case.now)
    assert case.core.blocker == 'POST_SPRAY_REVIEW_MISSING'


@pytest.mark.parametrize('limits', [
    {'linear_limit': .03}, {'angular_limit': .06}, {'linear_limit': True},
    {'angular_limit': 0}, {'timeout_s': .36}, {'timeout_s': True}, {'future_skew_ms': 21},
])
def test_configuration_cannot_weaken_stationary_limits(limits):
    with pytest.raises(ValueError):
        GlobalStopEvidence(**limits)
