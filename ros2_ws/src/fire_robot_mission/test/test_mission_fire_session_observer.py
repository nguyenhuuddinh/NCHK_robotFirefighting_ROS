"""Opt-in session telemetry must not change the existing control contract."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from fire_robot_mission import mission_supervisor
from fire_robot_mission.mission_supervisor import MissionSupervisorCore


GOAL = 'ab' * 16
NEXT_GOAL = 'cd' * 16


@pytest.mark.parametrize('enabled', [False, True])
def test_node_parameter_reaches_core_without_ros_initialization(enabled):
    class ParameterReader:
        _declare_parameters = mission_supervisor.FireMissionSupervisor._declare_parameters
        _read_parameters = mission_supervisor.FireMissionSupervisor._read_parameters

        def __init__(self):
            self.values = {}

        def declare_parameter(self, name, default):
            self.values[name] = default

        def get_parameter(self, name):
            return SimpleNamespace(value=self.values[name])

    node = ParameterReader()
    node._declare_parameters()
    assert node.values['enable_fire_session_observer'] is False
    node.values['enable_fire_session_observer'] = enabled
    node._read_parameters()
    core = MissionSupervisorCore(**node._core_parameters)
    assert (core._fire_session is not None) is enabled


def test_packaged_profile_keeps_observer_off():
    config = Path(__file__).resolve().parents[1] / 'config' / 'mission_params.yaml'
    params = yaml.safe_load(config.read_text())['fire_mission_supervisor']['ros__parameters']
    assert params['enable_fire_session_observer'] is False


def feed(core, now, present=True, age_ms=10.0):
    core.update_detection({
        'observe_only': True, 'state': 'ok', 'source': {'age_ms': age_ms},
        'fire': {'detected_now': present, 'confirmed': present},
    }, now)
    core.update_alignment({
        'observe_only': True, 'state': 'ALIGNING_LEFT',
        'preview': {'direction': 'LEFT', 'target_angular_z_rad_s': 0.3},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }, now)
    core.update_odom(0.0, 0.0, now)


def acquire(core, goal=GOAL):
    for now in (0.0, 0.11):
        core.update_goal_statuses([] if goal is None else [(goal, 2)], now)
        feed(core, now)
        result = core.evaluate(now)
    return result['lifecycle']['fire_session_observer']


def observer(core, now):
    return core.evaluate(now)['lifecycle']['fire_session_observer']


def test_off_does_not_construct_or_evaluate_a_session_core(monkeypatch):
    def unexpected(**kwargs):
        raise AssertionError('observer OFF must not construct a session core')

    monkeypatch.setattr(mission_supervisor, 'FireSessionCore', unexpected)
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    for now in (0.0, 0.11):
        feed(core, now)
        result = core.evaluate(now)
        assert 'fire_session_observer' not in result['lifecycle']
    assert core._fire_session is None


@pytest.mark.parametrize('goal', [None, GOAL])
def test_fresh_acquisition_records_real_binding_or_none_and_is_idempotent(goal):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    core.update_goal_statuses([] if goal is None else [(goal, 2)], 0.0)
    feed(core, 0.0)
    assert observer(core, 0.0)['phase'] == 'IDLE'
    # A repeated evaluation of the same frame is not another observation.
    assert observer(core, 0.1)['phase'] == 'IDLE'
    core.update_goal_statuses([] if goal is None else [(goal, 2)], 0.11)
    feed(core, 0.11)
    started = observer(core, 0.11)
    assert started['phase'] == 'ACTIVE'
    assert started['nav_goal_uuid'] == goal
    assert started['session_id'].startswith('fire:')
    assert started['actuation_authorized'] is False
    assert observer(core, 0.12)['session_id'] == started['session_id']


@pytest.mark.parametrize('records,expected', [
    (None, 'NAV2_STATUS_NOT_CURRENT'),
    ([(None, 2)], 'NAV2_GOAL_BINDING_AMBIGUOUS'),
    ([(GOAL, 2), (NEXT_GOAL, 2)], 'NAV2_GOAL_BINDING_AMBIGUOUS'),
    ([(GOAL, 2), (None, 2)], 'NAV2_GOAL_BINDING_AMBIGUOUS'),
])
def test_unknown_or_ambiguous_goal_cannot_be_relabelled_as_no_goal(records, expected):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    for now in (0.0, 0.11):
        if records is not None:
            core.update_goal_statuses(records, now)
        feed(core, now)
        state = observer(core, now)
    assert state['phase'] == 'IDLE'
    assert state['session_id'] is None
    assert state['blocker'] == expected


def test_old_nav_status_cannot_start_a_session():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    core.update_goal_statuses([], 0.0)
    for now in (1.0, 1.11):
        feed(core, now)
        state = observer(core, now)
    assert state['phase'] == 'IDLE'
    assert state['blocker'] == 'NAV2_STATUS_NOT_CURRENT'


def test_stale_detection_requests_safety_close_without_claiming_absence_or_reopening():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    started = acquire(core)
    expired = observer(core, 0.42)
    assert expired['phase'] == 'STOPPING'
    assert expired['close_reason'] == 'SAFETY_ABORT'
    assert expired['blocker'] == 'VERIFIED_SESSION_CLEANUP_NOT_CONNECTED'
    for now in (0.43, 0.55):
        core.update_goal_statuses([(NEXT_GOAL, 2)], now)
        feed(core, now)
        still = observer(core, now)
        assert still['phase'] == 'STOPPING'
        assert still['session_id'] == started['session_id']
        assert still['nav_goal_uuid'] == GOAL
        assert still['close_reason'] == 'SAFETY_ABORT'


@pytest.mark.parametrize('status,phase,reason', [
    (4, 'ACTIVE', None), (5, 'ACTIVE', None),
    (6, 'STOPPING', 'NAVIGATION_ABORTED'),
])
def test_exact_goal_cancel_is_not_mislabelled_as_operator_cancel(status, phase, reason):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    acquire(core)
    core.update_goal_statuses([(GOAL, status)], 0.12)
    feed(core, 0.12)
    result = observer(core, 0.12)
    assert result['phase'] == phase
    assert result['close_reason'] == reason


@pytest.mark.parametrize('bound_goal', [None, GOAL])
def test_new_goal_cannot_rebind_an_existing_session(bound_goal):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    started = acquire(core, bound_goal)
    core.update_goal_statuses([(NEXT_GOAL, 2)], 0.12)
    feed(core, 0.12)
    changed = observer(core, 0.12)
    assert changed['session_id'] == started['session_id']
    assert changed['nav_goal_uuid'] == bound_goal
    assert changed['phase'] == 'STOPPING'
    assert changed['close_reason'] == 'SAFETY_ABORT'


def test_absent_frame_and_zero_commands_are_not_release_or_absence_proof():
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    started = acquire(core, None)
    for now in (0.12, 0.2):
        core.update_goal_statuses([], now)
        feed(core, now, present=False)
        core.update_raw_command(0.0, 0.0, now)
        core.update_pump_owner_count(0)
        state = observer(core, now)
        assert state['phase'] == 'ACTIVE'
        assert state['session_id'] == started['session_id']
        assert state['close_reason'] is None
        assert state['blocker'] == 'TARGET_ABSENCE_PROOF_NOT_CONNECTED'


@pytest.mark.parametrize('after_spray', [False, True])
def test_legacy_clear_requests_closure_but_cannot_fabricate_owner_release(after_spray):
    core = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                                 enable_fire_session_observer=True)
    started = acquire(core)
    core.update_goal_statuses([(GOAL, 5)], 0.12)
    off = {'schema_version': 1, 'mode': 'COMMAND_TELEMETRY_ONLY',
           'pump_pwm_commanded_on': False, 'servo_pwm_commanded_on': False,
           'servo_phase': 'IDLE'}
    if after_spray:
        core.update_pump({'state': 'FIRING', 'pump_on': True,
                          'goal_uuid': GOAL}, 0.12)
    for now in (0.12, 0.9, 1.7):
        feed(core, now, present=False)
        core.update_raw_command(0.0, 0.0, now)
        core.update_actuator(off, now)
        core.update_ir({
            'schema_version': 1, 'mode': 'OBSERVE_ONLY', 'observe_only': True,
            'zone': 'UNKNOWN', 'ir_band': 'NO_FLAME_SIGNATURE',
            'stable_samples': 5, 'ir_receive_age_ms': 20.0,
            'blockers': ['NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED'],
        }, now)
        if after_spray:
            core.update_review({
                'schema_version': 1, 'mode': 'SHADOW_ONLY',
                'state': 'NO_FIRE_EVIDENCE', 'goal_uuid': GOAL,
                'pulse_index': 1, 'blockers': [],
                'safety': {'motion_command_publisher': False,
                           'pump_command_publisher': False,
                           'nav2_action_client': False,
                           'actuation_authorized': False},
            }, now)
        result = core.evaluate(now)
    state = result['lifecycle']['fire_session_observer']
    assert result['state'] == 'IDLE'  # Legacy behavior is retained.
    assert state['phase'] == 'STOPPING'
    assert state['session_id'] == started['session_id']
    assert state['close_reason'] == (
        'POST_SPRAY_CLEAR' if after_spray else 'TARGET_ABSENT_CONFIRMED')
    assert state['blocker'] == 'VERIFIED_SESSION_CLEANUP_NOT_CONNECTED'
    assert core._fire_session.last_closed is None
    if after_spray:
        assert core._pump_started_uuid == GOAL  # Never reset treatment history.


@pytest.mark.parametrize('terminal_status', [4, 5, 6])
def test_on_off_replay_preserves_every_existing_output_field(terminal_status):
    off = MissionSupervisorCore(fire_acquire_hold_s=0.1)
    on = MissionSupervisorCore(fire_acquire_hold_s=0.1,
                               enable_fire_session_observer=True)
    for now, present, statuses in (
            (0.0, False, [(GOAL, 2)]),
            (0.1, True, [(GOAL, 2)]),
            (0.21, True, [(GOAL, 2)]),
            (0.22, True, [(GOAL, terminal_status)]),
            (0.23, False, [(GOAL, terminal_status)]),
            (0.3, False, [(NEXT_GOAL, 2)]),
            (0.4, True, [(NEXT_GOAL, 2)]),
            (0.51, True, [(NEXT_GOAL, 2)]),
            (0.9, None, [(NEXT_GOAL, 2)])):
        snapshots = []
        for core in (off, on):
            core.update_goal_statuses(statuses, now)
            if present is not None:
                feed(core, now, present=present)
            snapshots.append(deepcopy(core.evaluate(now)))
        snapshots[1]['lifecycle'].pop('fire_session_observer')
        assert snapshots[0] == snapshots[1]
