"""Unit tests for mission-preview report accumulation."""

from fire_robot_mission.mission_diagnostics import (
    MissionDiagnosticsAccumulator,
)


def payload(state, intent, comparison='NAV_NEUTRAL'):
    """Build one valid observe-only mission payload."""
    return {
        'observe_only': True,
        'state': state,
        'navigation': {'fire_nav_turn_comparison': comparison},
        'decision': {
            'intent': intent,
            'eligible_for_future_alignment_handoff': False,
            'readiness_blockers': ['ODOM_MISSING_OR_STALE'],
            'command_published': False,
        },
        'robot': {'odom_fresh': False, 'stationary': False},
        'health': {'status': 'OK', 'blockers': []},
        'safety': {
            'motion_command_publisher': False,
            'actuator_command_publisher': False,
            'nav2_action_client': False,
        },
    }


def test_accumulator_counts_transitions_and_contracts():
    accumulator = MissionDiagnosticsAccumulator()
    accumulator.add(
        payload('FIRE_TRACKING', 'LEFT_PREVIEW'), receive_time=1.0)
    accumulator.add(
        payload('TARGET_LOST', 'REQUEST_SAFE_STOP'), receive_time=1.1)
    report = accumulator.summary()
    assert report['messages'] == 2
    assert report['state_counts'] == {
        'FIRE_TRACKING': 1,
        'TARGET_LOST': 1,
    }
    assert report['transitions'] == {
        'FIRE_TRACKING->TARGET_LOST': 1}
    assert report['safe_stop_intent_violations'] == 0
    assert report['safety_contract_violations'] == 0
    assert report['eligible_frames'] == 0
    assert report['eligibility_counts'] == {'ineligible': 2}
    assert report['readiness_blocker_counts'] == {
        'ODOM_MISSING_OR_STALE': 2}
    assert report['odom_fresh_counts'] == {'false': 2}
    assert report['stationary_counts'] == {'false': 2}


def test_accumulator_flags_unsafe_payload():
    accumulator = MissionDiagnosticsAccumulator()
    unsafe = payload('FAULT', 'KEEP_NAV2')
    unsafe['observe_only'] = False
    unsafe['decision']['command_published'] = True
    unsafe['safety']['motion_command_publisher'] = True
    accumulator.add(unsafe, receive_time=2.0)
    report = accumulator.summary()
    assert report['observe_only_violations'] == 1
    assert report['command_published_violations'] == 1
    assert report['safety_contract_violations'] == 1
    assert report['safe_stop_intent_violations'] == 1


def test_accumulator_counts_eligible_robot_state():
    accumulator = MissionDiagnosticsAccumulator()
    ready = payload('ALIGNMENT_READY', 'HOLD_CENTER_PREVIEW')
    ready['decision']['eligible_for_future_alignment_handoff'] = True
    ready['decision']['readiness_blockers'] = []
    ready['robot'] = {'odom_fresh': True, 'stationary': True}
    accumulator.add(ready, receive_time=3.0)
    report = accumulator.summary()
    assert report['eligible_frames'] == 1
    assert report['eligibility_counts'] == {'eligible': 1}
    assert report['readiness_blocker_counts'] == {}
    assert report['odom_fresh_counts'] == {'true': 1}
    assert report['stationary_counts'] == {'true': 1}
