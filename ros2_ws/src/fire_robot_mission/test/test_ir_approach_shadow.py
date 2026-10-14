"""Fail-closed and package-wiring tests for telemetry-only approach."""

import ast
import math
from pathlib import Path

import pytest

from fire_robot_mission.ir_approach_shadow_core import IrApproachShadowCore


ROOT = Path(__file__).resolve().parents[1]
COUNTS = {'ir': 1, 'scan': 1, 'odom': 1, 'cmd': 1, 'raw': 0}


def ir(zone='APPROACH'):
    return {
        'schema_version': 1, 'mode': 'OBSERVE_ONLY',
        'observe_only': True, 'zone': zone, 'ir_band': zone,
        'ir_receive_age_ms': 20.0,
        'stable_samples': 5,
        'filtered_counts': {'left': 14000, 'center': 19000,
                            'right': 1650},
        'blockers': [],
        'calibration': 'PROVISIONAL_SINGLE_FLAME',
        'validated_for_actuation': False,
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }


def scan(distance=1.0):
    return (-math.pi, 2 * math.pi / 359, 0.05, 8.0,
            tuple([distance] * 360))


def ready(zone='APPROACH'):
    core = IrApproachShadowCore()
    core.update('ir', ir(zone), 1.0)
    core.update('scan', scan(), 1.0)
    core.update('odom', (0.0,) * 6, 1.0)
    core.update('cmd', (0.0,) * 6, 1.0)
    return core


@pytest.mark.parametrize('zone', ['APPROACH', 'TOO_FAR'])
def test_far_zone_only_suggests_small_non_actuating_step(zone):
    result = ready(zone).snapshot(1.1, COUNTS)
    assert result['state'] == 'STEP_CANDIDATE'
    assert result['suggested_step_m'] == 0.05
    assert result['validated_for_actuation'] is False
    assert result['safety']['motion_command_publisher'] is False


@pytest.mark.parametrize('zone', ['SPRAY_RANGE', 'TOO_CLOSE', 'UNKNOWN'])
def test_spray_close_unknown_never_suggest_step(zone):
    result = ready(zone).snapshot(1.1, COUNTS)
    assert result['state'] == 'HOLD'
    assert result['suggested_step_m'] == 0.0


def test_obstacle_or_invalid_scan_blocks_step():
    core = ready()
    core.update('scan', scan(0.49), 1.0)
    assert 'FRONT_OBSTACLE_OR_INSUFFICIENT_MARGIN' in \
        core.snapshot(1.1, COUNTS)['blockers']
    core.update('scan', scan(math.inf), 1.0)
    assert 'SCAN_FRONT_UNOBSERVED' in core.snapshot(1.1, COUNTS)['blockers']
    core.update('scan', (0, 0, 0, 0, ()), 1.0)
    assert 'SCAN_INVALID' in core.snapshot(1.1, COUNTS)['blockers']


def test_clockwise_scan_is_valid_and_checks_front_sector():
    core = ready()
    clockwise = (math.pi, -2 * math.pi / 359, 0.05, 8.0,
                 tuple([1.0] * 360))
    core.update('scan', clockwise, 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'STEP_CANDIDATE'
    ranges = list(clockwise[4])
    ranges[179] = 0.30
    core.update('scan', clockwise[:4] + (tuple(ranges),), 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'HOLD'


def test_partial_scan_missing_one_side_of_front_is_not_clear():
    core = ready()
    partial = (0.0, -2 * math.pi / 359, 0.05, 8.0,
               tuple([1.0] * 129))
    core.update('scan', partial, 1.0)
    assert 'SCAN_FRONT_COVERAGE_LOW' in \
        core.snapshot(1.1, COUNTS)['blockers']


def test_loss_of_freshness_motion_or_publisher_conflict_blocks():
    core = ready()
    assert core.snapshot(1.6, COUNTS)['state'] == 'HOLD'
    core = ready()
    core.update('odom', (0.03, 0, 0, 0, 0, 0), 1.0)
    assert 'ODOM_NOT_FRESH_STATIONARY' in \
        core.snapshot(1.1, COUNTS)['blockers']
    core = ready()
    core.update('cmd', (0.01, 0, 0, 0, 0, 0), 1.0)
    assert 'CMD_VEL_NOT_FRESH_ZERO' in \
        core.snapshot(1.1, COUNTS)['blockers']
    assert 'RAW_MOTION_OWNER_ACTIVE' in \
        ready().snapshot(1.1, dict(COUNTS, raw=1))['blockers']


def test_bad_ir_contract_never_suggests_step_or_crashes():
    core = ready()
    bad = ir()
    bad['safety'] = []
    core.update('ir', bad, 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'HOLD'
    core.update('ir', None, 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'HOLD'


def test_ir_only_band_does_not_depend_on_camera_zone():
    core = ready()
    payload = ir()
    payload['zone'] = 'UNKNOWN'
    payload['blockers'] = ['FIRE_NOT_CENTERED']
    core.update('ir', payload, 1.0)
    result = core.snapshot(1.1, COUNTS)
    assert result['state'] == 'STEP_CANDIDATE'
    assert result['ir_zone'] == 'APPROACH'
    assert result['vision_zone'] == 'UNKNOWN'
    payload['blockers'] = ['IR_AMBIGUOUS']
    core.update('ir', payload, 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'HOLD'
    payload['blockers'] = []
    payload['ir_receive_age_ms'] = 301.0
    core.update('ir', payload, 1.0)
    assert core.snapshot(1.1, COUNTS)['state'] == 'HOLD'


def test_parameters_are_hard_bounded():
    with pytest.raises(ValueError):
        IrApproachShadowCore(step_m=0.06)
    with pytest.raises(ValueError):
        IrApproachShadowCore(front_margin_m=0.30)


def test_node_has_only_string_publisher_and_isolated_launch():
    source = ROOT / 'fire_robot_mission' / 'ir_approach_shadow.py'
    tree = ast.parse(source.read_text())
    publishers = [call for call in ast.walk(tree)
                  if isinstance(call, ast.Call) and
                  isinstance(call.func, ast.Attribute) and
                  call.func.attr == 'create_publisher']
    assert len(publishers) == 1
    assert isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'String'
    assert publishers[0].args[1].value == \
        '/fire_mission/ir_approach_shadow'
    launch = (ROOT / 'launch' / 'ir_approach_shadow.launch.py').read_text()
    assert "executable='ir_approach_shadow'" in launch
    assert 'live_alignment_takeover' not in launch
