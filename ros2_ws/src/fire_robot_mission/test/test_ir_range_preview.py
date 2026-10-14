"""Pure-core and static safety tests for the provisional IR preview."""

import ast
from pathlib import Path

import pytest
import yaml

from fire_robot_mission.ir_range_core import IrRangePreviewCore
from fire_robot_mission.ir_approach_live_core import provisional_spray_candidate
from fire_robot_mission.ir_approach_shadow_core import validated_ir_band


ROOT = Path(__file__).resolve().parents[1]


def core():
    return IrRangePreviewCore(
        baselines=(26060, 26140, 25825),
        baseline_tolerance=500,
        min_active_drop=1000,
        right_thresholds=(850, 1100, 1850),
        median_window=5,
        stable_samples=3,
        spray_confirm_samples=5,
        spray_reentry_margin_counts=50,
        ir_timeout_s=0.6,
        vision_timeout_s=0.5,
        max_ir_age_ms=300.0,
        max_image_age_ms=300.0,
    )


def ir(left, center, right, *, valid=True, age=31):
    return {
        'schema_version': 1, 'valid': valid,
        'left': left, 'center': center, 'right': right,
        'sample_age_ms': age, 'receive_age_ms': 35,
    }


def vision(model, now=0.0, *, centered=True, detected=True):
    model.update_detection({
        'schema_version': 1, 'observe_only': True, 'state': 'ok',
        'source': {'age_ms': 35},
        'fire': {'detected_now': detected, 'confirmed': detected},
    }, now)
    model.update_alignment({
        'schema_version': 1, 'observe_only': True,
        'state': 'CENTERED' if centered else 'ALIGNING_LEFT',
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False},
    }, now)


def feed(model, sample, start=0.0, count=5):
    for index in range(count):
        model.update_ir(sample, start + 0.05 * index)


@pytest.mark.parametrize('sample,band', [
    (ir(1100, 1040, 760), 'TOO_CLOSE'),
    (ir(1290, 1450, 940), 'SPRAY_RANGE'),
    (ir(5000, 13000, 1260), 'APPROACH'),
    (ir(10500, 17500, 1410), 'APPROACH'),
    (ir(14400, 19500, 1650), 'APPROACH'),
    (ir(16000, 20000, 2100), 'TOO_FAR'),
])
def test_observed_fire_bands_are_provisional_only(sample, band):
    model = core()
    vision(model, 0.2)
    feed(model, sample)
    result = model.snapshot(0.21)
    assert result['ir_band'] == band
    assert result['zone'] == band
    assert result['validated_for_actuation'] is False
    assert result['safety']['motion_command_publisher'] is False
    assert result['safety']['actuator_command_publisher'] is False


def test_no_flame_never_becomes_far_or_spray():
    model = core()
    vision(model, 0.2)
    feed(model, ir(26060, 26140, 25825))
    result = model.snapshot(0.21)
    assert result['ir_band'] == 'NO_FLAME_SIGNATURE'
    assert result['zone'] == 'UNKNOWN'
    assert 'NO_IR_FLAME_SIGNATURE' in result['blockers']


def test_sample_sequence_only_advances_on_valid_new_adc_input():
    model = core()
    assert model.snapshot(0.0)['ir_sample_seq'] == 0
    model.update_ir(ir(5000, 13000, 1260), 0.1)
    assert model.snapshot(0.2)['ir_sample_seq'] == 1
    assert model.snapshot(0.3)['ir_sample_seq'] == 1
    model.update_ir(ir(5000, 13000, 1260, valid=False), 0.31)
    assert model.snapshot(0.32)['ir_sample_seq'] == 1
    model.update_ir(ir(5000, 13000, 1260), 0.4)
    assert model.snapshot(0.41)['ir_sample_seq'] == 2


def test_range_requires_vision_and_centering():
    model = core()
    feed(model, ir(5000, 13000, 940))
    assert model.snapshot(0.21)['zone'] == 'UNKNOWN'
    vision(model, 0.2, centered=False)
    assert 'FIRE_NOT_CENTERED' in model.snapshot(0.21)['blockers']
    vision(model, 0.2, detected=False)
    assert 'FIRE_NOT_CONFIRMED' in model.snapshot(0.21)['blockers']
    vision(model, 0.2)
    assert model.snapshot(0.21)['zone'] == 'SPRAY_RANGE'


def test_alignment_without_safety_contract_does_not_release_zone():
    model = core()
    feed(model, ir(5000, 13000, 940))
    vision(model, 0.2)
    model.update_alignment({
        'schema_version': 1, 'observe_only': True, 'state': 'CENTERED',
    }, 0.2)
    result = model.snapshot(0.21)
    assert result['zone'] == 'UNKNOWN'
    assert 'ALIGNMENT_SAFETY_CONTRACT' in result['blockers']


def test_one_bad_eye_is_unknown_but_boundary_is_contiguous():
    model = core()
    vision(model, 0.2)
    feed(model, ir(26050, 13000, 940))
    assert model.snapshot(0.21)['zone'] == 'UNKNOWN'
    assert 'IR_AMBIGUOUS' in model.snapshot(0.21)['blockers']
    model = core()
    vision(model, 0.2)
    feed(model, ir(5000, 13000, 880))
    assert model.snapshot(0.21)['zone'] == 'SPRAY_RANGE'


@pytest.mark.parametrize('right,band', [
    (850, 'TOO_CLOSE'), (851, 'SPRAY_RANGE'),
    (1100, 'SPRAY_RANGE'), (1101, 'APPROACH'),
    (1850, 'APPROACH'), (1851, 'TOO_FAR'),
])
def test_contiguous_right_eye_boundaries(right, band):
    model = core()
    vision(model, 0.2)
    feed(model, ir(5000, 13000, right))
    assert model.snapshot(0.21)['zone'] == band


def test_invalid_or_stale_input_clears_previous_band():
    model = core()
    vision(model, 0.2)
    feed(model, ir(5000, 13000, 940))
    assert model.snapshot(0.21)['zone'] == 'SPRAY_RANGE'
    model.update_ir(ir(5000, 13000, 940, valid=False), 0.3)
    assert model.snapshot(0.31)['zone'] == 'UNKNOWN'
    assert model.snapshot(0.31)['ir_band'] == 'UNKNOWN'
    feed(model, ir(5000, 13000, 940, age=301), 0.4)
    assert 'IR_SAMPLE_STALE_OR_INVALID_AGE' in model.snapshot(0.61)['blockers']
    feed(model, ir(5000, 13000, 940), 0.7)
    assert model.snapshot(1.51)['zone'] == 'UNKNOWN'
    assert 'IR_TIMEOUT' in model.snapshot(1.51)['blockers']


def test_band_change_needs_new_distinct_ir_messages():
    model = core()
    vision(model, 0.2)
    feed(model, ir(5000, 13000, 940))
    assert model.snapshot(0.21)['zone'] == 'SPRAY_RANGE'
    feed(model, ir(14400, 19500, 1650), 0.3, count=3)
    result = model.snapshot(0.51)
    assert result['zone'] == 'UNKNOWN'
    assert result['ir_band'] == 'APPROACH'
    assert result['stable_samples'] == 1
    model.update_ir(ir(14400, 19500, 1650), 0.6)
    model.update_ir(ir(14400, 19500, 1650), 0.7)
    vision(model, 0.7)
    assert model.snapshot(0.71)['zone'] == 'APPROACH'


def test_spray_entry_needs_five_distinct_ir_samples():
    model = core()
    vision(model, 0.2)
    feed(model, ir(1300, 1600, 970), count=4)
    pending = model.snapshot(0.16)
    assert pending['ir_band'] == 'SPRAY_RANGE'
    assert pending['zone'] == 'UNKNOWN'
    assert 'IR_BAND_NOT_STABLE' in pending['blockers']
    assert validated_ir_band(pending) is None
    assert provisional_spray_candidate(pending)
    model.update_ir(ir(1300, 1600, 970), 0.2)
    confirmed = model.snapshot(0.21)
    assert confirmed['zone'] == 'SPRAY_RANGE'
    assert provisional_spray_candidate(confirmed)
    assert validated_ir_band(confirmed) == 'SPRAY_RANGE'


def test_early_spray_stop_requires_centered_camera():
    model = core()
    vision(model, 0.2, centered=False)
    feed(model, ir(1300, 1600, 970), count=4)
    pending = model.snapshot(0.21)
    assert pending['ir_band'] == 'SPRAY_RANGE'
    assert not provisional_spray_candidate(pending)


def test_spray_reentry_hysteresis_does_not_hold_spray_above_1100():
    model = core()
    vision(model, 0.2)
    feed(model, ir(1300, 1600, 970))
    assert model.snapshot(0.21)['zone'] == 'SPRAY_RANGE'
    feed(model, ir(5000, 13000, 1260), 0.3)
    vision(model, 0.5)
    assert model.snapshot(0.51)['zone'] == 'APPROACH'
    feed(model, ir(5000, 13000, 1080), 0.6, count=7)
    vision(model, 0.9)
    pending = model.snapshot(0.91)
    assert pending['ir_band'] == 'SPRAY_RANGE'
    assert pending['zone'] == 'UNKNOWN'
    assert 'IR_SPRAY_REENTRY_HYSTERESIS' in pending['blockers']
    assert validated_ir_band(pending) is None
    feed(model, ir(5000, 13000, 1040), 1.0)
    vision(model, 1.2)
    assert model.snapshot(1.21)['zone'] == 'UNKNOWN'
    model.update_ir(ir(5000, 13000, 1040), 1.25)
    model.update_ir(ir(5000, 13000, 1040), 1.3)
    vision(model, 1.3)
    assert model.snapshot(1.31)['zone'] == 'SPRAY_RANGE'


def test_no_flame_resets_spray_reentry_latch():
    model = core()
    vision(model, 0.2)
    feed(model, ir(1300, 1600, 970))
    feed(model, ir(5000, 13000, 1260), 0.3)
    feed(model, ir(26060, 26140, 25825), 0.6)
    assert model.snapshot(0.81)['zone'] == 'UNKNOWN'
    feed(model, ir(5000, 13000, 1080), 0.9, count=7)
    vision(model, 1.2)
    assert model.snapshot(1.21)['zone'] == 'SPRAY_RANGE'


def test_malformed_payload_and_stale_yolo_fail_closed():
    model = core()
    vision(model, 0.2)
    feed(model, ir(5000, 13000, 940))
    model.update_ir(ir(True, 13000, 940), 0.3)
    assert 'IR_COUNTS_INVALID' in model.snapshot(0.31)['blockers']
    feed(model, ir(5000, 13000, 940), 0.4)
    assert 'YOLO_MISSING_OR_STALE' in model.snapshot(0.71)['blockers']
    vision(model, 0.6)
    assert model.snapshot(0.61)['zone'] == 'SPRAY_RANGE'
    model.mark_ir_error(0.7)
    assert 'IR_JSON_INVALID' in model.snapshot(0.71)['blockers']
    feed(model, ir(5000, 13000, 940), 0.8)
    model.update_detection({
        'schema_version': 1, 'observe_only': True, 'state': 'ok',
        'fire': [], 'source': 'bad source',
    }, 1.0)
    model.update_alignment({
        'schema_version': 1, 'observe_only': True, 'state': 'CENTERED',
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False}}, 1.0)
    result = model.snapshot(1.01)
    assert result['zone'] == 'UNKNOWN'
    assert 'YOLO_FIRE_INVALID' in result['blockers']
    assert 'YOLO_SOURCE_STALE_OR_INVALID' in result['blockers']


def test_bad_parameters_are_rejected():
    with pytest.raises(ValueError):
        IrRangePreviewCore(
            baselines=(26060, 26140, 25825),
            baseline_tolerance=1000, min_active_drop=1000,
            right_thresholds=(850, 1100, 1850),
            median_window=5, stable_samples=3,
            ir_timeout_s=0.6, vision_timeout_s=0.5,
            max_ir_age_ms=300.0, max_image_age_ms=300.0,
        )
    with pytest.raises(ValueError):
        IrRangePreviewCore(
            baselines=(26060, 26140, 25825),
            baseline_tolerance=500, min_active_drop=1000,
            right_thresholds=(850, 1100, 1850),
            median_window=5, stable_samples=3,
            spray_confirm_samples=2, spray_reentry_margin_counts=50,
            ir_timeout_s=0.6, vision_timeout_s=0.5,
            max_ir_age_ms=300.0, max_image_age_ms=300.0,
        )
    with pytest.raises(ValueError):
        IrRangePreviewCore(
            baselines=(26060, 26140, 25825),
            baseline_tolerance=500, min_active_drop=1000,
            right_thresholds=(850, 1100, 1850),
            median_window=5, stable_samples=3,
            spray_confirm_samples=5, spray_reentry_margin_counts=250,
            ir_timeout_s=0.6, vision_timeout_s=0.5,
            max_ir_age_ms=300.0, max_image_age_ms=300.0,
        )


def test_config_and_source_cannot_publish_robot_commands():
    source = ROOT / 'fire_robot_mission' / 'ir_range_preview.py'
    tree = ast.parse(source.read_text())
    publishers = [node for node in ast.walk(tree)
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute)
                  and node.func.attr == 'create_publisher']
    assert len(publishers) == 1
    assert isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'String'
    config_text = (ROOT / 'config' / 'ir_range_preview.yaml').read_text()
    params = yaml.safe_load(config_text)[
        'ir_range_preview']['ros__parameters']
    assert params['preview_topic'] == '/fire_mission/ir_range_preview'
    forbidden = {
        '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav', '/pump_cmd',
        '/fire_target'}
    assert params['preview_topic'] not in forbidden
    assert params['ir_topic'] == '/fire_sensors/ir_raw'
    assert params['right_too_close_max'] == 850
    assert params['right_spray_max'] == 1100
    assert params['right_approach_max'] == 1850
    assert params['spray_confirm_samples'] == 5
    assert params['spray_reentry_margin_counts'] == 50
    trial = yaml.safe_load((ROOT / 'config' /
                            'ir_range_demo_trial.yaml').read_text())[
                                'ir_range_preview']['ros__parameters']
    assert trial['right_spray_max'] == 1100
    launch = (ROOT / 'launch' / 'ir_range_preview.launch.py').read_text()
    assert "executable='ir_range_preview'" in launch
    assert "executable='live_alignment_takeover'" not in launch
