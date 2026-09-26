"""Offline tests for observe-only fire alignment previews."""

from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
import yaml

from fire_robot_perception.fire_alignment_observer import (
    AlignmentPreviewController,
    extract_target_error,
    telemetry_qos,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def make_payload(*, error_x=0.3, age_ms=40.0, confirmed=True):
    """Create representative observe-only target telemetry."""
    return {
        'observe_only': True,
        'state': 'ok',
        'frame_index': 10,
        'source': {'age_ms': age_ms},
        'fire': {
            'detected_now': confirmed,
            'confirmed': confirmed,
            'smoothed_center_normalized': [error_x, 0.0],
        },
    }


def make_controller():
    """Return the conservative default preview controller."""
    return AlignmentPreviewController(
        deadband_enter=0.10,
        deadband_exit=0.13,
        proportional_gain=1.0,
        max_angular_speed=0.50,
        max_angular_acceleration=0.80,
    )


def test_telemetry_qos_does_not_build_a_preview_backlog():
    """Alignment telemetry uses sensor-style best-effort depth one."""
    qos = telemetry_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 1


def test_target_extraction_requires_current_confirmed_fresh_fire():
    """Invalid, missing and old targets cannot generate a yaw preview."""
    assert extract_target_error(make_payload(), 300.0) == (
        'TRACKABLE', 0.3, '')
    assert extract_target_error(
        make_payload(confirmed=False), 300.0)[0] == 'NO_TARGET'
    assert extract_target_error(
        make_payload(age_ms=301.0), 300.0)[0] == 'STALE_SOURCE'
    assert extract_target_error(
        make_payload(age_ms=float('nan')), 300.0)[0] == 'INVALID_TARGET'
    payload = make_payload()
    payload['observe_only'] = False
    assert extract_target_error(payload, 300.0)[0] == 'UNSAFE_INPUT'


def test_right_target_requests_negative_yaw_with_slew_limit():
    """Camera-right maps to a bounded right turn in the base convention."""
    controller = make_controller()
    first = controller.update(0.5, now=1.0)
    second = controller.update(0.5, now=1.1)
    assert first['state'] == 'ALIGNING_RIGHT'
    assert first['target_angular_z_rad_s'] == -0.4
    assert first['proposed_angular_z_rad_s'] == 0.0
    assert first['slew_limited'] is True
    assert second['proposed_angular_z_rad_s'] == -0.08


def test_deadband_hysteresis_prevents_center_chatter():
    """A centered target must cross the wider exit band before turning."""
    controller = make_controller()
    assert controller.update(0.05, now=1.0)['state'] == 'CENTERED'
    assert controller.update(0.12, now=1.1)['state'] == 'CENTERED'
    preview = controller.update(0.14, now=1.2)
    assert preview['state'] == 'ALIGNING_RIGHT'
    assert preview['proposed_angular_z_rad_s'] == -0.04


def test_target_loss_immediately_zeroes_and_resets_preview():
    """No target cannot coast on an earlier alignment value."""
    controller = make_controller()
    controller.update(-0.8, now=1.0)
    controller.update(-0.8, now=1.1)
    stopped = controller.inactive_preview('NO_TARGET')
    reacquired = controller.update(0.8, now=2.0)
    assert stopped['proposed_angular_z_rad_s'] == 0.0
    assert reacquired['proposed_angular_z_rad_s'] == 0.0
    assert reacquired['state'] == 'ALIGNING_RIGHT'


def test_alignment_defaults_are_conservative_and_observe_only():
    """YAML defaults retain bounded preview-only behavior."""
    config = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'yolo_params.yaml').read_text())
    params = config['fire_alignment_observer']['ros__parameters']
    assert params['input_topic'] == '/yolo/detections'
    assert params['preview_topic'] == '/yolo/alignment_preview'
    assert params['deadband_enter_x'] == 0.10
    assert params['deadband_exit_x'] == 0.13
    assert params['max_angular_speed_rad_s'] == 0.50
    assert params['max_angular_acceleration_rad_s2'] == 0.80


def test_alignment_source_has_no_robot_command_topics():
    """Preview implementation must not name a robot command interface."""
    source = (
        PACKAGE_DIR / 'fire_robot_perception' /
        'fire_alignment_observer.py'
    ).read_text()
    forbidden = ('/' + 'cmd_vel', '/' + 'pump_cmd', '/' + 'fire_target')
    assert all(topic not in source for topic in forbidden)
