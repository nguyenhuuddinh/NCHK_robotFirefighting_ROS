"""Offline tests for subscriber-only fire-target diagnostics."""

from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from fire_robot_perception.target_diagnostics import (
    TargetDiagnosticsAccumulator,
    diagnostic_qos,
    progress_line,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def make_payload(
        *, detected=True, confirmed=True, center=(0.0, 0.0),
        confidence=0.8, aim='CENTERED'):
    """Create one representative observer JSON object."""
    candidate = None
    detections = []
    if detected:
        candidate = {
            'class_name': 'fire',
            'confidence': confidence,
            'center_normalized': list(center),
        }
        detections = [candidate]
    else:
        aim = 'NO_TARGET'
    return {
        'observe_only': True,
        'state': 'ok',
        'source': {'age_ms': 20.0},
        'processing_ms': 25.0,
        'inference_ms': 18.0,
        'detections': detections,
        'fire': {
            'detected_now': detected,
            'confirmed': confirmed,
            'hits': 3 if confirmed else int(detected),
            'window_frames': 5,
            'center_filter': {
                'method': 'ema',
                'alpha': 0.65,
                'reset_on_miss': True,
            },
            'candidate': candidate,
            'smoothed_center_normalized': (
                list(center) if confirmed else None),
            'aim': {'state': aim},
        },
    }


def test_diagnostic_qos_is_best_effort_depth_one():
    """Diagnostics cannot create a telemetry backlog."""
    qos = diagnostic_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 1


def test_accumulator_reports_detection_jitter_and_transitions():
    """Summary captures target stability, loss and reacquisition."""
    accumulator = TargetDiagnosticsAccumulator()
    accumulator.add(
        make_payload(center=(-0.2, 0.0), aim='LEFT'), receive_time=1.0)
    accumulator.add(
        make_payload(center=(-0.1, 0.0), aim='CENTERED'), receive_time=1.1)
    accumulator.add(
        make_payload(detected=False, confirmed=False), receive_time=1.2)
    accumulator.add(
        make_payload(center=(0.2, 0.1), aim='RIGHT'), receive_time=1.3)

    report = accumulator.summary()
    assert report['mode'] == 'OBSERVE_ONLY'
    assert report['subscriber_only'] is True
    assert report['messages'] == 4
    assert report['observe_only_violations'] == 0
    assert report['fire']['detected_frames'] == 3
    assert report['fire']['confirmed_frames'] == 3
    assert report['fire']['confirmation_events'] == 2
    assert report['fire']['lost_events'] == 1
    assert report['fire']['reacquire_ms']['mean'] == 100.0
    assert report['fire']['confidence']['mean'] == 0.8
    assert report['fire']['smoothed_center']['x']['samples'] == 3
    assert report['fire']['center_filter_counts'] == {
        'ema(alpha=0.65)': 4,
    }
    assert report['fire']['aim_state_counts'] == {
        'CENTERED': 1,
        'LEFT': 1,
        'NO_TARGET': 1,
        'RIGHT': 1,
    }
    assert report['message_rate_hz'] == 10.0


def test_progress_line_is_compact_and_action_free():
    """Progress output exposes aiming evidence without command semantics."""
    line = progress_line(make_payload(center=(0.2, -0.1), aim='RIGHT'))
    assert 'confirmed=True' in line
    assert 'center=(+0.20,-0.10)' in line
    assert 'filter=ema(0.65)' in line
    assert 'aim=RIGHT' in line


def test_diagnostics_source_has_no_publishers():
    """The diagnostic executable must remain subscriber-only."""
    source = (
        PACKAGE_DIR / 'fire_robot_perception' / 'target_diagnostics.py'
    ).read_text()
    assert 'create_' + 'publisher' not in source
