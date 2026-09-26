"""Offline tests for observe-only YOLO helpers and configuration."""

from pathlib import Path

import numpy as np
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
import yaml

from fire_robot_perception.yolo_observer import (
    Detection,
    TargetCenterSmoother,
    TemporalFireGate,
    YoloObserver,
    aim_observation,
    normalized_center,
    normalized_to_pixel,
    sensor_qos,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def make_fire(x_normalized=0.0):
    """Create a compact fake fire detection for temporal-gate tests."""
    return Detection(
        class_id=1,
        class_name='fire',
        confidence=0.8,
        box=(100.0, 100.0, 200.0, 200.0),
        center=(150.0, 150.0),
        center_normalized=(x_normalized, 0.0),
    )


def test_sensor_qos_never_builds_an_image_backlog():
    """Camera input and perception telemetry are volatile depth-one data."""
    qos = sensor_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 1


def test_normalized_center_matches_contract_range():
    """Image center maps to zero and corners map to the signed limits."""
    assert normalized_center((0.0, 0.0, 640.0, 480.0), 640, 480) == (
        320.0, 240.0, 0.0, 0.0)
    assert normalized_center((0.0, 0.0, 0.0, 0.0), 640, 480) == (
        0.0, 0.0, -1.0, -1.0)


def test_normalized_pixel_round_trip_and_aim_deadband():
    """Overlay coordinates and direction labels follow the image contract."""
    assert normalized_to_pixel((0.0, 0.0), 640, 480) == (320.0, 240.0)
    assert aim_observation(None, 0.1, 0.1)['state'] == 'NO_TARGET'
    assert aim_observation((0.05, -0.05), 0.1, 0.1)['state'] == 'CENTERED'
    assert aim_observation((-0.2, 0.3), 0.1, 0.1)['state'] == 'LEFT+DOWN'
    assert aim_observation((0.2, -0.3), 0.1, 0.1)['state'] == 'RIGHT+UP'


def test_temporal_gate_requires_current_repeated_evidence():
    """Three hits confirm, while a missed current frame clears confirmation."""
    gate = TemporalFireGate(window_frames=5, minimum_detections=3)
    assert not gate.update(make_fire(-0.2))['confirmed']
    assert not gate.update(None)['confirmed']
    assert not gate.update(make_fire(0.0))['confirmed']
    status = gate.update(make_fire(0.2))
    assert status['confirmed']
    assert status['hits'] == 3
    assert not gate.update(None)['confirmed']


def test_temporal_gate_reset_removes_old_evidence():
    """Camera loss cannot leave a stale confirmed fire target behind."""
    gate = TemporalFireGate(window_frames=3, minimum_detections=2)
    gate.update(make_fire())
    assert gate.update(make_fire())['confirmed']
    gate.reset()
    status = gate.update(make_fire())
    assert not status['confirmed']
    assert status['hits'] == 1


def test_median_filter_rejects_one_position_outlier():
    """Median remains available for an A/B comparison with the old method."""
    smoother = TargetCenterSmoother('median', 5, 0.65)
    values = [0.02, 0.03, 0.80, 0.04, 0.03]
    result = None
    for value in values:
        result = smoother.update((value, 0.0))
    assert result == [0.03, 0.0]


def test_ema_follows_latest_center_and_resets_after_a_miss():
    """EMA is responsive but cannot carry a stale point across target loss."""
    smoother = TargetCenterSmoother('ema', 5, 0.65)
    assert smoother.update((-0.8, 0.0)) == [-0.8, 0.0]
    assert smoother.update((0.8, 0.0)) == [0.24, 0.0]
    assert smoother.update(None) is None
    assert smoother.update((0.5, -0.1)) == [0.5, -0.1]


def test_center_filter_rejects_unsafe_configuration():
    """Unknown methods and invalid EMA weights fail during node startup."""
    try:
        TargetCenterSmoother('unknown', 5, 0.65)
        assert False, 'unknown center filter must fail'
    except ValueError:
        pass
    try:
        TargetCenterSmoother('ema', 5, 0.0)
        assert False, 'zero EMA alpha must fail'
    except ValueError:
        pass


def test_annotation_draws_center_deadband_and_smoothed_target():
    """Annotated output exposes aiming evidence without running a model."""
    observer = object.__new__(YoloObserver)
    observer.aim_deadband_x = 0.10
    observer.aim_deadband_y = 0.10
    observer._fire_class_id = 1
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    fire = make_fire(0.2)
    fire_status = {
        'hits': 5,
        'window_frames': 5,
        'confirmed': True,
        'center_filter': {'method': 'ema', 'alpha': 0.65},
        'candidate': fire.as_dict(),
        'smoothed_center_normalized': [0.2, 0.0],
        'aim': aim_observation((0.2, 0.0), 0.10, 0.10),
    }

    annotated = observer._draw_annotation(
        frame,
        [fire],
        fire_status,
        source_age_ms=20.0,
        processing_ms=25.0,
    )

    assert annotated.shape == frame.shape
    assert annotated.sum() > 0
    assert annotated[216, 288].tolist() == [255, 255, 0]


def test_default_config_is_observe_only_and_gpu_bounded():
    """Defaults use the compressed topic and short temporal confirmation."""
    config = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'yolo_params.yaml').read_text())
    params = config['yolo_observer']['ros__parameters']
    assert params['input_topic'] == '/image_raw/compressed'
    assert params['device'] == '0'
    assert params['quantize'] == 16
    assert params['confidence_threshold'] == 0.25
    assert params['use_clahe'] is False
    assert params['confirmation_window_frames'] == 5
    assert params['confirmation_min_detections'] == 3
    assert params['center_filter'] == 'ema'
    assert params['ema_alpha'] == 0.60
    assert params['aim_deadband_x'] == 0.10
    assert params['aim_deadband_y'] == 0.10
    assert params['annotated_max_fps'] <= 10.0


def test_node_source_has_no_robot_command_publishers():
    """The first perception phase must not name any robot command topic."""
    source = (
        PACKAGE_DIR / 'fire_robot_perception' / 'yolo_observer.py'
    ).read_text()
    forbidden_topics = ('/' + 'cmd_vel', '/' + 'pump_cmd', '/' + 'fire_target')
    assert all(topic not in source for topic in forbidden_topics)
