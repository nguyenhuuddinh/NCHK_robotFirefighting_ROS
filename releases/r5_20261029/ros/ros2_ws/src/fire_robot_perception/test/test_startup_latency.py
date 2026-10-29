"""Offline fake dependency checks; no ROS Node init, GPU, images or sockets."""
from contextlib import nullcontext
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from fire_robot_perception.yolo_observer import LatencyWindow, YoloObserver


def test_latency_window_is_bounded_and_keeps_signed_clock_evidence():
    stats = LatencyWindow()
    for value in (None, float('nan'), float('inf'), -5.0, 15.0):
        stats.add('entry', value)
    assert stats.summary() == {'entry': {
        'count': 2, 'mean_ms': 5.0, 'min_ms': -5.0, 'max_ms': 15.0}}
    for _ in range(10000):
        stats.add('entry', 5.0)
    assert len(stats.values['entry']) == 4
    stats.values.clear()
    assert stats.summary() == {}


class Harness:
    """Call only model initialization with fake packages; never create a Node."""
    _check_startup_context = YoloObserver._check_startup_context
    _load_model = YoloObserver._load_model
    _startup_stage_begin = YoloObserver._startup_stage_begin
    _startup_stage_done = YoloObserver._startup_stage_done
    device = '0'
    quantize = 16
    warmup_runs = 2
    image_size = 8
    model_path = Path('/fake/model.pt')
    fire_class_name = 'fire'
    confidence = .25
    center_filter = 'ema'
    ema_alpha = .6

    def __init__(self):
        self.context = SimpleNamespace(ok=lambda: True)
        self.logs = []
        self.syncs = 0

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def _predict_arguments(self):
        return {'device': self.device}

    def _synchronize_gpu(self):
        self.syncs += 1


def test_model_initializes_once_and_preserves_warmup(monkeypatch):
    calls = []
    predicts = []
    model = SimpleNamespace(names={0: 'fire', 1: 'person'},
                            predict=lambda *a, **kw: predicts.append(kw))
    def load(path):
        calls.append(path)
        return model
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        inference_mode=nullcontext, cuda=SimpleNamespace(
            is_available=lambda: True, get_device_name=lambda _: 'FAKE_GPU')))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=load))
    node = Harness()
    node._load_model()
    assert calls == ['/fake/model.pt']
    assert predicts == [{'device': '0'}] * 2
    assert node.syncs == 1
    assert node._fire_class_id == 0
    for stage in ('dependency_import', 'device_check', 'model_load', 'gpu_warmup'):
        assert any('stage='+stage+' status=begin' in x for x in node.logs)
        assert any('stage='+stage+' status=done' in x for x in node.logs)


def test_missing_dependencies_fail_before_model_or_observations(monkeypatch):
    monkeypatch.setitem(sys.modules, 'torch', None)
    node = Harness()
    with pytest.raises(RuntimeError, match='dependencies are missing'):
        node._load_model()
    assert not hasattr(node, '_model')
    assert not any('status=done' in x for x in node.logs)


def test_requested_cuda_failure_does_not_fall_back_to_cpu(monkeypatch):
    def unexpected_model(path):
        pytest.fail('Model must not load after a failed GPU check')
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: False)))
    monkeypatch.setitem(
        sys.modules, 'ultralytics', SimpleNamespace(YOLO=unexpected_model))
    with pytest.raises(RuntimeError, match='CUDA is unavailable'):
        Harness()._load_model()
