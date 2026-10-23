"""Lifecycle regressions without ROS initialization, images or GPU work."""
import builtins
from contextlib import nullcontext
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from fire_robot_perception import yolo_observer as yolo

RCL_ERROR = yolo.rclpy_implementation.RCLError
TIMER_ERROR = 'failed to create timer: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called.'
WAIT_ERROR = 'failed to initialize wait set: the given context is not valid'


class ModelHarness:
    _check_startup_context = yolo.YoloObserver._check_startup_context
    _startup_stage_begin = yolo.YoloObserver._startup_stage_begin
    _startup_stage_done = yolo.YoloObserver._startup_stage_done
    _load_model = yolo.YoloObserver._load_model
    device = '0'
    quantize = 16
    image_size = 8
    warmup_runs = 2
    model_path = Path('/fake/model.pt')
    fire_class_name = 'fire'
    confidence = .25
    center_filter = 'ema'
    ema_alpha = .6

    def __init__(self):
        self.active = True
        self.context = SimpleNamespace(ok=lambda: self.active)
        self.calls = []
        self.logs = []

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def _predict_arguments(self):
        return {}

    def _synchronize_gpu(self):
        self.calls.append('sync')


@pytest.mark.parametrize('stage', ['dependency_import', 'device_check', 'model_load', 'warmup_first', 'warmup_second', 'gpu_sync'])
def test_shutdown_during_expensive_stage_skips_remaining_work(monkeypatch, stage):
    n = ModelHarness()
    def step(name):
        n.calls.append(name)
        if name == stage:
            n.active = False
    def cuda():
        step('device_check')
        return True
    def predict(*a, **kw):
        step('warmup_first' if 'warmup_first' not in n.calls else 'warmup_second')
    def model(path):
        step('model_load')
        return SimpleNamespace(names={0: 'fire'}, predict=predict)
    def sync():
        step('gpu_sync')
    n._synchronize_gpu = sync
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(inference_mode=nullcontext, cuda=SimpleNamespace(is_available=cuda, get_device_name=lambda _: 'GPU')))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=model))
    original_import = builtins.__import__
    def import_hook(name, *args, **kwargs):
        result = original_import(name, *args, **kwargs)
        if name == 'ultralytics':
            step('dependency_import')
        return result
    monkeypatch.setattr(builtins, '__import__', import_hook)
    with pytest.raises(yolo.ExternalShutdownException):
        n._load_model()
    order = ['dependency_import', 'device_check', 'model_load', 'warmup_first', 'warmup_second', 'gpu_sync']
    assert n.calls == order[:order.index(stage)+1]
    assert not any('[YOLO] loaded' in x for x in n.logs)


@pytest.mark.parametrize('active,error,expected', [
    (False,TIMER_ERROR,yolo.ExternalShutdownException),
    (True,TIMER_ERROR,RCL_ERROR),
    (False,'unrelated timer allocation failure',RCL_ERROR),
])
def test_timer_check_create_race_only_accepts_closed_context(monkeypatch, active, error, expected):
    state = {'ok': True}
    def create(*args):
        state['ok'] = active
        raise RCL_ERROR(error)
    n = SimpleNamespace(context=SimpleNamespace(ok=lambda: state['ok']), create_timer=create)
    n._check_startup_context = lambda: yolo.YoloObserver._check_startup_context(n)
    with pytest.raises(expected):
        yolo.YoloObserver._create_startup_timer(n, .5, lambda: None)


def test_closed_context_never_attempts_timer_creation():
    n = SimpleNamespace(context=SimpleNamespace(ok=lambda: False), create_timer=lambda *a: pytest.fail('timer created'))
    n._check_startup_context = lambda: yolo.YoloObserver._check_startup_context(n)
    with pytest.raises(yolo.ExternalShutdownException):
        yolo.YoloObserver._create_startup_timer(n, .5, lambda: None)


@pytest.mark.parametrize('exc', [yolo.ExternalShutdownException(), KeyboardInterrupt(), ValueError('bad params'), RCL_ERROR('unexpected')])
def test_partial_constructor_releases_node_and_preserves_error(monkeypatch, exc):
    calls = []
    monkeypatch.setattr(yolo.Node, '__init__', lambda self,*a: calls.append('base_init'))
    def fail(self):
        raise exc
    monkeypatch.setattr(yolo.YoloObserver, '_initialize_observer', fail)
    monkeypatch.setattr(yolo.YoloObserver, 'destroy_node', lambda self: calls.append('destroy'))
    with pytest.raises(type(exc)) as caught:
        yolo.YoloObserver()
    assert caught.value is exc
    assert calls == ['base_init', 'destroy']


@pytest.mark.parametrize('active,error,expected', [
    (False,WAIT_ERROR,None), (True,WAIT_ERROR,RCL_ERROR),
    (False,'unrelated error',RCL_ERROR), (True,None,None),
    (False,"Failed to publish: publisher's context is invalid",None),
    (True,"Failed to publish: publisher's context is invalid",RCL_ERROR),
    (False,'Failed to publish: unrelated allocation error',RCL_ERROR),
    (True,'Failed to publish: unrelated allocation error',RCL_ERROR),
])
def test_spin_teardown_preserves_real_failures(monkeypatch, active, error, expected):
    calls = []
    n = SimpleNamespace(context=SimpleNamespace(ok=lambda: active), destroy_node=lambda: calls.append('destroy'))
    monkeypatch.setattr(yolo.rclpy,'init',lambda **kw: None)
    monkeypatch.setattr(yolo.rclpy,'ok',lambda: active)
    monkeypatch.setattr(yolo.rclpy,'shutdown',lambda: calls.append('shutdown'))
    monkeypatch.setattr(yolo,'YoloObserver',lambda: n)
    def spin(node):
        if error:
            raise RCL_ERROR(error)
    monkeypatch.setattr(yolo.rclpy,'spin',spin)
    if expected:
        with pytest.raises(expected):
            yolo.main()
    else:
        yolo.main()
    assert calls == ['destroy'] + (['shutdown'] if active else [])
