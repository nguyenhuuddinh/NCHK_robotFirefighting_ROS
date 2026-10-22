"""Exercise the Humble wait-set teardown race without connecting to hardware."""

from importlib.util import module_from_spec, spec_from_file_location
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from rclpy.impl.implementation_singleton import rclpy_implementation


if os.environ.get('MISSION_SHUTDOWN_SOURCE'):
    SOURCE = Path(os.environ['MISSION_SHUTDOWN_SOURCE'])
    SPEC = spec_from_file_location('qa_mission_shutdown', SOURCE)
    mission = module_from_spec(SPEC)
    SPEC.loader.exec_module(mission)
else:
    from fire_robot_mission import mission_supervisor as mission
RCL_ERROR = rclpy_implementation.RCLError
CONTEXT_ERROR = (
    'failed to initialize wait set: the given context is not valid, either '
    'rcl_init() was not called or rcl_shutdown() was called., '
    'at ./src/rcl/wait.c:130')


@pytest.fixture
def harness(monkeypatch):
    """Record lifecycle calls without creating ROS entities."""
    calls = []
    state = {'ok': True}
    node = SimpleNamespace(
        context=SimpleNamespace(ok=lambda: state['ok']),
        destroy_node=lambda: calls.append('destroy'))
    monkeypatch.setattr(mission.rclpy, 'init', lambda **kw: calls.append('init'))
    monkeypatch.setattr(mission, 'FireMissionSupervisor', lambda: node)
    monkeypatch.setattr(mission.rclpy, 'ok', lambda: state['ok'])
    monkeypatch.setattr(mission.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    return calls, state, node


def set_spin(monkeypatch, state, error, invalidate):
    """Inject a spin outcome and matching context state."""
    def spin(node):
        if invalidate:
            state['ok'] = False
        if error:
            raise error
    monkeypatch.setattr(mission.rclpy, 'spin', spin)


def test_exact_shutdown_race_exits_cleanly_and_destroys_node(monkeypatch, harness):
    """Match the actual234730 traceback, after ROS invalidated its context."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, RCL_ERROR(CONTEXT_ERROR), True)
    mission.main()
    assert calls == ['init', 'destroy']


def test_identical_error_with_live_context_is_not_hidden(monkeypatch, harness):
    """An invalid entity while ROS is active still fails the process."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, RCL_ERROR(CONTEXT_ERROR), False)
    with pytest.raises(RCL_ERROR, match='wait set'):
        mission.main()
    assert calls == ['init', 'destroy', 'shutdown']


@pytest.mark.parametrize('active', [True, False])
def test_other_rcl_error_is_not_hidden(monkeypatch, harness, active):
    """A different RCL failure must not be turned into shutdown success."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, RCL_ERROR('publisher failed'), not active)
    with pytest.raises(RCL_ERROR, match='publisher failed'):
        mission.main()
    assert calls.count('destroy') == 1


def test_node_initialization_error_is_not_treated_as_teardown(monkeypatch, harness):
    """Limit the handler to spin, not construction."""
    calls, state, _ = harness
    def fail():
        state['ok'] = False
        raise RCL_ERROR(CONTEXT_ERROR)
    monkeypatch.setattr(mission, 'FireMissionSupervisor', fail)
    with pytest.raises(RCL_ERROR):
        mission.main()
    assert calls == ['init']


@pytest.mark.parametrize('exception', [KeyboardInterrupt, mission.ExternalShutdownException])
def test_existing_normal_shutdown_paths_preserved(monkeypatch, harness, exception):
    """Retain both previously supported ways to leave the event loop."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, exception(), True)
    mission.main()
    assert calls == ['init', 'destroy']


def test_unrelated_callback_error_remains_fatal(monkeypatch, harness):
    """Application failures are never swallowed just because context closed."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, RuntimeError('bad callback'), True)
    with pytest.raises(RuntimeError, match='bad callback'):
        mission.main()
    assert calls == ['init', 'destroy']


def test_normal_spin_return_cleanup(monkeypatch, harness):
    """Destroy once and shut down a still-live context."""
    calls, state, _ = harness
    set_spin(monkeypatch, state, None, False)
    mission.main()
    assert calls == ['init', 'destroy', 'shutdown']
