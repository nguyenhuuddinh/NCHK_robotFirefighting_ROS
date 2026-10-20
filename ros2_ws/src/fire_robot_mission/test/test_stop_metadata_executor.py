"""Exercise actual Humble executor hooks with handles, never a ROS context."""

import asyncio
from types import SimpleNamespace

import pytest

from fire_robot_mission.stop_metadata_executor import StopMetadataExecutor


class Executor(StopMetadataExecutor):
    def __init__(self):
        pass  # No ROS context/thread/waitset in this test.

    def __del__(self):
        pass


class Handle:
    def __init__(self, result):
        self.result, self.calls = result, []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def take_message(self, kind, raw):
        self.calls.append((kind, raw))
        return self.result


@pytest.mark.parametrize('selected', [False, True])
def test_selected_subscription_preserves_source_timestamp_and_legacy_discards_metadata(selected):
    message, metadata = object(), {'source_timestamp': 123, 'received_timestamp': 456}
    handle, calls = Handle((message, metadata)), []
    sub = SimpleNamespace(handle=handle, msg_type=object, raw=False,
                          callback=lambda msg: calls.append(('legacy', msg)))
    if selected:
        sub._fire_stop_metadata_callback = lambda msg, info: calls.append(('selected', msg, info))
    executor = Executor()
    taken = executor._take_subscription(sub)
    assert taken == ((message, metadata) if selected else message)
    asyncio.run(executor._execute_subscription(sub, taken))
    assert calls == ([('selected', message, metadata)] if selected else [('legacy', message)])
    assert handle.calls == [(object, False)]
    assert metadata == {'source_timestamp': 123, 'received_timestamp': 456}


def test_no_message_does_not_call_a_marked_callback_or_fabricate_metadata():
    calls = []
    sub = SimpleNamespace(handle=Handle(None), msg_type=object, raw=False,
                          _fire_stop_metadata_callback=lambda *args: calls.append(args))
    executor = Executor()
    taken = executor._take_subscription(sub)
    assert taken is None
    asyncio.run(executor._execute_subscription(sub, taken))
    assert calls == []


def test_missing_metadata_is_delivered_as_missing_not_filled_with_local_time():
    calls = []
    message = object()
    sub = SimpleNamespace(handle=Handle((message, {})), msg_type=object, raw=False,
                          _fire_stop_metadata_callback=lambda *args: calls.append(args))
    executor = Executor()
    asyncio.run(executor._execute_subscription(sub, executor._take_subscription(sub)))
    assert calls == [(message, {})]


def test_take_exception_is_not_swallowed_as_fresh_data():
    class FailedHandle(Handle):
        def take_message(self, kind, raw):
            raise RuntimeError('take failed')
    sub = SimpleNamespace(handle=FailedHandle(None), msg_type=object, raw=False,
                          _fire_stop_metadata_callback=lambda *args: None)
    with pytest.raises(RuntimeError):
        Executor()._take_subscription(sub)
