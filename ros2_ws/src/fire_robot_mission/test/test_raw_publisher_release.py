"""Exercise real node tick effects without ROS init, discovery or hardware.

The core stub deliberately asks RELEASE with a nonzero candidate command:
the adapter must send STOP and check destruction before acknowledging it.
Existing core tests separately exercise admission/stop-hold guards.
"""

from types import SimpleNamespace

import pytest

from fire_robot_mission.live_nav_stop_gateway import LiveNavStopGateway
from fire_robot_mission.live_alignment_takeover import LiveAlignmentTakeover
from fire_robot_mission.ir_approach_live import IrApproachLiveNode


TICKS = {
    'gateway': LiveNavStopGateway._tick,
    'alignment': LiveAlignmentTakeover._tick,
    'ir': IrApproachLiveNode._tick,
}


class Publisher:
    def __init__(self, events, label):
        self.events = events
        self.label = label
        self.messages = []

    def publish(self, message):
        self.events.append(self.label)
        self.messages.append(message)


class TickHarness:
    """Supply only the services used by actual adapter _tick methods."""

    def __init__(self, kind, destroy_result=True, action='RELEASE', owned=True):
        self.kind = kind
        self.destroy_result = destroy_result
        self.action = action
        self.events = []
        self.destroy_calls = []
        self.logs = []
        self.seen_owned = []
        self.release_acks = 0
        self.raw = Publisher(self.events, 'stop_or_command')
        self.telemetry = Publisher(self.events, 'telemetry')
        self._raw_publisher = self.raw if owned else None
        self._state_publisher = self.telemetry
        self.state_publisher = self.telemetry
        self.core = SimpleNamespace(
            step=self.core_step, evaluate=self.core_step,
            publisher_released=self.publisher_released,
            stop_reason='TEST_STOP')

    def core_step(self, *args):
        owned = args[2] if self.kind == 'ir' else args[1]
        self.seen_owned.append(owned)
        payload = {'state': 'COMPLETE', 'publisher_owned': owned}
        if self.kind == 'gateway':
            payload['released'] = bool(self.release_acks)
            return 0.07, 0.2, payload, self.action
        return 0.2, payload, self.action

    def publisher_released(self):
        self.events.append('ack')
        self.release_acks += 1

    def destroy_publisher(self, publisher):
        self.events.append('destroy')
        self.destroy_calls.append(publisher)
        if isinstance(self.destroy_result, Exception):
            raise self.destroy_result
        return self.destroy_result

    def count_publishers(self, _topic):
        return 1

    def _sources(self, _topic):
        return []

    def _counts(self):
        return {}

    def get_logger(self):
        return SimpleNamespace(warning=self.logs.append)

    def tick(self):
        TICKS[self.kind](self)


def command_values(message):
    return (message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z)


@pytest.mark.parametrize('kind', TICKS)
@pytest.mark.parametrize('result', [False, None])
def test_unconfirmed_destroy_sends_zero_but_never_acknowledges_release(kind, result):
    node = TickHarness(kind, destroy_result=result)
    with pytest.raises(RuntimeError, match='RAW_PUBLISHER_RELEASE_UNCONFIRMED'):
        node.tick()
    assert node.events == ['stop_or_command', 'destroy']
    assert command_values(node.raw.messages[0]) == (0.0,) * 6
    assert node._raw_publisher is node.raw  # Do not manufacture "not owned".
    assert node.release_acks == 0
    assert node.telemetry.messages == []
    assert node.logs == []


@pytest.mark.parametrize('kind', TICKS)
def test_successful_destroy_releases_only_after_zero_and_true_return(kind):
    node = TickHarness(kind)
    node.tick()
    expected = ['stop_or_command', 'destroy']
    if kind == 'gateway':
        expected.append('ack')
    expected.append('telemetry')
    assert node.events == expected
    assert command_values(node.raw.messages[0]) == (0.0,) * 6
    assert node.destroy_calls == [node.raw]
    assert node._raw_publisher is None
    assert node.release_acks == (1 if kind == 'gateway' else 0)
    assert len(node.logs) == 1

    # The old payload describes the pre-action snapshot. Next tick observes
    # no publisher; it must not destroy/acknowledge a second time.
    node.action = None
    node.tick()
    assert node.seen_owned == [True, False]
    assert node.destroy_calls == [node.raw]
    assert len(node.raw.messages) == 1


@pytest.mark.parametrize('kind', TICKS)
def test_destroy_exception_cannot_publish_a_release_receipt(kind):
    node = TickHarness(kind, destroy_result=RuntimeError('destroy failed'))
    with pytest.raises(RuntimeError, match='destroy failed'):
        node.tick()
    assert command_values(node.raw.messages[0]) == (0.0,) * 6
    assert node._raw_publisher is node.raw
    assert node.release_acks == 0
    assert node.telemetry.messages == []


@pytest.mark.parametrize('kind', TICKS)
def test_normal_forwarding_does_not_destroy_or_replace_the_core_command(kind):
    node = TickHarness(kind, action=None)
    node.tick()
    message = node.raw.messages[0]
    assert message.linear.x == (0.07 if kind == 'gateway' else
                                0.2 if kind == 'ir' else 0.0)
    assert message.angular.z == (0.0 if kind == 'ir' else 0.2)
    assert node._raw_publisher is node.raw
    assert node.destroy_calls == []
    assert node.release_acks == 0


@pytest.mark.parametrize('kind', TICKS)
def test_release_without_a_publisher_does_not_invent_a_successful_destroy(kind):
    node = TickHarness(kind, owned=False)
    node.tick()
    assert node.destroy_calls == []
    assert node.release_acks == 0
    assert node.raw.messages == []
