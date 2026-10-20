"""User-run offline scheduling regression; no sleeps, ROS init or hardware."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatusArray

from fire_robot_mission.global_stop_evidence import GlobalStopEvidence
from fire_robot_mission.mission_supervisor import _stop_clock_pair
from fire_robot_mission.stop_clock_sampling import sample_stop_clock
from test_global_stop_evidence import Case, ros
from test_global_stop_node import Harness


def read(mono, stamps):
    times, values = iter(mono), iter(stamps)
    return sample_stop_clock(lambda: next(values), lambda: next(times))


def test_old_unbracketed_pair_can_latch_fault_with_no_actual_clock_jump():
    stop = GlobalStopEvidence()
    assert stop._clock(ros(10), 10)
    # ROS was read at 10.1; the process resumed 30ms later to read monotonic.
    assert not stop._clock(ros(10.1), 10.13)
    assert stop.fault == 'GLOBAL_CLOCK_DISCONTINUITY'


def test_discard_wide_pair_and_read_again_without_extrapolating_ros():
    stop = GlobalStopEvidence()
    assert stop._clock(ros(10), 10)
    sample = read([10.1, 10.13, 10.13, 10.1301], [ros(10.1), ros(10.13)])
    stop.observe_clock_sample(sample)
    assert sample.attempts == 2 and sample.reason is None
    assert sample.ros_ns == ros(10.13) and sample.after == 10.1301
    assert stop._clock(sample.ros_ns, sample.after)
    assert stop.fault is None and stop.clock_sampling['retries'] == 1
    assert stop.clock_sampling['max_span_ns'] == 30_000_000


@pytest.mark.parametrize('jump_s', [-.05, .05, 1.0])
def test_real_clock_offset_change_still_latches_after_scheduling_retry(jump_s):
    stop = GlobalStopEvidence()
    assert stop._clock(ros(10), 10)
    sample = read([11, 11.03, 11.03, 11.0301],
                  [ros(11 + jump_s), ros(11.03 + jump_s)])
    stop.observe_clock_sample(sample)
    assert not stop._clock(sample.ros_ns, sample.after)
    assert stop.fault == 'GLOBAL_CLOCK_DISCONTINUITY'
    first = deepcopy(stop.clock_fault_evidence)
    stop._clock(ros(12), 12)
    assert stop.clock_fault_evidence == first and stop.fault is not None


def test_backward_ros_between_discarded_and_retry_read_is_never_hidden():
    sample = read([11, 11.03, 11.03, 11.0301], [ros(11), ros(10.9)])
    assert sample.reason == 'GLOBAL_CLOCK_BACKWARD'
    stop = GlobalStopEvidence()
    stop.observe_clock_sample(sample)
    assert not stop._clock(ros(12), 12)
    assert stop.fault == 'GLOBAL_CLOCK_BACKWARD'


def test_backward_first_discard_candidate_is_compared_to_previous_guard_read():
    values = iter([11, 11.03])
    sample = sample_stop_clock(lambda: ros(9.9), lambda: next(values),
                               previous_ros_ns=ros(10), previous_mono=10)
    assert sample.reason == 'GLOBAL_CLOCK_BACKWARD' and sample.attempts == 1


@pytest.mark.parametrize('times,stamps,reason', [
    ([10.1, 10.09], [ros(10.1)], 'GLOBAL_CLOCK_BACKWARD'),
    ([10.1, float('nan')], [ros(10.1)], 'GLOBAL_CLOCK_INVALID'),
    ([10.1, 10.1], [0], 'GLOBAL_CLOCK_INVALID'),
    ([10.1, 10.1], [True], 'GLOBAL_CLOCK_INVALID'),
    ([10.1, 10.13, 10.12, 10.1201], [ros(10.1), ros(10.12)], 'GLOBAL_CLOCK_BACKWARD'),
])
def test_invalid_or_backward_read_cannot_be_retried_into_healthy(times, stamps, reason):
    sample = read(times, stamps)
    assert sample.reason == reason


def test_three_wide_reads_block_existing_ready_proof_and_keep_first_fault():
    case = Case()
    assert case.ready().ready
    sample = read([11, 11.03, 11.03, 11.06, 11.06, 11.09],
                  [ros(11), ros(11.03), ros(11.06)])
    assert sample.attempts == 3 and sample.reason == 'GLOBAL_CLOCK_SAMPLE_UNCERTAIN'
    case.stop.observe_clock_sample(sample)
    first = deepcopy(case.stop.clock_fault_evidence)
    case.stop.observe_clock_sample(read([12, 12], [ros(12)]))
    result = case.stop.assess(case.owners(), ros_ns=ros(12), now=12)
    assert not result.ready and result.evidence is None
    assert case.stop.clock_fault_evidence == first
    assert case.stop._hold_since is None


def inject_one_read_delay(node, delay):
    original = node.get_clock
    pending = [True]

    def get_clock():
        def now():
            stamp = original().now().nanoseconds
            if pending:
                pending.pop()
                node.advance(delay)
            return SimpleNamespace(nanoseconds=stamp)
        return SimpleNamespace(now=now)

    node.get_clock = get_clock


def test_real_status_callback_retries_without_redating_dds_source(monkeypatch):
    node = Harness(monkeypatch, goal=None)
    assert node.ready().ready
    node.advance(.05)
    stamp = node.stamp
    inject_one_read_delay(node, .03)
    node._on_nav_status(GoalStatusArray(), {'source_timestamp': stamp})
    sample = node._global_stop.samples['nav_status']
    assert sample is not None and sample.stamp_ns == stamp
    assert sample.source_age_s == pytest.approx(.03)
    assert node._global_stop.fault is None
    assert node._global_stop.clock_sampling['retries'] == 1


def test_retry_cannot_make_a_400ms_old_source_current(monkeypatch):
    node = Harness(monkeypatch, goal=None)
    assert node.ready().ready
    node.advance(.05)
    stamp = node.stamp - 400_000_000
    inject_one_read_delay(node, .03)
    node._on_nav_status(GoalStatusArray(), {'source_timestamp': stamp})
    assert node._global_stop.samples['nav_status'] is None
    assert node._global_stop.reasons['nav_status'] == 'SOURCE_STALE'


def test_delay_before_state_admission_does_not_poison_fixed_clock_anchor(monkeypatch):
    node = Harness(monkeypatch)
    assert node.ready().ready
    node.advance(.05)
    inject_one_read_delay(node, .03)
    node._publish_state()
    assert node._global_stop.fault is None
    assert node._global_stop.clock_sampling['retries'] == 1
    source = node.messages[-1]['lifecycle']['fire_session_observer']['source']['stamp']
    assert source['sec'] * 1_000_000_000 + source['nanosec'] <= node.stamp


def test_pair_function_default_mode_uses_no_retry_or_stop_state():
    values = iter([ros(10)])
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=next(values))), _now=lambda: 10)
    assert _stop_clock_pair(node) == (ros(10), 10)
