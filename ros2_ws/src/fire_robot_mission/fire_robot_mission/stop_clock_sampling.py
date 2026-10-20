"""Bounded local clock reads for stop evidence; never alter a source stamp."""

from dataclasses import dataclass
import math


MAX_READ_SPAN_S = 0.001
MAX_ATTEMPTS = 3


@dataclass(frozen=True)
class ClockSample:
    ros_ns: object
    before: object
    after: object
    span_ns: object
    attempts: int
    reason: object
    widest_span_ns: int = 0


def _valid_time(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def sample_stop_clock(ros_now_ns, monotonic, *, previous_ros_ns=None, previous_mono=None):
    """Read monotonic/ROS/monotonic, retry only a scheduling-wide sample.

    Use the END of the bracket as receipt time, and the actual ROS read.
    Never extrapolate ROS time, sleep, refresh a message stamp or re-anchor
    the clock guard. Invalid/backward clocks are returned immediately so a
    later retry cannot conceal them. At most three ROS reads are performed.
    """
    previous_ros, previous_after = previous_ros_ns, previous_mono
    widest = 0
    for attempt in range(1, MAX_ATTEMPTS + 1):
        before = monotonic()
        ros_ns = ros_now_ns()
        after = monotonic()
        if (not _valid_time(before) or not _valid_time(after) or
                type(ros_ns) is not int or ros_ns <= 0):
            return ClockSample(ros_ns, before, after, None, attempt,
                               'GLOBAL_CLOCK_INVALID', widest)
        span = after - before
        widest = max(widest, round(span * 1e9))
        if (span < 0 or (previous_after is not None and before < previous_after) or
                (previous_ros is not None and ros_ns < previous_ros)):
            return ClockSample(ros_ns, before, after, round(span * 1e9), attempt,
                               'GLOBAL_CLOCK_BACKWARD', widest)
        if span <= MAX_READ_SPAN_S:
            return ClockSample(ros_ns, before, after, round(span * 1e9), attempt, None, widest)
        previous_ros, previous_after = ros_ns, after
    return ClockSample(ros_ns, before, after, round(span * 1e9), MAX_ATTEMPTS,
                       'GLOBAL_CLOCK_SAMPLE_UNCERTAIN', widest)
