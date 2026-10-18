"""Admit stamped odometry without renewing its source-age budget.

ROS clocks must be synchronized. A future stamp is rejected, not clamped.
The returned monotonic timestamp is backdated by the source age, so existing
consumer timeouts include transport delay. This cannot measure ESP sample age
before the bridge stamps the message.
"""

import math
import time


class OdomStampGuard:
    """Reject invalid, expired and non-increasing stamps within a clock epoch."""

    def __init__(self):
        self.last_stamp_ns = None
        self.last_ros_ns = None
        self.reason = None

    def admit(self, stamp, ros_now_ns, mono_now, timeout_s):
        """Return effective monotonic receipt time, or None on rejection."""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('odom timeout must be finite and positive')
        previous_ros = self.last_ros_ns
        self.last_ros_ns = ros_now_ns
        if previous_ros is not None and ros_now_ns < previous_ros:
            self.last_stamp_ns = None
            self.reason = 'ROS_CLOCK_BACKWARD'
            return None  # Require a subsequent sample in the new epoch.
        sec, nsec = stamp.sec, stamp.nanosec
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            self.reason = 'INVALID_STAMP'
            return None
        stamp_ns = sec * 1_000_000_000 + nsec
        age_ns = ros_now_ns - stamp_ns
        if stamp_ns <= 0:
            self.reason = 'ZERO_STAMP'
        elif age_ns < 0:
            self.reason = 'FUTURE_STAMP'
        elif age_ns > round(timeout_s * 1e9):
            self.reason = 'STALE_STAMP'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.reason = 'NON_INCREASING_STAMP'
        else:
            self.last_stamp_ns = stamp_ns
            self.reason = None
            return mono_now - age_ns / 1e9
        return None


def admit_odom(node, message, timeout_s):
    """Node adapter; each node owns its guard and logs rejection transitions."""
    previous = node._odom_stamp_guard.reason
    received = node._odom_stamp_guard.admit(
        message.header.stamp, node.get_clock().now().nanoseconds,
        time.monotonic(), timeout_s)
    reason = node._odom_stamp_guard.reason
    if reason is not None and reason != previous:
        node.get_logger().warn(f'Odom rejected: {reason}')
    elif reason is None and previous is not None:
        node.get_logger().info('Odom timestamp valid again')
    return received
