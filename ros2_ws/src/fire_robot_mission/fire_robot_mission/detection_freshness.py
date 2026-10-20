"""Check YOLO source stamps at receipt without nodes or hardware access."""

import math


class DetectionStampGuard:
    """Keep a single accepted stamp; reject stale/repeated source images."""

    def __init__(self, max_age_ms=300.0, max_future_skew_ms=20.0):
        if (type(max_age_ms) not in (int, float) or
                not math.isfinite(max_age_ms) or max_age_ms <= 0 or
                type(max_future_skew_ms) not in (int, float) or
                not math.isfinite(max_future_skew_ms) or
                max_future_skew_ms < 0):
            raise ValueError('source age limits must be finite and valid')
        self.max_age_ms = float(max_age_ms)
        self.max_future_skew_ms = float(max_future_skew_ms)
        self.last_stamp_ns = None
        self.last_ros_ns = None
        self.reason = None

    def admit(self, payload, ros_now_ns):
        """Return a receipt-aged payload copy, or None and a blocker reason.

        ROS clocks on publisher and receiver must be synchronized. A receiver
        clock rollback invalidates this call and starts a new stamp epoch.
        Monotonic aging after receipt belongs to MissionSupervisorCore.
        """
        self.reason = None
        if type(ros_now_ns) is not int or ros_now_ns <= 0:
            self.reason = 'DETECTION_CLOCK_INVALID'
            return None
        previous_ros = self.last_ros_ns
        self.last_ros_ns = ros_now_ns
        if previous_ros is not None and ros_now_ns < previous_ros:
            self.last_stamp_ns = None
            self.reason = 'DETECTION_CLOCK_BACKWARD'
            return None
        if (not isinstance(payload, dict) or
                payload.get('observe_only') is not True):
            self.reason = 'DETECTION_NOT_OBSERVE_ONLY'
            return None
        source = payload.get('source')
        stamp = source.get('stamp') if isinstance(source, dict) else None
        if not isinstance(stamp, dict):
            self.reason = 'DETECTION_SOURCE_STAMP_INVALID'
            return None
        sec, nanosec = stamp.get('sec'), stamp.get('nanosec')
        if (type(sec) is not int or type(nanosec) is not int or
                sec < 0 or not 0 <= nanosec < 1_000_000_000):
            self.reason = 'DETECTION_SOURCE_STAMP_INVALID'
            return None
        stamp_ns = sec * 1_000_000_000 + nanosec
        if stamp_ns <= 0:
            self.reason = 'DETECTION_SOURCE_STAMP_INVALID'
            return None
        measured_ns = ros_now_ns - stamp_ns
        reported_age = source.get('age_ms')
        if (type(reported_age) not in (int, float) or
                not math.isfinite(reported_age)):
            self.reason = 'SOURCE_AGE_MISSING'
        elif (measured_ns < -round(self.max_future_skew_ms * 1e6) or
              reported_age < -self.max_future_skew_ms):
            self.reason = 'SOURCE_TIMESTAMP_IN_FUTURE'
        elif (measured_ns > round(self.max_age_ms * 1e6) or
              reported_age > self.max_age_ms):
            self.reason = 'SOURCE_TIMESTAMP_STALE'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.reason = 'DETECTION_SOURCE_STAMP_NON_INCREASING'
        if self.reason is not None:
            return None
        self.last_stamp_ns = stamp_ns
        # Never make reported age younger. Small negative skew remains visible
        # to the core, which clamps its budget to zero and retains the warning.
        corrected_source = dict(source)
        corrected_source['age_ms'] = max(float(reported_age), measured_ns / 1e6)
        admitted = dict(payload)
        admitted['source'] = corrected_source
        return admitted
