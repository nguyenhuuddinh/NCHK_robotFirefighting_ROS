"""Admit stamped ACT reports; source authentication belongs to the ROS caller."""

import math


def _uint32(value):
    return type(value) is int and 0 <= value <= 0xFFFFFFFF


class ActuatorStampGuard:
    """Bounded source watermark; no command, clock polling or quota reset."""

    def __init__(self, *, max_age_ms, max_future_skew_ms):
        if (type(max_age_ms) not in (int, float) or
                not math.isfinite(max_age_ms) or max_age_ms <= 0 or
                type(max_future_skew_ms) not in (int, float) or
                not math.isfinite(max_future_skew_ms) or max_future_skew_ms < 0):
            raise ValueError('ACT source age limits must be finite and valid')
        self.max_age_ms = float(max_age_ms)
        self.max_future_skew_ms = float(max_future_skew_ms)
        self.last_ros_ns = None
        self.last_stamp_ns = None
        self.last_identity = None
        self.last_seq = None
        self.last_esp_ms = None
        self.reason = None
        self.context_changed = False
        self._clock_reset_pending = False

    def admit(self, payload, ros_now_ns):
        """Return age at receipt in ms, or None and a blocker reason.

        A source/clock transition invalidates old caller proofs via
        context_changed. It does not authorize cleanup or rearm. uint32
        sequence gaps are allowed; a half-range jump is ambiguous and blocked.
        """
        self.reason = None
        self.context_changed = False
        if type(ros_now_ns) is not int or ros_now_ns <= 0:
            self.reason = 'ACTUATOR_CLOCK_INVALID'
            return None
        previous_ros = self.last_ros_ns
        self.last_ros_ns = ros_now_ns
        if previous_ros is not None and ros_now_ns < previous_ros:
            self.last_stamp_ns = None
            self._clock_reset_pending = True
            self.reason = 'ACTUATOR_CLOCK_BACKWARD'
            return None
        if (not isinstance(payload, dict) or
                type(payload.get('schema_version')) is not int or
                payload.get('schema_version') != 1 or
                payload.get('mode') != 'COMMAND_TELEMETRY_ONLY' or
                type(payload.get('pump_pwm_commanded_on')) is not bool or
                type(payload.get('servo_pwm_commanded_on')) is not bool or
                payload.get('servo_phase') not in
                ('IDLE', 'SWEEPING', 'RETURNING_HOME', 'LOCKED') or
                payload.get('physical_pump_or_nozzle_verified') is not False):
            self.reason = 'ACTUATOR_REPORT_INVALID'
            return None
        source = payload.get('source')
        stamp = source.get('stamp') if isinstance(source, dict) else None
        if not isinstance(stamp, dict):
            self.reason = 'ACTUATOR_SOURCE_METADATA_INVALID'
            return None
        sec, nanosec = stamp.get('sec'), stamp.get('nanosec')
        instance = source.get('bridge_instance_id')
        epoch, seq = source.get('serial_epoch'), source.get('frame_seq')
        esp_ms = payload.get('esp_ms')
        if (type(sec) is not int or sec < 0 or type(nanosec) is not int or
                not 0 <= nanosec < 1_000_000_000 or
                type(instance) is not str or len(instance) != 32 or
                instance == '0' * 32 or
                any(c not in '0123456789abcdef' for c in instance) or
                type(epoch) is not int or epoch <= 0 or
                not _uint32(seq) or not _uint32(esp_ms)):
            self.reason = 'ACTUATOR_SOURCE_METADATA_INVALID'
            return None
        stamp_ns = sec * 1_000_000_000 + nanosec
        reported_age = payload.get('receive_age_ms')
        measured_ns = ros_now_ns - stamp_ns
        identity = (instance, epoch)
        if stamp_ns <= 0:
            self.reason = 'ACTUATOR_SOURCE_METADATA_INVALID'
        elif (type(reported_age) not in (int, float) or
              not math.isfinite(reported_age) or reported_age < 0):
            self.reason = 'ACTUATOR_SOURCE_AGE_INVALID'
        elif measured_ns < -round(self.max_future_skew_ms * 1e6):
            self.reason = 'ACTUATOR_SOURCE_IN_FUTURE'
        elif (measured_ns > round(self.max_age_ms * 1e6) or
              reported_age > self.max_age_ms):
            self.reason = 'ACTUATOR_SOURCE_STALE'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.reason = 'ACTUATOR_STAMP_NON_INCREASING'
        elif self.last_identity is not None and instance == self.last_identity[0]:
            if epoch < self.last_identity[1]:
                self.reason = 'ACTUATOR_EPOCH_BACKWARD'
            elif epoch == self.last_identity[1]:
                if not 0 < ((seq - self.last_seq) & 0xFFFFFFFF) < 0x80000000:
                    self.reason = 'ACTUATOR_SEQUENCE_NON_INCREASING'
                elif ((esp_ms - self.last_esp_ms) & 0xFFFFFFFF) >= 0x80000000:
                    self.reason = 'ACTUATOR_UPTIME_BACKWARD'
        if self.reason is not None:
            return None
        self.context_changed = self._clock_reset_pending or (
            self.last_identity is not None and identity != self.last_identity)
        self._clock_reset_pending = False
        self.last_identity = identity
        self.last_stamp_ns = stamp_ns
        self.last_seq = seq
        self.last_esp_ms = esp_ms
        return max(float(reported_age), measured_ns / 1e6, 0.0)
