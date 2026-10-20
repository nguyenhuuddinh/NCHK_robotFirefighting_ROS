"""One-shot, operator-armed IR-only forward/reverse jog state machine."""

import math

from fire_robot_mission.ir_approach_shadow_core import (
    _finite, _fresh, sector_clearance, validated_ir_band,
)


class VerifiedEndpointPins:
    """
    Keep an exact-name-verified DDS endpoint identity across name loss.

    Cross-host discovery can temporarily report an endpoint's node name as
    unknown while retaining its GID. A new GID cannot inherit a prior pin;
    the live node must be restarted after a real Pi endpoint restart.
    """

    def __init__(self):
        self._gids = {}
        self._reliabilities = {}

    def matches(self, key, info, *, name, message_type, reliability):
        if (info.topic_type != message_type or
                info.qos_profile.reliability != reliability):
            return False
        try:
            gid = bytes(info.endpoint_gid)
        except (TypeError, ValueError):
            return False
        if len(gid) < 16 or not any(gid):
            return False
        exact_name = (info.node_name == name and
                      info.node_namespace == '/')
        unknown_name = (
            info.node_name == '_NODE_NAME_UNKNOWN_' and
            info.node_namespace == '_NODE_NAMESPACE_UNKNOWN_')
        pinned_gid = self._gids.get(key)
        if pinned_gid is None:
            if not exact_name:
                return False
            self._gids[key] = gid
            self._reliabilities[key] = reliability
            return True
        return (gid == pinned_gid and
                reliability == self._reliabilities[key] and
                (exact_name or unknown_name))


def verified_raw_gate(pins, endpoints, gate_reliability,
                      recorder_reliabilities, require_supervisor=False):
    """Require the SafetyGate and only exact, read-only observers."""
    minimum = 2 if require_supervisor else 1
    if len(endpoints) not in (minimum, minimum + 1):
        return False
    gate = [info for info in endpoints if pins.matches(
        'raw_gate', info, name='safety_watchdog',
        message_type='geometry_msgs/msg/Twist',
        reliability=gate_reliability)]
    if len(gate) != 1:
        return False
    observers = [info for info in endpoints if info is not gate[0]]
    if require_supervisor:
        supervisor = [info for info in observers if pins.matches(
            'raw_supervisor', info, name='fire_mission_supervisor',
            message_type='geometry_msgs/msg/Twist',
            reliability=gate_reliability)]
        if len(supervisor) != 1:
            return False
        observers = [info for info in observers
                     if info is not supervisor[0]]
    if not observers:
        return True
    if len(observers) != 1:
        return False
    recorder = observers[0]
    if recorder.qos_profile.reliability not in recorder_reliabilities:
        return False
    return pins.matches(
        'raw_bag_recorder', recorder, name='rosbag2_recorder',
        message_type='geometry_msgs/msg/Twist',
        reliability=recorder.qos_profile.reliability)


ACQUIRING_DISCOVERY_BLOCKERS = frozenset({
    'SCAN_PUBLISHER_INVALID', 'SCAN_SOURCE_INVALID',
    'ODOM_PUBLISHER_INVALID', 'ODOM_SOURCE_INVALID',
    'CMD_PUBLISHER_INVALID', 'SAFETY_GATE_TOPOLOGY_INVALID',
})
MAX_INTEGRATED_STEPS = 20
MAX_INTEGRATED_DISTANCE_M = 0.30
MAX_TOTAL_CONTINUOUS_DISTANCE_M = 2.0
MAX_SEGMENT_OVERRUN_M = 0.03
MAX_CONTINUOUS_SPEED_M_S = 0.08
SPRAY_CONFIRM_WAIT_S = 1.5


def vision_far_candidate(payload):
    """
    Identify a fresh centered YOLO fire with stable baseline IR.

    It is not an IR distance measurement and can never authorize spraying.
    """
    if not isinstance(payload, dict):
        return False
    safety = payload.get('safety') or {}
    counts = payload.get('filtered_counts') or {}
    return (payload.get('schema_version') == 1 and
            payload.get('mode') == 'OBSERVE_ONLY' and
            payload.get('observe_only') is True and
            payload.get('calibration') == 'PROVISIONAL_SINGLE_FLAME' and
            payload.get('validated_for_actuation') is False and
            payload.get('zone') == 'UNKNOWN' and
            payload.get('ir_band') == 'NO_FLAME_SIGNATURE' and
            payload.get('blockers') == ['NO_IR_FLAME_SIGNATURE'] and
            type(payload.get('stable_samples')) is int and
            payload['stable_samples'] >= 3 and
            _finite(payload.get('ir_receive_age_ms')) and
            0 <= payload['ir_receive_age_ms'] <= 300 and
            all(type(counts.get(key)) is int and
                0 <= counts[key] <= 32767
                for key in ('left', 'center', 'right')) and
            safety.get('motion_command_publisher') is False and
            safety.get('actuator_command_publisher') is False)


def provisional_spray_candidate(payload):
    """Stop on the first valid spray candidate; never use it to start a pump."""
    if not isinstance(payload, dict):
        return False
    safety = payload.get('safety') or {}
    counts = payload.get('filtered_counts') or {}
    blockers = payload.get('blockers')
    return (payload.get('schema_version') == 1 and
            payload.get('mode') == 'OBSERVE_ONLY' and
            payload.get('observe_only') is True and
            payload.get('calibration') == 'PROVISIONAL_SINGLE_FLAME' and
            payload.get('validated_for_actuation') is False and
            safety.get('motion_command_publisher') is False and
            safety.get('actuator_command_publisher') is False and
            payload.get('ir_band') == 'SPRAY_RANGE' and
            payload.get('zone') in ('UNKNOWN', 'SPRAY_RANGE') and
            isinstance(blockers, list) and
            set(blockers) <= {'IR_BAND_NOT_STABLE',
                              'IR_SPRAY_REENTRY_HYSTERESIS'} and
            _finite(payload.get('ir_receive_age_ms')) and
            0 <= payload['ir_receive_age_ms'] <= 300.0 and
            type(payload.get('stable_samples')) is int and
            payload['stable_samples'] >= 1 and
            type(counts.get('right')) is int and
            0 <= counts['right'] <= 32767)


def realign_candidate(payload):
    """Identify a confirmed fire shifted sideways, not a lost sensor."""
    if not isinstance(payload, dict):
        return False
    safety = payload.get('safety') or {}
    return (payload.get('schema_version') == 1 and
            payload.get('mode') == 'OBSERVE_ONLY' and
            payload.get('observe_only') is True and
            payload.get('zone') == 'UNKNOWN' and
            payload.get('ir_band') in ('APPROACH', 'TOO_FAR') and
            payload.get('blockers') == ['FIRE_NOT_CENTERED'] and
            _finite(payload.get('ir_receive_age_ms')) and
            0 <= payload['ir_receive_age_ms'] <= 300.0 and
            safety.get('motion_command_publisher') is False and
            safety.get('actuator_command_publisher') is False)


def target_reacquire_candidate(payload):
    """
    Identify a fresh, unconfirmed target for safe stop and realignment.

    This is not used for stale/malformed IR or obstacle/owner faults.
    """
    if not isinstance(payload, dict):
        return False
    safety = payload.get('safety') or {}
    counts = payload.get('filtered_counts') or {}
    return (payload.get('schema_version') == 1 and
            payload.get('mode') == 'OBSERVE_ONLY' and
            payload.get('observe_only') is True and
            payload.get('calibration') == 'PROVISIONAL_SINGLE_FLAME' and
            payload.get('validated_for_actuation') is False and
            payload.get('zone') == 'UNKNOWN' and
            payload.get('ir_band') in (
                'APPROACH', 'TOO_FAR', 'NO_FLAME_SIGNATURE') and
            isinstance(payload.get('blockers'), list) and
            'FIRE_NOT_CONFIRMED' in payload['blockers'] and
            set(payload['blockers']) <= {
                'FIRE_NOT_CONFIRMED', 'FIRE_NOT_CENTERED',
                'NO_IR_FLAME_SIGNATURE'} and
            _finite(payload.get('ir_receive_age_ms')) and
            0 <= payload['ir_receive_age_ms'] <= 300.0 and
            type(counts.get('right')) is int and
            0 <= counts['right'] <= 32767 and
            safety.get('motion_command_publisher') is False and
            safety.get('actuator_command_publisher') is False)


class IrApproachLiveCore:
    """Bound one motion attempt; abort on any missing or contradictory input."""

    def __init__(self, *, enabled=False, allow_provisional_ir=False,
                 enable_reverse_jog=False, integrated_handoff=False,
                 continuous_approach=False,
                 step_m=0.05, speed_m_s=0.05, max_duration_s=1.0,
                 continuous_distance_m=0.30,
                 max_total_approach_m=2.0,
                 input_timeout_s=0.35, arm_timeout_s=0.40,
                 discovery_timeout_s=0.75, admission_hold_s=0.3,
                 zero_hold_s=0.5,
                 clearance_margin_m=0.45):
        if (type(integrated_handoff) is not bool or
                type(continuous_approach) is not bool or
                type(enabled) is not bool or
                type(allow_provisional_ir) is not bool or
                type(enable_reverse_jog) is not bool):
            raise ValueError('opt-in flags must be bool')
        values = (step_m, speed_m_s, max_duration_s,
                  continuous_distance_m,
                  input_timeout_s,
                  arm_timeout_s, discovery_timeout_s, admission_hold_s,
                  zero_hold_s,
                  clearance_margin_m)
        if any(not _finite(v) or v <= 0 for v in values):
            raise ValueError('IR jog parameters must be positive and finite')
        if not _finite(max_total_approach_m) or max_total_approach_m < 0:
            raise ValueError('max_total_approach_m must be nonnegative')
        if (step_m > 0.05 or max_duration_s > 1.0 or
                input_timeout_s > 0.35 or arm_timeout_s > 0.40 or
                discovery_timeout_s > 0.75 or admission_hold_s < 0.3 or
                zero_hold_s < 0.5 or
                clearance_margin_m < 0.45):
            raise ValueError('IR jog hard safety bound exceeded')
        if continuous_approach:
            if (not integrated_handoff or enable_reverse_jog or
                    speed_m_s > MAX_CONTINUOUS_SPEED_M_S or
                    continuous_distance_m > MAX_INTEGRATED_DISTANCE_M or
                    (max_total_approach_m >
                     MAX_TOTAL_CONTINUOUS_DISTANCE_M)):
                raise ValueError('continuous IR safety bound exceeded')
        elif speed_m_s > 0.06 or speed_m_s * max_duration_s > step_m:
            raise ValueError('IR jog hard safety bound exceeded')
        if not continuous_approach and max_total_approach_m == 0:
            raise ValueError('unbounded mode requires integrated approach')
        self.enabled = enabled
        self.allow_provisional_ir = allow_provisional_ir
        self.enable_reverse_jog = enable_reverse_jog
        self.integrated_handoff = integrated_handoff
        self.continuous_approach = continuous_approach
        self.step_m = float(step_m)
        self.speed = float(speed_m_s)
        self.max_duration = float(max_duration_s)
        self.continuous_distance = float(continuous_distance_m)
        self.max_total_approach = (math.inf if max_total_approach_m == 0 else
                                   float(max_total_approach_m))
        self.input_timeout = float(input_timeout_s)
        self.arm_timeout = float(arm_timeout_s)
        self.discovery_timeout = float(discovery_timeout_s)
        self.admission_hold = float(admission_hold_s)
        self.zero_hold = float(zero_hold_s)
        self.clearance_margin = float(clearance_margin_m)
        self.inputs = {}
        self.arm = False
        self.arm_at = None
        self.arm_false_seen = False
        self.arm_cycle_seen = False
        self.admission_since = None
        self.pending_direction = 0
        self.phase = 'WAITING_FOR_ADMISSION'
        self.direction = 0
        self.acquired_at = None
        self.moving_at = None
        self.start_pose = None
        self.last_progress_m = 0.0
        self.last_progress_at = None
        self.stopping_at = None
        self.zero_since = None
        self.stop_reason = ''
        self.stop_trigger_blockers = []
        self.goal_uuid = None
        self.step_count = 0
        self.step_progress_m = 0.0
        self.total_progress_m = 0.0
        self.last_review_pulse = 0
        self.pending_review = None
        self.used_goal_uuids = set()
        self.recovery_sample_seq = None
        self.sensor_recovery_active = False

    def update(self, name, value, now):
        self.inputs[name] = (value, float(now))

    def _mark_sensor_recovery(self):
        """Require two *new* accepted ADC samples after a recoverable stop."""
        self.sensor_recovery_active = True
        payload = self.inputs.get('ir', (None, None))[0]
        seq = payload.get('ir_sample_seq') if isinstance(payload, dict) else None
        self.recovery_sample_seq = seq if type(seq) is int and seq >= 0 else None

    def _sensor_recovery_pending(self):
        return self.sensor_recovery_active

    def _motion_ir_band(self, payload):
        """Integrated motion also requires the vision-gated stable zone."""
        if self.integrated_handoff and vision_far_candidate(payload):
            return 'VISION_FAR'
        band = validated_ir_band(payload)
        if self.integrated_handoff and (
                not isinstance(payload, dict) or
                payload.get('zone') != band or
                payload.get('blockers') != []):
            return None
        return band

    def _recovery_ready(self, now):
        """Allow re-centering after stable ADC recovery, never raw motion."""
        if not self._sensor_recovery_pending():
            return True
        payload = self._input('ir', now)
        if not isinstance(payload, dict):
            return False
        seq = payload.get('ir_sample_seq')
        if type(seq) is not int or seq < 0:
            return False
        if self.recovery_sample_seq is None:
            self.recovery_sample_seq = seq
            return False
        if seq < self.recovery_sample_seq:
            # The preview process restarted and its local counter began a
            # new epoch. Never compare it with the old process's counter.
            self.recovery_sample_seq = seq
            return False
        return (seq >= self.recovery_sample_seq + 2 and
                self._motion_ir_band(payload) is not None)

    def _clear_cycle(self):
        self.used_goal_uuids.add(self.goal_uuid)
        self.phase = 'WAITING_FOR_ADMISSION'
        self.goal_uuid = None
        self.direction = 0
        self.step_count = 0
        self.step_progress_m = 0.0
        self.total_progress_m = 0.0
        self.last_review_pulse = 0
        self.pending_review = None
        self.admission_since = None
        self.pending_direction = 0
        self.acquired_at = None
        self.moving_at = None
        self.start_pose = None
        self.last_progress_m = 0.0
        self.last_progress_at = None
        self.stopping_at = None
        self.zero_since = None
        self.stop_reason = ''
        self.stop_trigger_blockers = []
        self.recovery_sample_seq = None
        self.sensor_recovery_active = False

    def update_arm(self, armed, now):
        if armed is False:
            self.arm_false_seen = True
        elif armed is True and self.arm_false_seen:
            self.arm_cycle_seen = True
        self.arm = armed is True
        self.arm_at = float(now)

    def _input(self, name, now):
        value, received = self.inputs.get(name, (None, None))
        return value if _fresh(received, now, self.input_timeout) else None

    @staticmethod
    def _zero_twist(values, epsilon):
        return (isinstance(values, tuple) and len(values) == 6 and
                all(_finite(v) and abs(v) <= epsilon for v in values))

    @staticmethod
    def _odom_valid(odom):
        return (isinstance(odom, tuple) and len(odom) == 3 and
                isinstance(odom[0], tuple) and len(odom[0]) == 2 and
                all(_finite(v) for v in odom[0]) and
                isinstance(odom[1], tuple) and len(odom[1]) == 6 and
                all(_finite(v) for v in odom[1]) and
                isinstance(odom[2], str) and odom[2] == 'odom')

    def _checks(self, now, counts, owned, moving=False):
        blockers = []
        if not self.enabled:
            blockers.append('LIVE_IR_JOG_DISABLED')
        if not self.allow_provisional_ir:
            blockers.append('PROVISIONAL_IR_OVERRIDE_DISABLED')
        if not (self.arm_cycle_seen and self.arm and
                _fresh(self.arm_at, now, self.arm_timeout) and
                counts.get('arm') == 1):
            blockers.append('OPERATOR_ARM_MISSING_OR_STALE')
        for name in ('ir', 'scan', 'odom', 'cmd'):
            if counts.get(name) != 1:
                blockers.append(name.upper() + '_PUBLISHER_INVALID')
        for name in ('ir', 'scan', 'odom'):
            if not counts.get(name + '_owner'):
                blockers.append(name.upper() + '_SOURCE_INVALID')
        if not counts.get('safety_owner') or not counts.get('raw_gate'):
            blockers.append('SAFETY_GATE_TOPOLOGY_INVALID')
        if self.integrated_handoff:
            if (counts.get('nav') != 1 or
                    not counts.get('nav_shadow_ok') or
                    not counts.get('status_source_ok')):
                blockers.append('NAV2_TOPOLOGY_INVALID')
            proof = ('runtime_ok' if owned else 'handoff_ok')
            if not counts.get(proof) or not counts.get('goal_uuid') or (
                    self.goal_uuid is not None and
                    counts.get('goal_uuid') != self.goal_uuid):
                blockers.append('EXACT_HANDOFF_PROOF_INVALID')
        elif counts.get('nav') != 0 or counts.get('nav_cmd') != 0:
            blockers.append('NAV2_PRESENT')
        if counts.get('pump') != 0:
            blockers.append('PUMP_OWNER_PRESENT')
        if owned:
            acquiring_not_discovered = (
                self.phase == 'ACQUIRING' and counts.get('raw') == 0)
            if not acquiring_not_discovered and (
                    counts.get('raw') != 1 or not counts.get('raw_owned')):
                blockers.append('RAW_OWNER_CONFLICT')
        elif counts.get('raw') != 0:
            blockers.append('RAW_OWNER_CONFLICT')

        ir = self._input('ir', now)
        band = self._motion_ir_band(ir)
        if band is None:
            blockers.append('IR_BAND_INVALID_OR_STALE')
        direction = (1 if band in ('APPROACH', 'TOO_FAR', 'VISION_FAR') else
                     -1 if band == 'TOO_CLOSE' else 0)
        if direction == 0:
            blockers.append('IR_BAND_HOLD')
        elif direction < 0 and not self.enable_reverse_jog:
            blockers.append('REVERSE_JOG_DISABLED')
        elif moving and direction != self.direction:
            blockers.append('IR_DIRECTION_CHANGED')

        odom = self._input('odom', now)
        if not self._odom_valid(odom):
            blockers.append('ODOM_MISSING_OR_INVALID')
        elif moving:
            twist = odom[1]
            if (any(abs(v) > 0.12 for v in twist[:3]) or
                    any(abs(v) > 0.15 for v in twist[3:]) or
                    twist[0] * self.direction < -0.02):
                blockers.append('ODOM_MOTION_INVALID_OR_OVERSPEED')
        elif (any(abs(v) > 0.02 for v in odom[1][:3]) or
              any(abs(v) > 0.05 for v in odom[1][3:])):
            blockers.append('ROBOT_NOT_STATIONARY')

        cmd = self._input('cmd', now)
        if not isinstance(cmd, tuple) or len(cmd) != 6 or \
                not all(_finite(v) for v in cmd):
            blockers.append('CMD_VEL_MISSING_OR_INVALID')
        elif moving:
            if (abs(cmd[0]) > self.speed + 0.02 or
                    abs(cmd[1]) > 0.001 or abs(cmd[2]) > 0.001 or
                    any(abs(v) > 0.02 for v in cmd[3:]) or
                    cmd[0] * self.direction < -0.001):
                blockers.append('SAFETY_COMMAND_CONFLICT')
        elif not self._zero_twist(cmd, 0.001):
            blockers.append('CMD_VEL_NOT_ZERO')

        scan = self._input('scan', now)
        if direction:
            clearance, error = sector_clearance(
                scan, 0.0 if direction > 0 else math.pi, 0.35,
                self.clearance_margin + (0.08 if self.continuous_approach
                                         else self.step_m))
        else:
            clearance, error = None, None
        if error:
            blockers.append('SCAN_' + error)
        return blockers, direction, band, clearance

    def request_session_stop(self, reason, now, *, reset_hold=False):
        """Latch STOPPING without resetting distance, goal, arm or review history."""
        if reason not in ('SESSION_CONTEXT_INVALID', 'SESSION_CLOSE_REQUESTED'):
            raise ValueError('invalid IR session stop')
        if self.phase in ('ACQUIRING', 'MOVING', 'STOPPING'):
            odom = self._input('odom', now)
            if self.start_pose is not None and self._odom_valid(odom):
                self.step_progress_m = max(self.step_progress_m, math.hypot(
                    odom[0][0] - self.start_pose[0], odom[0][1] - self.start_pose[1]))
            if self.phase != 'STOPPING':
                self.phase = 'STOPPING'
                self.stop_reason = reason
                self.stopping_at = now
                self.zero_since = None
            elif self.stop_reason == 'SPRAY_CANDIDATE_STOP':
                # Closing a session needs a safe release, not spray confirmation.
                self.stop_reason = reason
            if reset_hold:
                self.zero_since = None

    def publisher_released(self, *, account_abort=False):
        """Commit a qualified release once, after the adapter confirms its effect."""
        if type(account_abort) is not bool:
            raise ValueError('IR abort accounting gate must be bool')
        if self.phase != 'STOPPING' or self.stop_reason == 'SPRAY_CANDIDATE_STOP':
            raise RuntimeError('IR_RELEASE_NOT_PENDING')
        self.phase = ('COMPLETE' if self.stop_reason in (
            'STEP_DISTANCE_REACHED', 'STEP_PROGRESS_TIME_LIMIT',
            'SPRAY_RANGE_REACHED', 'REALIGN_REQUIRED',
            'APPROACH_DISTANCE_LIMIT') else 'ABORTED')
        if self.integrated_handoff and (
                self.phase == 'COMPLETE' or account_abort):
            self.step_count += 1
            self.total_progress_m += self.step_progress_m

    def step(self, now, counts, owned, *, session_admission=True,
             session_release_ready=True, allow_rearm=True, defer_release=False):
        """Return (linear_x, telemetry, CREATE/RELEASE/None)."""
        now = float(now)
        if any(type(flag) is not bool for flag in (
                session_admission, session_release_ready, allow_rearm, defer_release)):
            raise ValueError('IR session gates must be bool')
        new_uuid = counts.get('goal_uuid')
        review = self._input('review', now) or {}
        actuator = self._input('actuator', now) or {}
        if (allow_rearm and self.phase == 'COMPLETE' and
                self.stop_reason == 'SPRAY_RANGE_REACHED' and
                self.goal_uuid and counts.get('review_source_ok') is True and
                review.get('schema_version') == 1 and
                review.get('mode') == 'SHADOW_ONLY' and
                review.get('state') == 'APPROACH_REQUIRED' and
                review.get('goal_uuid') == self.goal_uuid and
                review.get('blockers') == [] and
                type(review.get('pulse_index')) is int and
                review['pulse_index'] > self.last_review_pulse):
            self.pending_review = (review['pulse_index'], now)
        if self.pending_review is not None and \
                now - self.pending_review[1] > 30.0:
            self.pending_review = None
        review_resume = (
            self.pending_review is not None and
            counts.get('actuator_source_ok') is True and
            actuator.get('pump_pwm_commanded_on') is False and
            actuator.get('servo_pwm_commanded_on') is False and
            actuator.get('servo_phase') == 'IDLE' and
            isinstance(actuator.get('receive_age_ms'), (int, float)) and
            0 <= actuator['receive_age_ms'] < 350)
        # Re-centering can bring the flame into spray range without any
        # further IR motion. Resolve the old recoverable stop using fresh,
        # exact canceled-goal and stationary proof; never claim raw motion.
        if (allow_rearm and self.integrated_handoff and self.continuous_approach and
                self.phase == 'COMPLETE' and not owned and self.goal_uuid and
                new_uuid == self.goal_uuid and counts.get('pump') == 0 and
                self._recovery_ready(now) and
                counts.get('range_handoff_ok') is True and
                (self.stop_reason in (
                    'REALIGN_REQUIRED', 'APPROACH_DISTANCE_LIMIT') or
                 (self.stop_reason == 'SPRAY_RANGE_REACHED' and
                  review_resume))):
            if review_resume:
                self.last_review_pulse = self.pending_review[0]
                self.pending_review = None
            self.stop_reason = 'SPRAY_RANGE_REACHED'
        if (allow_rearm and self.integrated_handoff and self.continuous_approach and
                self.phase == 'COMPLETE' and not owned and
                (self.stop_reason in (
                    'REALIGN_REQUIRED', 'APPROACH_DISTANCE_LIMIT') or
                 (self.stop_reason == 'SPRAY_RANGE_REACHED' and
                  review_resume)) and
                self.total_progress_m < self.max_total_approach and
                self.goal_uuid and new_uuid == self.goal_uuid and
                self._recovery_ready(now) and
                counts.get('handoff_ok') is True and
                counts.get('pump') == 0 and
                self._motion_ir_band(self._input('ir', now)) in
                ('APPROACH', 'TOO_FAR', 'VISION_FAR')):
            if review_resume:
                self.last_review_pulse = self.pending_review[0]
                self.pending_review = None
            # A fresh centered-fire and canceled-goal proof has been rebuilt
            # after IR stopped. Retain the traveled-distance accounting and
            # do not reacquire raw motion until normal admission passes.
            self.phase = 'WAITING_FOR_ADMISSION'
            self.admission_since = None
            self.pending_direction = 0
            self.acquired_at = None
            self.moving_at = None
            self.start_pose = None
            self.last_progress_m = 0.0
            self.last_progress_at = None
            self.stopping_at = None
            self.zero_since = None
            self.stop_reason = ''
            self.stop_trigger_blockers = []
            self.recovery_sample_seq = None
            self.sensor_recovery_active = False
            self.step_progress_m = 0.0
        odom_clear = self._input('odom', now)
        clear_stationary = (
            self._odom_valid(odom_clear) and
            all(abs(v) <= 0.02 for v in odom_clear[1][:3]) and
            all(abs(v) <= 0.05 for v in odom_clear[1][3:]) and
            self._zero_twist(self._input('cmd', now), 0.001))
        if (allow_rearm and self.integrated_handoff and
                self.phase in ('COMPLETE', 'ABORTED') and not owned and
                self.goal_uuid and
                counts.get('pre_spray_clear_uuid') == self.goal_uuid and
                counts.get('pump') == 0 and counts.get('raw') == 0 and
                clear_stationary):
            self._clear_cycle()
        if (allow_rearm and self.integrated_handoff and self.phase == 'COMPLETE' and
                not owned and self.goal_uuid and new_uuid and
                new_uuid != self.goal_uuid and
                new_uuid not in self.used_goal_uuids and
                counts.get('handoff_ok') and counts.get('pump') == 0):
            self._clear_cycle()
        if (allow_rearm and self.integrated_handoff and not self.continuous_approach and
                self.phase == 'COMPLETE' and
                not owned and self.stop_reason != 'SPRAY_RANGE_REACHED' and
                self.step_count < MAX_INTEGRATED_STEPS and
                self.total_progress_m < MAX_INTEGRATED_DISTANCE_M and
                counts.get('handoff_ok') and
                counts.get('goal_uuid') == self.goal_uuid and
                self._motion_ir_band(self._input('ir', now)) in
                ('APPROACH', 'TOO_FAR')):
            self.phase = 'WAITING_FOR_ADMISSION'
            self.admission_since = None
            self.pending_direction = 0
            self.acquired_at = None
            self.moving_at = None
            self.start_pose = None
            self.last_progress_m = 0.0
            self.last_progress_at = None
            self.stopping_at = None
            self.zero_since = None
            self.stop_reason = ''
            self.stop_trigger_blockers = []
            self.recovery_sample_seq = None
            self.sensor_recovery_active = False
            self.step_progress_m = 0.0
        command = 0.0
        action = None
        blockers = []
        band = None
        clearance = None
        if self.phase == 'WAITING_FOR_ADMISSION':
            blockers, direction, band, clearance = self._checks(
                now, counts, owned=False)
            if not session_admission:
                blockers.append('SESSION_ADMISSION_MISSING')
            if blockers:
                self.admission_since = None
                self.pending_direction = 0
            elif self.admission_since is None or \
                    direction != self.pending_direction:
                self.admission_since = now
                self.pending_direction = direction
            elif now - self.admission_since >= self.admission_hold:
                self.direction = direction
                if self.integrated_handoff:
                    self.goal_uuid = counts['goal_uuid']
                self.phase = 'ACQUIRING'
                self.acquired_at = now
                action = 'CREATE'
        elif self.phase in ('ACQUIRING', 'MOVING'):
            moving = self.phase == 'MOVING'
            blockers, _, band, clearance = self._checks(
                now, counts, owned=owned, moving=moving)
            if not owned:
                blockers.append('RAW_PUBLISHER_LOST')
            if not self.continuous_approach and \
                    now - self.acquired_at > self.max_duration + \
                    self.discovery_timeout:
                blockers.append('TOTAL_TIME_LIMIT')
            retry_discovery = (
                self.phase == 'ACQUIRING' and blockers and
                set(blockers).issubset(ACQUIRING_DISCOVERY_BLOCKERS) and
                now - self.acquired_at <= self.discovery_timeout)
            if not blockers and self.phase == 'ACQUIRING':
                if (counts.get('raw') == 1 and counts.get('raw_owned') and
                        (not self.integrated_handoff or
                         counts.get('guard_owner_ack'))):
                    odom = self._input('odom', now)
                    self.start_pose = odom[0]
                    self.moving_at = now
                    self.last_progress_at = now
                    self.phase = 'MOVING'
                elif now - self.acquired_at > self.discovery_timeout:
                    blockers.append('OWN_PUBLISHER_NOT_DISCOVERED')
            if not blockers and self.phase == 'MOVING':
                if self.integrated_handoff and not counts.get(
                        'guard_owner_ack'):
                    blockers.append('IR_GUARD_ACK_LOST')
            spray_candidate = (self.continuous_approach and moving and
                               provisional_spray_candidate(
                                   self._input('ir', now)))
            spray_transition_only = (set(blockers) <= {
                'IR_BAND_INVALID_OR_STALE', 'IR_BAND_HOLD',
                'EXACT_HANDOFF_PROOF_INVALID'})
            if spray_candidate and spray_transition_only:
                odom = self._input('odom', now)
                if self._odom_valid(odom):
                    self.step_progress_m = math.hypot(
                        odom[0][0] - self.start_pose[0],
                        odom[0][1] - self.start_pose[1])
                    if (self.step_progress_m < self.continuous_distance and
                            self.total_progress_m + self.step_progress_m <
                            self.max_total_approach):
                        self.stop_reason = 'SPRAY_CANDIDATE_STOP'
                        self.stopping_at = now
                        self.phase = 'STOPPING'
                        blockers = []
            if not blockers and self.phase == 'MOVING':
                odom = self._input('odom', now)
                distance = math.hypot(odom[0][0] - self.start_pose[0],
                                      odom[0][1] - self.start_pose[1])
                if (self.continuous_approach and
                        self.total_progress_m + distance >=
                        self.max_total_approach):
                    self.step_progress_m = distance
                    self.stop_reason = 'APPROACH_TOTAL_DISTANCE_LIMIT'
                    self.phase = 'STOPPING'
                elif self.continuous_approach and \
                        distance >= self.continuous_distance:
                    self.step_progress_m = distance
                    self.stop_reason = 'APPROACH_DISTANCE_LIMIT'
                    self.phase = 'STOPPING'
                elif not self.continuous_approach and distance >= self.step_m:
                    self.step_progress_m = distance
                    self.stop_reason = 'STEP_DISTANCE_REACHED'
                    self.phase = 'STOPPING'
                elif not self.continuous_approach and \
                        now - self.moving_at >= self.max_duration:
                    self.step_progress_m = distance
                    self.stop_reason = ('STEP_PROGRESS_TIME_LIMIT' if
                                        self.integrated_handoff and
                                        distance >= 0.01 else
                                        'STEP_TIME_LIMIT')
                    self.phase = 'STOPPING'
                elif (self.continuous_approach and
                      now - self.last_progress_at >= 1.0 and
                      distance - self.last_progress_m < 0.01):
                    self.step_progress_m = distance
                    self.stop_reason = 'ODOM_PROGRESS_LOST'
                    self.phase = 'STOPPING'
                else:
                    if self.continuous_approach and \
                            distance - self.last_progress_m >= 0.01:
                        self.last_progress_m = distance
                        self.last_progress_at = now
                    command = self.direction * self.speed
            if blockers and not retry_discovery and self.phase != 'STOPPING':
                self.stop_trigger_blockers = list(blockers)
                if moving and band == 'SPRAY_RANGE' and \
                        set(blockers) == {'IR_BAND_HOLD'}:
                    odom = self._input('odom', now)
                    self.step_progress_m = math.hypot(
                        odom[0][0] - self.start_pose[0],
                        odom[0][1] - self.start_pose[1])
                    self.stop_reason = 'SPRAY_RANGE_REACHED'
                elif (moving and self.integrated_handoff and
                      self.continuous_approach and
                      (realign_candidate(self._input('ir', now)) or
                       target_reacquire_candidate(self._input('ir', now))) and
                      set(blockers) <= {
                          'IR_BAND_INVALID_OR_STALE', 'IR_BAND_HOLD',
                          'IR_DIRECTION_CHANGED',
                          'EXACT_HANDOFF_PROOF_INVALID'} and
                      counts.get('scan_owner') is True and
                      self._input('scan', now) is not None):
                    self.stop_reason = 'REALIGN_REQUIRED'
                elif (owned and self.integrated_handoff and
                      self.continuous_approach and
                      'IR_BAND_INVALID_OR_STALE' in blockers and
                      set(blockers) <= {'IR_BAND_INVALID_OR_STALE',
                                        'IR_BAND_HOLD',
                                        'EXACT_HANDOFF_PROOF_INVALID'} and
                      counts.get('goal_uuid') == self.goal_uuid):
                    # Stop immediately; only a fresh, multi-sample proof may
                    # later re-enter alignment/IR on this exact goal.
                    self.stop_reason = 'REALIGN_REQUIRED'
                    self._mark_sensor_recovery()
                else:
                    self.stop_reason = blockers[0]
                self.phase = 'STOPPING'
        elif self.phase == 'STOPPING':
            odom = self._input('odom', now)
            cmd = self._input('cmd', now)
            if self.continuous_approach and self._odom_valid(odom) and \
                    self.start_pose is not None:
                self.step_progress_m = math.hypot(
                    odom[0][0] - self.start_pose[0],
                    odom[0][1] - self.start_pose[1])
                if (self.continuous_approach and
                        self.step_progress_m > self.continuous_distance +
                        MAX_SEGMENT_OVERRUN_M):
                    self.stop_reason = 'APPROACH_SEGMENT_OVERRUN'
                elif self.total_progress_m + self.step_progress_m >= \
                        self.max_total_approach:
                    self.stop_reason = 'APPROACH_TOTAL_DISTANCE_LIMIT'
                elif (self.stop_reason != 'REALIGN_REQUIRED' and
                      self.step_progress_m >= self.continuous_distance):
                    self.stop_reason = 'APPROACH_DISTANCE_LIMIT'
            if self.stop_reason == 'SPRAY_CANDIDATE_STOP' and \
                    now - self.stopping_at >= SPRAY_CONFIRM_WAIT_S and \
                    self._motion_ir_band(self._input('ir', now)) != \
                    'SPRAY_RANGE':
                self.stop_reason = 'REALIGN_REQUIRED'
                self.stop_trigger_blockers.append('SPRAY_NOT_CONFIRMED')
                self._mark_sensor_recovery()
            if (owned and counts.get('raw') == 1 and
                    counts.get('raw_owned') and self._odom_valid(odom) and
                    all(abs(v) <= 0.02 for v in odom[1][:3]) and
                    all(abs(v) <= 0.05 for v in odom[1][3:]) and
                    self._zero_twist(cmd, 0.001)):
                if self.zero_since is None:
                    self.zero_since = now
                elif (session_release_ready and
                      now - self.zero_since >= self.zero_hold and
                      self.stop_reason != 'SPRAY_CANDIDATE_STOP'):
                    if not defer_release:
                        self.publisher_released()
                    action = 'RELEASE'
                elif (session_release_ready and
                      self.stop_reason == 'SPRAY_CANDIDATE_STOP' and
                      now - self.zero_since >= self.zero_hold and
                      self._motion_ir_band(self._input('ir', now)) ==
                      'SPRAY_RANGE' and
                      self._input('ir', now).get('zone') == 'SPRAY_RANGE' and
                      self._input('ir', now).get('blockers') == []):
                    self.stop_reason = 'SPRAY_RANGE_REACHED'
                    if not defer_release:
                        self.publisher_released()
                    action = 'RELEASE'
            else:
                self.zero_since = None
        payload = {
            'schema_version': 1,
            'state': ('DISABLED' if not self.enabled and self.phase ==
                      'WAITING_FOR_ADMISSION' else self.phase),
            'enabled': self.enabled,
            'one_shot': True,
            'integrated_handoff': self.integrated_handoff,
            'continuous_approach': self.continuous_approach,
            'goal_uuid': self.goal_uuid,
            'step_count': self.step_count,
            'step_progress_m': round(self.step_progress_m, 4),
            'total_progress_m': round(self.total_progress_m, 4),
            'approach_budget_exhausted': (
                self.integrated_handoff and
                ((not self.continuous_approach and
                  self.step_count >= MAX_INTEGRATED_STEPS) or
                 self.total_progress_m >= (
                     self.max_total_approach if self.continuous_approach
                     else MAX_INTEGRATED_DISTANCE_M))),
            'ir_band': band,
            'direction': ('FORWARD' if self.direction > 0 else
                          'REVERSE' if self.direction < 0 else 'STOP'),
            'command': {'linear_x_m_s': round(command, 4),
                        'angular_z_rad_s': 0.0},
            'clearance_m': clearance,
            'stop_reason': self.stop_reason,
            'stop_trigger_blockers': self.stop_trigger_blockers,
            'recovery_required': self.sensor_recovery_active,
            'recovery_ready': self._recovery_ready(now) if
            self.phase == 'COMPLETE' else False,
            'arm': {
                'false_seen': self.arm_false_seen,
                'cycle_seen': self.arm_cycle_seen,
                'heartbeat_true': self.arm,
                'receive_fresh': _fresh(self.arm_at, now, self.arm_timeout),
                'receive_age_ms': (
                    None if self.arm_at is None else
                    round(max(0.0, now - self.arm_at) * 1000, 1)),
                'publisher_count': counts.get('arm'),
            },
            'blockers': blockers,
            'publisher_owned': owned,
            'safety': {'pump_command_publisher': False,
                       'requires_operator_arm': True,
                       'one_step_only': not self.continuous_approach},
        }
        return command, payload, action
