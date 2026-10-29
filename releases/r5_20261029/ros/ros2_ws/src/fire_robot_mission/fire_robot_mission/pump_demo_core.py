"""Fail-closed, one-shot supervised pump pulse; no ROS imports."""

import math
from .no_goal_cycle import context_of,stopped_pair,same_cycle,registered_navigation


POSSIBLE_EARLY_EXTINGUISH = frozenset({
    'IR_NOT_CONFIRMED_SPRAY_RANGE',
    'MISSION_NOT_SAFE_AND_CENTERED',
})


def fresh(received_at, now, limit):
    return (received_at is not None and 0.0 <= now - received_at <= limit)


def zero_twist(values, limit):
    return (isinstance(values, (tuple, list)) and len(values) == 6 and
            all(type(value) in (int, float) and math.isfinite(value) and
                abs(value) <= limit for value in values))


class PumpDemoCore:
    """Issue at most one pulse, with continuous sensor and operator checks."""

    def __init__(self, *, enabled=False, allow_provisional_ir=False,
                 pulse_s=5.0, start_on_ack=False, hold_s=0.4, input_timeout_s=0.35,
                 arm_timeout_s=0.4):
        if type(enabled) is not bool or type(allow_provisional_ir) is not bool:
            raise ValueError('enable and provisional override must be bool')
        for value in (pulse_s, hold_s, input_timeout_s, arm_timeout_s):
            if (type(value) not in (int, float) or not math.isfinite(value) or
                    value <= 0):
                raise ValueError('pump timing must be finite and positive')
        if (type(start_on_ack) is not bool or pulse_s > (6.0 if start_on_ack else 5.0) or hold_s < 0.4 or input_timeout_s > 0.35 or
                arm_timeout_s > 0.4):
            raise ValueError('pump hard safety limit exceeded')
        self.enabled = enabled
        self.allow_provisional_ir = allow_provisional_ir
        self.pulse_s = float(pulse_s)
        self.hold_s = float(hold_s)
        self.input_timeout_s = float(input_timeout_s)
        self.arm_timeout_s = float(arm_timeout_s)
        self.r5_cycle_context=None;self.cycle_id=None
        self.inputs = {}
        self.arm = False
        self.arm_at = None
        self.arm_false_seen = False
        self.arm_cycle_seen = False
        self.odom = None
        self.odom_at = None
        self.cmd = None
        self.cmd_at = None
        self.statuses = None
        self.status_at = None
        self.ready_since = None
        self.on_since = None
        self.start_on_ack=start_on_ack
        self.ack_on_since=None;self.ack_esp_ms=None
        self.state = 'WAITING_FOR_ADMISSION'
        self.stop_reason = ''
        self.goal_uuid = None
        self.used_goal_uuids = set()

    def update(self, name, value, now):
        self.inputs[name] = (value, float(now))

    def update_arm(self, armed, now):
        if armed is False:
            self.arm_false_seen = True
        elif armed is True and self.arm_false_seen:
            self.arm_cycle_seen = True
        self.arm = armed is True
        self.arm_at = float(now)

    def update_odom(self, values, now):
        self.odom = values
        self.odom_at = float(now)

    def update_cmd(self, values, now):
        self.cmd = values
        self.cmd_at = float(now)

    def update_statuses(self, statuses, now):
        """Store (uuid, status, sec, nanosec) from the Nav2 action server."""
        self.statuses = tuple(statuses)
        self.status_at = float(now)

    def _input(self, name, now):
        payload, received_at = self.inputs.get(name, (None, None))
        limit = .5 if name == 'ir' else self.input_timeout_s
        if fresh(received_at, now, limit) and \
                isinstance(payload, dict):
            return payload
        return None

    def committed(self):
        """Only an admitted R5 pulse defers perception until OFF/home review."""
        from .cycle_release import ReleaseContext
        try:ReleaseContext.parse(self.r5_cycle_context)
        except (ValueError,TypeError):return False
        return self.state=='FIRING' and self.on_since is not None

    def acknowledge_on(self, received_at, esp_ms):
        """Anchor once to the first current ESP ON report, never a repeated ON."""
        if (self.state!='FIRING' or self.on_since is None
                or type(esp_ms) is not int or not 0<=esp_ms<2**32
                or not math.isfinite(received_at) or received_at<self.on_since):return False
        if self.ack_on_since is None:
            self.ack_on_since=received_at;self.ack_esp_ms=esp_ms
        return True

    def pulse_elapsed(self, now):
        start=self.ack_on_since if self.start_on_ack else self.on_since
        return max(0.,now-start) if start is not None else None

    def completion_hold_errors(self,now):
        """Fresh zero/owner contracts still apply when the flame is obscured."""
        errors=[];mission=self._input('mission',now);align=self._input('alignment',now)
        if (mission is None or mission.get('schema_version')!=1 or mission.get('observe_only') is not True
                or (mission.get('robot') or {}).get('odom_fresh') is not True
                or (mission.get('robot') or {}).get('stationary') is not True
                or (mission.get('navigation') or {}).get('goal_active') is not False
                or any((mission.get('safety') or {}).get(k) is not False for k in
                    ('motion_command_publisher','actuator_command_publisher','collision_check_bypassed'))):
            errors.append('MISSION_HOLD_CONTRACT_INVALID')
        if (align is None or align.get('schema_version')!=1 or align.get('enabled') is not True
                or align.get('publisher_owned') is not False
                or (align.get('command') or {}).get('angular_z_rad_s')!=0.
                or any((align.get('safety') or {}).get(k) is not False for k in
                    ('pump_command_publisher','linear_motion'))):
            errors.append('ALIGNMENT_HOLD_CONTRACT_INVALID')
        return errors

    def blockers(self, now, counts):
        """Return reasons; UNKNOWN or contradictory input never grants ON."""
        reasons = []
        if not self.enabled:
            reasons.append('PUMP_DEMO_DISABLED')
        if not self.allow_provisional_ir:
            reasons.append('PROVISIONAL_IR_OVERRIDE_DISABLED')
        if not (self.arm_cycle_seen and self.arm and
                fresh(self.arm_at, now, self.arm_timeout_s)):
            reasons.append('OPERATOR_ARM_MISSING_OR_STALE')
        for key in ('ir', 'mission', 'alignment', 'gateway', 'guard', 'status',
                    'odom', 'cmd', 'arm'):
            if counts.get(key) != 1:
                if not (key=='ir' and self.committed() and counts.get('ir_endpoint_absent') is True):
                    reasons.append(key.upper() + '_PUBLISHER_INVALID')
        if counts.get('pump') != 1:
            reasons.append('PUMP_OWNER_CONFLICT')
        if counts.get('pump_subscribers') != 1:
            reasons.append('BRIDGE_SUBSCRIBER_INVALID')
        if counts.get('raw', 0) != 0:
            reasons.append('RAW_MOTION_OWNER_ACTIVE')

        ir = self._input('ir', now)
        if (ir is None or ir.get('schema_version') != 1 or
                ir.get('mode') != 'OBSERVE_ONLY' or
                ir.get('observe_only') is not True or
                ir.get('zone') != 'SPRAY_RANGE' or ir.get('blockers') != [] or
                ir.get('calibration') != 'PROVISIONAL_SINGLE_FLAME' or
                ir.get('validated_for_actuation') is not False or
                (ir.get('safety') or {}).get('motion_command_publisher')
                is not False or
                (ir.get('safety') or {}).get('actuator_command_publisher')
                is not False):
            reasons.append('IR_NOT_CONFIRMED_SPRAY_RANGE')

        context=context_of(self)
        no_goal=(context is not None and stopped_pair(self._input('gateway',now),self._input('guard',now),context))
        mission = self._input('mission', now)
        if (mission is None or mission.get('schema_version') != 1 or
                mission.get('observe_only') is not True or
                mission.get('state') not in
                ('ALIGNMENT_READY', 'CENTERED_HOLD') or
                (mission.get('perception') or {}).get('detected_now')
                is not True or
                (mission.get('perception') or {}).get('confirmed')
                is not True or
                (mission.get('perception') or {}).get('alignment_state')
                != 'CENTERED' or
                (mission.get('navigation') or {}).get('goal_active')
                is not False or
                (not no_goal and (mission.get('navigation') or {}).get('goal_state') != 'CANCELED') or
                (mission.get('robot') or {}).get('odom_fresh') is not True or
                (mission.get('robot') or {}).get('stationary') is not True or
                (mission.get('health') or {}).get('status') != 'OK' or
                (mission.get('health') or {}).get('blockers') or
                (mission.get('safety') or {}).get('motion_command_publisher')
                is not False or
                (mission.get('safety') or {}).get('actuator_command_publisher')
                is not False or
                (mission.get('decision') or {}).get(
                    'eligible_for_future_alignment_handoff') is not True or
                (mission.get('decision') or {}).get('readiness_blockers')
                != [] or
                (mission.get('safety') or {}).get('collision_check_bypassed')
                is not False):
            reasons.append('MISSION_NOT_SAFE_AND_CENTERED')

        align = self._input('alignment', now)
        if (align is None or align.get('schema_version') != 1 or
                align.get('state') != 'COMPLETE' or
                align.get('enabled') is not True or
                align.get('one_shot') is not True or
                align.get('stop_reason') != 'TARGET_CENTERED' or
                align.get('publisher_owned') is not False or
                (not align.get('goal_uuid') and not (no_goal and same_cycle(align,context) and align.get('cycle_id')==context['cycle_id'])) or
                (align.get('command') or {}).get('angular_z_rad_s') != 0.0 or
                (align.get('safety') or {}).get('pump_command_publisher')
                is not False or
                (align.get('safety') or {}).get('linear_motion')
                is not False):
            reasons.append('LIVE_ALIGNMENT_NOT_COMPLETE')

        uuid = align.get('goal_uuid') if align else None
        if uuid in self.used_goal_uuids:
            reasons.append('GOAL_UUID_ALREADY_USED')
        gateway = self._input('gateway', now)
        if not no_goal and (gateway is None or gateway.get('schema_version') != 1 or
                gateway.get('state') not in (('HANDOFF_READY','RELEASED_STOPPED') if self.committed() else ('HANDOFF_READY',)) or
                gateway.get('enabled') is not True or
                gateway.get('fire_latched') is not True or
                gateway.get('ever_forwarded') is not True or
                gateway.get('released') is not True or
                gateway.get('stop_reason') != 'FIRE_ACQUIRED' or
                (not self.committed() and gateway.get('handoff_ready') is not True) or
                gateway.get('raw_publishers') != 0 or
                gateway.get('nav_shadow_fingerprint_ok') is not True or
                gateway.get('safety_gate_fingerprint_ok') is not True or
                gateway.get('cmd_vel_fresh_zero') is not True or
                gateway.get('odom_stationary') is not True or
                gateway.get('goal_active') is not False or
                (gateway.get('safety') or {}).get('raw_command_publisher')
                is not False):
            reasons.append('GATEWAY_HANDOFF_PROOF_INVALID')

        guard = self._input('guard', now)
        if not no_goal and (guard is None or guard.get('schema_version') != 1 or
                guard.get('state') != 'CANCEL_CONFIRMED_STOPPED' or
                guard.get('enabled') is not True or
                guard.get('goal_uuid') != uuid or
                guard.get('send_count') != 1 or
                guard.get('cancel_service_accepted') is not True or
                guard.get('cancel_status_seen') is not True or
                guard.get('goal_status') != 5 or
                guard.get('cmd_vel_fresh_zero') is not True or
                guard.get('odom_fresh_stationary') is not True or
                guard.get('arm_fresh') is not True or
                guard.get('nav_shadow_fingerprint_ok') is not True or
                guard.get('raw_cmd_publishers') != 0 or
                guard.get('blockers') != [] or
                (guard.get('safety') or {}).get('motion_command_publisher')
                is not False or
                (guard.get('safety') or {}).get('actuator_command_publisher')
                is not False):
            reasons.append('CANCEL_GUARD_PROOF_INVALID')

        status_ok = False
        # Nav2 action status is event-driven, not a heartbeat. Its exact
        # canceled UUID is retained while the continuously fresh guard,
        # gateway, mission, odom and command provide liveness.
        if (self.status_at is not None and self.statuses and uuid and
                len(uuid) == 32):
            valid = all(
                isinstance(item, tuple) and len(item) == 4 and
                isinstance(item[0], str) and
                all(type(value) is int for value in item[1:])
                for item in self.statuses)
            if valid:
                latest = max(self.statuses, key=lambda item: item[2:4])
                status_ok = (
                    latest[0] == uuid and latest[1] == 5 and
                    not any(item[1] in (1, 2, 3) for item in self.statuses))
        if self.r5_cycle_context is not None:
            rows=[(x[0],x[1]) for x in self.statuses or () if isinstance(x,(tuple,list)) and len(x)==4]
            status_ok=(self.statuses is not None and len(rows)==len(self.statuses)
                and registered_navigation(self,rows,self.status_at,now,uuid))
        if not status_ok:
            reasons.append('EXACT_CANCEL_PROOF_INVALID')

        if (not fresh(self.odom_at, now, self.input_timeout_s) or
                not zero_twist(self.odom, 0.02)):
            reasons.append('ODOM_NOT_STATIONARY')
        if (not fresh(self.cmd_at, now, self.input_timeout_s) or
                not zero_twist(self.cmd, 0.001)):
            reasons.append('CMD_VEL_NOT_ZERO')
        if self.committed():
            # Admission above was required before the first ON. IR/YOLO can
            # disappear in spray; re-evaluate them after this bounded pulse.
            # Never suppress current cancel, stop, heartbeat or owner checks.
            reasons=[r for r in reasons if r not in (
                'IR_NOT_CONFIRMED_SPRAY_RANGE','MISSION_NOT_SAFE_AND_CENTERED','LIVE_ALIGNMENT_NOT_COMPLETE')]
            reasons+=self.completion_hold_errors(now)
        return reasons

    def step(self, now, counts):
        """Return (pump_on, diagnostic). OFF/abort is latched for this node."""
        now = float(now)
        if self.state in ('COMPLETE', 'ABORTED') and self.goal_uuid and \
                self.stop_reason in ({'PULSE_TIME_LIMIT'} |
                                     POSSIBLE_EARLY_EXTINGUISH) and \
                self.status_at is not None and \
                self.statuses is not None:
            active = [uid for uid, status, _, _ in self.statuses
                      if status in (1, 2, 3)]
            if len(active) == 1 and active[0] != self.goal_uuid and \
                    len(active[0]) == 32 and \
                    active[0] != '0' * 32 and \
                    all(c in '0123456789abcdef' for c in
                        active[0].lower()) and \
                    active[0] not in self.used_goal_uuids:
                self.used_goal_uuids.add(self.goal_uuid)
                self.state = 'WAITING_FOR_ADMISSION'
                self.goal_uuid = None
                self.ready_since = None
                self.on_since = None
                self.stop_reason = ''
                self.inputs.clear()
        reasons = self.blockers(now, counts)
        on = False
        if self.state == 'WAITING_FOR_ADMISSION':
            if reasons:
                self.ready_since = None
            elif self.ready_since is None:
                self.ready_since = now
            elif now - self.ready_since >= self.hold_s:
                self.state = 'FIRING'
                alignment = self._input('alignment', now) or {}
                self.goal_uuid = alignment.get('goal_uuid')
                if context_of(self) is not None:self.cycle_id=context_of(self)['cycle_id']
                self.on_since = now
                on = True
        elif self.state == 'FIRING':
            if reasons:
                self.state = 'ABORTED'
                self.stop_reason = reasons[0]
            elif self.pulse_elapsed(now) is not None and self.pulse_elapsed(now) >= self.pulse_s:
                self.state = 'COMPLETE'
                self.stop_reason = 'PULSE_TIME_LIMIT'
            else:
                on = True
        return on, {
            'schema_version': 1,
            'mode': 'SUPERVISED_DEMO',
            'state': self.state if self.enabled else 'DISABLED',
            'one_shot': True,
            'goal_uuid': self.goal_uuid,'cycle_id':self.cycle_id,'r5_cycle_context':self.r5_cycle_context,
            'pump_on': on,
            'pulse_clock': 'FIRST_FRESH_ESP_PWM_ON_ACK' if self.start_on_ack else 'FIRST_HOST_ON_COMMAND',
            'on_ack_at': self.ack_on_since,'on_ack_esp_ms':self.ack_esp_ms,
            'pulse_elapsed_s':self.pulse_elapsed(now),
            'stop_reason': self.stop_reason,
            'blockers': reasons,
            'perception_policy': 'REVIEW_AFTER_OFF_HOME' if self.committed() else 'REQUIRED_FOR_ADMISSION',
            'safety': {'requires_operator_arm': True,
                       'provisional_ir_override': self.allow_provisional_ir,
                       'motion_command_publisher': False,
                       'servo_command_publisher': False},
        }
