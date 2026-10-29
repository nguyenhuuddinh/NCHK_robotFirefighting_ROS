"""Explicit pre-spray cancellation. No command publisher or post-spray proof.

Two-phase release: continuous clear evidence -> mission ACK -> a NEW goal.
Missing evidence holds STOP; it never restores the old handoff or queues motion.
"""
import math

REQUEST_TOPIC = '/fire_mission/prespray_cancel_request'


def object_dict(v):
    return v if isinstance(v, dict) else {}


def number(v):
    return type(v) in (int, float) and math.isfinite(v)


def uuid_ok(v):
    return isinstance(v, str) and len(v) == 32 and v != '0'*32 and all(c in '0123456789abcdef' for c in v)


def fresh(at, now, limit=.35):
    return number(at) and 0 <= now-at <= limit


def aged(payload, field, at, now, limit=300):
    age = payload.get(field)
    return fresh(at, now) and number(age) and 0 <= age and age+(now-at)*1000 <= limit


def vision_clear_fresh(p, at, now):
    source, received = p.get('source_age_ms'), p.get('detection_receive_age_ms')
    return (fresh(at, now) and number(source) and number(received)
            and source >= 0 and received >= 0 and source+received+(now-at)*1000 <= 300)


def request_ok(p):
    return (isinstance(p, dict) and p.get('schema_version') == 1 and p.get('active') is True
            and uuid_ok(p.get('request_id')) and uuid_ok(p.get('goal_uuid')))


class PreSprayRecovery:
    def __init__(self, enabled=False):
        self.enabled = enabled is True
        self.inputs = {}
        self.graph_ok = False
        self.base_graph_ok = False
        self.pump_count = -1
        self.token = self.goal = None
        self.state = 'IDLE'
        self.blockers = []
        self.since = self.ready_at = None
        self.last_seq = None
        self.samples = 0
        self.mission_samples = set()
        self.new_goal = None

    def update(self, name, payload, now):
        self.inputs[name] = (payload, now)

    def get(self, name, now):
        p, at = self.inputs.get(name, ({}, None))
        return p if isinstance(p, dict) and fresh(at, now) else {}

    @property
    def active(self):
        return self.token is not None and self.state != 'HANDED_BACK'

    def proof_blockers(self, c, now, stationary, cmd_zero, raw_count, *, new_goal=False):
        r = self.get('request', now)
        errors = []
        if not request_ok(r) or r.get('request_id') != self.token or r.get('goal_uuid') != self.goal:
            errors.append('CANCEL_HEARTBEAT_MISSING_OR_CHANGED')
        if not self.graph_ok or not self.base_graph_ok or self.pump_count != 0 or raw_count != 0:
            errors.append('OWNER_OR_GRAPH_NOT_RELEASED')
        if not fresh(c.arm_at, now, .4) or c.arm is not True:
            errors.append('OPERATOR_ARM_NOT_FRESH')
        if not stationary or not cmd_zero:
            errors.append('WAIT_STOP_FEEDBACK')
        a = c.actuator if isinstance(c.actuator, dict) else {}
        if not (c._actuator_off(now) and aged(a, 'receive_age_ms', c.actuator_at, now, 350)
                and a.get('mode') == 'COMMAND_TELEMETRY_ONLY' and a.get('pump_pwm_ready') is True):
            errors.append('WAIT_FRESH_ACTUATOR_OFF')
        m = c.mission if isinstance(c.mission, dict) else {}
        p = object_dict(m.get('perception'))
        if not (m.get('observe_only') is True and m.get('state') in ('TARGET_LOST', 'IDLE', 'NAV2_NAVIGATING')
                and object_dict(m.get('health')).get('status') == 'OK'
                and object_dict(m.get('health')).get('blockers') == []
                and p.get('detected_now') is False and p.get('confirmed') is False
                and vision_clear_fresh(p, c.mission_at, now)):
            errors.append('WAIT_YOLO_CLEAR_FRESH')
        ir = self.get('ir', now)
        at = self.inputs.get('ir', ({}, None))[1]
        if not (ir.get('schema_version') == 1 and ir.get('observe_only') is True
                and ir.get('mode') == 'OBSERVE_ONLY' and ir.get('ir_band') == 'NO_FLAME_SIGNATURE'
                and aged(ir, 'ir_receive_age_ms', at, now)
                and type(ir.get('stable_samples')) is int and ir['stable_samples'] >= 5
                and type(ir.get('ir_sample_seq')) is int and ir['ir_sample_seq'] > 0
                and isinstance(ir.get('blockers'), list)
                and all(isinstance(b, str) for b in ir['blockers'])
                and set(ir['blockers']) <= {'NO_IR_FLAME_SIGNATURE', 'FIRE_NOT_CONFIRMED', 'FIRE_NOT_CENTERED'}):
            errors.append('WAIT_IR_CLEAR_FRESH')
        actor = self.get('actor', now)
        ack = object_dict(actor.get('prespray_cancel'))
        if not (ack.get('request_id') == self.token and ack.get('goal_uuid') == self.goal
                and ack.get('closed') is True and actor.get('request_used') is False
                and actor.get('proposed_pump_on') is False and actor.get('pump_publisher_registered') is False):
            errors.append('WAIT_OLD_PUMP_CLOSED')
        al = self.get('alignment', now)
        live = self.get('ir_live', now)
        old_alignment = (al.get('goal_uuid') == self.goal and al.get('state') == 'COMPLETE'
            and al.get('stop_reason') == 'TARGET_CENTERED')
        # The real alignment core retires its COMPLETE cycle on action status,
        # before the gateway can necessarily observe that same new status.
        retired_alignment = ((new_goal or (self.state == 'READY' and fresh(self.ready_at, now))) and al.get('state') == 'WAITING_FOR_ADMISSION'
            and al.get('goal_uuid') is None and al.get('stop_reason') == ''
            and isinstance(al.get('retired_goal_uuids'), list)
            and self.goal in al['retired_goal_uuids'])
        if not ((old_alignment or retired_alignment) and al.get('publisher_owned') is False
                and al.get('command') == {'linear_x_m_s': 0.0, 'angular_z_rad_s': 0.0}
                and live.get('publisher_owned') is False and live.get('state') in ('WAITING_FOR_ADMISSION', 'COMPLETE')
                and (live.get('state') != 'COMPLETE' or (live.get('goal_uuid') == self.goal
                     and live.get('stop_reason') == 'SPRAY_RANGE_REACHED' and live.get('recovery_required') is False))):
            errors.append('WAIT_OLD_MOTION_RELEASED')
        g = c.guard if isinstance(c.guard, dict) else {}
        guard_base = (fresh(c.guard_at, now) and g.get('enabled') is True
            and g.get('arm_fresh') is True and g.get('cmd_vel_fresh_zero') is True
            and g.get('odom_fresh_stationary') is True and g.get('nav_shadow_fingerprint_ok') is True
            and g.get('raw_cmd_publishers') == 0 and g.get('failure_evidence') is None)
        old_cancel = (guard_base and g.get('state') == 'CANCEL_CONFIRMED_STOPPED'
            and g.get('goal_uuid') == self.goal and g.get('blockers') == []
            and g.get('send_count') == 1 and g.get('cancel_service_accepted') is True
            and g.get('cancel_status_seen') is True and g.get('goal_status') == 5)
        retired = g.get('retired_goal_uuids')
        guard_rearmed = (guard_base and g.get('state') == 'WAITING_FOR_ADMISSION'
            and g.get('goal_uuid') is None and g.get('send_count') == 0
            and isinstance(retired, list) and self.goal in retired
            and g.get('alignment_handoff_active') is False and g.get('ir_approach_handoff_active') is False)
        if new_goal:
            if not guard_rearmed:
                errors.append('WAIT_CANCEL_GUARD_REARM_ACK' if old_cancel else 'WAIT_EXACT_CANCEL_STOP')
        elif not old_cancel and not (guard_rearmed and self.state == 'READY' and fresh(self.ready_at, now)):
            errors.append('WAIT_EXACT_CANCEL_STOP')
        if (self.goal, 5) not in (c.statuses or ()):
            errors.append('OLD_GOAL_NOT_CANCELED')
        return errors

    def step(self, c, now, stationary, cmd_zero, raw_count, nav_blockers):
        r = self.get('request', now)
        if not self.enabled or self.state == 'HANDED_BACK':
            return False
        if self.token is None:
            if not (self.graph_ok and request_ok(r) and r['goal_uuid'] == c.goal_uuid
                    and c.released and c.fire_latched and c.stop_reason == 'FIRE_ACQUIRED'
                    and c.cancel_confirmed_uuid == c.goal_uuid):
                return False
            self.token, self.goal = r['request_id'], r['goal_uuid']
            self.state = 'WAIT_CLEAR'
        if self.state == 'FAULT':
            return False
        active = [u for u, status in c.statuses or () if status in (1,2,3)]
        if len(active)>1 or len([u for u,status in c.statuses or () if u==self.goal])!=1:
            self.state='FAULT';self.blockers=['AMBIGUOUS_GOAL_STATUS'];return False
        new = c._active_goal(now)
        if new:
            if (new == self.goal or new in c.used_goal_uuids or self.state != 'READY'
                    or not fresh(self.ready_at, now) or not number(c.status_at)
                    or c.status_at <= self.ready_at or (self.new_goal and new != self.new_goal)):
                self.state = 'FAULT'; self.blockers = ['GOAL_BEFORE_READY_OR_REPLAY']; return False
            self.new_goal = new
        errors = self.proof_blockers(c, now, stationary, cmd_zero, raw_count, new_goal=bool(new))
        if new:
            # Permit normal status/mission/command ordering for <=350ms only.
            self.blockers = errors + list(nav_blockers)
            if errors and errors != ['WAIT_CANCEL_GUARD_REARM_ACK']:
                self.state = 'FAULT'; return False
            if not self.blockers:
                c._start_next_goal(new)
                self.state = 'HANDED_BACK'
                return True
            return False
        self.new_goal = None
        if errors:
            self.state = 'WAIT_CLEAR'; self.blockers = errors
            self.since = self.ready_at = None
            self.samples = 0; self.last_seq = None; self.mission_samples.clear()
            return False
        # Peer cores may receive the action status before this gateway. Keep
        # the last READY proof for its existing <=350ms lifetime; never refresh
        # that proof or create an owner until our exact new status arrives.
        peers_retired = ((self.get('alignment', now).get('goal_uuid') is None)
                         or (object_dict(c.guard).get('goal_uuid') is None))
        if peers_retired:
            self.blockers = ['WAIT_NEW_GOAL_STATUS']
            return False
        seq = self.get('ir', now)['ir_sample_seq']
        if self.last_seq is not None and seq < self.last_seq:
            self.state = 'FAULT'; self.blockers = ['IR_SEQUENCE_REGRESSION']; return False
        if self.since is None:
            self.since = now
        if seq != self.last_seq:
            self.samples += 1; self.last_seq = seq
        if len(self.mission_samples) < 3:
            self.mission_samples.add(c.mission_at)
        self.blockers = []
        if now-self.since < 1.0 or self.samples < 5 or len(self.mission_samples) < 3:
            self.state = 'WAIT_CLEAR'; self.blockers = ['WAIT_CLEAR_HOLD_1S_5_IR_SAMPLES']
        else:
            ack = object_dict((c.mission or {}).get('prespray_cancel_ack'))
            if ack == {'request_id': self.token, 'goal_uuid': self.goal}:
                self.state = 'READY'
                self.ready_at = now
            else:
                self.state = 'COMMIT'
        return False

    def report(self):
        return dict(state=self.state, request_id=self.token, goal_uuid=self.goal,
                    blockers=list(self.blockers), clear_samples=self.samples,
                    next_goal_uuid=self.new_goal, pump_completion_claimed=False)
