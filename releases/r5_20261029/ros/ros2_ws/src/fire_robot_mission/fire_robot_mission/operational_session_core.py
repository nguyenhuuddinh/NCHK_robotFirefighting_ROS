"""Persistent operator session; proposes stage intents, never motor/pump commands.

Uses the accepted R4 report contracts at the integration boundary. Missing
no-goal live consumer proofs remain visible waits; retirement requires an
exact typed receipt from all release participants;
cycle IDs are never substituted for Nav2 UUIDs to satisfy legacy consumers.
"""
import copy

from .actuator_freshness import fresh_off, fresh_motion_off
from .ir_approach_shadow_core import validated_ir_band
from .ir_budget_stop import cap_report_valid
from .operational_identity import IdentityContinuity,identity_current
from .operational_prearm import ir_readiness
from .no_goal_cycle import stopped_pair,same_cycle
from .operational_cycle import CycleRegistry, NavigationSnapshot, finite, zero


ZERO_COMMAND = {'linear_x_m_s': 0., 'angular_z_rad_s': 0.}


def obj(value):
    return value if isinstance(value, dict) else {}




class OperationalSessionCore:
    """Camera/observer lifetime is independent of individual stage outcomes.

    An I budget runs from accepted I, including target loss and recentering.
    All output is advisory until an independently reviewed intent executor is
    connected. A report is not a replacement for any live consumer's guards.
    """
    INPUTS = frozenset(('vision', 'preview', 'mission', 'gateway', 'guard',
        'alignment', 'ir_live', 'ir', 'actuator', 'cmd', 'odom', 'status', 'graph',
        'actor','review','identity','ir_raw'))

    def __init__(self, id_factory=None, *, allow_close_stationary=False, require_boot_identity=False, unbounded_approach=False, ir_confirm_samples=5):
        if type(ir_confirm_samples) is not int or ir_confirm_samples not in (3,5):raise ValueError('IR_CONFIRM_SAMPLES_INVALID')
        self.ir_confirm_samples=ir_confirm_samples
        self.unbounded_approach=unbounded_approach is True
        self.cycles = CycleRegistry(id_factory, unbounded_approach=self.unbounded_approach)
        self.require_boot=require_boot_identity is True
        self.identity_lease=IdentityContinuity()
        self.navigation_only=False;self.navigation_home_before=None
        self.identity_hold=False;self.identity_saved_phase=None
        self.inputs = {}
        self.armed = self.fire_permission = self.turn_permission = False
        self.allow_close = allow_close_stationary is True
        self.phase = 'WAIT_ARM'
        self.reason = 'WAIT_STABLE_INPUTS'
        self.fault = None
        self.closed = False
        self.recovery_clear_policy = False
        self.recovery_require_ir_clear = False
        self.cancel_requested = self.p_used = False
        self.last_now = self.stable_since = self.release_since = None
        self.arm_requested_at = None
        self.arm_request_index = None
        self.ir_stop_step = None
        self.last_ir_step = 0
        self.ir_recovering = False
        self.ir_has_run = False
        self.ir_baseline_seq = None
        self.confirm_samples = []
        self.ir_confirmation_seen_at = None
        self.pending_stationary = False
        self.feedback = {'key': None, 'accepted': False, 'reason': 'NO_KEY'}
        self.event_index = 0
        self.handback_completed=False
        self.p_request_baseline = None
        self.p_request_feedback = None
        self.navigation_cancel_uuid = None
        self.graph_hold=False
        self.interruption_reason=None
        self.retry_fire_clear_required=False
        self.retry_clear_since=None
        self.input_hold=False;self.input_hold_since=None;self.input_recovery_since=None
        self.input_hold_blockers=[]

    def observe_input_hold(self,now,blockers):
        """Missing reports pause authority, not the lifetime of an armed session.

        Recovery needs the same authenticated graph/boot, fresh OFF and actual
        stopped motion for 0.5s. A surviving raw owner may only hold zero.
        Hard source/epoch faults remain latched by their existing validators.
        """
        if not self.armed or self.closed or self.fault:
            return
        if blockers:
            if not self.input_hold:self.input_hold_since=now
            self.input_hold=True;self.input_recovery_since=None
            self.input_hold_blockers=list(blockers)
            self.confirm_samples=[];self.release_since=None
            return
        if not self.input_hold:return
        graph=obj(self.get('graph',now))
        raw_known=(graph.get('raw_count')==0 or graph.get('raw_count')==1 and graph.get('raw_owner') in
            ('live_nav_stop_gateway','live_alignment_takeover','ir_approach_live'))
        stopped=(graph.get('valid') is True and raw_known and not self.identity_hold and
            zero(self.get('cmd',now)) and zero(self.get('odom',now),.02,.05) and self._motion_off(now))
        if not stopped:self.input_recovery_since=None;return
        if self.input_recovery_since is None:self.input_recovery_since=now
        if now-self.input_recovery_since<.5:return
        self.input_hold=False;self.input_hold_since=self.input_recovery_since=None
        self.input_hold_blockers=[]

    def interrupt_cycle(self, reason):
        """End an unused fire attempt. Release still requires real STOP/OFF."""
        if self.cycles.current is None or self.p_used or self.fault or self.closed:
            return False
        self.interruption_reason=self.interruption_reason or reason
        self.cancel_requested=True
        self.turn_permission=False
        self.pending_stationary=False
        self.confirm_samples=[]
        self.retry_fire_clear_required=True
        self.retry_clear_since=None
        self.phase,self.reason='CANCEL_WAIT_CLEAR','INTERRUPTED:'+self.interruption_reason
        return True

    def _observe_retry_clear(self, now):
        if not self.retry_fire_clear_required:return
        vision=obj(self.get('vision',now));fire=obj(vision.get('fire'))
        clear=(vision.get('state')=='ok' and vision.get('schema_version')==1
            and vision.get('observe_only') is True and fire.get('detected_now') is False
            and fire.get('confirmed') is False)
        if self.recovery_require_ir_clear:
            raw=obj(self.get('ir_raw',now))
            clear=clear and ir_readiness(self.inputs,now)['grant_fresh'] and raw.get('schema_version')==1 and raw.get('valid') is True and all(
                type(raw.get(k)) is int and 20000<raw[k]<=32767 for k in ('left','center','right'))
        if not clear:self.retry_clear_since=None
        elif self.retry_clear_since is None:self.retry_clear_since=now
        elif now-self.retry_clear_since>=(3. if self.recovery_clear_policy else .5):
            self.retry_fire_clear_required=False;self.recovery_clear_policy=False;self.recovery_require_ir_clear=False

    def update(self, name, value, at):
        if name not in self.INPUTS or not finite(at):
            raise ValueError('SESSION_INPUT_INVALID')
        self.inputs[name] = (copy.deepcopy(value), float(at))

    def get(self, name, now, limit=.35):
        value, at = self.inputs.get(name, (None, None))
        if name == 'ir' and limit == .35:
            limit = .5
        if at is None or not 0 <= now-at <= limit:
            return None
        if name in ('vision', 'preview'):
            age = obj(obj(value).get('source')).get('age_ms')
            if not finite(age) or age < 0 or age+(now-at)*1000 > 300:
                return None
        if name == 'ir':
            age = obj(value).get('ir_receive_age_ms')
            if not finite(age) or age < 0 or age+(now-at)*1000 > 500 + 1e-6:
                return None
        return value

    def stop_fault(self, reason):
        if self.fault is None:
            self.fault = reason
        self.input_hold=False;self.input_hold_since=self.input_recovery_since=None
        self.input_hold_blockers=[]
        self.armed = self.fire_permission = self.turn_permission = False
        self.clear_prearm()
        self.phase = 'FAULT'
        self.reason = self.fault

    def _off(self, now):
        value, at = self.inputs.get('actuator', (None, None))
        return (at is not None and self.get('actuator', now) is not None
                and fresh_off(value, at, now) and obj(value).get('pump_pwm_ready') is True)

    def _motion_off(self, now):
        if not self.navigation_only: return self._off(now)
        value, at = self.inputs.get('actuator', (None,None))
        return fresh_motion_off(value, at, now)

    def _rest(self, now):
        graph = obj(self.get('graph', now))
        return (graph.get('raw_count') == 0 and graph.get('raw_owner') is None
                and zero(self.get('cmd', now))
                and zero(self.get('odom', now), .02, .05) and self._motion_off(now))

    def _standby_proof(self, now, *, boot_ok=None):
        """Explain pre-arm readiness without changing any admission gate."""
        graph = obj(self.get('graph', now))
        if boot_ok is None:
            boot, boot_at = self.inputs.get('identity', (None, None))
            boot_ok = identity_current(boot, boot_at, now) if self.require_boot else True
        checks = dict(
            graph_valid=graph.get('valid') is True,
            boot_current=boot_ok is True,
            raw_owner_clear=graph.get('raw_count') == 0 and graph.get('raw_owner') is None,
            cmd_zero=zero(self.get('cmd', now)),
            odom_stationary=zero(self.get('odom', now), .02, .05),
            actuator_off=self._motion_off(now),
            vision_fresh=self.get('vision', now) is not None,
            ir_continuity=ir_readiness(self.inputs, now)['continuity'],
        )
        return dict(checks=checks, blockers=[name for name, passed in checks.items() if not passed],
                    ir=ir_readiness(self.inputs, now),
                    arm_request_pending=self.arm_requested_at is not None,
                    arm_request_remaining_s=(max(0., 2.-(now-self.arm_requested_at))
                                             if self.arm_requested_at is not None else None),
                    stable_s=(max(0., now-self.stable_since) if self.stable_since is not None else 0.))

    def clear_prearm(self):
        """Loss of any standby/adapter proof also discards an explicit pending A."""
        self.stable_since = self.arm_requested_at = None
        self.arm_request_index = None

    def _try_pending_arm(self, now):
        if self.arm_requested_at is None:
            return
        requested_at = self.arm_requested_at
        request_index = self.arm_request_index
        if now-requested_at >= 2.:
            self.arm_requested_at = None
            self.feedback = dict(index=request_index, key='A', accepted=False,
                                 reason='ARM_REQUEST_EXPIRED_PRESS_A_AGAIN', at=now)
            return
        if (self.stable_since is None or now-self.stable_since < 3.
                or self._standby_proof(now)['blockers'] or not self._rest(now)):
            self.clear_prearm()
            self.feedback.update(accepted=False, reason='ARM_REQUEST_CANCELLED_PROOF_LOST', at=now)
            return
        if not ir_readiness(self.inputs, now)['grant_fresh']:
            return
        boot, at = self.inputs.get('identity', (None, None))
        if self.require_boot and not self.identity_lease.observe(boot, at, now, armed=True, stopped=True):
            self.clear_prearm()
            return
        self.arm_requested_at = None
        self.armed = True
        self.feedback = dict(index=request_index, key='A', accepted=True,
                             reason='ARM_ACCEPTED', at=now, requested_at=requested_at)

    def _fire(self, now):
        vision = obj(self.get('vision', now))
        fire = obj(vision.get('fire'))
        return (vision.get('schema_version') == 1 and vision.get('observe_only') is True
                and vision.get('state') == 'ok' and fire.get('confirmed') is True
                and fire.get('detected_now') is True)

    def _centered(self, now):
        preview = obj(self.get('preview', now))
        error = obj(preview.get('preview')).get('error_x_normalized')
        return (self._fire(now) and preview.get('schema_version') == 1
                and preview.get('observe_only') is True and preview.get('state') == 'CENTERED'
                and finite(error) and abs(error) <= .13)

    def _status(self, now):
        graph = obj(self.get('graph',now))
        rows = self.get('status', now, float('inf') if graph.get('nav_status_event_stream') is True else .35)
        if rows is None:
            return None
        try:
            return NavigationSnapshot.parse(rows, self.inputs['status'][1])
        except ValueError as exc:
            self.stop_fault(str(exc))
            return None

    def no_goal_context(self,now):
        from .cycle_release import ReleaseContext
        from .operational_identity import identity_key
        cycle=self.cycles.current;value,at=self.inputs.get('identity',(None,None))
        if cycle is None or cycle.nav_goal_uuid is not None or not identity_current(value,at,now):return None
        return ReleaseContext(cycle.cycle_id,None,*identity_key(value)).as_dict()

    def _handoff(self, now, *, initial_zero_only=False):
        """Accepted R4 exact cancel, plus current receiver-side stop evidence."""
        identity = self.cycles.current
        if identity is None:return False
        if identity.nav_goal_uuid is None:
            context=self.no_goal_context(now)
            status=self._status(now);m=obj(self.get('mission',now))
            return bool(context and status and status.active_uuid is None
                and stopped_pair(obj(self.get('gateway',now)),obj(self.get('guard',now)),context)
                and obj(m.get('health')).get('status')=='OK' and obj(m.get('health')).get('blockers')==[]
                and obj(m.get('navigation')).get('goal_active') is False
                and obj(m.get('decision')).get('eligible_for_future_alignment_handoff') is True
                and obj(m.get('decision')).get('readiness_blockers')==[]
                and (self._rest(now) or initial_zero_only and zero(self.get('cmd',now))
                    and zero(self.get('odom',now),.02,.05) and self._off(now)))
        uid = identity.nav_goal_uuid
        status = self._status(now)
        m = obj(self.get('mission', now)); g = obj(self.get('gateway', now))
        c = obj(self.get('guard', now))
        # Nav2 may prune a terminal record. The fresh exact guard below still
        # proves the real response and observed CANCELED result for this UUID;
        # an active or contradictory record must continue to block admission.
        return bool(status and status.active_uuid is None
            and not any(u==uid and s!=5 for u,s in status.rows)
            and g.get('cycle_goal_uuid') == uid and g.get('state') == 'HANDOFF_READY'
            and g.get('handoff_ready') is True and g.get('released') is True
            and g.get('fire_latched') is True and g.get('raw_publishers') == 0
            and g.get('cmd_vel_fresh_zero') is True and g.get('odom_fresh') is True
            and g.get('odom_stationary') is True
            and c.get('goal_uuid') == uid and c.get('state') == 'CANCEL_CONFIRMED_STOPPED'
            and c.get('enabled') is True and c.get('send_count') == 1
            and c.get('cancel_service_accepted') is True and c.get('cancel_status_seen') is True
            and c.get('goal_status') == 5 and c.get('arm_fresh') is True
            and c.get('cmd_vel_fresh_zero') is True and c.get('odom_fresh_stationary') is True
            and c.get('nav_shadow_fingerprint_ok') is True and c.get('raw_cmd_publishers') == 0
            and c.get('blockers') == [] and obj(m.get('health')).get('status') == 'OK'
            and obj(m.get('navigation')).get('goal_active') is False
            and obj(m.get('decision')).get('eligible_for_future_alignment_handoff') is True
            and obj(m.get('decision')).get('readiness_blockers') == []
            and (self._rest(now) or (initial_zero_only and zero(self.get('cmd',now))
                 and zero(self.get('odom',now),.02,.05) and self._off(now))))

    def _aligned(self, now, *, initial_zero_only=False):
        a = obj(self.get('alignment', now))
        uid = self.cycles.current.nav_goal_uuid if self.cycles.current else None
        return (self._handoff(now,initial_zero_only=initial_zero_only) and self._centered(now) and a.get('schema_version') == 1
                and a.get('enabled') is True and a.get('goal_uuid') == uid
                and (uid is not None or same_cycle(a,self.no_goal_context(now))
                     and a.get('cycle_id')==self.cycles.current.cycle_id)
                and a.get('state') == 'COMPLETE' and a.get('stop_reason') == 'TARGET_CENTERED'
                and a.get('publisher_owned') is False and a.get('blockers') == []
                and a.get('command') == ZERO_COMMAND)

    def _bind_live(self, live):
        return (self.cycles.current is not None
                and (self.cycles.current.nav_goal_uuid is not None
                    or live.get('cycle_id')==self.cycles.current.cycle_id
                    and live.get('r5_cycle_context')==self.no_goal_context(self.last_now))
                and live.get('schema_version') == 1 and live.get('enabled') is True
                and live.get('goal_uuid') == self.cycles.current.nav_goal_uuid
                and live.get('integrated_handoff') is True
                and live.get('continuous_approach') is True)

    def _alignment_owned(self, now):
        a = obj(self.get('alignment', now)); graph = obj(self.get('graph', now))
        return (self.cycles.current is not None and self.turn_permission
                and a.get('goal_uuid') == self.cycles.current.nav_goal_uuid
                and (self.cycles.current.nav_goal_uuid is not None or
                     same_cycle(a,self.no_goal_context(now)) and a.get('cycle_id')==self.cycles.current.cycle_id)
                and a.get('enabled') is True and a.get('publisher_owned') is True
                and a.get('state') in ('ACQUIRING', 'TURNING', 'STOPPING', 'TARGET_HOLD')
                and graph.get('raw_count') == 1 and graph.get('raw_owner') == 'live_alignment_takeover')

    def _range_samples(self, now, live):
        current=self.get('ir',now)
        # A stale poll cannot grant P. Keep already validated distinct ADC
        # samples across a brief inter-report gap, while all alignment/STOP
        # proof stays current. Any fresh contradictory sample resets below.
        if current is None and self.ir_confirmation_seen_at is not None and self._aligned(now):
            previous,at=self.inputs.get('ir',(None,None))
            age=obj(previous).get('ir_receive_age_ms')
            if (finite(at) and finite(age) and age>=0 and 0<=now-at<=.6
                    and age+(now-at)*1000<=600 and now-self.ir_confirmation_seen_at<=.6):
                return False
        ir = obj(current); seq = ir.get('ir_sample_seq')
        band = validated_ir_band(ir)
        bands = ('SPRAY_RANGE', 'TOO_CLOSE') if self.allow_close else ('SPRAY_RANGE',)
        stable = (band in bands and ir.get('zone') == band and ir.get('blockers') == []
                  and type(ir.get('stable_samples')) is int and ir['stable_samples'] >= self.ir_confirm_samples
                  and type(seq) is int and seq > 0 and self._aligned(now))
        if not stable:
            self.confirm_samples = []
            self.ir_confirmation_seen_at = None
            self.ir_baseline_seq = seq
            return False
        self.ir_confirmation_seen_at = now
        previous = self.confirm_samples[-1][0] if self.confirm_samples else self.ir_baseline_seq
        if type(previous) is not int:
            self.ir_baseline_seq = seq
            return False
        if seq < previous:
            self.stop_fault('IR_SEQUENCE_REGRESSION')
            return False
        if seq > previous and (not self.confirm_samples or now-self.confirm_samples[-1][1] >= .099):
            self.confirm_samples.append((seq, now))
            self.confirm_samples = self.confirm_samples[-self.ir_confirm_samples:]
        return len(self.confirm_samples) == self.ir_confirm_samples and now-self.confirm_samples[0][1] >= .399

    def _actor_cycle_current(self, actor):
        cycle = self.cycles.current
        context = obj(actor.get('r5_cycle_context'))
        return bool(cycle and context.get('cycle_id') == cycle.cycle_id
            and context.get('nav_goal_uuid') == cycle.nav_goal_uuid
            and self.identity_lease.bound == (context.get('bridge_instance_id'),
                context.get('connection_epoch'), context.get('elf_sha256')))

    def pulse_committed(self, now):
        """A current actor's already-admitted ON, not permission for a new ON."""
        actor=obj(self.get('actor',now))
        return bool(self.p_used and not self.cancel_requested and not self.closed
            and not self.fault and self._actor_cycle_current(actor)
            and actor.get('state')=='FIRING' and actor.get('proposed_pump_on') is True
            and obj(actor.get('pulse_core')).get('state')=='FIRING')

    def _pulse_admission(self, now):
        if self.navigation_only:return 'P_WAIT_ACTUATOR_HOME_REPAIR_NAV_AVAILABLE'
        actor = obj(self.get('actor', now))
        receipt = obj(actor.get('pulse_request_feedback'))
        if (not self._actor_cycle_current(actor) or actor.get('state') != 'WAIT_P'
                or actor.get('request_used') is not False
                or actor.get('pump_publisher_registered') is not True
                or type(receipt.get('index')) is not int or receipt['index'] < 0):
            return 'P_WAIT_CURRENT_ACTOR_READY'
        if actor.get('pulse_arm_active') is not False or actor.get('pulse_request_pending') is not False:
            return 'P_WAIT_ACTOR_FALSE_HEARTBEAT'
        blockers = actor.get('blockers')
        if not isinstance(blockers, list):return 'P_WAIT_ACTOR_PROOF'
        blockers = [s for s in blockers if s != 'OPERATOR_HEARTBEAT_MISSING']
        return 'P_BLOCKED_BY_ACTOR:'+str(blockers[0]) if blockers else None

    def _pulse_receipt(self, now):
        actor = obj(self.get('actor', now))
        receipt = obj(actor.get('pulse_request_feedback'))
        if (self.p_request_baseline is None or not self._actor_cycle_current(actor)
                or type(receipt.get('index')) is not int
                or receipt['index'] != self.p_request_baseline+1):return
        if (receipt.get('accepted') is False and actor.get('request_used') is False
                and actor.get('state') == 'WAIT_P' and self._rest(now)):
            # A negative receipt proves that this edge never started a pulse.
            # Withdraw true; a new P is blocked until the actor observes false.
            # An absent/ambiguous receipt never resets a used P or auto-retries.
            self.p_used = False
            self.feedback = dict(self.p_request_feedback, accepted=False,
                reason='P_REJECTED_BY_ACTOR:'+str(receipt.get('reason')), at=now)
            self.p_request_baseline = self.p_request_feedback = None

    def _approach(self, now):
        budget = self.cycles.budget
        live = obj(self.get('ir_live', now))
        bands=('SPRAY_RANGE','TOO_CLOSE') if self.allow_close else ('SPRAY_RANGE',)
        if (not self.ir_has_run and not self.ir_recovering
                and live.get('schema_version')==1 and live.get('enabled') is True
                and live.get('state')=='WAITING_FOR_ADMISSION' and live.get('goal_uuid') is None
                and live.get('step_count')==0 and live.get('total_progress_m')==0
                and live.get('publisher_owned') is False and live.get('command')==ZERO_COMMAND
                and live.get('recovery_required') is False and self._rest(now) and self._aligned(now)
                and validated_ir_band(self.get('ir',now)) in bands):
            # A newer ADC may enter range after I but before the first motion
            # owner exists. Use initial stationary proof, not a forward step.
            self.pending_stationary=True
            self.phase,self.reason='IR_CONFIRM','IR_INITIAL_RANGE_BEFORE_FIRST_STEP'
            return
        if not self._bind_live(live):
            # The idle live node has not bound a UUID until it receives I arm.
            # Requiring that binding before proposing I creates an admission
            # deadlock. This initial intent is still zero-only, not ownership.
            if (live.get('schema_version') == 1 and live.get('enabled') is True
                    and live.get('goal_uuid') is None
                    and live.get('integrated_handoff') is True and live.get('continuous_approach') is True
                    and live.get('state') == 'WAITING_FOR_ADMISSION'
                    and live.get('publisher_owned') is False and live.get('command') == ZERO_COMMAND
                    and not self.ir_has_run and not self.ir_recovering
                    and self._aligned(now) and budget.report(now)['forward_budget_available']):
                self.phase, self.reason = 'IR_RUNNING', 'I_ACCEPTED_WAIT_FIRST_IR_UUID_BIND'
                return
            if live.get('goal_uuid') not in (None, self.cycles.current.nav_goal_uuid):
                self.stop_fault('IR_REPORT_UUID_CHANGED'); return
            self.phase, self.reason = 'WAIT_DATA', 'IR_REPORT_OR_UUID_MISSING'
            return
        distance = live.get('total_progress_m')
        if not budget.observe(self.cycles.current.cycle_id, distance, now):
            self.stop_fault(budget.fault)
            return
        b = budget.report(now)
        if b['fault']:
            self.stop_fault(b['fault']); return
        step = live.get('step_count')
        if type(step) is not int or step < 0:
            self.stop_fault('IR_STEP_INVALID'); return
        if step < self.last_ir_step or step > self.last_ir_step+1:
            self.stop_fault('IR_STEP_REGRESSION_OR_SKIP'); return
        if live.get('state') == 'COMPLETE' and step == 0:
            self.stop_fault('IR_COMPLETE_WITHOUT_RELEASED_STEP'); return
        reason = live.get('stop_reason')
        state = live.get('state')
        cap = cap_report_valid(live)
        if self.ir_recovering and self._alignment_owned(now):
            self.phase, self.reason = ('REALIGNING', 'RECENTER_SAME_I_BUDGET') if self._fire(now) else ('WAIT_TARGET', 'WAIT_ALIGNMENT_ZERO_RELEASE')
            return
        # A released complete stage is required; STOPPING step0 is not reentry.
        if state == 'STOPPING' or (not self._centered(now) and state in ('ACQUIRING','MOVING')):
            self.ir_recovering = True
            if self.ir_stop_step is None:
                self.ir_stop_step = step
            self.phase, self.reason = 'IR_RELEASE', reason or 'WAIT_ZERO_AND_RELEASE'
            self.release_since = None
            return
        complete_release = (state == 'COMPLETE' and live.get('publisher_owned') is False
                            and live.get('command') == ZERO_COMMAND and self._rest(now))
        if complete_release and reason in ('REALIGN_REQUIRED', 'APPROACH_TOTAL_DISTANCE_LIMIT', 'SPRAY_RANGE_REACHED','CLOSE_RANGE_REACHED', 'ODOM_PROGRESS_LOST'):
            if self.ir_stop_step is not None and step != self.ir_stop_step + 1:
                self.stop_fault('IR_REENTRY_STEP_SEQUENCE'); return
            self.last_ir_step = step
            if self.release_since is None:
                self.release_since = now
            if now-self.release_since < (3. if reason == 'ODOM_PROGRESS_LOST' else .5):
                self.phase, self.reason = 'IR_RELEASE', ('STALL_RETRY_WAIT_3S' if reason=='ODOM_PROGRESS_LOST' else 'WAIT_RELEASE_HOLD'); return
            self.ir_recovering = True
            if self._range_samples(now, live):
                self.phase, self.reason = 'WAIT_P', 'FRESH_IR_STOP_CONFIRMED_PRESS_P'; return
            if not self._fire(now):
                self.phase, self.reason = 'WAIT_TARGET', 'IR_RETAINED_WAIT_FRESH_YOLO'; return
            if not self._aligned(now):
                self.phase, self.reason = 'REALIGNING', 'RELEASED_RECENTER_SAME_I_BUDGET'; return
            if cap or b['exhausted']:
                self.phase, self.reason = 'IR_CAP_HOLD', 'FORWARD_LOCKED_WAIT_RANGE_OR_X'; return
            if reason in ('REALIGN_REQUIRED', 'ODOM_PROGRESS_LOST'):
                self.phase, self.reason = 'IR_RESUME', 'SAME_I_WAIT_LIVE_REENTRY'; return
            self.phase, self.reason = 'IR_CONFIRM', 'WAIT_NEW_IR_SAMPLES'; return
        if state == 'WAITING_FOR_ADMISSION' and self.ir_recovering:
            expected = self.ir_stop_step + 1 if self.ir_stop_step is not None else step
            if (step != expected or live.get('recovery_required') is not False
                    or live.get('publisher_owned') is not False or not self._rest(now)):
                self.phase, self.reason = 'IR_RELEASE', 'WAIT_VALID_REENTRY_STEP'; return
            if not self._fire(now):
                self.phase, self.reason = 'WAIT_TARGET', 'WAIT_FRESH_YOLO'; return
            if not self._aligned(now):
                self.phase, self.reason = 'REALIGNING', 'RECENTER_BEFORE_IR_REENTRY'; return
            bands=('SPRAY_RANGE','TOO_CLOSE') if self.allow_close else ('SPRAY_RANGE',)
            if validated_ir_band(self.get('ir',now)) in bands and self._off(now):
                # A released I step may already be in spray range before
                # reentry. Confirm at rest; never request another forward
                # step or renew the original distance/time budget.
                self.pending_stationary=True
                self.last_ir_step=step
                self.phase,self.reason='IR_CONFIRM','IR_REENTRY_STATIONARY_CONFIRM'
                return
            if b['exhausted']:
                self.phase, self.reason = 'IR_CAP_HOLD', 'ORIGINAL_I_BUDGET_EXHAUSTED'; return
            self.ir_stop_step = None
            self.phase, self.reason = 'IR_RESUME', 'SAME_I_BUDGET_NO_NEW_REQUEST'; return
        if state == 'ABORTED':
            self.phase, self.reason = 'IR_HOLD', 'IR_ABORTED:'+str(reason)
            return
        if b['exhausted']:
            self.phase, self.reason = 'IR_CAP_HOLD', 'ORIGINAL_I_BUDGET_EXHAUSTED'; return
        if not self._fire(now):
            self.phase, self.reason = 'WAIT_TARGET', 'WAIT_FRESH_YOLO'; return
        self.release_since = None
        self.ir_has_run |= state == 'MOVING'
        self.phase, self.reason = 'IR_RUNNING', 'I_ACCEPTED_APPROACH_UNTIL_IR_RANGE'

    def _key(self, key, now):
        self.event_index += 1
        accepted = False; reason = 'KEY_NOT_ALLOWED_IN_'+self.phase
        if key in ('Q', 'ESC', 'WINDOW_CLOSED'):
            self.clear_prearm()
            self.closed = True; self.armed = False
            self.phase, self.reason = 'STOPPED', 'OPERATOR_STOP'
            accepted, reason = True, 'STOP_ACCEPTED'
        elif key == 'A' and not self.armed and self.fault is None:
            if self.stable_since is not None and now-self.stable_since >= 3. and self._rest(now):
                # R4 policy: A is remembered only AFTER readiness, for <=2 s.
                # Repeats never extend the deadline. No permission until fresh.
                if self.arm_requested_at is None:
                    self.arm_requested_at = now
                    self.arm_request_index = self.event_index
                reason = 'A_RECEIVED_WAIT_FRESH_IR_MAX_2S'
            else:
                reason = 'WAIT_THREE_SECONDS_FRESH_STANDBY'
        elif key == 'F' and self.armed and self.cycles.current is None:
            self.fire_permission = True; accepted, reason = True, 'FIRE_WATCH_ENABLED'
        elif key == 'T' and self.phase == 'WAIT_T':
            self.turn_permission = True; accepted, reason = True, 'TURN_REQUEST_ACCEPTED'
        elif key == 'I' and self.phase == 'WAIT_IR_I':
            band = validated_ir_band(self.get('ir', now))
            allowed = ('APPROACH', 'TOO_FAR', 'SPRAY_RANGE', 'TOO_CLOSE') if self.allow_close else ('APPROACH', 'TOO_FAR', 'SPRAY_RANGE')
            accepted = self.cycles.budget.request(now, admission_ready=self._aligned(now) and band in allowed)
            if not accepted:
                reason = 'I_BLOCKED_IR_STALE_OR_RANGE_OR_OPTIN'
            if accepted:
                self.pending_stationary = validated_ir_band(self.get('ir', now)) in ('SPRAY_RANGE', 'TOO_CLOSE')
                self.ir_baseline_seq = obj(self.get('ir', now)).get('ir_sample_seq')
                reason = 'I_ACCEPTED_STATIONARY_CONFIRM' if self.pending_stationary else 'I_ACCEPTED_APPROACH'
        elif key == 'P' and self.phase == 'WAIT_P' and not self.p_used:
            actor_blocker = self._pulse_admission(now)
            if actor_blocker:
                reason = actor_blocker
            elif self._aligned(now) and self._off(now):
                self.p_used = True; accepted, reason = True, 'P_ACCEPTED_ONE_PULSE'
                self.p_request_baseline = self.get('actor', now)['pulse_request_feedback']['index']
        elif key == 'X' and self.cycles.current is not None:
            if self.p_used:
                self.cancel_requested=True;self.turn_permission=False
                self.interruption_reason='OPERATOR_CANCEL_AFTER_PULSE'
                self.retry_fire_clear_required=True;self.retry_clear_since=None
            else:self.interrupt_cycle('OPERATOR_CANCEL')
            accepted, reason = True, 'CANCEL_CYCLE_REQUEST_ACCEPTED'
        elif key == 'X' and self.armed and self.cycles.current is None:
            status=self._status(now)
            if status is not None:
                self.navigation_cancel_uuid=self.navigation_cancel_uuid or status.active_uuid
                accepted=True
                reason='NAV_GOAL_CANCEL_REQUESTED' if self.navigation_cancel_uuid else 'NO_ACTIVE_NAV_GOAL'
        self.feedback = dict(index=self.event_index, key=key, accepted=accepted, reason=reason, at=now)
        if key == 'P' and accepted:self.p_request_feedback = dict(self.feedback)
        if key == 'A':
            self._try_pending_arm(now)

    def apply_intent(self, intent, now, *, origin):
        """Semantic automatic request; run the same stage admission checks."""
        keys={'ARM':'A','FIRE_WATCH':'F','ALIGN':'T','APPROACH':'I','PULSE':'P'}
        if origin!='AUTOMATIC' or intent not in keys:raise ValueError('AUTOMATIC_INTENT_INVALID')
        self.step(now,(keys[intent],))
        self.feedback.update(origin=origin,intent=intent)
        if intent=='PULSE' and self.feedback.get('accepted') is True:
            self.p_request_feedback=dict(self.feedback)
        return self.report(now)

    def step(self, now, keys=()):
        if not finite(now) or (self.last_now is not None and now < self.last_now):
            self.stop_fault('CLOCK_DISCONTINUITY')
            return self.report(self.last_now if self.last_now is not None else 0.)
        if self.last_now is not None and now-self.last_now > .35:
            self.release_since = None
            self.confirm_samples = []
            if not self.armed:
                self.clear_prearm()
        self.last_now = now
        graph = obj(self.get('graph', now))
        boot,boot_at=self.inputs.get('identity',(None,None))
        boot_ok=identity_current(boot,boot_at,now) if self.require_boot else True
        if self.require_boot:
            raw_known=(graph.get('raw_count')==0 or graph.get('raw_count')==1 and graph.get('raw_owner') in
                ('live_nav_stop_gateway','live_alignment_takeover','ir_approach_live'))
            stopped=raw_known and zero(self.get('cmd',now)) and zero(self.get('odom',now),.02,.05) and self._motion_off(now)
            ready=self.identity_lease.observe(boot,boot_at,now,armed=self.armed,stopped=stopped)
            self.identity_hold=self.armed and not ready
            if self.identity_lease.fault:self.stop_fault(self.identity_lease.fault)
            if not self.identity_hold and self.identity_saved_phase is not None and self.fault is None:
                self.phase=self.identity_saved_phase;self.identity_saved_phase=None
        if graph.get('fault'):
            self.stop_fault(str(graph['fault']))
        live_actuator=obj(self.get('actuator',now))
        if live_actuator.get('servo_phase')=='LOCKED' and not self.navigation_only:self.stop_fault('ACTUATOR_LOCKED')
        if self.armed and not self.p_used and live_actuator and live_actuator.get('pump_pwm_commanded_on') is True:
            self.stop_fault('UNEXPECTED_PUMP_ON')
        if self.identity_hold and self.fault is None and not self.closed:
            if self.identity_saved_phase is None:self.identity_saved_phase=self.phase
            self.phase,self.reason='WAIT_IDENTITY','WAIT_FRESH_SAME_BOOT_AND_STOP_HOLD'
            self.release_since=None;self.confirm_samples=[]
            for key in keys:
                if str(key).upper() in ('Q','ESC','WINDOW_CLOSED','X'):self._key(str(key).upper(),now)
                else:
                    self.event_index+=1
                    self.feedback=dict(index=self.event_index,key=str(key).upper(),accepted=False,
                        reason='WAIT_CURRENT_BOOT_IDENTITY',at=now)
            return self.report(now)
        if self.graph_hold and self.fault is None and not self.closed:
            self.phase,self.reason='WAIT_GRAPH','WAIT_NAV_RAW_OWNER_NAME_ZERO_ONLY'
            for key in keys:
                if str(key).upper() in ('Q','ESC','WINDOW_CLOSED','X'):self._key(str(key).upper(),now)
            return self.report(now)
        if self.input_hold and self.fault is None and not self.closed:
            self.phase,self.reason='WAIT_INPUT','WAIT_INPUT_RECOVERY_STOP_0_5S'
            for key in keys:
                if str(key).upper() in ('Q','ESC','WINDOW_CLOSED','X'):self._key(str(key).upper(),now)
                else:
                    self.event_index+=1
                    self.feedback=dict(index=self.event_index,key=str(key).upper(),accepted=False,
                        reason=self.reason,at=now)
            return self.report(now)
        healthy = graph.get('valid') is True
        if not self.closed and self.fault is None:
            if not healthy:
                self.clear_prearm()
                if self.armed:
                    self.stop_fault('GRAPH_OR_DATA_PROOF_LOST')
                else:
                    self.phase, self.reason = 'WAIT_ARM', 'WAIT_VALID_GRAPH_AND_DATA'
            elif not self.armed:
                standby = not self._standby_proof(now, boot_ok=boot_ok)['blockers']
                if not standby:
                    self.clear_prearm()
                elif self.stable_since is None:
                    self.stable_since = now
                self.phase = 'WAIT_ARM'
                self.reason = ('WAIT_CURRENT_BOOT_IDENTITY' if not boot_ok else
                    'READY_PRESS_A' if self.stable_since is not None and now-self.stable_since >= 3. else 'WAIT_STABLE_STANDBY')
            else:
                self._advance(now)
            for key in keys:
                if isinstance(key, str):
                    self._key(key.upper(), now)
            if not self.closed and not self.armed and self.fault is None:
                self._try_pending_arm(now)
            if self.armed and self.fault is None and not self.closed:
                self._advance(now)
        elif keys:
            for key in keys:
                if str(key).upper() in ('Q', 'ESC', 'WINDOW_CLOSED'):
                    self._key(str(key).upper(), now)
                else:
                    self.event_index += 1
                    self.feedback = dict(index=self.event_index, key=str(key).upper(),
                        accepted=False, reason='FAULT_BLOCKED:'+str(self.fault) if self.fault else 'SESSION_CLOSED', at=now)
        return self.report(now)

    def accept_handback(self,receipt,now,*,expected_context):
        if self.fault or self.closed or not self.armed:
            raise ValueError('SESSION_CANNOT_ACCEPT_HANDBACK')
        self.cycles.retire(receipt,now,expected_context=expected_context)
        self.cancel_requested=self.p_used=self.turn_permission=self.fire_permission=False
        self.release_since=self.ir_stop_step=self.ir_baseline_seq=None
        self.last_ir_step=0;self.ir_recovering=self.ir_has_run=self.pending_stationary=False
        self.confirm_samples=[];self.handback_completed=True
        self.ir_confirmation_seen_at=None
        self.p_request_baseline = self.p_request_feedback = None
        self.phase,self.reason='ARMED_IDLE','NAV2_READY_NEW_GOAL'

    def _advance(self, now):
        self._observe_retry_clear(now)
        if self.p_used:self._pulse_receipt(now)
        actuator = obj(self.get('actuator', now))
        if self.navigation_only:
            count=actuator.get('home_command_completed_count')
            if (self._off(now) and type(count) is int and self.navigation_home_before is not None
                    and count>self.navigation_home_before):
                self.navigation_only=False;self.navigation_home_before=None
        if actuator.get('servo_phase') == 'LOCKED' and not self.navigation_only:
            self.stop_fault('ACTUATOR_LOCKED'); return
        if not actuator:
            self.phase, self.reason = 'WAIT_DATA', 'WAIT_FRESH_ACTUATOR'; return
        if not self.p_used and actuator.get('pump_pwm_commanded_on') is not False:
            self.stop_fault('UNEXPECTED_PUMP_ON'); return
        status = self._status(now)
        if self.fault:
            return
        if status is None:
            self.phase, self.reason = 'WAIT_DATA', 'WAIT_FRESH_NAV_STATUS'; return
        if self.navigation_cancel_uuid is not None:
            terminal=[s for u,s in status.rows if u==self.navigation_cancel_uuid]
            if terminal in ([4],[5],[6]) and self._rest(now):
                # A goal submitted before cancel/STOP release must also settle;
                # otherwise the gateway can never accumulate its idle proof.
                self.navigation_cancel_uuid=status.active_uuid
                if self.navigation_cancel_uuid is not None:
                    self.phase,self.reason='NAV_CANCEL_WAIT','CANCEL_NEW_GOAL_BEFORE_RELEASE';return
            else:
                receipt=obj(obj(self.get('guard',now)).get('navigation_cancel'))
                detail=receipt.get('state') if receipt.get('goal_uuid')==self.navigation_cancel_uuid else None
                self.phase='NAV_CANCEL_WAIT'
                self.reason='NAV_CANCEL_'+detail if detail else 'WAIT_REAL_NAV_TERMINAL_AND_STOP'
                return
        cycle = self.cycles.current
        if cycle is None:
            if self.fire_permission and not self.navigation_only and not self.retry_fire_clear_required and self._fire(now):
                self.handback_completed=False
                self.interruption_reason=None
                self.cycles.begin(status.rows, self.inputs['status'][1], now, armed=True, fire_confirmed=True,
                                  retained_source_current=obj(self.get('graph',now)).get('nav_status_event_stream') is True)
                cycle = self.cycles.current
            else:
                g = obj(self.get('gateway', now))
                self.phase = 'NAVIGATION' if status.active_uuid else 'ARMED_IDLE'
                self.reason = ('NAV2_ACTIVE_GOAL' if status.active_uuid else
                               'NAV2_READY_NEW_GOAL' if self._rest(now) and (self.handback_completed or g.get('normal_rearm_ready') is True)
                               else 'WAIT_GOAL_OR_F')
                return
        # An action can finish between visual cycle registration and cancel.
        # Retire this unused cycle with its REAL terminal result, then let the
        # still-visible fire enter a fresh no-goal cycle. Never invent cancel.
        if (cycle.nav_goal_uuid is not None and not self.turn_permission
                and self.cycles.budget.requested_at is None and not self.p_used
                and not self.cancel_requested and status.active_uuid is None
                and [s for u,s in status.rows if u==cycle.nav_goal_uuid] in ([4],[5],[6])):
            guard=obj(self.get('guard',now))
            if ([s for u,s in status.rows if u==cycle.nav_goal_uuid] != [5]
                    or not (guard.get('goal_uuid')==cycle.nav_goal_uuid
                    and guard.get('cancel_service_accepted') is True
                    and guard.get('send_count')==1)):
                self.interrupt_cycle('NAV_GOAL_TERMINAL_BEFORE_FIRE_HANDOFF')
                self.retry_fire_clear_required=False
        if status.active_uuid not in (None, cycle.nav_goal_uuid):
            self.phase, self.reason = 'GOAL_BLOCKED', 'NEW_GOAL_BEFORE_CYCLE_RELEASE'; return
        if self.cancel_requested:
            self.phase, self.reason = 'CANCEL_WAIT_CLEAR', ('INTERRUPTED:'+self.interruption_reason if self.interruption_reason else 'WAIT_CLEAR_RELEASE_ACK_R5_3'); return
        if self.p_used:
            actor = obj(self.get('actor', now))
            sequence=obj(actor.get('spray_sequence'))
            if sequence.get('stage')=='REVIEW':
                self.phase=('POSTSPRAY_REALIGN' if sequence.get('can_align') is True else
                    'IR_RESUME' if sequence.get('can_approach') is True else 'POSTSPRAY_WAIT')
                self.reason=str(sequence.get('reason') or 'WAIT_POSTSPRAY_REVIEW')
                return
            if sequence.get('stage')=='ARM_NEXT' and actor.get('pump_publisher_registered') is not True:
                self.phase,self.reason='POSTSPRAY_WAIT','WAIT_RETRY_PUMP_REGISTRATION';return
            self.phase = 'POSTSPRAY_HOLD'
            self.reason = ('P_WAIT_ACTOR_RECEIPT_NO_AUTOMATIC_RETRY'
                if actor.get('request_used') is not True else
                'P_ACTOR_ABORTED:'+str(actor.get('reason')) if actor.get('state') == 'ABORTED' else
                'WAIT_ACTOR_OFF_HOME_REVIEW_HANDOFF_R5_3')
            return
        if self.cycles.budget.requested_at is not None:
            if self.pending_stationary:
                # Stationary I used to wait forever when centering was lost.
                # Reuse the same released turn and I budget; no forward motion.
                if not self._fire(now):
                    self.confirm_samples=[]
                    self.phase,self.reason='WAIT_TARGET','STATIONARY_WAIT_FRESH_YOLO';return
                if self._alignment_owned(now) or not self._centered(now):
                    self.confirm_samples=[]
                    self.phase,self.reason='REALIGNING','STATIONARY_RECENTER_SAME_I_BUDGET';return
                actor=obj(self.get('actor',now));confirmation=obj(actor.get('stationary_confirmation'))
                if self._actor_cycle_current(actor) and confirmation.get('state')=='ABORTED':
                    if confirmation.get('restartable') is True and actor.get('request_used') is False:
                        # This phase withdraws stationary ACTIVE in the mapper.
                        # Wait for the actor's actual OFF/STOP/endpoint drain and
                        # false-edge reset; never relabel ABORTED as COMPLETE.
                        self.phase,self.reason='IR_RELEASE','WAIT_STATIONARY_WINDOW_RESTART'
                    else:
                        self.interrupt_cycle('STATIONARY_CONFIRMATION_ABORTED:'+str(confirmation.get('reason')))
                    return
                endpoint_clear=(self._actor_cycle_current(actor)
                    and actor.get('request_used') is False
                    and actor.get('pump_publisher_registered') is False
                    and obj(actor.get('counts')).get('pump_total')==0)
                band=validated_ir_band(self.get('ir',now))
                if band in ('APPROACH','TOO_FAR'):
                    # Range can move away after stationary I. Revoke that
                    # proof first, then use the existing initial/reentry IR
                    # path with the same cycle, step counter and task budget.
                    live=obj(self.get('ir_live',now))
                    if (endpoint_clear and self._rest(now) and self._aligned(now)
                            and live.get('state')=='WAITING_FOR_ADMISSION'
                            and live.get('publisher_owned') is False
                            and live.get('command')==ZERO_COMMAND):
                        self.pending_stationary=False;self.confirm_samples=[]
                        self.release_since=None
                        self._approach(now)
                    else:
                        self.phase,self.reason='IR_RELEASE','WAIT_STATIONARY_RELEASE_BEFORE_APPROACH'
                    return
                if confirmation.get('state')=='WAIT_I' and actor.get('pump_publisher_registered') is True:
                    self.phase,self.reason='IR_RELEASE','WAIT_STATIONARY_OFF_ENDPOINT_RELEASE'
                    return
                # The actor already proves the configured fresh IR samples and STOP.
                # Do not run a second competing stationary confirmation loop.
                if (self._actor_cycle_current(actor) and confirmation.get('state')=='COMPLETE'
                        and confirmation.get('goal_uuid')==cycle.nav_goal_uuid
                        and self._aligned(now) and self._off(now)):
                    self.phase, self.reason = 'WAIT_P', 'I_CONFIRMED_STATIONARY_PRESS_P'
                else:
                    self.phase, self.reason = 'IR_CONFIRM', 'I_ACCEPTED_WAIT_NEW_IR_CENTERED_STOP'
                return
            self._approach(now); return
        if self._alignment_owned(now):
            self.phase, self.reason = ('ALIGNING', 'TURN_REQUEST_ACTIVE') if self._fire(now) else ('WAIT_TARGET', 'WAIT_ALIGNMENT_ZERO_RELEASE')
            return
        # A COMPLETE turn is not permission to start another turn while the
        # target is still centered. Let gateway/guard STOP reports catch up;
        # their independently timed zero hold is re-established after release.
        alignment = obj(self.get('alignment', now))
        if (self.turn_permission and self._centered(now)
                and alignment.get('state') == 'COMPLETE'
                and alignment.get('stop_reason') == 'TARGET_CENTERED'
                and alignment.get('publisher_owned') is False
                and alignment.get('goal_uuid') == cycle.nav_goal_uuid
                and (cycle.nav_goal_uuid is not None or
                     same_cycle(alignment, self.no_goal_context(now)))):
            if self._aligned(now):
                self.phase, self.reason = 'WAIT_IR_I', 'CENTERED_RELEASED_PRESS_I'
            else:
                self.phase, self.reason = 'WAIT_ALIGN_STOP', 'CENTERED_WAIT_FRESH_STOP_HANDOFF'
            return
        if not self._handoff(now):
            self.phase,self.reason=(('WAIT_NO_GOAL_HANDSHAKE','WAIT_NO_ACTIVE_GOAL_STOP_RELEASE')
                if cycle.nav_goal_uuid is None else ('WAIT_STOP','WAIT_EXACT_CANCEL_ZERO_RELEASE'));return
        if not self._fire(now):
            self.phase, self.reason = 'WAIT_TARGET', 'IR_RETAINED_WAIT_YOLO_OR_CLEAR_HANDOFF'; return
        if self._aligned(now):
            self.phase, self.reason = 'WAIT_IR_I', 'CENTERED_RELEASED_PRESS_I'; return
        self.phase, self.reason = ('ALIGNING', 'TURN_REQUEST_ACTIVE') if self.turn_permission else ('WAIT_T', 'STOP_PROVED_PRESS_T')

    def task_report(self, now):
        cycle=self.cycles.current
        if self.closed or self.fault:return dict(stage='STOP',outcome='INTERRUPTED',reason=self.fault or self.reason)
        if not self.armed:return dict(stage='STARTUP',outcome='RUNNING',reason=self.reason)
        if cycle is None:
            if self.phase=='NAVIGATION' or self.phase=='ARMED_IDLE' and self.reason=='NAV2_READY_NEW_GOAL':
                return dict(stage='NAVIGATION',outcome='RUNNING' if self.phase=='NAVIGATION' else 'COMPLETED',
                    reason=self.reason,last_interruption_reason=self.interruption_reason,
                    retry_after_new_fire=self.retry_fire_clear_required)
            return dict(stage='NAVIGATION',outcome='INTERRUPTED' if self.interruption_reason else
                'COMPLETED' if self.handback_completed else 'RUNNING',
                reason=self.interruption_reason or self.reason,
                retry_after_new_fire=self.retry_fire_clear_required)
        if self.cancel_requested:return dict(stage='RETURN_NAV2',outcome='INTERRUPTED',reason=self.interruption_reason or self.reason)
        if self.p_used:
            actor=obj(self.get('actor',now))
            return dict(stage='SPRAY',outcome='COMPLETED' if actor.get('state')=='DONE' else
                'INTERRUPTED' if actor.get('state')=='ABORTED' else 'RUNNING',reason=actor.get('reason') or self.reason)
        stage='IR' if self.cycles.budget.requested_at is not None else 'ALIGN'
        done=self.phase=='WAIT_P' if stage=='IR' else self._aligned(now)
        return dict(stage=stage,outcome='COMPLETED' if done else 'RUNNING',reason=self.reason)

    def report(self, now):
        budget = self.cycles.budget.report(now) if self.cycles.budget else None
        align = (self.phase in ('ALIGNING', 'REALIGNING') or self._alignment_owned(now)) and self.turn_permission
        align = ((align and not self.p_used) or self.phase=='POSTSPRAY_REALIGN') and not self.cancel_requested
        ir = (self.phase in ('IR_RUNNING', 'IR_RESUME') and not self.pending_stationary) or self.p_used and obj(obj(self.get('actor',now)).get('spray_sequence')).get('can_approach') is True
        active = self.armed and not self.closed and self.fault is None and not self.identity_hold and not self.graph_hold and not self.input_hold
        intents = dict(navigation=active and self.cycles.current is None and self.navigation_cancel_uuid is None,
                       cancel=active and self.cycles.current is not None,
                       alignment=active and align, ir=active and ir,
                       stationary_confirmation=active and self.pending_stationary and self.phase=='IR_CONFIRM',
                       pulse=active and self.p_used and self.phase=='POSTSPRAY_HOLD',
                       cancel_cycle=active and self.cancel_requested)
        ir_data = obj(self.get('ir', now))
        counts = ir_data.get('filtered_counts')
        if (not isinstance(counts,dict) or not all(type(counts.get(k)) is int
                and 0<=counts[k]<=32767 for k in ('left','center','right'))):
            counts = None
        display_band=validated_ir_band(ir_data)
        if (display_band is None and ir_data.get('schema_version')==1
                and ir_data.get('observe_only') is True and ir_data.get('ir_band')=='NO_FLAME_SIGNATURE'
                and counts is not None and all(n>20000 for n in counts.values())):
            display_band='NO_FLAME_SIGNATURE'
        last_ir,ir_at=self.inputs.get('ir',(None,None));last_ir=obj(last_ir)
        source_age=last_ir.get('ir_receive_age_ms')
        source_age=(source_age+(now-ir_at)*1000 if finite(source_age) and finite(ir_at) and now>=ir_at else None)
        raw_ir,raw_at=self.inputs.get('ir_raw',(None,None))
        raw_age=obj(raw_ir).get('receive_age_ms')
        raw_age=(raw_age+(now-raw_at)*1000 if finite(raw_age) and finite(raw_at) and now>=raw_at else None)
        guard=obj(self.get('guard',now))
        return dict(schema_version=1, scope='R5_SESSION_INTENT_ONLY', task=self.task_report(now),phase=self.phase,
                    reason=self.reason, fault=self.fault, navigation_only=self.navigation_only,
                    navigation_home_before=self.navigation_home_before, armed=self.armed, closed=self.closed,identity_hold=self.identity_hold,graph_hold=self.graph_hold,
                    input_hold=self.input_hold,input_hold_blockers=list(self.input_hold_blockers),
                    input_recovery_stable_s=(max(0.,now-self.input_recovery_since) if self.input_recovery_since is not None else 0.),
                    armed_identity=list(self.identity_lease.bound) if self.identity_lease.bound else None,
                    navigation_cancel_uuid=self.navigation_cancel_uuid,
                    interruption_reason=self.interruption_reason,retry_fire_clear_required=self.retry_fire_clear_required,
                    recovery_clear_policy=self.recovery_clear_policy,recovery_require_ir_clear=self.recovery_require_ir_clear,
                    startup_readiness=self._standby_proof(now),
                    cycle=self.cycles.retirement_request(), budget=budget,
                    operator_feedback=copy.deepcopy(self.feedback),
                    spray_sequence=copy.deepcopy(obj(obj(self.get('actor',now)).get('spray_sequence'))),
                    p_admission_blocker=(self._pulse_admission(now) if self.phase == 'WAIT_P' and not self.p_used else None),
                    intents=intents, ir_band=validated_ir_band(ir_data),
                    ir_counts=counts, ir_display_band=display_band, yolo_fresh=self.get('vision', now) is not None,
                    ir_source_age_ms=round(source_age,1) if source_age is not None else None,
                    ir_raw_source_age_ms=round(raw_age,1) if raw_age is not None else None,
                    range_confirm_samples=(obj(obj(self.get('actor',now)).get('stationary_confirmation')).get('sample_count',0)
                        if self.pending_stationary else len(self.confirm_samples)),
                    ir_confirm_required=self.ir_confirm_samples,
                    stationary_confirmation_reason=obj(obj(self.get('actor',now)).get('stationary_confirmation')).get('reason'),
                    aligned_stop_proof=self._aligned(now) if self.cycles.current else False,
                    cancel_guard_state=guard.get('state'),cancel_guard_blockers=guard.get('blockers'),
                    blocked_goal_cancel=guard.get('blocked_goal_cancel'),
                    fire_confirmed=self._fire(now), centered=self._centered(now),
                    nav_status_receive_age_s=(max(0.,now-self.inputs['status'][1])
                        if 'status' in self.inputs else None),
                    physical_commands_published=False, live_intents_published=False,
                    handback_implemented=False, motion_authorized=False)
