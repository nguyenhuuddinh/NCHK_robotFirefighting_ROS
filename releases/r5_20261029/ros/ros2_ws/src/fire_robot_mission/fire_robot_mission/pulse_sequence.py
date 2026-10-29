"""Pulse sequence with a legacy policy and an explicit R5 automatic policy.

R5 admits three consecutive rounds and grants a new quota only after an
actual completed reposition. Legacy defaults retain their 5s travel-budget
policy. Neither policy fabricates pulse, home or STOP receipts.
"""
import copy
from .pulse_session import PulseSession, number
from .operational_cycle import zero
from .ir_approach_shadow_core import validated_ir_band

# A same-connection permission gap ends the old ON; fresh OFF/home/STOP and
# complete admission may start a NEW round. Actuator faults and explicit
# STOP/cancel remain terminal. Already admitted rounds retain their quota.
RECOVERABLE_PERMISSION_GAPS=frozenset(('OPERATOR_ARM_MISSING_OR_STALE',
                                    'OPERATOR_HEARTBEAT_MISSING'))

class PulseSequenceSession:
    def __init__(self, *, enabled=False, allow_provisional_ir=False,
                 allow_close_stationary=False, max_pulses=3,
                 max_total_on_s=15., settle_s=2., retry_window_s=30.,
                 pulse_s=5., start_on_ack=False, ir_confirm_samples=5,
                 unbounded_approach=False, reposition_resets_quota=False):
        if type(max_pulses) is not int or not 1 <= max_pulses <= 3:
            raise ValueError('pulse sequence limit must be 1..3')
        if (not all(number(v) for v in (max_total_on_s, settle_s, retry_window_s))
                or not pulse_s <= max_total_on_s <= 3*pulse_s
                or settle_s < (0 if reposition_resets_quota else 2)
                or not (0 <= retry_window_s <= 30 if reposition_resets_quota else 2 < retry_window_s <= 30)):
            raise ValueError('pulse sequence timing invalid')
        self.r5_initialization = {k:v for k,v in locals().items() if k not in ('self','__class__')}
        self.base_config=dict(enabled=enabled, allow_provisional_ir=allow_provisional_ir,
                              allow_close_stationary=allow_close_stationary,pulse_s=pulse_s,
                              start_on_ack=start_on_ack,ir_confirm_samples=ir_confirm_samples,unbounded_approach=unbounded_approach)
        self.current=PulseSession(**self.base_config)
        self.reposition_resets_quota=reposition_resets_quota is True
        self.consecutive=0;self.can_approach=False;self.reposition_performed=False
        self.post_off_decision=None;self.target_lost_since=None;self.quota_resets=0
        self.max_pulses=max_pulses;self.max_total_on_s=float(max_total_on_s)
        self.settle_s=float(settle_s);self.retry_window_s=float(retry_window_s)
        self.index=0;self.reserved_on_s=0.;self.first_home=None
        self.stage='ROUND';self.reason='';self.history=[]
        self.review_started=None;self.off_since=None;self.samples=[]
        self.clear_since=None;self.last_now=None;self.can_align=False
        self.cancelled=False;self.hard_fault=None;self.last_report={}
        self.retry_confirmed_at=None;self.retry_band=None;self.retry_proof=[]
        self.actuation_started=False;self.last_started_home=None

    def __getattr__(self, name):
        return getattr(self.current, name)

    @property
    def r5_cycle_context(self):return self.current.r5_cycle_context

    @r5_cycle_context.setter
    def r5_cycle_context(self, value):self.current.r5_cycle_context=value

    @property
    def used(self):return self.index > 0 or self.current.used

    @property
    def phase(self):
        if self.stage=='ROUND':return self.current.phase
        if self.stage=='ARM_NEXT':return 'WAIT_P'
        if self.stage=='TERMINAL':return 'ABORTED' if self.cancelled or not self.current.normal_end else 'DONE'
        return 'RETRY_WAIT'

    @property
    def start_home_count(self):return self.first_home

    @property
    def start_home(self):
        # During next-round registration no new pulse has started yet.
        # Endpoint withdrawal must still use the last actual pulse baseline.
        if self.actuation_started:return self.last_started_home
        value=self.current.start_home
        return value if type(value) is int else self.history[-1]['home_before'] if self.history else None

    def registration_session(self):
        return self.current if self.stage in ('ROUND','ARM_NEXT') else None

    def recoverable_interruption(self):
        return (not self.current.normal_end and self.current.on_ack
                and self.current.reason in RECOVERABLE_PERMISSION_GAPS)

    def _safe_off(self, now, counts):
        a=self.actuator(now)
        return bool(a and self.idle(a) and counts.get('actuator')==1
            and counts.get('raw')==0 and zero(self.get('cmd',now))
            and zero(self.get('odom',now),.02,.05)
            and type(self.current.start_home) is int
            and (a['home_command_completed_count']>self.current.start_home
                 if self.current.pump.on_since is not None else
                 a['home_command_completed_count']>=self.current.start_home))

    def _clear(self, now):
        v=self.get('vision',now) or {};fire=v.get('fire') or {}
        ir=self.get('ir',now) or {};values=ir.get('filtered_counts')
        source=v.get('source') or {};age=source.get('age_ms')
        vat=self.inputs.get('vision',(None,None))[1]
        iat=self.inputs.get('ir',(None,None))[1];ia=ir.get('ir_receive_age_ms')
        return bool(v.get('schema_version')==1 and v.get('observe_only') is True
            and v.get('state')=='ok' and fire.get('detected_now') is False
            and fire.get('confirmed') is False and number(age) and number(vat)
            and 0<=age+(now-vat)*1000<=300 and number(ia) and number(iat)
            and 0<=ia+(now-iat)*1000<=500 and ir.get('ir_band')=='NO_FLAME_SIGNATURE'
            and isinstance(values,dict) and all(type(values.get(k)) is int
                and 20000<values[k]<=32767 for k in ('left','center','right')))

    def _new_round(self, now):
        old=self.current;new=PulseSession(**self.base_config)
        new.inputs=copy.deepcopy(old.inputs);new.r5_cycle_context=copy.deepcopy(old.r5_cycle_context)
        new.cycle_id=old.cycle_id;new.pump.r5_cycle_context=copy.deepcopy(old.r5_cycle_context)
        new.stationary=copy.deepcopy(old.stationary)
        # Only stationary actuation may outlive I's travel deadline. A new
        # physical home counter and this local used sequence authorize that
        # distinction; it never grants an IR motion request.
        new.respray_after_home=old.start_home
        new.feed_pump(now)
        return new

    def _assess_after_off(self, now):
        """Evaluate perception as soon as OFF is requested, including home travel."""
        v=self.get('vision',now) or {};f=v.get('fire') or {}
        age=(v.get('source') or {}).get('age_ms');at=self.inputs.get('vision',(None,None))[1]
        fire=(v.get('schema_version')==1 and v.get('observe_only') is True
            and v.get('state')=='ok' and f.get('detected_now') is True
            and f.get('confirmed') is True and number(age) and number(at)
            and 0<=age+(now-at)*1000<=300)
        if not fire:
            if self.target_lost_since is None:self.target_lost_since=now
            return 'WAIT_TARGET'
        self.target_lost_since=None
        p=self.get('preview',now) or {};pa=(p.get('source') or {}).get('age_ms')
        pat=self.inputs.get('preview',(None,None))[1]
        if not (number(pa) and number(pat) and 0<=pa+(now-pat)*1000<=300):return 'WAIT_FRESH_YOLO_AND_IR'
        if p.get('state')!='CENTERED':return 'REALIGN_REQUIRED'
        ir=self.get('ir',now) or {};band=validated_ir_band(ir)
        at=self.inputs.get('ir',(None,None))[1];age=ir.get('ir_receive_age_ms')
        if not (number(at) and number(age) and 0<=age+(now-at)*1000<=500):return 'WAIT_FRESH_YOLO_AND_IR'
        if band in ('APPROACH','TOO_FAR'):return 'APPROACH_REQUIRED'
        if band in (('SPRAY_RANGE','TOO_CLOSE') if self.allow_close_stationary else ('SPRAY_RANGE',)):
            return 'SPRAY_READY'
        return 'WAIT_FRESH_YOLO_AND_IR'

    def _review_automatic(self, now, counts):
        prior_align,prior_approach=self.can_align,self.can_approach
        self.can_align=self.can_approach=False
        self.post_off_decision=self._assess_after_off(now)
        if self.cancelled:
            self.stage='TERMINAL';self.reason='OPERATOR_CANCEL_AFTER_PULSE';return
        if not self.current.normal_end and not self.recoverable_interruption():
            self.stage='TERMINAL';self.reason='PULSE_INTERRUPTED:'+str(self.current.reason);return
        if self.consecutive>=self.max_pulses:
            self.stage='TERMINAL';self.reason='CONSECUTIVE_PULSE_LIMIT_RETURN_NAV2';return
        if self.target_lost_since is not None and now-self.target_lost_since>=3.:
            self.stage='TERMINAL';self.reason='TARGET_NOT_RECOVERED_RETURN_NAV2';return
        align=self.get('alignment',now) or {};live=self.get('ir_live',now) or {}
        if (counts.get('raw')==1 and counts.get('pump_total')==0
                and self.idle(self.actuator(now))):
            for payload,kind in ((align,'ALIGN'),(live,'IR')):
                if (payload.get('r5_cycle_context')==self.r5_cycle_context
                        and (payload.get('publisher_owned') is True or
                             payload.get('state')=='ACQUIRING'
                             and payload.get('command')=={'linear_x_m_s':0.,'angular_z_rad_s':0.}
                             and (prior_align if kind=='ALIGN' else prior_approach))):
                    cmd=payload.get('command') or {}
                    if kind=='ALIGN':
                        self.can_align=True
                        self.reposition_performed |= abs(cmd.get('angular_z_rad_s',0.))>0
                    else:
                        self.can_approach=True
                        self.reposition_performed |= cmd.get('linear_x_m_s',0.)>0
                    self.samples=[];self.reason='WAIT_REPOSITION_STOP_RELEASE';return
        if not self._safe_off(now,counts):
            self.samples=[];self.reason='WAIT_FRESH_OFF_NEW_HOME_AND_STOP';return
        decision=self.post_off_decision
        if decision=='WAIT_TARGET':
            self.samples=[];self.reason='WAIT_TARGET_AFTER_OFF_3S';return
        if decision=='REALIGN_REQUIRED':
            self.samples=[];self.can_align=counts.get('pump_total')==0
            self.reason='WAIT_RECENTER_SAME_CYCLE';return
        if decision=='APPROACH_REQUIRED':
            self.samples=[];self.can_approach=counts.get('pump_total')==0
            self.reason='WAIT_APPROACH_SAME_CYCLE';return
        if decision!='SPRAY_READY':
            self.samples=[];self.reason=decision;return
        # A completed real reposition, not a stale report or phase name, grants
        # a new three-round quota. Next ON still passes fresh complete gates.
        if self.reposition_performed:
            if (align.get('state')!='COMPLETE' or align.get('stop_reason')!='TARGET_CENTERED'
                    or align.get('publisher_owned') is not False):
                self.samples=[];self.reason='WAIT_REPOSITION_COMPLETED';return
            self.consecutive=0;self.quota_resets+=1;self.reposition_performed=False
        if (align.get('state')!='COMPLETE' or align.get('stop_reason')!='TARGET_CENTERED'
                or align.get('publisher_owned') is not False):
            # Centered perception may arrive before the released alignment
            # engine rebuilds its COMPLETE proof. Ask that existing engine
            # to finish; a zero-only admission does not reset the quota.
            self.can_align=counts.get('pump_total')==0
            self.samples=[];self.reason='WAIT_FRESH_ALIGNMENT_COMPLETION';return
        probe=self._new_round(now)
        # A real post-spray IR advance has its own COMPLETE stop proof; do not
        # keep the earlier stationary-only I proof over a changed live stage.
        if live.get('state')=='COMPLETE':
            from .stationary_confirmation import StationaryConfirmation
            probe.stationary=StationaryConfirmation(required_samples=probe.ir_confirm_samples)
        errors=probe.pump.blockers(now,counts)+probe.extra_blockers(now,counts,require_idle=True)
        errors=sorted(set(errors)-{'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'})
        if counts.get('pump_total')!=0:errors.append('WAIT_OLD_PUMP_ENDPOINT_RELEASE')
        if errors:
            self.samples=[];self.reason='WAIT_RETRY_PROOF:'+errors[0];return
        ir=probe.get('ir',now) or {};seq=ir.get('ir_sample_seq')
        previous=self.samples[-1][0] if self.samples else None
        if type(seq) is not int or previous is not None and seq<previous:
            self.samples=[];self.reason='WAIT_NEW_IR_SEQUENCE';return
        if previous is None or seq>previous:self.samples.append((seq,self.inputs['ir'][1],now))
        self.reason='WAIT_THREE_NEW_IR_SAMPLES'
        if len(self.samples)>=self.current.ir_confirm_samples and now-self.samples[0][2]>=.399:
            self.retry_confirmed_at=now;self.retry_band=ir.get('ir_band');self.retry_proof=copy.deepcopy(self.samples)
            self.previous=self.current;self.current=probe;self.stage='ARM_NEXT'
            self.reason='RETRY_READY_WAIT_PUMP_REGISTRATION'

    def _review(self, now, counts):
        if self.reposition_resets_quota:return self._review_automatic(now,counts)
        turning=self.can_align
        self.can_align=False
        align=self.get('alignment',now) or {}
        if (turning and not self.cancelled and self.index<self.max_pulses
                and now-self.review_started<self.retry_window_s and counts.get('raw')==1
                and counts.get('pump_total')==0 and counts.get('alignment')==1
                and align.get('publisher_owned') is True
                and align.get('r5_cycle_context')==self.r5_cycle_context
                and (align.get('command') or {}).get('linear_x_m_s')==0.
                and self.idle(self.actuator(now))):
            # Once the real turn owner exists, motion is expected. Keep its
            # arm through its own zero/release; stationary admission is for
            # the NEXT pulse, not a requirement for continuing this turn.
            self.can_align=True;self.samples=[]
            self.reason='WAIT_RECENTER_SAME_CYCLE';return
        if not self._safe_off(now,counts):
            self.off_since=None;self.clear_since=None;self.samples=[]
            self.reason='WAIT_FRESH_OFF_NEW_HOME_AND_STOP';return
        if self.off_since is None:self.off_since=now
        if self.cancelled:
            self.stage='TERMINAL';self.reason='OPERATOR_CANCEL_AFTER_PULSE';return
        if self._clear(now):
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since>=1:
                self.stage='TERMINAL';self.reason='NO_FIRE_EVIDENCE'
            else:self.reason='WAIT_FRESH_BOTH_CLEAR_HOLD'
            self.samples=[];return
        self.clear_since=None
        if self.index>=self.max_pulses or self.reserved_on_s+5>self.max_total_on_s+1e-9:
            self.reason='PULSE_LIMIT_REACHED_X_CANCEL_OR_WAIT_CLEAR';return
        if now-self.review_started>=self.retry_window_s:
            self.reason='RETRY_WINDOW_EXPIRED_X_CANCEL_OR_WAIT_CLEAR';return
        if now-self.off_since<self.settle_s:
            self.reason='OFF_HOME_SETTLING';return
        vision=self.get('vision',now) or {};fire=vision.get('fire') or {}
        preview=self.get('preview',now) or {};source=vision.get('source') or {}
        vat=self.inputs.get('vision',(None,None))[1];age=source.get('age_ms')
        fresh_fire=(vision.get('state')=='ok' and fire.get('detected_now') is True
            and fire.get('confirmed') is True and number(age) and number(vat)
            and 0<=age+(now-vat)*1000<=300)
        if not fresh_fire:
            self.samples=[];self.reason='WAIT_FRESH_YOLO_AND_IR';return
        if preview.get('state')!='CENTERED':
            self.can_align=counts.get('pump_total')==0
            self.samples=[];self.reason='WAIT_RECENTER_SAME_CYCLE';return
        probe=self._new_round(now)
        errors=probe.pump.blockers(now,counts)+probe.extra_blockers(now,counts,require_idle=True)
        errors=sorted(set(errors)-{'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'})
        if counts.get('pump_total')!=0:errors.append('WAIT_OLD_PUMP_ENDPOINT_RELEASE')
        if errors:
            self.samples=[];self.reason='WAIT_RETRY_PROOF:'+errors[0];return
        ir=probe.get('ir',now) or {};seq=ir.get('ir_sample_seq');received=probe.inputs['ir'][1]
        previous=self.samples[-1][0] if self.samples else None
        if type(seq) is not int or previous is not None and seq<previous:
            self.samples=[];self.reason='WAIT_NEW_IR_SEQUENCE';return
        if previous is None or seq>previous:self.samples.append((seq,received,now))
        self.reason='WAIT_FIVE_NEW_IR_SAMPLES'
        if len(self.samples)>=5 and now-self.samples[0][2]>=.399:
            self.retry_confirmed_at=now;self.retry_band=ir.get('ir_band')
            self.retry_proof=copy.deepcopy(self.samples)
            self.previous=self.current
            self.current=probe;self.stage='ARM_NEXT';self.reason='RETRY_READY_WAIT_PUMP_REGISTRATION'

    def step(self, now, counts, *, request=False, stop=False):
        if not number(now) or self.last_now is not None and now<self.last_now:
            self.hard_fault='SEQUENCE_CLOCK_INVALID';stop=True
        self.last_now=now
        # A completed terminal receipt is history. PREPARE's STOP must not
        # turn that DONE into ABORTED after the coordinator chose its policy.
        if stop and self.stage!='TERMINAL':self.cancelled=True
        if self.cancelled and self.stage=='ARM_NEXT':
            self.current=self.previous;self.stage='REVIEW'
        if self.stage=='ARM_NEXT' and not self.cancelled:
            self.current.feed_pump(now)
            errors=self.current.pump.blockers(now,counts)+self.current.extra_blockers(now,counts,require_idle=True)
            errors=set(errors)-{'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'}
            if (errors or validated_ir_band(self.get('ir',now))!=self.retry_band
                    or now-self.retry_confirmed_at>1.):
                self.current=self.previous;self.stage='REVIEW';self.samples=[];self.off_since=now
                self.reason='RETRY_CONFIRMATION_CHANGED_RECOLLECT';return self.report(False,sorted(errors))
        if self.stage in ('REVIEW','TERMINAL'):
            if self.stage!='TERMINAL':self._review(now,counts)
            return self.report(False,[])
        automatic=(self.stage=='ARM_NEXT' and not self.cancelled and self.get('operator',now,.4) is True
                   and counts.get('pump')==1 and (self.reposition_resets_quota or self.reserved_on_s+self.pump.pulse_s<=self.max_total_on_s+1e-9)
                   and (self.consecutive if self.reposition_resets_quota else self.index)<self.max_pulses)
        was_used=self.current.used
        report=self.current.step(now,counts,request=(request and self.index==0) or automatic,stop=stop)
        if self.current.pump.on_since is not None:
            self.actuation_started=True;self.last_started_home=self.current.start_home
        if not was_used and self.current.used:
            self.index+=1;self.consecutive+=1;self.reserved_on_s+=self.pump.pulse_s
            if self.first_home is None:self.first_home=self.current.start_home
            self.stage='ROUND'
        if self.reposition_resets_quota and self.current.off_at is not None and self.current.normal_end:
            self.post_off_decision=self._assess_after_off(now)
        self.last_report=copy.deepcopy(report)
        if self.current.used and (report['state'] in ('DONE','ABORTED')
                or report['state']=='REVIEW' and self._safe_off(now,counts)):
            self.history.append(dict(pulse_index=self.index,state=report['state'],reason=report['reason'],
                home_before=self.current.start_home,pump_on_feedback_seen=self.current.on_ack,
                actuation_started=self.current.pump.on_since is not None,
                servo_sweep_feedback_seen=self.current.sweep_ack,pulse_core=copy.deepcopy(self.current.pulse_core)))
            self.stage='REVIEW';self.review_started=now;self.off_since=None;self.samples=[]
            self._review(now,counts)
        return self.report(report['proposed_pump_on'],report['blockers'])

    def report(self, proposed, blockers):
        report=self.current.report(proposed and self.stage=='ROUND' and not self.cancelled,blockers)
        report.update(state=self.phase,request_used=self.used,
            reason=self.reason if self.stage!='ROUND' else report['reason'],
            automatic_respray=True,start_home_count=self.first_home,
            spray_sequence=dict(schema_version=1,stage=self.stage,pulse_index=self.index,
                max_pulses=self.max_pulses,reserved_on_s=self.reserved_on_s,
                max_total_on_s=self.max_total_on_s,settle_s=self.settle_s,
                reason=self.reason,can_align=self.can_align and not self.cancelled,
                can_approach=self.can_approach and not self.cancelled,
                consecutive_count=self.consecutive,quota_resets=self.quota_resets,
                post_off_decision=self.post_off_decision,pulse_s=self.pump.pulse_s,
                quota_policy='CONSECUTIVE_AFTER_REPOSITION' if self.reposition_resets_quota else 'PER_CYCLE',
                release_without_clear=self.reposition_resets_quota,
                require_new_fire_event=self.reposition_resets_quota and self.reason=='CONSECUTIVE_PULSE_LIMIT_RETURN_NAV2',
                confirmation_band=self.retry_band,confirmation_samples=copy.deepcopy(self.retry_proof),
                confirmation_sample_count=len(self.samples) if self.stage=='REVIEW' else len(self.retry_proof),
                confirmation_required=self.current.ir_confirm_samples,
                cancelled=self.cancelled,hard_fault=self.hard_fault,history=copy.deepcopy(self.history)))
        report['spray_sequence']['recoverable_permission_gap']=self.recoverable_interruption()
        report['spray_sequence']['actuation_started']=self.actuation_started
        report['spray_sequence'].update(pulse_state=self.current.phase,
            perception_policy=report.get('perception_policy'),pulse_remaining_s=report.get('pulse_remaining_s'))
        return report
