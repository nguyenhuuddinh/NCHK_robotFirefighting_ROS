"""Map actual installed pulse telemetry to canonical subscriber-only review.

A completed pulse is history. It may remain while that exact actor resets via
COMMIT, but is not replaced by DONE or a fabricated new firing sequence.
Fresh camera, IR and stopped-robot evidence is reevaluated on every spin.
"""
import copy
from .cycle_release import ReleaseContext
from .operational_cycle import finite
from .post_spray_review_core import PostSprayReviewCore
from .actuator_freshness import fresh_off
from .pulse_sequence import RECOVERABLE_PERMISSION_GAPS


class OperationalReview:
    def __init__(self,*,settle_s=1.,confirm_s=1.,ir_confirm_samples=5):
        self.config=dict(settle_s=settle_s,confirm_s=confirm_s,max_pulses=1,ir_confirm_samples=ir_confirm_samples)
        self.core=PostSprayReviewCore(**self.config)
        self.inputs={};self.context=None;self.committed=False
        self.terminal=None;self.retained=False;self.last_now=None;self.fault=None
        self.round_index=0

    def update(self,name,value,now):
        if name not in ('actor','vision','ir','alignment','mission') or not finite(now):
            raise ValueError('REVIEW_INPUT_INVALID')
        self.inputs[name]=(copy.deepcopy(value),now)

    def _actor(self,actor):
        ack=actor.get('cycle_release_ack') if isinstance(actor,dict) else None
        try:
            if (not isinstance(ack,dict) or ack.get('protocol')!='CYCLE_RELEASE_V1'
                    or ack.get('role')!='actor'):raise ValueError('WAIT_ACTOR_CYCLE_ACK')
            ctx=ReleaseContext.parse(ack.get('context'))
            if self.context is None:self.context=ctx
            elif ctx!=self.context:
                if not self.committed or ack.get('phase')!='ACTIVE':raise ValueError('REVIEW_CYCLE_CHANGED_BEFORE_COMMIT')
                self.context=ctx;self.committed=False;self.terminal=None;self.round_index=0
                self.core=PostSprayReviewCore(**self.config)
            sequence=actor.get('spray_sequence')
            if isinstance(sequence,dict) and actor.get('request_used') is True:
                index=sequence.get('pulse_index')
                duration=sequence.get('pulse_s',5)
                unlimited=sequence.get('quota_policy')=='CONSECUTIVE_AFTER_REPOSITION'
                if (type(index) is not int or index<1 or not unlimited and index>3 or index<self.round_index
                        or index>self.round_index+1 or duration not in (5,6)
                        or sequence.get('reserved_on_s')!=index*duration):
                    raise ValueError('WAIT_ACTUAL_PULSE_SEQUENCE')
                if index>self.round_index:
                    self.round_index=index;self.terminal=None
                    self.core=PostSprayReviewCore(**self.config)
            pulse=actor.get('pulse_core')
            committed=(ack.get('phase')=='COMMITTED' and ack.get('blockers')==[]
                       and ack.get('reset_completed') is True)
            if committed:self.committed=True
            if (committed and pulse is None and self.terminal is not None):
                self.retained=True
                return copy.deepcopy(self.terminal)
            self.retained=False
            if (not isinstance(pulse,dict) or pulse.get('schema_version')!=1
                    or pulse.get('mode')!='SUPERVISED_DEMO' or pulse.get('one_shot') is not True
                    or pulse.get('goal_uuid')!=ctx.nav_goal_uuid
                    or actor.get('request_used') is not True
                    or actor.get('goal_uuid')!=ctx.nav_goal_uuid
                    or ctx.nav_goal_uuid is None and (actor.get('cycle_id')!=ctx.cycle_id
                        or pulse.get('cycle_id')!=ctx.cycle_id or pulse.get('r5_cycle_context')!=ctx.as_dict())):raise ValueError('WAIT_ACTUAL_PULSE_CORE_REPORT')
            if (pulse.get('state')=='COMPLETE' and pulse.get('pump_on') is False
                    and pulse.get('stop_reason')=='PULSE_TIME_LIMIT'):
                self.terminal=copy.deepcopy(pulse)
            return copy.deepcopy(pulse)
        except (ValueError,TypeError,AttributeError) as exc:
            # Missing initial ACK is an expected WAIT; identity changes latch.
            if str(exc)=='REVIEW_CYCLE_CHANGED_BEFORE_COMMIT':self.fault=str(exc)
            return None

    def step(self,now,*,graph_ok):
        if not finite(now) or self.last_now is not None and now<self.last_now:
            self.fault='REVIEW_CLOCK_DISCONTINUITY'
        if self.last_now is not None and finite(now) and now-self.last_now>.35:
            self.core.candidate_since=None;self.core.candidate=None
        self.last_now=now
        if self.fault or graph_ok is not True:
            self.core.update('pump',None,now)
        else:
            actor,at=self.inputs.get('actor',(None,None))
            pulse=self._actor(actor) if at is not None and 0<=now-at<=.35 else None
            self.core.update('pump',pulse,now)
        for name in ('vision','ir','alignment','mission'):
            value,at=self.inputs.get(name,(None,None))
            limit=.5 if name=='ir' else .35
            value=copy.deepcopy(value) if isinstance(value,dict) and at is not None and 0<=now-at<=limit else None
            if value is not None:
                if name in ('vision','alignment'):
                    source=value.get('source');age=source.get('age_ms') if isinstance(source,dict) else None
                    if not finite(age) or age<0:value=None
                    else:source['age_ms']=age+(now-at)*1000
                if name=='ir':
                    age=value.get('ir_receive_age_ms')
                    if not finite(age) or age<0:value=None
                    else:value['ir_receive_age_ms']=age+(now-at)*1000
            self.core.update(name,value,now)
        report=self.core.step(now)
        actor,at=self.inputs.get('actor',(None,None))
        if graph_ok and not self.fault and isinstance(actor,dict) and finite(at) and 0<=now-at<=.35:
            sequence=actor.get('spray_sequence');sequence=sequence if isinstance(sequence,dict) else {}
            history=sequence.get('history');history=history if isinstance(history,(list,tuple)) else []
            last=history[-1] if history and isinstance(history[-1],dict) else {}
            actuator=actor.get('actuator');actuator=actuator if isinstance(actuator,dict) else {}
            home=actuator.get('home_command_completed_count')
            candidate,errors=self.core._evidence(now)
            # A failed ON is never relabeled COMPLETE. This is a fresh,
            # subscriber-only positioning decision after verified OFF/home.
            # IR still checks the source/context, fresh actuator and handoff.
            if (isinstance(pulse,dict) and pulse.get('state')=='ABORTED' and pulse.get('pump_on') is False
                    and self.context and actor.get('r5_cycle_context')==self.context.as_dict()
                    and (actor.get('cycle_release_ack') or {}).get('phase')=='ACTIVE'
                    and actor.get('state')=='RETRY_WAIT' and sequence.get('stage')=='REVIEW'
                    and sequence.get('recoverable_permission_gap') is True
                    and sequence.get('cancelled') is False and sequence.get('can_approach') is True
                    and last.get('reason') in RECOVERABLE_PERMISSION_GAPS
                    and last.get('pump_on_feedback_seen') is True
                    and type(last.get('home_before')) is int and type(home) is int and home>last['home_before']
                    and actor.get('proposed_pump_on') is False and actor.get('pump_publisher_registered') is False
                    and fresh_off(actuator,at,now) and candidate=='APPROACH_CANDIDATE' and not errors):
                report.update(state='APPROACH_REQUIRED',candidate=candidate,blockers=[],
                    pulse_index=self.round_index,recovery_after_interrupted_pulse=True)
        report.update(scope='R5_CANONICAL_REVIEW_FROM_ACTUAL_PULSE_CORE',
            cycle_context=self.context.as_dict() if self.context else None,
            historical_pulse_retained=self.retained,fault=self.fault)
        if self.round_index and report['pulse_index']:
            report['pulse_index']=self.round_index
        if self.fault:
            report['blockers']=sorted(set(report['blockers']+[self.fault]))
        return report
