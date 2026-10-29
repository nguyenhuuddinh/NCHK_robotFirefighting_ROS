"""Independent stationary I confirmation. No synthetic live-IR completion/motion.

The actor collects five distinct fresh preview sequences AFTER explicit I.
The real PumpDemoCore still checks every input and operator P before any ON.
"""
from .no_goal_cycle import context_of,same_cycle,stopped_pair
from .cycle_release import ReleaseContext
import copy
import math
from fire_robot_mission.ir_approach_shadow_core import validated_ir_band
from .close_stationary_policy import POLICY, allowed_bands, respray_context

from .operational_contract import STATIONARY_REQUEST_TOPIC as TOPIC
MODE='STATIONARY_IR_CONFIRMATION'

# An unused R5 confirmation may be revoked while its OFF-only endpoint is
# draining. These failures discard evidence, not the surrounding fire cycle.
# STOP, foreign UUID/token/owner and sequence regression are not retry grants.
RESTARTABLE_ABORTS=frozenset((
    'MISSION_NOT_SAFE_AND_CENTERED','LIVE_ALIGNMENT_NOT_COMPLETE',
    'CANCEL_GUARD_PROOF_INVALID','GATEWAY_HANDOFF_PROOF_INVALID',
    'IR_NOT_CONFIRMED_SPRAY_RANGE','STATIONARY_IR_RANGE_NOT_FRESH',
    'STATIONARY_IR_BAND_CHANGED','STATIONARY_IR_OWNER_NOT_IDLE',
    'STATIONARY_ACTUATOR_STALE','STATIONARY_RAW_OWNER_PRESENT',
    'STATIONARY_PUMP_BEFORE_CONFIRMATION','STATIONARY_NO_FALSE_BEFORE_I',
    'STATIONARY_CONFIRMATION_DEADLINE_2S'))


def number(v):return type(v) in (int,float) and math.isfinite(v)
def uid_ok(v):return isinstance(v,str) and len(v)==32 and v!='0'*32 and all(c in '0123456789abcdef' for c in v)


def common_errors(session,now,counts,*,require_idle):
    errors=[]
    live=session.get('ir_live',now);ir=session.get('ir',now)
    request=session.get('stationary_request',now)
    context=context_of(session)
    no_goal=(context is not None and isinstance(request,dict) and request.get('cycle_context')==context
             and request.get('goal_uuid') is None)
    reentry=request.get('ir_reentry') if isinstance(request,dict) else None
    bound_context=getattr(session,'r5_cycle_context',None)
    try:bound_context=ReleaseContext.parse(bound_context).as_dict()
    except (ValueError,TypeError):bound_context=None
    released_reentry=(isinstance(reentry,dict) and set(reentry)=={'step_count','total_progress_m','budget_started_at'}
        and bound_context is not None and request.get('cycle_context')==bound_context
        and request.get('goal_uuid')==bound_context['nav_goal_uuid']
        and isinstance(live,dict) and live.get('r5_cycle_context')==bound_context
        and live.get('goal_uuid')==bound_context['nav_goal_uuid']
        # Real-goal IR nodes retain their UUID; cycle_id is their no-goal
        # discriminator only. Both paths still require the full R5 context.
        and (bound_context['nav_goal_uuid'] is not None
             or live.get('cycle_id')==bound_context['cycle_id'])
        and type(reentry.get('step_count')) is int and reentry['step_count']>0
        and live.get('step_count')==reentry['step_count']
        and number(reentry.get('total_progress_m')) and 0<=reentry['total_progress_m'] and (session.unbounded_approach or reentry['total_progress_m']<=.30+1e-6)
        and live.get('total_progress_m')==reentry['total_progress_m']
        and number(reentry.get('budget_started_at')) and 0<=now-reentry['budget_started_at']
        and (session.unbounded_approach or now-reentry['budget_started_at']<=20.
             or respray_context(session,now)))
    bands=allowed_bands(session,now)
    idle_blockers=('OPERATOR_ARM_MISSING_OR_STALE','EXACT_HANDOFF_PROOF_INVALID','IR_BAND_HOLD')
    if 'TOO_CLOSE' in bands:idle_blockers+=('REVERSE_JOG_DISABLED',)
    # The IR motion node must remain idle after our own OFF-only pump endpoint
    # is registered. Its refusal to move with that endpoint is expected.
    if session.stationary.state=='COMPLETE' and counts.get('pump_total')==counts.get('pump')==1:
        idle_blockers+=('PUMP_OWNER_PRESENT',)
    a=session.actuator(now)
    if counts.get('stationary_request')!=1:errors.append('STATIONARY_REQUEST_OWNER_INVALID')
    if counts.get('ir_live')!=1:errors.append('IR_LIVE_OWNER_INVALID')
    for key in ('actuator','vision','preview'):
        if counts.get(key)!=1:errors.append('STATIONARY_'+key.upper()+'_OWNER_INVALID')
    if not (isinstance(request,dict) and type(request.get('schema_version')) is int and request.get('schema_version')==1
            and request.get('active') is True and uid_ok(request.get('request_id'))
            and (uid_ok(request.get('goal_uuid')) or no_goal)):errors.append('STATIONARY_I_HEARTBEAT_INVALID')
    if not (isinstance(live,dict) and live.get('schema_version')==1
            and live.get('state')=='WAITING_FOR_ADMISSION'
            and live.get('enabled') is True and live.get('integrated_handoff') is True
            and live.get('continuous_approach') is True and live.get('publisher_owned') is False
            and live.get('one_shot') is True
            and (released_reentry or reentry is None and live.get('step_count')==0 and live.get('total_progress_m')==0)
            and live.get('stop_reason')==''
            and (live.get('goal_uuid') is None or released_reentry)
            and live.get('recovery_required') is False
            and isinstance(live.get('blockers'),list) and all(isinstance(x,str) and x in
                idle_blockers
                for x in live['blockers'])
            and (live.get('safety') or {}).get('pump_command_publisher') is False
            and live.get('command')=={'linear_x_m_s':0.,'angular_z_rad_s':0.}):
        errors.append('STATIONARY_IR_OWNER_NOT_IDLE')
    if not (isinstance(ir,dict) and validated_ir_band(ir) in bands
            and ir.get('zone')==ir.get('ir_band') and ir.get('blockers')==[]
            and type(ir.get('ir_sample_seq')) is int and ir['ir_sample_seq']>0
            and type(ir.get('stable_samples')) is int and ir['stable_samples']>=session.ir_confirm_samples
            and number(ir.get('ir_receive_age_ms'))
            and 0<=ir['ir_receive_age_ms']+(now-session.inputs['ir'][1])*1000<=500 + 1e-6):
        errors.append('STATIONARY_IR_RANGE_NOT_FRESH')
    if session.stationary.band is not None and (ir or {}).get('ir_band')!=session.stationary.band and not respray_context(session,now):
        errors.append('STATIONARY_IR_BAND_CHANGED')
    parts=[session.get(k,now) for k in ('alignment','guard','gateway')]
    align,guard,gw=[x if isinstance(x,dict) else {} for x in parts]
    uid=align.get('goal_uuid')
    if not ((uid_ok(uid) and uid==guard.get('goal_uuid')==gw.get('cycle_goal_uuid')
            and isinstance(request,dict) and uid==request.get('goal_uuid'))
            or no_goal and all(same_cycle(v,context) for v in (align,guard,gw))
            and align.get('cycle_id')==context['cycle_id'] and stopped_pair(gw,guard,context)):
        errors.append('STATIONARY_UUID_MISMATCH')
    if counts.get('raw')!=0:errors.append('STATIONARY_RAW_OWNER_PRESENT')
    if any(type(counts.get(k)) is not int for k in ('raw','stationary_request','ir_live','pump_total','pump')):
        errors.append('STATIONARY_COUNT_EVIDENCE_INVALID')
    if counts.get('pump_total') not in (0,1) or (counts.get('pump_total')==1 and counts.get('pump')!=1):
        errors.append('STATIONARY_FOREIGN_PUMP_OWNER')
    if a is None:errors.append('STATIONARY_ACTUATOR_STALE')
    elif require_idle and not session.idle(a):errors.append('STATIONARY_ACTUATOR_NOT_OFF')
    return errors


class StationaryConfirmation:
    def __init__(self, *, required_samples=5):
        if type(required_samples) is not int or required_samples not in (3,5):raise ValueError('IR_CONFIRM_SAMPLES_INVALID')
        self.required_samples=required_samples
        self.state='WAIT_I';self.false_seen=False;self.token=None;self.goal=None
        self.false_at=None
        self.started=None;self.baseline_seq=None;self.samples=[];self.proof=None;self.reason=''
        self.band=None;self.range_policy=None;self.cycle_context=None
        self.restart_count=0;self.last_abort_reason=None

    def restartable(self):
        return self.state=='ABORTED' and self.reason in RESTARTABLE_ABORTS

    def _restart_after_withdrawal(self,session,request,now,counts,stop):
        from .pump_demo_core import zero_twist
        try:context=ReleaseContext.parse(getattr(session,'r5_cycle_context',None)).as_dict()
        except (ValueError,TypeError):return False
        # The full admission will run again on ACTIVE. First require an
        # authenticated false edge, actual OFF/STOP and the old endpoint gone.
        return (self.restartable() and not stop and not session.used
            and counts.get('stationary_request')==counts.get('actuator')==1
            and all(type(counts.get(k)) is int and counts[k]==0 for k in ('raw','pump','pump_total'))
            and (self.token is None or request.get('request_id')==self.token)
            and self.goal==context['nav_goal_uuid']
            and self.cycle_context in (None,context)
            and session.idle(session.actuator(now))
            and zero_twist(session.get('cmd',now),.001)
            and zero_twist(session.get('odom',now),.02))

    def step(self,session,now,counts,*,stop=False):
        request=session.get('stationary_request',now)
        if isinstance(request,dict) and request.get('active') is False:
            if self._restart_after_withdrawal(session,request,now,counts,stop):
                reason=self.reason;count=self.restart_count+1
                self.__init__(required_samples=self.required_samples)
                self.restart_count=count;self.last_abort_reason=reason
            if (getattr(session,'r5_cycle_context',None) is not None and not session.used
                    and self.state in ('CONFIRMING','COMPLETE')):
                # Recenter revokes the old proof. A new window is required
                # after the same bounded I task regains centered STOP.
                self.__init__(required_samples=self.required_samples)
            self.false_seen=True
            self.false_at=session.inputs['stationary_request'][1]
        active=isinstance(request,dict) and request.get('active') is True
        if self.state in ('COMPLETE','ABORTED'):return
        if stop:
            self.state='ABORTED';self.reason='STATIONARY_OPERATOR_STOP';return
        if self.state=='WAIT_I':
            if not active:return
            if not self.false_seen or not number(self.false_at) or not 0<=now-self.false_at<=.35:
                self.state='ABORTED';self.reason='STATIONARY_NO_FALSE_BEFORE_I';return
            self.token=request.get('request_id');self.goal=request.get('goal_uuid')
            self.cycle_context=copy.deepcopy(request.get('cycle_context'))
            self.started=now
            ir=session.get('ir',now) or {};self.baseline_seq=ir.get('ir_sample_seq')
            self.band=ir.get('ir_band');self.range_policy=request.get('range_policy')
            self.state='CONFIRMING'
        errors=common_errors(session,now,counts,require_idle=True)
        # Use the baseline checker, allowing only its own absent pump endpoint
        # and absent P at this confirmation step. ON never omits those checks.
        errors+=list(set(session.pump.blockers(now,counts))-
                     {'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'})
        if counts.get('pump_total')!=0:errors.append('STATIONARY_PUMP_BEFORE_CONFIRMATION')
        if (not active or request.get('request_id')!=self.token or request.get('goal_uuid')!=self.goal
                or request.get('cycle_context')!=self.cycle_context):
            errors.append('STATIONARY_I_CHANGED_OR_LOST')
        # R5 has one bounded I task. A transient stale/range/centering report
        # invalidates this sample window, not the whole unused actuator cycle.
        # Every new window still passes the full baseline checker; P is separate.
        r5_wait=(getattr(session,'r5_cycle_context',None) is not None and not session.used
            and active and request.get('request_id')==self.token
            and request.get('goal_uuid')==self.goal and request.get('cycle_context')==self.cycle_context
            and counts.get('pump_total')==0 and not stop and now>=self.started)
        permanent={'STATIONARY_I_CHANGED_OR_LOST','STATIONARY_REQUEST_OWNER_INVALID',
            'STATIONARY_FOREIGN_PUMP_OWNER','STATIONARY_COUNT_EVIDENCE_INVALID',
            'STATIONARY_UUID_MISMATCH','STATIONARY_ACTUATOR_NOT_OFF'}
        if r5_wait and (errors or now-self.started>=2) and not permanent.intersection(errors):
            self.samples=[];self.proof=None;self.started=now
            ir=session.get('ir',now) or {};self.baseline_seq=ir.get('ir_sample_seq')
            self.band=ir.get('ir_band')
            self.reason='WAIT_NEW_WINDOW:'+(','.join(sorted(set(errors))) if errors else 'FIVE_NEW_IR_SAMPLES')
            return
        if errors:
            self.state='ABORTED';self.reason=sorted(errors)[0];return
        if now-self.started>=2 or now<self.started:
            self.state='ABORTED';self.reason='STATIONARY_CONFIRMATION_DEADLINE_2S';return
        ir=session.get('ir',now);seq=ir['ir_sample_seq'];received=session.inputs['ir'][1]
        previous=self.samples[-1]['seq'] if self.samples else self.baseline_seq
        if type(previous) is not int:
            self.baseline_seq=seq;self.samples=[];self.started=now;return
        if seq<previous:
            self.state='ABORTED';self.reason='STATIONARY_IR_SEQUENCE_REGRESSED';return
        # Five new sequences, not five repeated checks of one old sample.
        if seq>previous and received>self.started:
            snapshot={k:copy.deepcopy(session.inputs[k]) for k in
                      ('ir','ir_live','mission','alignment','gateway','guard','actuator','odom','cmd','status','stationary_request')}
            self.samples.append(dict(seq=seq,at=now,inputs=snapshot,counts=copy.deepcopy(counts)))
        if len(self.samples)>=self.required_samples and now-self.started>=.4:
            self.state='COMPLETE';self.reason=('THREE' if self.required_samples==3 else 'FIVE')+'_FRESH_IR_SAMPLES_CENTERED_STOP'
            self.proof=dict(mode=MODE,goal_uuid=self.goal,request_id=self.token,cycle_context=self.cycle_context,
                started_at=self.started,completed_at=now,baseline_seq=self.baseline_seq,
                samples=copy.deepcopy(self.samples),motion_performed=False)
            self.proof.update(ir_band=self.band,range_policy=self.range_policy,required_samples=self.required_samples,unbounded_approach=session.unbounded_approach)

    def report(self):
        return dict(mode=MODE,state=self.state,reason=self.reason,sample_count=len(self.samples),goal_uuid=self.goal,
                    request_id=self.token,cycle_context=self.cycle_context,proof=self.proof,ir_band=self.band,range_policy=self.range_policy,
                    restartable=self.restartable(),restart_count=self.restart_count,last_abort_reason=self.last_abort_reason)


def proof_errors(proof):
    """Validate stored evidence using real baseline admission; no command output."""
    from .pulse_session import PulseSession
    errors=[]
    if not isinstance(proof,dict):return ['STATIONARY_PROOF_MISSING']
    required=proof.get('required_samples',5)
    if type(required) is not int or required not in (3,5):return ['STATIONARY_PROOF_SAMPLE_LIMIT_INVALID']
    context=proof.get('cycle_context');no_goal=False
    if context is not None:
        try:no_goal=ReleaseContext.parse(context).nav_goal_uuid is None and proof.get('goal_uuid') is None
        except (ValueError,TypeError):return ['STATIONARY_PROOF_CONTEXT_INVALID']
    start=proof.get('started_at');end=proof.get('completed_at');samples=proof.get('samples')
    if (proof.get('mode')!=MODE or proof.get('motion_performed') is not False
            or not (uid_ok(proof.get('goal_uuid')) or no_goal) or not uid_ok(proof.get('request_id'))
            or not number(start) or not number(end) or not .4<=end-start<2
            or type(proof.get('baseline_seq')) is not int or proof['baseline_seq']<=0
            or not isinstance(samples,list) or len(samples)!=required):return ['STATIONARY_PROOF_HEADER_INVALID']
    previous=proof['baseline_seq'];prev_time=start
    for row in samples:
        try:
            at=row['at'];seq=row['seq'];inputs=row['inputs'];counts=row['counts']
            if (not number(at) or not prev_time<at<=end or type(seq) is not int or seq<=previous
                    or not start<inputs['ir'][1]<=at):raise ValueError()
            session=PulseSession(enabled=True,allow_provisional_ir=True,
                allow_close_stationary=proof.get('range_policy')==POLICY,ir_confirm_samples=required,
                unbounded_approach=proof.get('unbounded_approach') is True)
            session.r5_cycle_context=context;session.pump.r5_cycle_context=context
            for name,(value,received) in inputs.items():
                # JSON arrays round-trip tuple statuses into lists. Restore the
                # ROS adapter's tuple representation before the baseline check.
                if name=='status':value=[tuple(item) for item in value]
                session.update(name,value,received)
            session.feed_pump(at)
            session.stationary.state='CONFIRMING';session.stationary.token=proof['request_id']
            session.stationary.cycle_context=context
            session.stationary.goal=proof['goal_uuid'];session.stationary.band=proof.get('ir_band','SPRAY_RANGE')
            request=session.get('stationary_request',at) or {};ir=session.get('ir',at) or {}
            if request.get('cycle_context')!=context:raise ValueError()
            if request.get('range_policy')!=proof.get('range_policy'):raise ValueError()
            if request.get('goal_uuid')!=proof['goal_uuid'] or request.get('request_id')!=proof['request_id'] or ir.get('ir_sample_seq')!=seq:raise ValueError()
            blockers=common_errors(session,at,counts,require_idle=True)
            blockers+=list(set(session.pump.blockers(at,counts))-
                           {'PUMP_OWNER_CONFLICT','OPERATOR_ARM_MISSING_OR_STALE'})
            if counts.get('pump_total')!=0 or blockers:raise ValueError()
            previous=seq;prev_time=at
        except (KeyError,TypeError,ValueError,IndexError,AttributeError,OverflowError):
            errors.append('STATIONARY_SAMPLE_ADMISSION_INVALID');break
    if not isinstance(samples[-1],dict) or samples[-1].get('at')!=end:errors.append('STATIONARY_COMPLETION_TIME_MISMATCH')
    return errors
