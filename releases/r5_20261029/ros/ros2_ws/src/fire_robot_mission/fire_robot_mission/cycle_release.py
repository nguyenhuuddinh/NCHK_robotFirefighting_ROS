"""Cycle retirement: STOP/OFF/release -> PREPARED -> COMMIT -> peer resets.

No command, action, publisher, or reset is hidden in the coordinator. Each
participant must execute its own local reset before it can ACK COMMITTED.
Graph/source checks are supplied by the ROS boundary, not by operator keys.
"""
import copy
from dataclasses import dataclass
import uuid

from .operational_cycle import finite, valid_uuid, NavigationSnapshot, zero
from .operational_contract import FIRMWARE_ELF_SHA256
from .operational_identity import identity_current, identity_key
from .actuator_freshness import fresh_off

CONTROL_TOPIC='/fire_mission/cycle_control'
ROLES=('gateway','guard','alignment','ir_live','actor','mission')


def interrupted_cancel_upgrade(previous, current):
    """Allow only an explicit cancel of an already interrupted pulse.

    Context, token, real home baseline and every STOP/reset obligation stay
    fixed. Cancellation removes the clear wait; it never admits actuation.
    """
    return (previous is not None
        and previous == (True, previous[1], True, True, False, False, False)
        and current == (True, previous[1], False, True, True, False, False))


@dataclass(frozen=True)
class ReleaseContext:
    cycle_id:str
    nav_goal_uuid:str | None
    bridge_instance_id:str
    connection_epoch:int
    elf_sha256:str=FIRMWARE_ELF_SHA256

    @classmethod
    def parse(cls,value):
        if not isinstance(value,dict) or set(value)!={'cycle_id','nav_goal_uuid','bridge_instance_id','connection_epoch','elf_sha256'}:
            raise ValueError('RELEASE_CONTEXT_INVALID')
        if (not valid_uuid(value['cycle_id']) or not valid_uuid(value['bridge_instance_id'])
                or value['nav_goal_uuid'] is not None and not valid_uuid(value['nav_goal_uuid'])
                or value['cycle_id']==value['nav_goal_uuid'] or type(value['connection_epoch']) is not int
                or value['connection_epoch']<1 or value['elf_sha256']!=FIRMWARE_ELF_SHA256):
            raise ValueError('RELEASE_CONTEXT_INVALID')
        return cls(**value)

    def as_dict(self):return dict(self.__dict__)
    @property
    def identity(self):return self.bridge_instance_id,self.connection_epoch,self.elf_sha256


class ReleaseInputs:
    def __init__(self):self.inputs={};self.cancel_seen=set();self.terminal_seen=set()
    def update(self,name,value,at):
        if not finite(at):raise ValueError('RELEASE_TIME_INVALID')
        self.inputs[name]=(copy.deepcopy(value),at)
    def get(self,name,now,limit=.35):
        value,at=self.inputs.get(name,(None,None))
        if name=='ir' and limit==.35:limit=.5
        return value if at is not None and 0<=now-at<=limit else None

    def stop_errors(self,ctx,now,graph_ok,raw_count,pump_count,*,after_pulse=False,home_before=None,terminal_goal_release=False):
        errors=[]
        if graph_ok is not True:errors.append('WAIT_EXACT_GRAPH')
        if raw_count!=0:errors.append('WAIT_RAW_RELEASE')
        if pump_count!=0:errors.append('WAIT_PUMP_RELEASE')
        if not zero(self.get('cmd',now)):errors.append('WAIT_FRESH_CMD_ZERO')
        if not zero(self.get('odom',now),.02,.05):errors.append('WAIT_FRESH_STATIONARY')
        a,at=self.inputs.get('actuator',(None,None))
        if (not finite(at) or not isinstance(a,dict) or not fresh_off(a,at,now)
                or a.get('schema_version')!=1 or a.get('mode')!='COMMAND_TELEMETRY_ONLY'
                or a.get('pump_pwm_ready') is not True):errors.append('WAIT_FRESH_ACTUATOR_OFF')
        if after_pulse and (type(home_before) is not int or home_before<0 or not isinstance(a,dict)
                or type(a.get('home_command_completed_count')) is not int
                or a['home_command_completed_count']<=home_before):errors.append('WAIT_POST_PULSE_HOME')
        identity,received_identity=self.inputs.get('identity',(None,None))
        if not identity_current(identity,received_identity,now) or identity_key(identity)!=ctx.identity:
            errors.append('WAIT_SAME_BOOT_EPOCH')
        # Event-driven status is retained only while the adapter verifies its
        # pinned source. It is not sufficient alone for no-goal STOP proof.
        try:
            rows,received=self.inputs.get('status',(None,None))
            status=NavigationSnapshot.parse(rows,received)
            if received>now or status.active_uuid is not None:errors.append('WAIT_NO_ACTIVE_GOAL')
            if ctx.nav_goal_uuid is not None:
                matches=[s for u,s in status.rows if u==ctx.nav_goal_uuid]
                stamp=(ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity)
                if (graph_ok is True and status.active_uuid is None and matches in ([4],[5],[6])
                        and identity_current(identity,received_identity,now)
                        and identity_key(identity)==ctx.identity):
                    self.terminal_seen.add(stamp)
                terminal_ok=(terminal_goal_release is True and not after_pulse
                    and stamp in self.terminal_seen and status.active_uuid is None
                    and matches in ([],[4],[5],[6]))
                guard=self.get('guard',now)
                guard=guard if isinstance(guard,dict) else {}
                exact=(guard.get('goal_uuid')==ctx.nav_goal_uuid and guard.get('state')=='CANCEL_CONFIRMED_STOPPED'
                       and guard.get('cancel_service_accepted') is True and guard.get('cancel_status_seen') is True
                       and guard.get('goal_status')==5 and guard.get('send_count')==1 and guard.get('blockers')==[])
                if (graph_ok is True and exact and matches in ([],[5])
                        and identity_current(identity,received_identity,now) and identity_key(identity)==ctx.identity):
                    self.cancel_seen.add((ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity))
                # Target loss can invalidate alignment or IR motion handoff;
                # it does not undo an already observed exact Nav2 cancel.
                # A downstream task failure cannot revoke a real cancel.
                # Retirement still needs its exact historical receipt and
                # every independent current STOP/OFF/identity/source gate.
                downstream_stop = (guard.get('state')=='FAILED'
                    and guard.get('goal_uuid')==ctx.nav_goal_uuid
                    and guard.get('goal_status')==5
                    and guard.get('cancel_service_accepted') is True
                    and guard.get('cancel_status_seen') is True
                    and guard.get('send_count')==1
                    and (ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity) in self.cancel_seen)
                if not terminal_ok and (matches not in ([],[5]) or (ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity) not in self.cancel_seen
                        or guard.get('state')=='FAILED' and not downstream_stop):
                    errors.append('WAIT_REAL_GOAL_CANCELED')
        except (ValueError,TypeError):errors.append('WAIT_VALID_NAV_STATUS')
        return errors


class CycleReleaseParticipant(ReleaseInputs):
    def __init__(self,role):
        super().__init__()
        if role not in ROLES:raise ValueError('RELEASE_ROLE_INVALID')
        self.role=role;self.context=None;self.token=None;self.request=None;self.request_at=None
        self.phase='IDLE';self.since=None;self.last_now=None;self.retired=set();self.error=None
        self.prepared_token=None;self.policy=None

    def receive(self,message,now):
        try:
            if message.get('protocol')!='CYCLE_RELEASE_V1' or not finite(now):
                raise ValueError('RELEASE_REQUEST_INVALID')
            ctx=ReleaseContext.parse(message.get('context'))
            phase=message.get('phase');token=message.get('request_id')
            if phase not in ('ACTIVE','PREPARE','COMMIT') or phase!='ACTIVE' and not valid_uuid(token):
                raise ValueError('RELEASE_REQUEST_INVALID')
            if ctx.cycle_id in self.retired:
                # A repeated COMMIT may ACK the same already-committed token,
                # but must never re-run reset or bind another request token.
                if not (self.phase=='COMMITTED' and ctx==self.context and token==self.token and phase=='COMMIT'):
                    raise ValueError('RETIRED_CYCLE_REPLAY')
            elif self.context is not None and ctx!=self.context and self.phase!='COMMITTED':
                raise ValueError('RELEASE_CONTEXT_CHANGED')
            if self.context is None or ctx!=self.context:
                if phase!='ACTIVE':raise ValueError('WAIT_ACTIVE_CYCLE_REGISTRATION')
                self.context=ctx;self.token=None;self.phase='ACTIVE';self.since=None
                self.prepared_token=None;self.policy=None
            if self.token is not None and phase=='ACTIVE' and self.phase!='COMMITTED':
                raise ValueError('RELEASE_DOWNGRADE_TO_ACTIVE')
            if self.token is not None and token!=self.token and phase!='ACTIVE':
                raise ValueError('RELEASE_TOKEN_CHANGED')
            if phase in ('PREPARE','COMMIT'):
                after=message.get('after_pulse');home=message.get('home_before')
                if type(after) is not bool or after and (type(home) is not int or home<0):
                    raise ValueError('RELEASE_POLICY_INVALID')
                if not after and home is not None:raise ValueError('RELEASE_POLICY_INVALID')
                clear=message.get('require_clear',True)
                interrupted=message.get('pulse_interrupted',False)
                cancel=message.get('operator_cancel',False)
                if type(cancel) is not bool or cancel and (not after or not interrupted):
                    raise ValueError('RELEASE_POLICY_INVALID')
                completed=message.get('completed_sequence_release',False)
                if type(completed) is not bool or completed and (not after or interrupted):raise ValueError('RELEASE_POLICY_INVALID')
                if type(clear) is not bool or after and not clear and not (cancel or completed or interrupted):raise ValueError('RELEASE_POLICY_INVALID')
                if type(interrupted) is not bool or interrupted and not after:
                    raise ValueError('RELEASE_POLICY_INVALID')
                terminal=message.get('terminal_goal_release',False)
                if type(terminal) is not bool or terminal and (after or ctx.nav_goal_uuid is None):
                    raise ValueError('RELEASE_POLICY_INVALID')
                policy=(after,home,clear,interrupted,cancel,terminal,completed)
                if (self.policy is not None and policy!=self.policy
                        and not interrupted_cancel_upgrade(self.policy,policy)):
                    raise ValueError('RELEASE_POLICY_CHANGED')
                self.policy=policy;self.token=token
            self.request=copy.deepcopy(message);self.request_at=now
            return True
        except (ValueError,TypeError,AttributeError) as exc:
            self.error=str(exc);return False

    @property
    def closing(self):return self.request is not None and self.request.get('phase') in ('PREPARE','COMMIT') and self.phase!='COMMITTED'

    def step(self,now,*,graph_ok,raw_count,pump_count,owns_raw=False,owns_pump=False,pending_cancel=False,reset=None):
        errors=[]
        if not finite(now) or self.last_now is not None and now<self.last_now:
            self.error='RELEASE_CLOCK_DISCONTINUITY'
        if self.last_now is not None and finite(now) and now-self.last_now>.35:self.since=None
        self.last_now=now
        if self.context is None:return self.report(['WAIT_ACTIVE_CYCLE_REGISTRATION'])
        if self.error:return self.report([self.error])
        if self.request_at is None or not 0<=now-self.request_at<=.35:
            self.since=None;return self.report(['WAIT_FRESH_CYCLE_CONTROL'])
        if self.phase=='COMMITTED':return self.report([])
        if not self.closing:
            # Remember a real cancel receipt while its guard is still in the
            # stopped state. Later IR/STOP withdrawal may change that report.
            # This records history only; retirement still needs fresh STOP.
            self.stop_errors(self.context,now,graph_ok,raw_count,pump_count)
            return self.report([])
        r=self.request
        errors=self.stop_errors(self.context,now,graph_ok,raw_count,pump_count,
            after_pulse=r.get('after_pulse') is True,home_before=r.get('home_before'),
            terminal_goal_release=r.get('terminal_goal_release') is True)
        if owns_raw or owns_pump:errors.append('WAIT_LOCAL_PUBLISHER_DESTROYED')
        if pending_cancel:errors.append('WAIT_CANCEL_RESPONSE_SETTLED')
        if errors:
            self.since=None;self.phase='PREPARING';return self.report(errors)
        if self.since is None:self.since=now
        if now-self.since<.5:return self.report(['WAIT_STOP_HOLD'])
        if r['phase']=='PREPARE':
            self.phase='PREPARED';self.prepared_token=self.token
        elif r['phase']=='COMMIT':
            if self.prepared_token!=self.token:return self.report(['COMMIT_WITHOUT_PREPARED'])
            if reset is None:return self.report(['WAIT_LOCAL_RESET'])
            try:reset(self.context)
            except Exception as exc:
                self.error='LOCAL_RESET_FAILED:'+type(exc).__name__
                return self.report([self.error])
            self.phase='COMMITTED';self.retired.add(self.context.cycle_id)
        return self.report([])

    def report(self,errors):
        return dict(protocol='CYCLE_RELEASE_V1',role=self.role,phase=self.phase,
            context=self.context.as_dict() if self.context else None,request_id=self.token,
            blockers=list(errors),reset_completed=self.phase=='COMMITTED')


@dataclass(frozen=True)
class ReleaseReceipt:
    context:ReleaseContext
    request_id:str
    completed_at:float
    roles:tuple


class CycleReleaseCoordinator(ReleaseInputs):
    def __init__(self,id_factory=None):
        super().__init__();self.factory=id_factory or (lambda:uuid.uuid4().hex)
        self.context=None;self.token=None;self.after_pulse=False;self.home_before=None
        self.phase='IDLE';self.since=None;self.last_seq=None;self.samples=0;self.require_clear=True
        self.last_now=None;self.receipt=None;self.blockers=[];self.fault=None;self.pulse_proven=False
        self.pulse_interrupted=False
        self.operator_cancel=False;self.terminal_goal_release=False;self.completed_sequence_release=False
        self.pulse_index=1

    def begin(self,context,now,*,after_pulse=False,home_before=None,require_clear=True,pulse_interrupted=False,operator_cancel=False,terminal_goal_release=False,completed_sequence_release=False):
        if self.context is not None:raise ValueError('RELEASE_ALREADY_ACTIVE')
        ctx=ReleaseContext.parse(context)
        if not finite(now):raise ValueError('RELEASE_TIME_INVALID')
        if type(after_pulse) is not bool:raise ValueError('RELEASE_POLICY_INVALID')
        if type(pulse_interrupted) is not bool or pulse_interrupted and not after_pulse:
            raise ValueError('RELEASE_POLICY_INVALID')
        if after_pulse and (type(home_before) is not int or home_before<0):raise ValueError('PULSE_HOME_BASELINE_REQUIRED')
        if not after_pulse and home_before is not None:raise ValueError('RELEASE_POLICY_INVALID')
        token=self.factory()
        if not valid_uuid(token):raise ValueError('RELEASE_TOKEN_INVALID')
        if type(operator_cancel) is not bool or operator_cancel and (not after_pulse or not pulse_interrupted):
            raise ValueError('RELEASE_POLICY_INVALID')
        if type(completed_sequence_release) is not bool or completed_sequence_release and (not after_pulse or pulse_interrupted):raise ValueError('RELEASE_POLICY_INVALID')
        if type(require_clear) is not bool or after_pulse and not require_clear and not (operator_cancel or completed_sequence_release or pulse_interrupted):raise ValueError('RELEASE_POLICY_INVALID')
        self.completed_sequence_release=completed_sequence_release
        if type(terminal_goal_release) is not bool or terminal_goal_release and (after_pulse or ctx.nav_goal_uuid is None):
            raise ValueError('RELEASE_POLICY_INVALID')
        self.terminal_goal_release=terminal_goal_release
        self.context=ctx;self.token=token
        self.after_pulse=after_pulse;self.home_before=home_before;self.require_clear=require_clear
        self.pulse_interrupted=pulse_interrupted
        self.operator_cancel=operator_cancel
        self.phase='PREPARE';self.last_now=now

    def control(self):
        if self.context is None:return None
        result=dict(protocol='CYCLE_RELEASE_V1',phase='COMMIT' if self.phase=='READY' else self.phase,
            context=self.context.as_dict(),request_id=self.token,after_pulse=self.after_pulse,home_before=self.home_before,require_clear=self.require_clear)
        if self.pulse_interrupted:result['pulse_interrupted']=True
        if self.operator_cancel:result['operator_cancel']=True
        if self.completed_sequence_release:result['completed_sequence_release']=True
        if self.terminal_goal_release:result['terminal_goal_release']=True
        return result

    def cancel_interrupted_release(self):
        """Apply the current operator's late X without changing release IDs."""
        if (self.context is None or self.phase not in ('PREPARE','COMMIT')
                or not self.after_pulse or not self.pulse_interrupted
                or self.completed_sequence_release or self.terminal_goal_release):
            return False
        if not self.require_clear:return True
        if not self.operator_cancel:
            self.operator_cancel=True;self.require_clear=False
            self.since=None;self.samples=0;self.last_seq=None
        return True

    def _clear(self,now):
        errors=[];v=self.get('vision',now);ir=self.get('ir',now)
        if not isinstance(v,dict) or v.get('schema_version')!=1 or v.get('observe_only') is not True or v.get('state')!='ok':
            errors.append('WAIT_FRESH_YOLO_CLEAR')
        else:
            fire=v.get('fire');source=v.get('source');age=source.get('age_ms') if isinstance(source,dict) else None
            received=self.inputs['vision'][1]
            if (not isinstance(fire,dict) or fire.get('detected_now') is not False or fire.get('confirmed') is not False
                    or not finite(age) or age<0 or age+(now-received)*1000>300 + 1e-6):errors.append('WAIT_FRESH_YOLO_CLEAR')
        if not isinstance(ir,dict):errors.append('WAIT_IR_CLEAR')
        else:
            counts=ir.get('filtered_counts');age=ir.get('ir_receive_age_ms');received=self.inputs['ir'][1]
            blockers=ir.get('blockers');safety=ir.get('safety')
            if (ir.get('schema_version')!=1 or ir.get('mode')!='OBSERVE_ONLY' or ir.get('observe_only') is not True
                    or ir.get('calibration')!='PROVISIONAL_SINGLE_FLAME' or ir.get('validated_for_actuation') is not False
                    or ir.get('ir_band')!='NO_FLAME_SIGNATURE'
                    or not isinstance(blockers,list) or not all(isinstance(b,str) for b in blockers)
                    or not set(blockers)<={'NO_IR_FLAME_SIGNATURE','FIRE_NOT_CONFIRMED','FIRE_NOT_CENTERED'}
                    or not isinstance(safety,dict) or safety.get('motion_command_publisher') is not False
                    or safety.get('actuator_command_publisher') is not False
                    or ir.get('range_policy')!='YOLO_RIGHT_ADC_20261025' or ir.get('vision_presence')!='CLEAR'
                    or ir.get('no_flame_requires_all_eyes') is not True
                    or type(ir.get('stable_samples')) is not int or ir['stable_samples']<5
                    or not isinstance(counts,dict) or not all(type(counts.get(k)) is int and 20000<counts[k]<=32767 for k in ('left','center','right'))
                    or not finite(age) or age<0 or age+(now-received)*1000>500 + 1e-6
                    or type(ir.get('ir_sample_seq')) is not int or ir['ir_sample_seq']<=0):errors.append('WAIT_IR_CLEAR')
        return errors

    def _after_pulse_errors(self,now):
        if not self.after_pulse:return []
        actor=self.get('actor',now);review=self.get('review',now)
        if not self.pulse_proven and isinstance(actor,dict):
            index=(actor.get('spray_sequence') or {}).get('pulse_index',1)
            consecutive=(actor.get('spray_sequence') or {}).get('quota_policy')=='CONSECUTIVE_AFTER_REPOSITION'
            if type(index) is not int or index<1 or not consecutive and index>3:return ['WAIT_ACTUAL_PULSE_SEQUENCE']
            self.pulse_index=index
        if self.pulse_interrupted:
            # An aborted request may have ended before ON/sweep telemetry was
            # sampled. Never call it a completed pulse or synthesize review.
            # Require its actual bound used-request and starting home count;
            # Independent STOP/OFF/new home always apply. Sensor clear is a
            # separate policy; R5 may return Nav2 while waiting for a new fire event.
            if not self.pulse_proven and (not isinstance(actor,dict)
                    or actor.get('state') not in (('ABORTED','DONE') if self.operator_cancel else ('ABORTED',)) or actor.get('request_used') is not True
                    or actor.get('proposed_pump_on') is not False
                    or actor.get('r5_cycle_context')!=self.context.as_dict()
                    or type(actor.get('start_home_count')) is not int
                    or actor.get('start_home_count')!=self.home_before
                    or actor.get('goal_uuid')!=self.context.nav_goal_uuid):
                return ['WAIT_INTERRUPTED_PULSE_EVIDENCE']
            return []
        if not self.pulse_proven and (not isinstance(actor,dict) or actor.get('request_used') is not True
                or actor.get('state')!='DONE' or actor.get('proposed_pump_on') is not False
                or actor.get('pump_on_feedback_seen') is not True or actor.get('servo_sweep_feedback_seen') is not True
                or actor.get('goal_uuid')!=self.context.nav_goal_uuid
                or self.context.nav_goal_uuid is None and actor.get('cycle_id')!=self.context.cycle_id):return ['WAIT_COMPLETED_PULSE_EVIDENCE']
        if self.completed_sequence_release:
            sequence=(actor or {}).get('spray_sequence') or {}
            # Completion is not a claim that the fire is out. The new policy
            # may return navigation with residual fire after three rounds.
            # Keep actual ON/sweep/DONE/context plus independent OFF/new-home
            # and all six STOP/reset proofs; never fabricate NO_FIRE_EVIDENCE.
            if (not self.pulse_proven and (sequence.get('release_without_clear') is not True
                    or sequence.get('quota_policy')!='CONSECUTIVE_AFTER_REPOSITION'
                    or sequence.get('stage')!='TERMINAL'
                    or sequence.get('reason') not in ('CONSECUTIVE_PULSE_LIMIT_RETURN_NAV2','TARGET_NOT_RECOVERED_RETURN_NAV2')
                    or not sequence.get('history'))):return ['WAIT_COMPLETED_SEQUENCE_EVIDENCE']
            return []
        if (not isinstance(review,dict) or review.get('schema_version')!=1 or review.get('mode')!='SHADOW_ONLY'
                or review.get('cycle_context')!=self.context.as_dict()
                or review.get('scope')!='R5_CANONICAL_REVIEW_FROM_ACTUAL_PULSE_CORE'
                or review.get('state')!='NO_FIRE_EVIDENCE' or review.get('goal_uuid')!=self.context.nav_goal_uuid
                or type(review.get('pulse_index')) is not int or review['pulse_index']!=self.pulse_index
                or review.get('blockers')!=[]):return ['WAIT_POSTSPRAY_REVIEW']
        return []

    def step(self,now,*,graph_ok,raw_count,pump_count):
        if self.context is None:return self.report()
        if not finite(now) or self.last_now is not None and now<self.last_now:
            self.fault='RELEASE_CLOCK_DISCONTINUITY'
        if self.last_now is not None and finite(now) and now-self.last_now>.35:
            self.since=None;self.samples=0;self.last_seq=None
        self.last_now=now
        if self.fault:
            self.blockers=[self.fault];self.receipt=None;return self.report()
        errors=self.stop_errors(self.context,now,graph_ok,raw_count,pump_count,
            after_pulse=self.after_pulse,home_before=self.home_before,terminal_goal_release=self.terminal_goal_release)+(self._clear(now) if self.require_clear else [])+self._after_pulse_errors(now)
        expected='PREPARED' if self.phase=='PREPARE' else 'COMMITTED'
        for role in ROLES:
            report=self.get(role,now);ack=report.get('cycle_release_ack') if isinstance(report,dict) else None
            if (not isinstance(ack,dict) or ack.get('protocol')!='CYCLE_RELEASE_V1' or ack.get('role')!=role
                    or ack.get('context')!=self.context.as_dict() or ack.get('request_id')!=self.token
                    or ack.get('phase')!=expected or ack.get('blockers')!=[]
                    or expected=='COMMITTED' and ack.get('reset_completed') is not True):
                errors.append('WAIT_'+role.upper()+'_'+expected)
        if errors:
            self.since=None;self.samples=0;self.last_seq=None;self.receipt=None;self.blockers=errors
            return self.report()
        self.blockers=[]
        seq=self.get('ir',now)['ir_sample_seq'] if self.require_clear else now
        if self.last_seq is not None and seq<self.last_seq:
            self.fault='IR_SEQUENCE_REGRESSION';return self.step(now,graph_ok=graph_ok,raw_count=raw_count,pump_count=pump_count)
        if self.since is None:self.since=now
        if seq!=self.last_seq:self.samples+=1;self.last_seq=seq
        if self.samples<5 or now-self.since<1.:
            self.blockers=['WAIT_CLEAR_HOLD_1S_5_NEW_IR' if self.require_clear else 'WAIT_STOP_OFF_RELEASE_1S'];self.receipt=None
            return self.report()
        if self.phase=='PREPARE':
            # Historical ON/sweep/completion is established before peers reset.
            # Fresh ACT OFF/home, clear, and review are still checked each spin.
            self.pulse_proven=self.after_pulse
            self.phase='COMMIT'
        elif self.phase in ('COMMIT','READY'):
            self.phase='READY'
            self.receipt=ReleaseReceipt(self.context,self.token,now,ROLES)
        return self.report()

    def report(self):
        return dict(protocol='CYCLE_RELEASE_V1',phase=self.phase,request_id=self.token,
            context=self.context.as_dict() if self.context else None,after_pulse=self.after_pulse,require_clear=self.require_clear,
            pulse_outcome=('INTERRUPTED' if self.pulse_interrupted else 'COMPLETED') if self.after_pulse else 'NO_PULSE',
            blockers=list(self.blockers),fault=self.fault,clear_samples=self.samples,
            ready=self.phase=='READY' and self.receipt is not None,
            old_goal_resumed=False,motion_authorized=False)
