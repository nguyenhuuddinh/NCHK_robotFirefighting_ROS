"""Bounded stop verification for R5 operator teardown; no ROS or device I/O.

False arms stop authority immediately. This object only checks fresh telemetry;
it never grants motion, invents a home completion, or repeats a pulse.
"""
import copy
from .operational_cycle import finite,zero,valid_uuid
from .operational_identity import identity_current,identity_key,IdentityContinuity
from .actuator_freshness import fresh_off


class StopVerification:
    TIMEOUT_S=4.
    HOLD_S=.4

    def __init__(self):
        self.start=None;self.last_at=None;self.since=None;self.done=False;self.confirmed=False
        self.reason='NOT_REQUESTED';self.cause=None;self.expected_identity=None;self.fault=None
        self.home_before=None;self.home_required=False;self.act_esp=None;self.act_progress_at=None
        self.blockers=[]

    def begin(self,now,*,cause='Q',expected_identity=None):
        if self.start is not None:return
        if not finite(now):raise ValueError('STOP_CLOCK_INVALID')
        self.start=self.last_at=now;self.cause=str(cause);self.expected_identity=expected_identity
        self.reason='WAIT_STOP_PROOF'

    @staticmethod
    def fresh(at,now):return finite(at) and 0<=now-at<=.35

    def step(self,now,inputs,*,sources_ok,raw_count,pump_count):
        if self.start is None:raise ValueError('STOP_NOT_REQUESTED')
        if self.done:return self.report(self.last_at)
        if not finite(now) or now<self.last_at:
            self.done=True;self.fault='STOP_CLOCK_DISCONTINUITY';self.reason='STOP_UNCONFIRMED'
            return self.report(self.last_at)
        if now-self.last_at>.35:self.since=None
        self.last_at=now
        def entry(name):return inputs.get(name,(None,None))
        def current(name):
            value,at=entry(name);return value if self.fresh(at,now) else None
        errors=[]
        if sources_ok is not True:errors.append('WAIT_PINNED_STOP_SOURCES')
        if raw_count!=0:errors.append('WAIT_RAW_RELEASE')
        if pump_count!=0:errors.append('WAIT_PUMP_RELEASE')
        if not zero(current('cmd')):errors.append('WAIT_FRESH_CMD_ZERO')
        # Retain the accepted actuator-drain limit, including all angular axes.
        if not zero(current('odom'),.02,.02):errors.append('WAIT_FRESH_ODOM_STATIONARY')
        identity,at=entry('identity');key=identity_key(identity)
        if self.expected_identity is not None and isinstance(identity,dict):
            instance=identity.get('bridge_instance_id');epoch=identity.get('connection_epoch');elf=identity.get('elf_sha256')
            if (valid_uuid(instance) and type(epoch) is int and epoch>0
                    and (instance,epoch)!=self.expected_identity[:2]):self.fault='STOP_BOOT_EPOCH_CHANGED'
            if isinstance(elf,str) and len(elf)==64 and elf!=self.expected_identity[2]:self.fault='STOP_BOOT_EPOCH_CHANGED'
            if identity.get('reason') in IdentityContinuity.HARD_REASONS:self.fault='STOP_IDENTITY_FAULT'
        if identity_current(identity,at,now):
            if self.expected_identity is None:self.expected_identity=key
            elif key!=self.expected_identity:self.fault='STOP_BOOT_EPOCH_CHANGED'
        else:errors.append('WAIT_CURRENT_BOOT_IDENTITY')
        a,act_at=entry('actuator');a=a if isinstance(a,dict) else {}
        if not (self.fresh(act_at,now) and fresh_off(a,act_at,now) and a.get('schema_version')==1
                and a.get('mode')=='COMMAND_TELEMETRY_ONLY' and a.get('pump_pwm_ready') is True):
            errors.append('WAIT_FRESH_ACTUATOR_OFF_HOME')
        count=a.get('home_command_completed_count');count_ok=type(count) is int and count>=0
        if not count_ok:errors.append('WAIT_HOME_COUNTER')
        # Historical ON/sweep is only read from the actual actor/current ACT.
        # An early P key alone does not invent an ON or a required home cycle.
        actor=current('actor');actor=actor if isinstance(actor,dict) else {}
        if sources_ok and (actor.get('pump_on_feedback_seen') is True or actor.get('published_pump_request') is True):
            self.home_required=True
            before=actor.get('start_home_count')
            if self.home_before is None and type(before) is int and before>=0:self.home_before=before
        if sources_ok and self.fresh(act_at,now) and (a.get('pump_pwm_commanded_on') is True
                or a.get('servo_pwm_commanded_on') is True or a.get('servo_phase') in ('SWEEPING','RETURNING_HOME')):
            self.home_required=True
            if self.home_before is None and count_ok:self.home_before=count
        if self.home_required and (self.home_before is None or not count_ok or count<=self.home_before):
            errors.append('WAIT_NEW_POST_PULSE_HOME')
        esp=a.get('esp_ms')
        if sources_ok and self.fresh(act_at,now) and type(esp) is int and 0<=esp<2**32:
            if self.act_esp is None:self.act_esp=esp;self.act_progress_at=now
            elif esp!=self.act_esp:
                if (esp-self.act_esp)%(2**32)>=2**31:self.fault='STOP_ACT_CLOCK_REGRESSION'
                else:self.act_esp=esp;self.act_progress_at=now
        if self.act_progress_at is None or now-self.act_progress_at>.35:errors.append('WAIT_NEW_ACT_SAMPLE')
        if self.fault:errors.append(self.fault)
        self.blockers=errors
        if errors:self.since=None
        elif self.since is None:self.since=now
        if self.since is not None and now-self.since>=self.HOLD_S:
            self.done=self.confirmed=True;self.reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED'
        elif now-self.start>=self.TIMEOUT_S:
            self.done=True;self.reason='STOP_UNCONFIRMED'
        return self.report(now)

    def abort(self,reason):
        if not self.done:
            self.done=True;self.confirmed=False;self.reason='STOP_UNCONFIRMED'
            self.blockers.append(str(reason))

    def report(self,now):
        return dict(scope='R5_STOP_TELEMETRY_VERIFICATION',requested=self.start is not None,
            done=self.done,confirmed=self.confirmed,cause=self.cause,reason=self.reason,
            elapsed_s=0. if self.start is None else max(0.,now-self.start),
            blockers=copy.deepcopy(self.blockers),fault=self.fault,
            home_required=self.home_required,home_before=self.home_before,
            motion_authorized=False,pulse_replayed=False)
