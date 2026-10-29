"""Bounded recovery decisions only. No ROS, processes, network or device I/O.

Abort the old attempt rather than rebinding an old motion/pulse lease. A new
process generation is admitted only after all old owned groups exited and a
retained read-only observer proves current zero/OFF/home. Nav2 and its existing
localization stay alive; no Nav2 restart or new Initial Pose is requested.
"""
import math


def number(x):
    return type(x) in (int,float) and math.isfinite(x)


class RuntimeRecovery:
    WINDOW_S=600.
    MAX_RESTARTS=3
    QUIET_S=1.
    HANDOFF_S=10.
    FAULTS=frozenset(('BOOT_OR_SERIAL_EPOCH_CHANGED',
        'INPUT_GID_CHANGED:/navigate_to_pose/_action/status',
        'LIVE_EXECUTOR_PROOF_LOST:INPUT_GID_CHANGED:/navigate_to_pose/_action/status',
        'LIVE_EXECUTOR_PROOF_LOST:WAIT_EXACT_OPERATIONAL_GRAPH', 'ACTUATOR_LOCKED'))

    def __init__(self):
        self.stage='RUNNING';self.reason=None;self.generation=0;self.last=None
        self.origin_bridge=None;self.handoff_since=None;self.backend_since=None;self.since=None;self.key=None
        self.start_times=[];self.retired_at=None;self.history=[]
        self.home_before=None;self.home_required=False;self.last_home=None;self.pulses_reserved=0
        self.act_esp=None;self.act_progress=None;self.actor_pulse_key=None
        self.navigation_only=False

    def observe_actuation(self,h):
        r=h.get('runtime_recovery',{});a=r.get('actuator') or {};actor=r.get('actor') or {}
        count=a.get('home_command_completed_count')
        actual_on=(a.get('pump_pwm_commanded_on') is True or a.get('servo_pwm_commanded_on') is True
            or a.get('servo_phase') in ('SWEEPING','RETURNING_HOME'))
        sequence=actor.get('spray_sequence') or {}
        index=sequence.get('pulse_index')
        if type(index) is int and index>0:self.pulses_reserved=max(self.pulses_reserved,index)
        actor_on=(actor.get('pump_on_feedback_seen') is True or sequence.get('actuation_started') is True)
        pulse_key=((actor.get('r5_cycle_context') or {}).get('cycle_id'),sequence.get('pulse_index'))
        if actual_on:
            self.home_required=True
            if type(count) is int and count>=0:
                self.home_before=max(self.home_before if self.home_before is not None else count,count)
        if actor_on and pulse_key!=self.actor_pulse_key:
            self.home_required=True;self.actor_pulse_key=pulse_key
            baseline=actor.get('start_home_count',self.last_home)
            if self.home_before is None and type(baseline) is int and baseline>=0:self.home_before=baseline
        if type(count) is int and count>=0:self.last_home=count

    def step(self,now,h=None,*,retired=False,stopped=False):
        if not number(now) or self.last is not None and now<self.last:
            raise ValueError('RECOVERY_CLOCK_INVALID')
        if self.last is not None and now-self.last>.75:self.since=None
        self.last=now
        if stopped:return []
        h=h if isinstance(h,dict) else {}
        sample=h.get('sample_mono')
        if (h.get('schema_version')!=1 or h.get('commands_published') is not False
                or not number(sample) or not 0<=now-sample<=.75):h={}
        if self.stage=='RUNNING':
            self.observe_actuation(h)
            r=h.get('runtime_recovery',{});op=r.get('operator') or {}
            trusted=r.get('operator_source_current') is True
            trigger=op.get('fault') if trusted and op.get('fault') in self.FAULTS else None
            if trusted and op.get('phase')=='WAIT_STOP':
                if self.handoff_since is None:self.handoff_since=now
                if now-self.handoff_since>=self.HANDOFF_S:trigger='EXACT_GOAL_HANDOFF_TIMEOUT'
            else:self.handoff_since=None
            if (trusted and op.get('armed') is True and op.get('closed') is False
                    and h.get('lifecycle_active') is False):
                if self.backend_since is None:self.backend_since=now
                if now-self.backend_since>=5.:trigger='NAV2_BACKEND_LOST'
            else:self.backend_since=None
            if trigger:
                return self.retire(now,trigger,op)
        elif self.stage=='RETIRING' and retired:
            self.retired_at=now;self.stage='WAIT_CURRENT_STOP';self.since=None;self.key=None
            return []
        elif self.stage=='WAIT_CURRENT_STOP':
            self.start_times=[t for t in self.start_times if now-t<self.WINDOW_S]
            if len(self.start_times)>=self.MAX_RESTARTS:
                self.reason='RESTART_RATE_LIMIT_WAIT';self.since=None;return []
            r=h.get('runtime_recovery',{});a=r.get('actuator') or {}
            key=h.get('identity_key');sample=h.get('sample_mono')
            # A genuine servo fault inhibits spray independently of navigation.
            # This alternative still requires current identity, PWM OFF, no
            # actuator motion, actual stopped wheels and released publishers.
            count=a.get('home_command_completed_count')
            home_pending=(self.home_required and (self.home_before is None or type(count) is not int or count<=self.home_before))
            self.navigation_only=((a.get('servo_phase')=='LOCKED' or home_pending)
                and a.get('servo_phase') in ('IDLE','LOCKED')
                and a.get('pump_pwm_commanded_on') is False
                and a.get('servo_pwm_commanded_on') is False)
            if self.navigation_only and self.home_before is None and type(count) is int and count>=0:
                self.home_before=count
            io_name='io_navigation' if self.navigation_only else 'io'
            self.reason='WAIT_CURRENT_STOP_OFF_AND_IO'
            current=(number(sample) and self.retired_at is not None and sample>self.retired_at
                and 0<=now-sample<=.75 and h.get('checks',{}).get(io_name) is True
                and h.get('stable_s',{}).get(io_name,0)>=self.QUIET_S
                and r.get('io_sources_current') is True
                and type(r.get('raw_count')) is int and r['raw_count']==0
                and type(r.get('pump_count')) is int and r['pump_count']==0
                and isinstance(key,list) and len(key)==3
                and isinstance(key[0],str) and type(key[1]) is int and key[1]>0
                and isinstance(key[2],str) and len(key[2])==64
                and self.quiet(h.get('io_diagnostics',{}).get('cmd'),0.)
                and self.quiet(h.get('io_diagnostics',{}).get('odom'),.02))
            if self.origin_bridge is not None and (not isinstance(key,list) or not key or key[0]!=self.origin_bridge):
                self.reason='WAIT_ORIGINAL_PAIRED_PI_BRIDGE';current=False
            count=a.get('home_command_completed_count')
            if self.home_required and not self.navigation_only and (self.home_before is None or type(count) is not int or count<=self.home_before):
                self.reason='WAIT_NEW_POST_PULSE_HOME';current=False
            esp=a.get('esp_ms')
            if type(esp) is not int or not 0<=esp<2**32:current=False
            if key!=self.key:
                self.key=key;self.since=None;self.act_esp=None;self.act_progress=None
            if current:
                if self.act_esp is None:self.act_esp=esp;self.act_progress=now;self.since=None
                elif esp!=self.act_esp:
                    if (esp-self.act_esp)%(2**32)>=2**31:
                        self.reason='WAIT_ACT_CLOCK_CONTINUITY';self.since=None;current=False
                    self.act_esp=esp;self.act_progress=now
                if self.act_progress is None or now-self.act_progress>.35:current=False
            if not current:self.since=None;return []
            if self.since is None:self.since=now
            if now-self.since>=self.QUIET_S:
                self.stage='WAIT_EXISTING_NAV';self.reason='WAIT_EXISTING_NAV_DATA'
                return []
        elif self.stage=='WAIT_EXISTING_NAV':
            c=h.get('checks',{});s=h.get('stable_s',{})
            io_name='io_navigation' if self.navigation_only else 'io'
            pose_name='pose_navigation' if self.navigation_only else 'pose'
            if c.get(io_name) is not True or h.get('identity_key')!=self.key:
                self.stage='WAIT_CURRENT_STOP';self.reason='WAIT_CURRENT_IO_AGAIN'
                self.since=None;self.key=None;return []
            if (c.get(io_name) is True and c.get(pose_name) is True and s.get(pose_name,0)>=5.
                    and c.get('yolo') is True and h.get('lifecycle_active') is True
                    and h.get('identity_key')==self.key):
                self.start_times.append(now);self.stage='WAIT_OPERATOR';self.reason='WAIT_OPERATOR_REBIND_ACK'
                return ['START_CONSUMERS']
        elif self.stage=='WAIT_OPERATOR':
            op=h.get('runtime_recovery',{}).get('operator') or {}
            key=h.get('identity_key')
            if isinstance(key,list) and len(key)==3 and key!=self.key:
                return self.retire(now,'IDENTITY_CHANGED_DURING_REBIND',op)
            rebind=op.get('runtime_rebind') or {}
            if (h.get('checks',{}).get('operator_present') is True
                    and h.get('stable_s',{}).get('operator_present',0)>=.5
                    and rebind.get('applied_generation')==self.generation and rebind.get('held') is False):
                self.stage='RUNNING';self.reason='RECOVERED_NEW_GENERATION_NO_REPLAY'
                self.handoff_since=self.backend_since=None
                self.home_required=False;self.home_before=None;self.pulses_reserved=0
                self.history[-1]['recovered_at']=now
                return ['ENABLE_NEW_GENERATION']
        return []

    def retire(self,now,reason,op):
        bound=op.get('armed_identity')
        if self.origin_bridge is None and isinstance(bound,list) and len(bound)==3:self.origin_bridge=bound[0]
        self.reason=reason;self.stage='RETIRING';self.generation+=1
        self.history.append(dict(generation=self.generation,reason=reason,at=now,
            aborted_reserved_pulses=self.pulses_reserved,home_required=self.home_required))
        return ['RETIRE_OWNED']

    @staticmethod
    def quiet(values,tolerance):
        return (isinstance(values,(tuple,list)) and len(values)==6
                and all(number(v) and abs(v)<=tolerance for v in values))

    def report(self):
        return dict(stage=self.stage,reason=self.reason,generation=self.generation,
            navigation_only=self.navigation_only,home_required=self.home_required,home_before=self.home_before,pulses_reserved=self.pulses_reserved,history=list(self.history),
            max_mission_rebuilds_per_600s=self.MAX_RESTARTS,old_goal_replayed=False,pulse_replayed=False,
            physical_commands_published=False,nav_restart_allowed=False,new_initial_pose_required=False)
