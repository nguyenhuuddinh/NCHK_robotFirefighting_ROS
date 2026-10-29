"""Caller-side recovery effects, restricted to the launcher's owned local roles.

Pi, Nav2, localization, startup observer, camera, YOLO and RViz stay up.
Only mission leases are retired; operator/GUI rebinds in place after fresh proof.
No SSH/NTP/device commands or old goal/pump replay are issued by this helper.
"""
from pathlib import Path
import uuid
from operational_entry import atomic,read_health
from runtime_recovery import RuntimeRecovery


class RuntimeRecoveryRunner:
    ROLES=('mission',)

    def __init__(self,local,out,session,packet,command_factory,automatic_start):
        self.local=local;self.out=Path(out).resolve();self.session=session;self.packet=packet
        self.command_factory=command_factory;self.automatic_start=automatic_start
        self.core=RuntimeRecovery();self.retirement=None;self.retired=False
        self.current_start=None;self.current_session=None;self.current_commands=None
        self.last_notice=None;self.last_write=None
        self.rebind_path=self.automatic_start.parent/'runtime_rebind.json';self.rebind=None

    @property
    def excluded(self):
        return self.ROLES if self.core.stage=='RETIRING' else ()

    def step(self,now,*,stop=False):
        try:h=read_health(self.out/'health.json',self.session,lambda:now)
        except (OSError,ValueError,KeyError,TypeError):h={}
        if self.core.stage=='RETIRING' and self.retirement is not None and not stop:
            self.retired=self.local.tick_retirement(self.retirement,now)
            if not self.retired and self.retirement['stack'].done:
                self.core.reason='WAIT_PROVEN_OLD_GROUP_EXIT_CTRL_C_TO_STOP'
        effects=self.core.step(now,h,retired=self.retired,stopped=stop)
        for effect in effects:
            if effect=='RETIRE_OWNED':
                self.rebind=dict(schema_version=1,kind='R5_RUNTIME_REBIND',session=self.session,
                    packet=self.packet,generation=self.core.generation,nonce=str(uuid.uuid4()),
                    phase='HOLD',reason=self.core.reason,
                    after_pulse=self.core.home_required or self.core.pulses_reserved>0)
                atomic(self.rebind_path,self.rebind)
                dest=self.out/('runtime_recovery_'+str(self.core.generation))/'retired'
                self.retirement=self.local.begin_retirement(now,dest,self.ROLES)
                self.retired=False
            elif effect=='START_CONSUMERS':
                folder=self.out/('runtime_recovery_'+str(self.core.generation));folder.mkdir(exist_ok=True)
                self.current_start=self.automatic_start;self.current_session=self.session
                self.current_commands=self.command_factory(self.current_start,self.current_session,
                    self.core.home_required or self.core.pulses_reserved>0)
                self.local.start('mission',self.current_commands['mission'],launch_leader=True)
                self.rebind.update(phase='RESUME',identity_key=self.core.key,
                    navigation_only=self.core.navigation_only,home_before=self.core.home_before)
                atomic(self.rebind_path,self.rebind)
            elif effect=='ENABLE_NEW_GENERATION':
                atomic(self.out/('runtime_recovery_'+str(self.core.generation))/'rebound.json',
                    dict(generation=self.core.generation,nav_kept=True,operator_kept=True))
        report=self.core.report()
        if self.core.generation and (self.last_write is None or now-self.last_write>=1. or effects):
            atomic(self.out/'runtime_recovery.json',report);self.last_write=now
            if self.rebind is not None and self.rebind.get('phase')=='HOLD':
                self.rebind.update(reason=self.core.reason,current_stage=self.core.stage)
                atomic(self.rebind_path,self.rebind)
        signature=(report['generation'],report['stage'],report['reason'])
        if self.core.generation and (signature!=self.last_notice):
            print('R5_RUNTIME_RECOVERY '+str(report),flush=True)
            if report['stage']=='WAIT_EXISTING_NAV':
                print('WAIT_NAV_RECOVERY: Nav2 kept running; waiting for current I/O/localization. No new Initial Pose requested.',flush=True)
            self.last_notice=signature
        return report
