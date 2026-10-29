"""Shared observe-only entrypoint plumbing. Imports perform no launch/network I/O."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid
from operational_logs import OwnedLog,OwnedLogRegistry,strict_json
from operational_owned_process import OwnedProcess
from operational_stream_log import StreamingOwnedLog
from operational_stack import StackCleanup

ROOT=Path(__file__).resolve().parents[1]

def atomic(path,value):
    path=Path(path);temp=path.with_name(path.name+'.tmp')
    temp.write_text(json.dumps(value,allow_nan=False,indent=2)+'\n');os.chmod(temp,0o600);os.replace(temp,path)

def packet_digest(root=ROOT):
    root=Path(root);manifest=root/'operational_packet.sha256';raw=manifest.read_bytes()
    rows=raw.decode().splitlines();assert rows,'EMPTY_PACKET'
    seen=set()
    for line in rows:
        sha,name=line.split('  ',1);p=(root/name).resolve()
        if p==manifest or not p.is_relative_to(root.resolve()) or name in seen:raise ValueError('INVALID_PACKET_ENTRY')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=sha:raise ValueError('PACKET_SOURCE_CHANGED:'+name)
        seen.add(name)
    return hashlib.sha256(raw).hexdigest()

def read_health(path,session,now):
    value=strict_json(Path(path).read_text());at=value.get('sample_mono')
    # Atomic replacement may finish after a caller sampled its clock. Production
    # callers supply the clock, so freshness is checked after reading the file.
    now=now() if callable(now) else now
    if (value.get('schema_version')!=1 or value.get('session')!=session or value.get('commands_published') is not False
            or type(at) not in (int,float) or not math.isfinite(at) or not 0<=now-at<=.75):
        raise ValueError('STALE_OR_FOREIGN_STARTUP_HEALTH')
    return value

def ros(executable,*args):return ['ros2','run','fire_robot_mission',executable,*map(str,args)]
def launch(package,name,*args):return ['ros2','launch','--noninteractive',package,name,*map(str,args)]

class LocalProcesses:
    def __init__(self,out,session,*,backend_factory=None):
        self.out=Path(out);self.session=session;self.registry=OwnedLogRegistry();self.processes={}
        self.backend_factory=backend_factory;self.stack=None;self.history=[]
    def start(self,role,argv,*,launch_leader=False):
        if role in self.processes:raise ValueError('ROLE_ALREADY_RUNNING:'+role)
        backend=None if self.backend_factory is None else self.backend_factory(role)
        p=OwnedProcess(role,launch_leader=launch_leader,backend=backend)
        log=StreamingOwnedLog(self.out/(role+'.log'))
        try:log.start(p,argv)
        except BaseException:
            # Retain any process already created so finally can clean it up.
            if p.process is not None:
                self.processes[role]=p;self.registry.register(role,log)
            else:log.close()
            raise
        self.processes[role]=p;self.registry.register(role,log);self.history.append(dict(event='START',role=role,argv=argv))
        return p
    def check(self,now,*,allow_operator_exit=False,exclude=()):
        operator_done=False
        for role,p in self.processes.items():
            if role in exclude:continue
            p.tick(now)
            if p.launch_leader and self.registry.logs[role].scan()['errors']:
                raise RuntimeError('OWNED_LAUNCH_CHILD_ERROR:'+role)
            if role=='operator' and allow_operator_exit and p.done and p.rc==0 and p.error is None:
                operator_done=True;continue
            if p.done or p.error is not None or not any(m['pid']==p.pid for m in p.live):
                raise RuntimeError('OWNED_CHILD_EXIT:'+role)
        return operator_done
    def begin_retirement(self,now,destination,roles):
        if self.stack is not None:raise ValueError('RETIRE_DURING_FINAL_CLEANUP')
        roles=tuple(r for r in roles if r in self.processes)
        registry=OwnedLogRegistry()
        for r in roles:registry.register(r,self.registry.logs[r])
        stack=(StackCleanup.from_logs(registry) if 'operator' in registry.logs else
               StackCleanup(lambda:None,child_logs=registry))
        for r in roles:stack.register(r,self.processes[r])
        stack.stop(now,'RUNTIME_RECOVERY_ABORT_OLD_GENERATION')
        return dict(stack=stack,roles=roles,destination=Path(destination))
    def tick_retirement(self,retirement,now):
        stack=retirement['stack'];stack.tick(now)
        if not stack.done:return False
        result=stack.report()
        if not all(p['group_exit_confirmed'] and not p['live_pids'] for p in result['processes'].values()):
            return False
        dest=retirement['destination'];dest.mkdir(mode=0o700,parents=True,exist_ok=False)
        atomic(dest/'retirement.json',result)
        for role in retirement['roles']:
            log=self.registry.logs.pop(role);log.close()
            log.path.rename(dest/(role+'.log'))
            self.processes.pop(role)
        self.history.append(dict(event='RETIRED',out=str(dest),audit=result))
        return True
    def begin_stop(self,now,cause='Q'):
        if self.stack is None:
            self.stack=(StackCleanup.from_logs(self.registry) if 'operator' in self.registry.logs
                        else StackCleanup(lambda:None,child_logs=self.registry))
            for role,p in self.processes.items():self.stack.register(role,p)
        return self.stack.stop(now,cause)
    def drain(self,stop_flag=lambda:False,*,clock=time.monotonic,sleep=time.sleep,cause='FINALLY'):
        # Stop flag/repeated SIGINT never skips zero/OFF drain or extends budgets.
        self.begin_stop(clock(),cause)
        while not self.stack.done:
            self.stack.tick(clock());sleep(.05)
        result=self.stack.report();atomic(self.out/'cleanup.json',result);return result
    def close_logs(self):
        for log in self.registry.logs.values():log.close()


def startup_observations(health):
    """Current stop/I/O proof plus independently accumulated localization.

    Never cache an admission across health samples or transport epochs. Keep
    strict pose (including I/O) true now; only its five-second timer is split.
    """
    checks=health.get('checks',{});stable=health.get('stable_s',{})
    def settled(key,seconds):
        age=stable.get(key)
        return checks.get(key) is True and type(age) in (int,float) and math.isfinite(age) and age>=seconds
    key=health.get('identity_key')
    same_identity=(isinstance(key,list) and len(key)==3 and key==health.get('localization_identity_key'))
    return dict(io=settled('io',1.),camera=settled('camera',1.),yolo=settled('yolo',1.),
        pose=(checks.get('pose') is True and same_identity and settled('localization',5.)),
        operator_present=settled('operator_present',.5))


class StartupSequence:
    """Pure finite startup: effects go through a caller, never accepts live arms."""
    LIMITS=dict(PI_IO=60.,CAMERA=30.,SEAL_PI=20.,POSE=185.,YOLO=180.,CONSUMERS=180.)
    REQUIRED=dict(PI_IO=('io',),CAMERA=('io','camera'),SEAL_PI=('pi_sealed','io','camera'),
        POSE=('camera','pose'),YOLO=('io','camera','pose','yolo'),
        CONSUMERS=('io','camera','pose','yolo','operator_present'))
    def __init__(self,now,*,live=False):
        if type(live) is not bool:raise ValueError('STARTUP_LIVE_FLAG_MUST_BE_BOOL')
        self.live=live
        self.stage='CLOCK';self.since=now;self.last=now;self.retry=0;self.done=False;self.error=None
        self.blockers=[];self.bridge=None
    def advance(self,stage,now):self.stage=stage;self.since=now
    def step(self,now,observation):
        if self.done:return []
        if not math.isfinite(now) or now<self.last:self.error='STARTUP_CLOCK_INVALID';self.done=True;return ['STOP']
        self.last=now;effects=[]
        self.blockers=[k for k in self.REQUIRED.get(self.stage,()) if observation.get(k) is not True]
        # Reopens of this unarmed serial bridge may settle under a new epoch;
        # replacing the paired bridge itself requires a new session/pairing.
        key=observation.get('identity_key')
        if observation.get('io') is True and isinstance(key,list) and len(key)==3:
            if self.bridge is None:self.bridge=key[0]
            elif key[0]!=self.bridge:
                self.error='STARTUP_PAIRED_BRIDGE_CHANGED';self.done=True;return ['STOP']
        if observation.get('fatal'):
            self.error=str(observation['fatal']);self.done=True;return ['STOP']
        if self.stage in self.LIMITS and now-self.since>=self.LIMITS[self.stage]:
            if self.stage=='CAMERA' and self.retry==0:
                self.retry=1;self.advance('RETRY_PI',now);return ['RETRY_PI','RESTART_HEALTH']
            self.error='STARTUP_TIMEOUT:'+self.stage;self.done=True;return ['STOP']
        if self.stage=='RETRY_PI' and now-self.since>=20.:
            self.error='STARTUP_TIMEOUT:RETRY_PI';self.done=True;return ['STOP']
        if self.stage=='CLOCK':
            if observation.get('clock_ok') is True:self.advance('PI_IO',now);effects=['START_PI','START_HEALTH']
        elif self.stage=='PI_IO' and observation.get('io') is True:
            self.advance('CAMERA',now)
        elif self.stage=='CAMERA' and observation.get('io') is True and observation.get('camera') is True:
            self.advance('SEAL_PI',now);effects=['SEAL_PI']
        elif self.stage=='SEAL_PI' and all(observation.get(k) is True for k in ('pi_sealed','io','camera')):
            self.advance('POSE',now);effects=['START_NAV','START_RVIZ']
        elif self.stage=='POSE' and observation.get('camera') is True and observation.get('pose') is True:
            self.advance('YOLO',now);effects=['START_YOLO']
        elif self.stage=='YOLO' and all(observation.get(k) is True for k in ('io','camera','pose','yolo')):
            self.advance('CONSUMERS',now);effects=['START_CONSUMERS','START_OPERATOR']
        elif self.stage=='CONSUMERS' and all(observation.get(k) is True for k in ('io','camera','pose','yolo','operator_present')):
            self.advance('READY_GATE2' if self.live else 'READY_OBSERVE',now);self.done=True
        if self.stage=='RETRY_PI' and observation.get('retry_ready') is True:
            self.advance('PI_IO',now)
        return effects
    def report(self):return dict(stage=self.stage,done=self.done,error=self.error,retries=self.retry,live=self.live)
    def diagnostics(self):
        limit=self.LIMITS.get(self.stage)
        return dict(self.report(),
            entered_mono=self.since,elapsed_s=max(0.,self.last-self.since),limit_s=limit,
            remaining_s=None if limit is None else max(0.,limit-(self.last-self.since)),
            blockers=[] if self.done and self.error is None else list(self.blockers))


def run_clock_gate(out,stop,*,backend=None,clock=time.monotonic,sleep=time.sleep):
    """Own the clock helper group; allow its nested probe cleanup to finish."""
    from operational_logs import strict_json
    out=Path(out);process=OwnedProcess('clock_gate',backend=backend)
    log=OwnedLog(out/'clock_gate.log');started=clock();aborted=False
    try:
        log.start(process,['bash',str(ROOT/'scripts/operational_support/clock_gate.sh'),str(out/'clock'),'10.0.0.253'],started)
        while not process.done:
            now=clock()
            if stop() or now-started>=130.:aborted=True;process.stop(now)
            else:process.tick(now)
            if not process.done:sleep(.05)
        atomic(out/'clock_process.json',process.report())
        if aborted or process.forced or process.rc!=0 or process.error is not None:raise RuntimeError('CLOCK_GROUP_FAILED_OR_INTERRUPTED')
        result=strict_json((out/'clock/settle.json').read_text())
        if (result.get('status')!='CLOCK_SETTLED' or result.get('measurement_budget_ms')!=50
                or result.get('limit_s')!=120 or result.get('pass_separation_s')!=5):raise ValueError('CLOCK_GATE_NOT_ORIGINAL_PROFILE')
        return result
    finally:
        try:
            if process.process is not None and not process.done:
                process.stop(clock())
                while not process.done:process.stop(clock());sleep(.05)
        finally:log.close()
