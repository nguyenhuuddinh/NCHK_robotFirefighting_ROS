#!/usr/bin/env python3
"""Explicit observe-only Pi entrypoint; default prints a plan and starts nothing."""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from operational_entry import ROOT,LocalProcesses,atomic,packet_digest,read_health,ros,launch
from operational_logs import strict_json
from operational_preflight import preflight
from operational_peer import Mailbox,binding

PI_ARGV=launch('fire_robot_bringup','robot.launch.py','use_sim_time:=false','camera_fps:=15',
    'native_camera_publication_mode:=ASYNCHRONOUS','native_camera_transport:=wifi_nonblocking',
    'cmd_vel_input_topic:=/cmd_vel_raw','enable_serial_recovery_diagnostics:=true')

class PiController:
    def __init__(self,bind,start,begin_stop,poll_stop,health,now):
        self.bind=bind;self.start_io=start;self.begin_stop=begin_stop;self.poll_stop=poll_stop;self.health=health
        self.phase='WAIT_LAPTOP';self.error=None;self.retries=0;self.sealed=False;self.cleanup=None
        self.started_at=now;self.last=now;self.stop_started=False
        self.seal_requested_at=None;self.seal_health_at_request=None;self.seal_confirmed_at=None
    def _start(self,now):
        try:self.start_io()
        except Exception:
            self.error='PI_IO_START_FAILED';self.begin_stop();self.stop_started=True;self.phase='STOPPING'
            raise
        self.phase='IO_STARTING';self.started_at=now
    def command(self,cmd,now):
        if cmd=='START':
            if self.phase!='WAIT_LAPTOP':raise ValueError('PI_START_WRONG_PHASE')
            self._start(now)
        elif cmd=='RETRY_PREARM':
            if self.sealed or self.seal_requested_at is not None or self.phase!='IO_STARTING' or self.retries:raise ValueError('PI_RETRY_NOT_PREARM')
            self.retries=1;self.begin_stop();self.phase='RETRY_STOPPING'
        elif cmd=='SEAL':
            if self.phase!='IO_STARTING':raise ValueError('PI_SEAL_WRONG_PHASE')
            if self.seal_requested_at is None:
                self.seal_requested_at=now;self.seal_health_at_request=copy.deepcopy(self.health())
            self._try_seal(now)
        elif cmd=='STOP':
            if self.phase!='DONE' and not self.stop_started:
                self.begin_stop();self.stop_started=True;self.phase='STOPPING'
        elif cmd!='STATE':raise ValueError('PI_UNKNOWN_COMMAND')
        return self.report()
    def _try_seal(self,now):
        if self.phase=='IO_STARTING' and self.seal_requested_at is not None:
            if now-self.seal_requested_at>=20.:
                self.error='PI_SEAL_HEALTH_TIMEOUT';self.command('STOP',now)
            elif self.health().get('io') is True:
                self.sealed=True;self.seal_confirmed_at=now;self.phase='RUNNING_OBSERVE'

    def tick(self,now):
        if now<self.last:raise ValueError('PI_RUNNER_CLOCK_REGRESSION')
        self.last=now
        self._try_seal(now)
        if self.phase=='WAIT_LAPTOP' and now-self.started_at>=360.:self.command('STOP',now)
        if self.phase in ('STOPPING','RETRY_STOPPING'):
            result=self.poll_stop()
            if result is not None:
                if self.phase=='RETRY_STOPPING' and result.get('remote_groups_exited') and result.get('children_clean') and not result.get('forced'):
                    self.cleanup=result;self._start(now)
                else:
                    self.cleanup=result;self.phase='DONE'
        return self.report()
    def report(self):return dict(session=self.bind['session'],phase=self.phase,error=self.error,retries=self.retries,
        sealed=self.sealed,seal_requested=self.seal_requested_at is not None,
        seal_requested_at=self.seal_requested_at,seal_confirmed_at=self.seal_confirmed_at,
        seal_health_at_request=self.seal_health_at_request,cleanup=self.cleanup,health=self.health() if self.phase in ('IO_STARTING','RUNNING_OBSERVE') else {},live=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-observe',action='store_true')
    p.add_argument('--out',type=Path);p.add_argument('--max-runtime-s',type=int,default=28800)
    a=p.parse_args(argv)
    if not a.run_observe:print(json.dumps(dict(mode='PLAN_ONLY_LIVE_FALSE',robot=PI_ARGV,wait_laptop_s=360,max_prearm_retries=1)));return 0
    if a.out is None or not 60<=a.max_runtime_s<=28800:p.error('--out and runtime60..28800 required')
    # The configured rawSerialV2 device, runtime environment and installed
    # workspace must already have been reviewed; this entrypoint never flashes.
    a.out.mkdir(mode=0o700)
    try:packet=packet_digest();preflight(ROOT,'pi')
    except Exception as exc:
        atomic(a.out/'failure.json',dict(error='PI_PREFLIGHT:'+type(exc).__name__+':'+str(exc),hardware_started=False));return 2
    (a.out/'requests').mkdir();(a.out/'responses').mkdir()
    lock=(Path.home()/'.cache/fire_robot_r5_pi.lock');lock.parent.mkdir(exist_ok=True)
    lockfd=lock.open('a');fcntl.flock(lockfd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    bind=binding(a.out,packet);atomic(a.out/'pairing.json',bind);mail=Mailbox(a.out,bind)
    stop=[False];previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    for sig in previous:signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    local=None;controller=None;generation=0;archive=[];rc=2
    def start():
        nonlocal local,generation
        generation+=1;folder=a.out/('generation_'+str(generation));folder.mkdir(mode=0o700)
        local=LocalProcesses(folder,bind['session']);local.start('pi',PI_ARGV,launch_leader=True)
        local.start('startup_health',ros('r5_startup_health','--out',folder/'health.json','--session',bind['session'],'--role','pi'))
    def begin():
        if local is not None:local.begin_stop(time.monotonic(),'PEER_STOP')
    def poll():
        if local is None:return dict(remote_groups_exited=True,children_clean=True,forced=False,groups={})
        local.stack.tick(time.monotonic())
        if not local.stack.done:return None
        groups={r:p.report() for r,p in local.processes.items()}
        try:audit=local.registry.audit(local.processes)
        except (OSError,ValueError) as exc:audit=dict(clean=False,error=str(exc))
        result=dict(remote_groups_exited=all(x['group_exit_confirmed'] for x in groups.values()),
            children_clean=audit['clean'],forced=any(x['forced'] or x['leader_rc']!=0 or x['error'] for x in groups.values()),groups=groups,child_audit=audit)
        atomic(local.out/'pi_cleanup.json',result);archive.append(result);local.close_logs();return result
    def health():
        try:
            row=read_health(local.out/'health.json',bind['session'],time.monotonic)
            return {**row['checks'],'sample_mono':row['sample_mono'],
                'io_stable_s':row.get('stable_s',{}).get('io',0),
                'io_blockers':row.get('io_diagnostics',{}).get('blockers',[])}
        except (OSError,ValueError,TypeError,AttributeError,KeyError) as exc:
            return dict(io=False,health_error=type(exc).__name__+':'+str(exc))
    try:
        # Keep controlling TTY for existing sudo chronyc. No ROS yet. This exact
        # accepted helper is bounded and changes no NTP config/service/manual time.
        refresh=['timeout','--foreground','--signal=TERM','--kill-after=5s','100s','bash',str(ROOT/'scripts/operational_support/pi_refresh.sh')]
        if subprocess.run(refresh,check=False).returncode!=0:raise RuntimeError('PI_NTP_REFRESH_FAILED')
        controller=PiController(bind,start,begin,poll,health,time.monotonic())
        print('PI_PAIRING='+str(a.out/'pairing.json'),flush=True)
        deadline=time.monotonic()+a.max_runtime_s
        while controller.phase!='DONE':
            now=time.monotonic()
            if stop[0] or now>=deadline:controller.command('STOP',now)
            for path,request in mail.requests():
                try:state=controller.command(request['command'],now)
                except (ValueError,RuntimeError) as exc:state={**controller.report(),'request_error':str(exc)}
                mail.reply(path,request,state)
            if local is not None and controller.phase in ('IO_STARTING','RUNNING_OBSERVE'):local.check(time.monotonic())
            atomic(a.out/'state.json',controller.tick(now));time.sleep(.1)
        state=controller.report();atomic(a.out/'DONE.json',state);rc=0 if state['cleanup'] and state['cleanup']['remote_groups_exited'] and state['cleanup']['children_clean'] and not state['cleanup']['forced'] else 2
    except BaseException as exc:
        atomic(a.out/'error.json',dict(error=type(exc).__name__+':'+str(exc)))
    finally:
        try:
            if controller is not None and controller.phase!='DONE':
                controller.command('STOP',time.monotonic())
                while controller.phase!='DONE':controller.tick(time.monotonic());time.sleep(.05)
                state=controller.report();state['error']='PI_RUNNER_FAILED';atomic(a.out/'DONE.json',state)
            elif local is not None and controller is None:local.drain()
        finally:
            for sig,h in previous.items():signal.signal(sig,h)
            fcntl.flock(lockfd,fcntl.LOCK_UN);lockfd.close()
    return rc

if __name__=='__main__':raise SystemExit(main())
