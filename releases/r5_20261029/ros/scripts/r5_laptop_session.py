#!/usr/bin/env python3
"""R5 paired entrypoint. Default is offline; Gate2 needs an explicit token."""
import argparse
import fcntl
import json
from pathlib import Path
import signal
import subprocess
import time
import uuid
from operational_entry import ROOT,LocalProcesses,StartupSequence,atomic,packet_digest,read_health,ros,launch,run_clock_gate,startup_observations
from operational_peer import SshPi
from operational_logs import strict_json
from operational_preflight import preflight,validate_gate2_profile

GATE2_TOKEN='R5_GATE2_SUPERVISED_LIVE'
AUTOMATIC_TOKEN='R5_AUTOMATIC_FIRE_LIVE'


def commands(map_path,rviz,*,gate2=False,automatic=False,auto_start=None,auto_session=None,auto_packet=None,recovery=False,recovery_after_pulse=False):
    if recovery and not automatic:raise ValueError('RECOVERY_REQUIRES_AUTOMATIC_PROFILE')
    if automatic and (not gate2 or None in (auto_start,auto_session,auto_packet)):
        raise ValueError('AUTOMATIC_COMMAND_BINDING_REQUIRED')
    result=dict(nav=launch('fire_robot_navigation','nav2_composed_shadow.launch.py',
        'map:='+str(map_path),'rviz:=false','use_sim_time:=false','refresh_bt_action_status:=true'),
        rviz=['rviz2','-d',str(rviz)],yolo=launch('fire_robot_perception','perception.launch.py','use_sim_time:=false'),
        mission=launch('fire_robot_mission','operational_stack_gate2.launch.py' if gate2 else 'operational_stack_observe.launch.py'),
        operator=ros('fire_robot_operator','--ros-args','-p','show_gui:=true','-p',
            'enable_live_intents:='+str(gate2).lower(),'-p','allow_close_stationary:='+str(gate2).lower()))
    if gate2:
        result['mission']+=['external_ir_preview:=true']
        config=ROOT/'ros2_ws/src/fire_robot_mission/config'
        result['ir_monitor']=ros('ir_range_preview','--ros-args','--params-file',str(config/'ir_range_preview.yaml'),
            '--params-file',str(config/'operational_gate2.yaml'),'-p','use_sim_time:=false')
        result['operator']+=['--params-file',str(ROOT/'ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml')]
    if automatic:
        result['operator']+=['-p','automatic_mode:=true','-p','automatic_start_file:='+str(auto_start),
            '-p','automatic_runner_session:='+auto_session,'-p','automatic_packet:='+auto_packet]
    if recovery:result['operator']+=['-p','require_fire_clear_on_start:=true',
        '-p','recovery_require_ir_clear:='+str(recovery_after_pulse is True).lower()]
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);mode=p.add_mutually_exclusive_group()
    mode.add_argument('--run-observe',action='store_true');mode.add_argument('--run-gate2',action='store_true')
    mode.add_argument('--run-automatic',action='store_true')
    p.add_argument('--automatic-token')
    p.add_argument('--gate2-token')
    p.add_argument('--pairing',type=Path);p.add_argument('--remote-script');p.add_argument('--out',type=Path)
    p.add_argument('--map',type=Path,default=ROOT/'ros2_ws/src/fire_robot_navigation/maps/fire_room_9_10_2026.yaml')
    p.add_argument('--rviz',type=Path,default=ROOT/'ros2_ws/src/fire_robot_mission/config/operational_navigation.rviz')
    p.add_argument('--max-runtime-s',type=int,default=28800);a=p.parse_args(argv)
    automatic=a.run_automatic is True
    gate2=a.run_gate2 is True or automatic
    if automatic and a.automatic_token!=AUTOMATIC_TOKEN:p.error('--run-automatic requires exact --automatic-token '+AUTOMATIC_TOKEN)
    if not automatic and a.automatic_token is not None:p.error('--automatic-token is only valid with --run-automatic')
    if automatic and a.gate2_token is not None:p.error('automatic profile does not use --gate2-token')
    if a.run_gate2 and a.gate2_token!=GATE2_TOKEN:p.error('--run-gate2 requires exact --gate2-token '+GATE2_TOKEN)
    if not a.run_gate2 and a.gate2_token is not None:p.error('--gate2-token is only valid with --run-gate2')
    argv_by_role=commands(a.map,a.rviz,gate2=gate2)
    if not a.run_observe and not gate2:
        print(json.dumps(dict(mode='PLAN_ONLY_LIVE_FALSE',gate2_token_required=GATE2_TOKEN,
            order=['CLOCK','PI_IO','CAMERA','POSE','YOLO','CONSUMERS','OPERATOR'],commands=argv_by_role)));return 0
    if any(x is None for x in (a.out,a.pairing,a.remote_script)) or not 60<=a.max_runtime_s<=28800:p.error('pairing/remote-script/out and runtime60..28800 required')
    a.out.mkdir(mode=0o700)
    try:
        if not a.map.is_file() or not a.rviz.is_file():raise ValueError('MAP_OR_RVIZ_NOT_FOUND')
        packet=packet_digest();preflight(ROOT,'laptop')
        if gate2:validate_gate2_profile(ROOT)
        bind=strict_json(a.pairing.read_text())
        if bind.get('packet')!=packet:raise ValueError('PAIRED_PACKET_MISMATCH')
    except Exception as exc:
        atomic(a.out/'failure.json',dict(error='LAPTOP_PREFLIGHT:'+type(exc).__name__+':'+str(exc),hardware_started=False));return 2
    peer=SshPi(bind,a.remote_script);session=str(uuid.uuid4())
    auto_start=a.out.resolve()/'automatic_start.json'
    if automatic:
        argv_by_role=commands(a.map,a.rviz,gate2=True,automatic=True,
            auto_start=auto_start,auto_session=session,auto_packet=packet)
    lock=Path.home()/'.cache/fire_robot_r5_laptop.lock';lock.parent.mkdir(exist_ok=True)
    lockfd=lock.open('a');fcntl.flock(lockfd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    local=LocalProcesses(a.out,session);stop=[False];previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    for sig in previous:signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    state=None;rc=2;remote_started=False;clock_child=None;recovery_runner=None
    def start_health():
        try:(a.out/'health.json').unlink()
        except FileNotFoundError:pass
        local.start('startup_health',ros('r5_startup_health','--out',a.out/'health.json','--session',session,'--role','laptop'))
    def halt_health():
        # Camera retry occurs before Nav2/operator, and the old observer must
        # release its GID pins before the new Pi connection is considered.
        proc=local.processes['startup_health'];proc.stop(time.monotonic())
        while not proc.done:proc.stop(time.monotonic());time.sleep(.05)
        old=local.registry.logs.pop('startup_health');audit=proc.report();old.close()
        local.processes.pop('startup_health');atomic(a.out/'health_retry_exit.json',audit)
        if not audit['group_exit_confirmed'] or audit['forced'] or audit['leader_rc']!=0:raise RuntimeError('HEALTH_RETRY_EXIT_UNCLEAN')
        (a.out/'startup_health.log').rename(a.out/'startup_health_before_retry.log')
    try:
        initial=peer.startup('STATE')
        if initial.get('phase')!='WAIT_LAPTOP':raise ValueError('PI_NOT_WAITING_FOR_THIS_LAPTOP')
        # Accepted read-only pair gate. No ROS/Pi I/O has started yet.
        run_clock_gate(a.out,lambda:stop[0])
        state=StartupSequence(time.monotonic(),live=gate2);obs={'clock_ok':True};peer_next=0.;pi_state=initial;notice_at=0.
        while not state.done and not stop[0]:
            now=time.monotonic()
            if remote_started and state.stage in ('PI_IO','CAMERA','RETRY_PI','SEAL_PI') and now>=peer_next:
                pi_state=peer.startup('STATE');peer_next=time.monotonic()+2.
                if state.stage=='SEAL_PI':atomic(a.out/'pi_seal.json',pi_state)
                if pi_state.get('phase')=='DONE':raise RuntimeError('PI_ENDED_DURING_STARTUP')
            if local.processes:local.check(time.monotonic())
            try:
                health=read_health(a.out/'health.json',session,time.monotonic);checks=health['checks'];stable=health.get('stable_s',{})
                obs.update(startup_observations(health))
                obs['identity_key']=health.get('identity_key')
            except (OSError,ValueError,KeyError,TypeError):
                health={};checks={};stable={};obs.update(io=False,camera=False,pose=False,yolo=False,
                    operator_present=False,identity_key=None)
            obs['pi_sealed']=(pi_state.get('phase')=='RUNNING_OBSERVE' and pi_state.get('sealed') is True
                              and pi_state.get('health',{}).get('io') is True)
            obs['retry_ready']=pi_state.get('phase')=='IO_STARTING' and pi_state.get('retries')==1
            obs['operator_present']=('operator' in local.processes and obs.get('operator_present') is True)
            effects=state.step(time.monotonic(),obs)
            # Commit the decision before executing STOP or any fallible effect.
            atomic(a.out/'startup.json',state.diagnostics())
            for effect in effects:
                if effect=='START_PI':peer.startup('START');remote_started=True
                elif effect=='START_HEALTH':start_health()
                elif effect=='RETRY_PI':peer.startup('RETRY_PREARM')
                elif effect=='RESTART_HEALTH':halt_health();start_health();obs.update(io=False,camera=False)
                elif effect=='SEAL_PI':
                    pi_state=peer.startup('SEAL');peer_next=time.monotonic()+.5
                    atomic(a.out/'pi_seal.json',pi_state)
                elif effect in ('START_NAV','START_RVIZ','START_YOLO','START_CONSUMERS','START_OPERATOR'):
                    role={'START_NAV':'nav','START_RVIZ':'rviz','START_YOLO':'yolo','START_CONSUMERS':'mission','START_OPERATOR':'operator'}[effect]
                    if role=='mission' and gate2 and 'ir_monitor' not in local.processes:
                        local.start('ir_monitor',argv_by_role['ir_monitor'],launch_leader=False)
                    local.start(role,argv_by_role[role],launch_leader=role in ('nav','yolo','mission'))
                    if role=='rviz':print('WAIT_INITIAL_POSE: set 2D Pose Estimate; do not arm/set goal before the ready line.',flush=True)
                elif effect=='STOP':raise RuntimeError(state.error)
            if time.monotonic()>=notice_at:
                required=('io','camera','pose','yolo')+(('operator_present',) if state.stage=='CONSUMERS' else ())
                operator_diag=health.get('operator_diagnostics',{})
                print('R5_STARTUP_STAGE='+state.stage+' missing='+','.join(k for k in required if not obs.get(k))+
                      ' remaining_s='+str(round(state.diagnostics().get('remaining_s') or 0.,1))+
                      ' localization_stable_s='+str(round(stable.get('localization',0),3))+
                      ' io_blockers='+','.join(health.get('io_diagnostics',{}).get('blockers',[]))+
                      ' io_stable_s='+str(round(stable.get('io',0),3))+
                      ' operator_reason='+str(operator_diag.get('reason'))+
                      ' operator_blockers='+','.join(operator_diag.get('blockers',[])),flush=True)
                notice_at=time.monotonic()+5.
            time.sleep(.1)
        if stop[0]:raise InterruptedError('OPERATOR_STOP_DURING_STARTUP')
        if state.error:raise RuntimeError(state.error)
        if automatic:
            atomic(auto_start,dict(schema_version=1,kind='R5_AUTOMATIC_START',session=session,
                packet=packet,stage='READY_GATE2',ready=True))
            print('R5_AUTOMATIC_READY: wait AUTO armed/goal-ready in camera, then set goal. Fire handling is automatic; X cancel, Q STOP.',flush=True)
        elif gate2:
            print('R5_GATE2_READY: live profile loaded; A remains blocked until READY_PRESS_A. No command is automatic.',flush=True)
        else:print('R5_OBSERVE_READY: all live/protocol flags false; this does not accept motion/pump runtime.',flush=True)
        if automatic:
            from runtime_recovery_runner import RuntimeRecoveryRunner
            recovery_runner=RuntimeRecoveryRunner(local,a.out,session,packet,
                lambda path,nonce,after_pulse:commands(a.map,a.rviz,gate2=True,automatic=True,
                    auto_start=path,auto_session=nonce,auto_packet=packet,recovery=True,recovery_after_pulse=after_pulse),auto_start)
        deadline=time.monotonic()+a.max_runtime_s
        # No recurring SSH health probes while running. ROS freshness/identity/
        # independent Pi watchdogs continue to enforce their original policies.
        while not stop[0] and time.monotonic()<deadline:
            if recovery_runner is not None:recovery_runner.step(time.monotonic(),stop=stop[0])
            excluded=recovery_runner.excluded if recovery_runner is not None else ()
            if local.check(time.monotonic(),allow_operator_exit=True,exclude=excluded):break
            time.sleep(.1)
        rc=0
    except BaseException as exc:
        if state is not None:
            state.done=True;state.error=state.error or type(exc).__name__+':'+str(exc)
            atomic(a.out/'startup.json',state.diagnostics())
        # Preserve pre-drain proof separately; the health child may update its
        # last snapshot while responding to SIGINT during cleanup.
        try:atomic(a.out/'health_at_failure.json',read_health(a.out/'health.json',session,time.monotonic))
        except (OSError,ValueError,KeyError,TypeError) as health_error:
            atomic(a.out/'health_at_failure.json',dict(valid=False,error=type(health_error).__name__+':'+str(health_error)))
        atomic(a.out/'failure.json',dict(error=type(exc).__name__+':'+str(exc),startup=None if state is None else state.report()))
    finally:
        try:
            try:local_result=local.drain(cause='LAPTOP_FINALLY') if local.processes else dict(clean=False,reason='NO_LOCAL_PROCESSES')
            except Exception as exc:local_result=dict(clean=False,error='LOCAL_CLEANUP_FAILED:'+str(exc))
            # Pi must outlive operator STOP drain, owner release and local
            # child audits. Only a Pi-generated final receipt can prove exit.
            remote_result=peer.stop();atomic(a.out/'remote_cleanup.json',remote_result)
            if not local_result.get('clean') or not remote_result['confirmed']:rc=2
            atomic(a.out/'receipt.json',dict(rc=rc,local=local_result,remote=remote_result,
                runtime_recovery=recovery_runner.core.report() if recovery_runner is not None else None,
                retired_generations=[row for row in local.history if row.get('event')=='RETIRED'],
                profile='R5_AUTOMATIC' if automatic else 'R5_GATE2_SUPERVISED' if gate2 else 'R5_OBSERVE_ONLY',live=gate2))
        finally:
            local.close_logs()
            for sig,h in previous.items():signal.signal(sig,h)
            fcntl.flock(lockfd,fcntl.LOCK_UN);lockfd.close()
    return rc

if __name__=='__main__':raise SystemExit(main())
