"""Session-bound Pi file mailbox. Invoked by explicit operator/SSH commands only.

Local SSH exit is never remote process proof. STOP proof is the pinned Pi
runner's final group/child receipt. No clock, ROS, device or network on import.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid
from operational_entry import atomic
from operational_logs import strict_json

class PeerTransportError(RuntimeError):pass

COMMANDS=frozenset(('STATE','START','RETRY_PREARM','SEAL','STOP'))

def binding(root,packet):
    stat=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()
    return dict(session=str(uuid.uuid4()),secret=uuid.uuid4().hex,packet=packet,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        runner_pid=os.getpid(),runner_start_ticks=int(stat[19]),root=str(Path(root).resolve()))

def same_binding(a,b):return isinstance(a,dict) and all(a.get(k)==b.get(k) for k in ('session','secret','packet','boot_id','runner_pid','runner_start_ticks','root'))

class Mailbox:
    def __init__(self,root,bind):
        self.root=Path(root);self.bind=bind
        if self.root.is_symlink() or self.root.stat().st_uid!=os.getuid() or self.root.stat().st_mode&0o077:
            raise ValueError('PRIVATE_PI_SESSION_REQUIRED')
    def requests(self):
        for p in sorted((self.root/'requests').glob('*.json'))[:16]:
            try:
                if p.stat().st_size>8192:raise ValueError('REQUEST_TOO_BIG')
                value=strict_json(p.read_text())
                if type(value.get('expires_mono')) not in (int,float) or not math.isfinite(value['expires_mono']) or not time.monotonic()<value['expires_mono']:raise ValueError('PEER_REQUEST_EXPIRED')
                if not same_binding(value.get('binding'),self.bind) or value.get('command') not in COMMANDS:raise ValueError('FOREIGN_PEER_REQUEST')
                request=value.get('request');assert str(uuid.UUID(request))==request
                yield p,value
            except (OSError,ValueError,TypeError,AssertionError):p.rename(p.with_suffix('.rejected'))
    def reply(self,path,request,state):
        atomic(self.root/'responses'/path.name,dict(request=request['request'],binding=self.bind,state=state))
        path.unlink()


def call(root,bind,command,*,timeout=6.,clock=time.monotonic,sleep=time.sleep):
    if command not in COMMANDS:raise ValueError('UNKNOWN_PEER_COMMAND')
    root=Path(root);mail=Mailbox(root,bind)
    actual=strict_json((root/'pairing.json').read_text())
    if not same_binding(actual,bind):raise ValueError('PI_SESSION_REPLACED')
    request=str(uuid.uuid4());path=root/'requests'/(request+'.json')
    response=root/'responses'/path.name
    # Completed proof is accepted only for an already ended session, never as
    # fresh startup/BOOT evidence. The binding is exact to this session.
    if command=='STATE' and (root/'DONE.json').exists():
        return dict(request=request,binding=bind,state=strict_json((root/'DONE.json').read_text()))
    temp=path.with_suffix('.pending')
    fd=os.open(temp,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as f:json.dump(dict(binding=bind,request=request,command=command,expires_mono=clock()+timeout),f)
    os.replace(temp,path)
    deadline=clock()+timeout
    while clock()<deadline:
        if response.exists():
            result=strict_json(response.read_text());response.unlink()
            if result.get('request')!=request or not same_binding(result.get('binding'),bind):raise ValueError('STALE_OR_FOREIGN_PEER_REPLY')
            return result
        sleep(.05)
    # No late START/RETRY request may become authority after caller timed out.
    try:path.unlink()
    except FileNotFoundError:pass
    raise TimeoutError('PI_MAILBOX_TIMEOUT:'+command)


class SshPi:
    def __init__(self,bind,remote_script,*,run=subprocess.run):
        self.bind=bind;self.remote_script=str(remote_script);self.run=run;self.sealed=False
    def request(self,command):
        payload=json.dumps(self.bind,separators=(',',':'))
        remote=shlex.join(['python3',self.remote_script,'--root',self.bind['root'],'--binding',payload,'--command',command])
        try:
            result=self.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=3','-o','ServerAliveInterval=2',
                             '-o','ServerAliveCountMax=2','pi@10.0.0.1',remote],text=True,capture_output=True,timeout=9,check=False)
        except (OSError,subprocess.SubprocessError) as exc:raise PeerTransportError('PI_CONTROL_TRANSPORT:'+str(exc)) from exc
        if result.returncode!=0:
            if result.returncode==255 or 'PI_MAILBOX_TIMEOUT:' in getattr(result,'stderr',''):
                raise PeerTransportError('PI_CONTROL_TRANSPORT:'+command)
            raise RuntimeError('PI_CONTROL_SSH_FAILED:'+command)
        reply=strict_json(result.stdout)
        if not same_binding(reply.get('binding'),self.bind):raise ValueError('PI_CONTROL_BINDING_CHANGED')
        state=reply.get('state')
        if not isinstance(state,dict) or state.get('session')!=self.bind['session']:raise ValueError('PI_CONTROL_STATE_INVALID')
        if state.get('request_error'):raise RuntimeError('PI_REQUEST_REJECTED:'+str(state['request_error']))
        if state.get('error') and state.get('phase')!='DONE':raise RuntimeError('PI_RUNNER_ERROR:'+str(state['error']))
        return state
    def startup(self,command,*,sleep=time.sleep):
        """Bounded startup transport retry; never replay a lost mutation."""
        if command=='STATE':
            last=None
            for attempt in range(3):
                try:return self.request('STATE')
                except PeerTransportError as exc:
                    last=exc
                    if attempt<2:sleep(1.)
            raise last
        try:return self.request(command)
        except PeerTransportError:
            state=self.startup('STATE',sleep=sleep)
            applied=((command=='START' and state.get('phase')=='IO_STARTING' and state.get('retries')==0)
                or (command=='RETRY_PREARM' and state.get('phase') in ('RETRY_STOPPING','IO_STARTING') and state.get('retries')==1)
                or (command=='SEAL' and ((state.get('phase')=='RUNNING_OBSERVE' and state.get('sealed') is True)
                    or (state.get('phase')=='IO_STARTING' and state.get('seal_requested') is True and state.get('sealed') is False))))
            if not applied:raise PeerTransportError('PI_MUTATION_ACK_LOST_NOT_PROVEN:'+command)
            return state

    def stop(self,*,clock=time.monotonic,sleep=time.sleep):
        deadline=clock()+25.;errors=[];result=None;sent=False
        while clock()<deadline:
            try:
                state=self.request('STATE')
                if state.get('phase')=='DONE':
                    result=state;break
                if not sent:self.request('STOP');sent=True
            except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:errors.append(str(exc))
            sleep(min(1.,max(0.,deadline-clock())))
        receipt=None if result is None else result.get('cleanup')
        confirmed=(result is not None and not result.get('error') and isinstance(receipt,dict) and receipt.get('remote_groups_exited') is True
            and receipt.get('children_clean') is True and receipt.get('forced') is False)
        return dict(confirmed=confirmed,scope='PINNED_PI_RUNNER_REMOTE_GROUPS_AND_CHILD_LOGS',
            binding={k:v for k,v in self.bind.items() if k!='secret'},receipt=receipt,errors=errors,runner_error=None if result is None else result.get('error'),
            reason='REMOTE_EXIT_PROVEN' if confirmed else 'REMOTE_EXIT_UNCONFIRMED',local_SSH_exit_is_proof=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--binding',required=True);p.add_argument('--command',choices=sorted(COMMANDS),required=True)
    a=p.parse_args(argv)
    try:reply=call(a.root,strict_json(a.binding),a.command);print(json.dumps(reply));return 0
    except (OSError,ValueError,RuntimeError,TimeoutError) as exc:print(str(exc),file=sys.stderr);return 2

if __name__=='__main__':raise SystemExit(main())
