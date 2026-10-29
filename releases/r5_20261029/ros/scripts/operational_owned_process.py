"""One explicitly started setsid process group, bounded cleanup, no ROS/device I/O.

Importing this module starts nothing. Keep the group leader unreaped until all
live group members are gone, so its PID/SID cannot be reused during cleanup.
A group exit is separate from ROS child-log acceptance and robot STOP proof.
"""
import math
import os
import signal
import subprocess
import time
from operational_support.cleanup_group import inspect


class Backend:
    def start(self,argv,output):
        return subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=output,
                                stderr=subprocess.STDOUT,start_new_session=True)
    def snapshot(self,pid):return inspect(pid)[0]
    def send(self,pid,sig,leader_only):
        try:
            if leader_only:os.kill(pid,sig)
            else:os.killpg(pid,sig)
        except ProcessLookupError:pass  # Reinspect next tick, never declare exit here.
    def reap(self,process):return process.wait(timeout=0)


class OwnedProcess:
    def __init__(self,label,*,launch_leader=False,backend=None,int_grace=8.,term_grace=3.,kill_grace=2.):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
               for v in (int_grace,term_grace,kill_grace)):raise ValueError('INVALID_CLEANUP_BUDGET')
        self.label=label;self.launch_leader=launch_leader;self.io=backend or Backend()
        self.budgets=(int_grace,term_grace,kill_grace);self.process=None;self.pid=None
        self.known=set();self.leader_birth=None;self.last=None;self.deadline=None
        self.phase='NOT_STARTED';self.done=False;self.forced=False;self.rc=None;self.error=None
        self.signals=[];self.live=[]

    def start(self,argv,output,now=None):
        if self.process is not None:raise ValueError('PROCESS_ALREADY_STARTED')
        if not isinstance(argv,(list,tuple)) or not argv or not all(isinstance(x,str) and x for x in argv):
            raise ValueError('STRUCTURED_ARGV_REQUIRED')
        self.process=self.io.start(list(argv),output);self.pid=self.process.pid
        if type(self.pid) is not int or self.pid<=1 or self.pid==os.getpgrp():raise ValueError('INVALID_OWNED_PID')
        self.phase='RUNNING';self.last=time.monotonic() if now is None else now
        members=self._snapshot()
        leader=next((m for m in members if m['pid']==self.pid),None)
        if leader is None:raise ValueError('OWNED_LEADER_NOT_PROVEN')
        self.leader_birth=leader['start_ticks'];return self.report()

    def _snapshot(self):
        members=self.io.snapshot(self.pid)
        if any(m['pgid']!=self.pid or m['sid']!=self.pid for m in members):raise ValueError('OWNERSHIP_SID_CHANGED')
        identities={(m['pid'],m['start_ticks']) for m in members}
        leader=next((m for m in members if m['pid']==self.pid),None)
        if self.leader_birth is not None:
            if leader is not None and leader['start_ticks']!=self.leader_birth:raise ValueError('LEADER_PID_REUSED')
            if identities and not identities.intersection(self.known):raise ValueError('OWNERSHIP_NOT_PROVEN')
        self.known.update(identities);self.live=[m for m in members if m['state'] not in ('Z','X')]
        return members

    def stop(self,now):
        if self.phase=='RUNNING':self.phase='REQUEST_STOP'
        return self.tick(now)

    def tick(self,now):
        if self.done or self.process is None:return self.report()
        if type(now) not in (int,float) or not math.isfinite(now) or now<self.last:
            self.done=True;self.error='CLEANUP_CLOCK_INVALID';return self.report()
        self.last=now
        try:
            self._snapshot()
            if not self.live:
                self.rc=self.io.reap(self.process);self.done=True;self.phase='GROUP_EXITED';return self.report()
            if self.phase=='RUNNING':return self.report()
            stages={'REQUEST_STOP':(signal.SIGINT,0,'WAIT_INT'),
                    'WAIT_INT':(signal.SIGTERM,1,'WAIT_TERM'),
                    'WAIT_TERM':(signal.SIGKILL,2,'WAIT_KILL')}
            if self.phase!='REQUEST_STOP' and now<self.deadline:return self.report()
            if self.phase=='WAIT_KILL':
                self.done=True;self.error='GROUP_STILL_LIVE_AFTER_KILL';return self.report()
            sig,index,next_phase=stages[self.phase]
            leader_only=sig==signal.SIGINT and self.launch_leader and any(m['pid']==self.pid for m in self.live)
            self.io.send(self.pid,sig,leader_only)
            self.signals.append(dict(signal=sig.name,scope='LEADER' if leader_only else 'GROUP',at=now))
            self.forced|=sig!=signal.SIGINT;self.phase=next_phase;self.deadline=now+self.budgets[index]
        except (OSError,ValueError,KeyError,subprocess.TimeoutExpired) as exc:
            self.done=True;self.error=type(exc).__name__+':'+str(exc)
        return self.report()

    def report(self):
        return dict(label=self.label,pid=self.pid,phase=self.phase,done=self.done,
                    forced=self.forced,leader_rc=self.rc,error=self.error,
                    live_pids=[m['pid'] for m in self.live],signals=list(self.signals),
                    group_exit_confirmed=self.done and self.phase=='GROUP_EXITED' and self.error is None,
                    ROS_child_acceptance='NOT_EVALUATED',robot_stop_proof='NOT_EVALUATED')
