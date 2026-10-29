import json
from types import SimpleNamespace as N
import pytest
from operational_entry import LocalProcesses


def proof():return dict(scope='R5_STOP_TELEMETRY_VERIFICATION',requested=True,done=True,confirmed=True,
    reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED',blockers=[],fault=None,motion_authorized=False,pulse_replayed=False)

class Backend:
    def __init__(self,role,pid,events,now):self.role=role;self.pid=pid;self.events=events;self.now=now;self.stop_at=None;self.output=None;self.rc=0
    def start(self,argv,out):
        self.output=out
        if self.role in ('mission','nav'):out.write(f'[INFO] [{self.role}-1]: process started with pid [{self.pid+100}]\n'.encode())
        return N(pid=self.pid)
    def snapshot(self,pid):
        exited=self.stop_at is not None and self.now[0]-self.stop_at>=(.5 if self.role=='operator' else .1)
        return [dict(pid=pid,ppid=1,pgid=pid,sid=pid,state='Z' if exited else 'S',start_ticks=10)]
    def send(self,pid,sig,leader):
        self.events.append((self.role,sig.name,leader,self.now[0]));self.stop_at=self.now[0]
        if self.role=='operator':self.output.write(('[INFO] [123.456] [fire_robot_operator]: OPERATOR_STOP '+json.dumps(proof())+'\n').encode())
        if self.role in ('mission','nav'):self.output.write(f'[INFO] [{self.role}-1]: process has finished cleanly [pid {self.pid+100}]\n'.encode())
    def reap(self,p):return self.rc


def test_real_local_registry_ownership_and_operator_drain_before_all_other_signals(tmp_path,monkeypatch):
    events=[];now=[1.];backends={}
    monkeypatch.setattr('operational_owned_process.time.monotonic',lambda:now[0])
    def factory(role):
        io=Backend(role,456789+len(backends),events,now);backends[role]=io;return io
    local=LocalProcesses(tmp_path,'session',backend_factory=factory)
    for role in ('operator','mission','nav','startup_health'):local.start(role,['fake'],launch_leader=role in ('mission','nav'))
    local.check(1.)
    r=local.drain(clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert r['clean'] and r['stop_confirmed']
    assert events[0][:2]==('operator','SIGINT')
    assert all(at>=1.5 for role,sig,leader,at in events if role!='operator')
    assert all(sig=='SIGINT' for role,sig,leader,at in events)
    local.close_logs()


def test_normal_operator_Q_exit_is_not_runtime_crash(tmp_path,monkeypatch):
    now=[1.];io=Backend('operator',456789,[],now)
    monkeypatch.setattr('operational_owned_process.time.monotonic',lambda:now[0])
    local=LocalProcesses(tmp_path,'session',backend_factory=lambda role:io);local.start('operator',['fake'])
    io.stop_at=1.;now[0]=2.
    assert local.check(2.,allow_operator_exit=True) is True
    local.close_logs()


def test_unexpected_child_exit_detected_before_start_next_stage(tmp_path,monkeypatch):
    now=[1.];io=Backend('mission',456789,[],now)
    monkeypatch.setattr('operational_owned_process.time.monotonic',lambda:now[0])
    local=LocalProcesses(tmp_path,'session',backend_factory=lambda role:io);local.start('mission',['fake'],launch_leader=True)
    io.stop_at=1.;now[0]=2.
    with pytest.raises(RuntimeError,match='OWNED_CHILD_EXIT:mission'):local.check(2.)
    local.close_logs()
