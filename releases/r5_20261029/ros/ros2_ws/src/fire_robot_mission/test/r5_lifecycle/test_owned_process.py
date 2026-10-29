"""Fake subprocess I/O only: never start or signal an OS process."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).parents[5]/'scripts'))
from operational_owned_process import OwnedProcess


def member(pid=456789,state='S',birth=12,sid=456789):
    return dict(pid=pid,ppid=1,pgid=456789,sid=sid,state=state,start_ticks=birth)

class Fake:
    def __init__(self):self.rows=[member()];self.sent=[];self.reaped=0;self.rc=0
    def start(self,argv,output):return SimpleNamespace(pid=456789)
    def snapshot(self,pid):return self.rows
    def send(self,pid,sig,leader):self.sent.append((sig.name,leader))
    def reap(self,p):self.reaped+=1;return self.rc


def started(launch=True):
    io=Fake();p=OwnedProcess('fake',backend=io,launch_leader=launch);p.start(['fake-only'],None,0);return p,io


def test_launch_receives_one_SIGINT_then_waits_for_descendants_and_preserves_exit():
    p,io=started();p.stop(0);p.stop(1);assert io.sent==[('SIGINT',True)]
    io.rows=[member(state='Z'),member(pid=456790,birth=14)]
    p.tick(2);assert not p.done and io.reaped==0
    io.rows=[member(state='Z')];io.rc=-2;r=p.tick(3)
    assert r['group_exit_confirmed'] and r['leader_rc']==-2 and not r['forced'] and io.reaped==1
    assert r['ROS_child_acceptance']=='NOT_EVALUATED' and r['robot_stop_proof']=='NOT_EVALUATED'


def test_leader_gone_initially_signals_group_once_not_two_SIGINTs():
    p,io=started();io.rows=[member(pid=456790,birth=14),member(state='Z')]
    p.tick(.1);io.rows=[member(pid=456790,birth=14)];p.stop(.2);p.stop(.3)
    assert io.sent==[('SIGINT',False)]


def test_nonlaunch_initial_SIGINT_group_and_bounded_fallback_not_clean():
    p,io=started(False);p.stop(0);p.tick(7.9);p.tick(8);p.tick(11);r=p.tick(13)
    assert io.sent==[('SIGINT',False),('SIGTERM',False),('SIGKILL',False)]
    assert r['done'] and r['forced'] and not r['group_exit_confirmed'] and r['live_pids']


def test_forced_exit_stays_forced_even_with_launch_zero():
    p,io=started();p.stop(0);p.tick(8);io.rows=[member(state='Z')];r=p.tick(8.1)
    assert r['group_exit_confirmed'] and r['leader_rc']==0 and r['forced']


@pytest.mark.parametrize('fault',['sid','reused','unknown','clock'])
def test_bad_ownership_or_clock_never_signals(fault):
    p,io=started()
    if fault=='sid':io.rows=[member(sid=777)]
    if fault=='reused':io.rows=[member(birth=99)]
    if fault=='unknown':io.rows=[member(pid=456790,birth=99)]
    r=p.stop(-1 if fault=='clock' else 1)
    assert r['done'] and r['error'] and not r['group_exit_confirmed'] and not io.sent


def test_exited_before_signal_reaps_actual_rc_and_idempotent_stop():
    p,io=started();io.rows=[];io.rc=7;r=p.stop(1);p.stop(2)
    assert r['group_exit_confirmed'] and r['leader_rc']==7 and not io.sent and io.reaped==1


def test_unstructured_argv_and_double_start_rejected_without_extra_process():
    io=Fake();p=OwnedProcess('fake',backend=io)
    with pytest.raises(ValueError):p.start('shell string',None,0)
    p.start(['fake'],None,0)
    with pytest.raises(ValueError):p.start(['fake'],None,1)
