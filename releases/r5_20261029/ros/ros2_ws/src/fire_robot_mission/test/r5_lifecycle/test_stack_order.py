"""Real OwnedProcess and stack controller, fake processes/receipts; no OS I/O."""
from types import SimpleNamespace as N
import pytest
from operational_owned_process import OwnedProcess
from operational_stack import StackCleanup


def receipt():
    return dict(scope='R5_STOP_TELEMETRY_VERIFICATION',requested=True,done=True,confirmed=True,
                reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED',blockers=[],fault=None,
                motion_authorized=False,pulse_replayed=False)

class Fake:
    def __init__(self,pid):self.pid=pid;self.live=True;self.sent=[];self.rc=0
    def start(self,argv,output):return N(pid=self.pid)
    def snapshot(self,pid):
        return [dict(pid=pid,ppid=1,pgid=pid,sid=pid,state='S' if self.live else 'Z',start_ticks=10)]
    def send(self,pid,sig,leader):self.sent.append(sig.name)
    def reap(self,p):return self.rc


def stack(reader=receipt):
    s=StackCleanup(reader);ios={}
    for i,role in enumerate(('operator','actor','alignment','ir_live','mission','guard','gateway','review','yolo','nav','rviz','pi')):
        io=Fake(456789+i);p=OwnedProcess(role,backend=io);p.start(['fake'],None,0)
        s.register(role,p);ios[role]=io
    return s,ios


def finish(s,ios,now=1):
    for io in ios.values():io.live=False
    for i in range(6):s.tick(now+i*.1)
    return s.report()


def test_actor_and_pi_survive_operator_proof_then_owners_then_nav_then_pi():
    s,ios=stack();s.stop(0);s.stop(1)
    assert ios['operator'].sent==['SIGINT'] and not any(io.sent for r,io in ios.items() if r!='operator')
    ios['operator'].live=False;s.tick(1.1);assert s.stop_confirmed and not ios['actor'].sent
    s.tick(1.2);assert ios['actor'].sent==['SIGINT'] and not ios['nav'].sent and not ios['pi'].sent
    for r in s.GROUPS[1]:ios[r].live=False
    s.tick(1.3);s.tick(1.4);assert ios['nav'].sent==['SIGINT'] and not ios['pi'].sent
    for r in s.GROUPS[2]:ios[r].live=False
    s.tick(1.5);s.tick(1.6);assert ios['pi'].sent==['SIGINT']
    ios['pi'].live=False;r=s.tick(1.7)
    assert r['done'] and r['group_cleanup_clean'] and not r['clean'] and r['ROS_child_acceptance']=='PENDING_CHILD_LOG_AUDIT'


@pytest.mark.parametrize('bad',['missing','false','wrong_scope','pulse_replay'])
def test_missing_or_bad_stop_proof_never_clean_but_cleanup_continues(bad):
    r=receipt()
    if bad=='missing':r=None
    if bad=='false':r['confirmed']=False
    if bad=='wrong_scope':r['scope']='SAVED_OLD_RECEIPT'
    if bad=='pulse_replay':r['pulse_replayed']=True
    s,ios=stack(lambda:r);s.stop(0);out=finish(s,ios)
    assert out['done'] and not out['group_cleanup_clean'] and 'OPERATOR_STOP_UNCONFIRMED' in out['errors']


def test_operator_forced_deadline_does_not_hang_and_pi_stays_until_last_stage():
    s,ios=stack();s.stop(0);s.stop(8);s.stop(11);s.stop(13)
    assert s.processes['operator'].done and not s.stop_confirmed and not ios['pi'].sent
    r=finish(s,ios,14);assert r['done'] and not r['group_cleanup_clean'] and 'PROCESS_UNCLEAN:operator' in r['errors']


def test_operator_nonzero_does_not_accept_even_good_receipt():
    s,ios=stack();ios['operator'].rc=2;s.stop(0);r=finish(s,ios)
    assert not r['group_cleanup_clean'] and not r['stop_confirmed']


def test_missing_operator_not_a_confirmed_shutdown():
    s,ios=stack();del s.processes['operator'];s.stop(0);r=finish(s,ios)
    assert r['done'] and not r['group_cleanup_clean'] and not r['stop_confirmed']


def test_registration_rejects_duplicate_role_pid_and_after_cleanup():
    s,ios=stack()
    with pytest.raises(ValueError):s.register('actor',s.processes['operator'])
    del s.processes['pi']
    with pytest.raises(ValueError):s.register('pi',s.processes['operator'])
    s.stop(0)
    with pytest.raises(ValueError):s.register('pi',s.processes['operator'])


def test_clock_discontinuity_reports_failure_and_never_claims_clean():
    s,ios=stack();s.stop(1);r=s.tick(0)
    assert r['done'] and not r['group_cleanup_clean'] and 'STACK_CLOCK_INVALID' in r['errors']
