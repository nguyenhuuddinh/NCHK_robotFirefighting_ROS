from types import SimpleNamespace as N
import pytest
from operational_owned_process import OwnedProcess
from operational_logs import OwnedLog,OwnedLogRegistry
from operational_stack import StackCleanup
from test_owned_logs import Fake,line,begin,end,die


def completed(tmp_path,role,text,pid,launch=False):
    class IO(Fake):
        def start(self,argv,output):self.output=output;return N(pid=pid)
    io=IO();p=OwnedProcess(role,backend=io,launch_leader=launch);log=OwnedLog(tmp_path/(role+'.log'))
    log.start(p,['fake'],0);io.output.write(text.encode());io.live=False;p.tick(1)
    return log,p


@pytest.mark.parametrize('child_ok',[True,False])
def test_bound_stack_accepts_only_actual_child_clean_logs_after_operator_proof(tmp_path,child_ok):
    registry=OwnedLogRegistry();oplog,op=completed(tmp_path,'operator',line(),456789)
    ylog,yolo=completed(tmp_path,'yolo',begin()+(end() if child_ok else die()),456790,True)
    registry.register('operator',oplog);registry.register('yolo',ylog)
    stack=StackCleanup.from_logs(registry);stack.register('operator',op);stack.register('yolo',yolo)
    stack.stop(2)
    for i in range(5):r=stack.tick(2+i*.1)
    assert r['done'] and r['stop_confirmed'] and r['clean'] is child_ok
    assert r['ROS_child_acceptance']==('PASS' if child_ok else 'FAIL')
    assert yolo.rc==0  # The failed child must not be hidden by launch leader RC0.
    oplog.close();ylog.close()


def test_registry_rejects_log_of_another_process_and_duplicate_role(tmp_path):
    registry=OwnedLogRegistry();log,p=completed(tmp_path,'operator',line(),456789)
    other,q=completed(tmp_path,'other',line(),456790)
    registry.register('operator',log);s=StackCleanup.from_logs(registry)
    with pytest.raises(ValueError):s.register('operator',q)
    with pytest.raises(ValueError):registry.register('other',log)
    with pytest.raises(ValueError):registry.register('operator',other)
    log.close();other.close()


def test_audit_memory_limit_is_blocked_not_partial_clean(tmp_path):
    log,p=completed(tmp_path,'operator','x'*(1024*1024)+line(),456789)
    with pytest.raises(ValueError,match='AUDIT_LIMIT'):log.receipt()
    log.close()
