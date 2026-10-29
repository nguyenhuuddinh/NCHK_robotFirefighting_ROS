import json
from pathlib import Path
from types import SimpleNamespace as N
import pytest
from operational_entry import run_clock_gate

class Backend:
    def __init__(self,out,now,slow=False,rc=0):self.out=out;self.now=now;self.slow=slow;self.rc=rc;self.signal_at=None;self.signals=[]
    def start(self,args,output):
        assert args[-1]=='10.0.0.253';(self.out/'clock').mkdir()
        (self.out/'clock/settle.json').write_text(json.dumps(dict(status='CLOCK_SETTLED',measurement_budget_ms=50,limit_s=120,pass_separation_s=5)))
        return N(pid=456789)
    def snapshot(self,pid):
        exited=self.now[0]>=2. if not self.slow else self.signal_at is not None and self.now[0]-self.signal_at>=3.
        return [dict(pid=pid,pgid=pid,sid=pid,ppid=1,state='Z' if exited else 'S',start_ticks=10)]
    def send(self,pid,sig,leader):self.signals.append(sig.name);self.signal_at=self.now[0]
    def reap(self,p):return self.rc


def test_clock_caller_requires_helper_exit_and_original_profile_no_clock_algorithm_retest(tmp_path):
    now=[1.];io=Backend(tmp_path,now)
    result=run_clock_gate(tmp_path,lambda:False,backend=io,clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert result['measurement_budget_ms']==50 and not io.signals


def test_clock_interrupt_allows_nested_probe_drain_more_than_old_one_second(tmp_path):
    now=[1.];io=Backend(tmp_path,now,True)
    with pytest.raises(RuntimeError):run_clock_gate(tmp_path,lambda:True,backend=io,clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
    assert io.signals==['SIGINT'] and now[0]>=4.


def test_clock_nonzero_cannot_pass_from_settled_file_alone(tmp_path):
    now=[1.];io=Backend(tmp_path,now,rc=2)
    with pytest.raises(RuntimeError):run_clock_gate(tmp_path,lambda:False,backend=io,clock=lambda:now[0],sleep=lambda s:now.__setitem__(0,now[0]+s))
