"""Only temporary log files and fake processes. No ROS/devices/signals/SSH."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as N
import pytest
sys.path.insert(0,str(Path(__file__).parents[5]/'scripts'))
from operational_owned_process import OwnedProcess
from operational_logs import OwnedLog,operator_receipt,audit_launch_children


def proof():return dict(scope='R5_STOP_TELEMETRY_VERIFICATION',requested=True,done=True,confirmed=True,reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED',blockers=[],fault=None,motion_authorized=False,pulse_replayed=False)
def line(value=None,node='fire_robot_operator'):
    return '[INFO] [123.456] ['+node+']: OPERATOR_STOP '+json.dumps(proof() if value is None else value)+'\n'
def begin(label='yolo_observer-1',pid=111):return f'[INFO] [{label}]: process started with pid [{pid}]\n'
def end(label='yolo_observer-1',pid=111):return f'[INFO] [{label}]: process has finished cleanly [pid {pid}]\n'
def die(label='yolo_observer-1',pid=111,rc=1):return f"[ERROR] [{label}]: process has died [pid {pid}, exit code {rc}, cmd 'fake']\n"

class Fake:
    def __init__(self):self.live=True;self.output=None
    def start(self,argv,output):self.output=output;return N(pid=456789)
    def snapshot(self,pid):return [dict(pid=pid,ppid=1,pgid=pid,sid=pid,state='S' if self.live else 'Z',start_ticks=1)]
    def send(self,*a):raise AssertionError('No signal expected')
    def reap(self,p):return 0


def bound(tmp_path,text):
    io=Fake();p=OwnedProcess('operator',backend=io);log=OwnedLog(tmp_path/'fresh.log')
    log.start(p,['fake'],0);io.output.write(text.encode());io.live=False;p.tick(1)
    return log,p


def test_fresh_log_binds_exact_output_process_and_current_receipt(tmp_path):
    log,p=bound(tmp_path,line());assert log.receipt()==proof();log.close()


def test_preexisting_log_never_reused(tmp_path):
    p=tmp_path/'old.log';p.write_text(line())
    with pytest.raises(FileExistsError):OwnedLog(p)


@pytest.mark.parametrize('bad',['wrong_node','duplicate','partial','nan','duplicate_key','saved_plain_json'])
def test_bad_receipt_is_not_source_proof(bad):
    text={'wrong_node':line(node='other'),'duplicate':line()+line(),'partial':line()[:-1],
          'nan':line().replace('true','NaN',1),'duplicate_key':line().replace('"confirmed": true','"confirmed": true, "confirmed": false'),
          'saved_plain_json':json.dumps(proof())+'\n'}[bad]
    with pytest.raises(ValueError):operator_receipt(text)


def test_replaced_file_not_read_as_owned_receipt(tmp_path):
    log,p=bound(tmp_path,line());log.path.unlink();log.path.write_text(line())
    with pytest.raises(ValueError):log.receipt()
    log.close()


def test_alive_or_unbound_log_cannot_prove_stop(tmp_path):
    log=OwnedLog(tmp_path/'new.log')
    with pytest.raises(ValueError):log.receipt()
    io=Fake();p=OwnedProcess('operator',backend=io);log.start(p,['fake'],0);io.output.write(line().encode())
    with pytest.raises(ValueError):log.receipt()
    log.close()


def test_all_children_must_exit_cleanly_same_pid():
    r=audit_launch_children(begin()+begin('child-2',222)+end()+end('child-2',222))
    assert r['clean'] and r['started']==r['terminal']==2


@pytest.mark.parametrize('text',[begin(),end(),begin()+end(pid=222),begin()+die(),begin()+die(rc=-6)+end(),begin()+begin()+end(),'',begin()+end()[:-1]])
def test_launch_zero_cannot_hide_child_failure_or_missing_exit(text):
    assert not audit_launch_children(text)['clean']


def test_restart_does_not_erase_earlier_failed_child():
    r=audit_launch_children(begin()+die()+begin(pid=222)+end(pid=222));assert not r['clean']


def test_bound_launch_audit_reads_own_log_not_other_log(tmp_path):
    log,p=bound(tmp_path,begin()+end());assert log.child_audit()['clean'];log.close()


@pytest.mark.parametrize('rc',[1,-6])
def test_real_launch_death_line_trailing_period_preserves_exact_child_code(rc):
    text=begin()+die(rc=rc).rstrip('\n')+'.\n'
    result=audit_launch_children(text)
    assert not result['clean'] and result['terminal']==1 and not result['missing']
    assert 'CHILD_DIED:yolo_observer-1:'+str(rc) in result['errors']
