import json
from types import SimpleNamespace as N
import pytest
from operational_stream_log import StreamingOwnedLog,Audit
from operational_owned_process import OwnedProcess
from operational_entry import LocalProcesses
from operational_logs import audit_launch_children

class Backend:
    def __init__(self):self.live=True;self.out=None
    def start(self,argv,out):self.out=out;return N(pid=456789)
    def snapshot(self,pid):return [dict(pid=pid,pgid=pid,sid=pid,ppid=1,start_ticks=1,state='S' if self.live else 'Z')]
    def reap(self,p):return 0
    def send(self,*a):raise AssertionError('No real/fake signal needed')


def test_streamed_large_log_preserves_earliest_failure_instead_of_clean_tail(tmp_path):
    io=Backend();p=OwnedProcess('yolo',backend=io,launch_leader=True);log=StreamingOwnedLog(tmp_path/'fresh.log');log.start(p,['fake'],1.)
    io.out.write(b"[INFO] [python-1]: process started with pid [123]\n[ERROR] [python-1]: process has died [pid 123, exit code 1, cmd 'fake'].\n")
    log.scan();io.out.write(b'normal line\n'*100000);io.live=False;p.tick(2.)
    r=log.child_audit();assert not r['clean'] and 'CHILD_DIED:python-1:1' in r['errors']
    log.close()


def test_large_clean_log_and_one_stop_receipt_are_not_rejected_by_legacy_1MiB_limit(tmp_path):
    io=Backend();p=OwnedProcess('operator',backend=io);log=StreamingOwnedLog(tmp_path/'operator.log');log.start(p,['fake'],1.)
    io.out.write(b'normal line\n'*100000)
    proof=dict(scope='R5_STOP_TELEMETRY_VERIFICATION',confirmed=True)
    io.out.write(('[INFO] [123.456] [fire_robot_operator]: OPERATOR_STOP '+json.dumps(proof)+'\n').encode())
    io.live=False;p.tick(2.);assert log.receipt()==proof;log.close()


def test_partial_lines_wait_until_complete_and_split_utf8_is_preserved():
    a=Audit();text='[INFO] [child-1]: process started with pid [123]\n'.encode();a.feed(text[:20]);assert a.report()['started']==0
    a.feed(text[20:]);a.feed('bình thường\n'.encode()[:2]);a.feed('bình thường\n'.encode()[2:]);a.feed(b'[INFO] [child-1]: process has finished cleanly [pid 123]\n')
    assert a.report(final=True)['clean']


@pytest.mark.parametrize('bad',['truncate','replace','symlink'])
def test_streaming_keeps_same_owned_inode_and_append_only(tmp_path,bad):
    io=Backend();p=OwnedProcess('operator',backend=io);log=StreamingOwnedLog(tmp_path/'operator.log');log.start(p,['fake'],1.)
    io.out.write(b'normal\n');log.scan()
    if bad=='truncate':io.out.truncate(0)
    else:
        log.path.unlink();other=tmp_path/'other';other.write_text('replacement\n')
        if bad=='replace':log.path.write_text('new')
        else:log.path.symlink_to(other)
    with pytest.raises(ValueError):log.scan()
    log.close()


def test_unbounded_single_line_is_error_not_unbounded_memory():
    a=Audit()
    for i in range(40):a.feed(b'x'*65536)
    assert len(a.pending)<=65536 and 'LOG_LINE_TOO_LONG' in a.report()['errors']


def test_live_child_error_detected_while_launch_leader_still_alive(tmp_path,monkeypatch):
    io=Backend();monkeypatch.setattr('operational_owned_process.time.monotonic',lambda:1.)
    local=LocalProcesses(tmp_path,'session',backend_factory=lambda role:io);local.start('yolo',['fake'],launch_leader=True)
    io.out.write(b"[INFO] [python-1]: process started with pid [123]\n[ERROR] [python-1]: process has died [pid 123, exit code 1, cmd 'fake'].\n")
    with pytest.raises(RuntimeError,match='OWNED_LAUNCH_CHILD_ERROR:yolo'):local.check(1.)
    local.close_logs()


@pytest.mark.parametrize('malformed',[b"[OTHER] [child-1]: process has died [pid 123, exit code 1]\n",b'\xffinvalid utf8\n'])
def test_malformed_process_record_or_encoding_never_hidden_by_later_clean_exit(malformed):
    a=Audit();a.feed(b'[INFO] [child-1]: process started with pid [123]\n');a.feed(malformed)
    a.feed(b'[INFO] [child-1]: process has finished cleanly [pid 123]\n')
    assert not a.report(final=True)['clean'] and a.report()['errors']


@pytest.mark.parametrize('streaming',[False,True])
def test_launch_internal_sigterm_fallback_is_reported_not_clean_rc0(streaming):
    lines=(
        "[INFO] [nav_container-1]: process started with pid [123]\n"
        "[ERROR] [nav_container-1]: process[nav_container-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'\n"
        "[INFO] [nav_container-1]: process has finished cleanly [pid 123]\n")
    if streaming:
        a=Audit();a.feed(lines.encode());report=a.report(final=True)
    else:report=audit_launch_children(lines)
    assert report['terminal']==1 and not report['clean']
    assert report['internal_signal_fallback'] is True
    assert report['fallbacks']==[dict(label='nav_container-1',pid=123,timeout_s=5.0,
                                      from_signal='SIGINT',to_signal='SIGTERM')]
    assert 'CHILD_SIGNAL_ESCALATION:nav_container-1:SIGINT_TO_SIGTERM' in report['errors']


def test_unmatched_or_spoofed_internal_escalation_is_rejected():
    a=Audit();a.feed(b"[INFO] [nav_container-1]: process started with pid [123]\n")
    a.feed(b"[ERROR] [other-1]: process[nav_container-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'\n")
    assert not a.report(final=True)['clean']
    assert 'UNMATCHED_OR_INVALID_CHILD_ESCALATION' in a.report()['errors']
