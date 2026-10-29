"""Offline only: subprocess fixtures, mocked clock/sudo, no ROS or SSH target."""
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

import pytest

import fire_robot_pi_session as pi


class Server:
    def __init__(self, tmp_path, command=None, lease=4, idle='lambda: None', extra=''):
        self.lines = queue.Queue()
        source = Path(pi.__file__).read_text().split("if __name__ == '__main__':")[0]
        source += '\n'+extra+'\npi_serve(launch_command='+repr(command or [
            sys.executable, '-c', 'import time; time.sleep(30)'])+\
            ', lease='+repr(lease)+', check_idle='+idle+\
            ', lock_path='+repr(str(tmp_path/'pi.lock'))+')\n'
        self.proc = subprocess.Popen([sys.executable, '-u', '-c', source],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True,
                                     start_new_session=True)
        self.out = None
        def read():
            for line in self.proc.stdout:
                self.lines.put(json.loads(line))
        threading.Thread(target=read, daemon=True).start()

    def request(self, op, **fields):
        self.proc.stdin.write(json.dumps({'op': op, **fields})+'\n')
        self.proc.stdin.flush()
        return self.response()

    def response(self, timeout=13):
        r = self.lines.get(timeout=timeout)
        self.out = r.get('log_dir', self.out)
        return r

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=13)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3)
        assert self.proc.returncode == 0, self.proc.stderr.read()


@pytest.mark.parametrize('ending', ['stop', 'eof', 'signal', 'lease'])
def test_owned_launch_cleanup(tmp_path, ending):
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    srv = Server(tmp_path, lease=.5 if ending == 'lease' else 4)
    try:
        start = srv.request('start')
        assert start['ok']
        assert srv.request('ping')['running']
        if ending == 'stop':
            result = srv.request('stop')
        else:
            if ending == 'eof':
                srv.proc.stdin.close()
            elif ending == 'signal':
                srv.proc.send_signal(signal.SIGINT)
            result = srv.response()
        assert result['ok'] and result['remaining_pids'] == []
        assert result['reason'] == {'stop': 'stop_requested', 'eof': 'controller_eof',
                                   'signal': 'signal', 'lease': 'controller_timeout'}[ending]
        assert unrelated.poll() is None
        assert not Path(f'/proc/{start["pid"]}').exists()
        assert json.loads((Path(result['log_dir'])/'result.json').read_text()) == result
    finally:
        srv.close()
        unrelated.terminate(); unrelated.wait(timeout=3)


def test_launch_failure_is_detected_and_cleanup_reported(tmp_path):
    srv = Server(tmp_path, command=[sys.executable, '-c', 'raise SystemExit(7)'])
    try:
        assert srv.request('start')['ok']
        result = srv.response()
        assert result['reason'] == 'launch_exited'
        assert result['remaining_pids'] == []
    finally:
        srv.close()


def test_cleanup_includes_orphans_setsid_and_sigkill_fallback(tmp_path):
    marker = tmp_path/'grandchild.pid'
    grandchild = ('import os,signal,time; from pathlib import Path; '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                  f'Path({str(marker)!r}).write_text(str(os.getpid())); time.sleep(30)')
    parent = ('import subprocess,sys,time; '
              f'subprocess.Popen([sys.executable,"-c",{grandchild!r}],start_new_session=True); '
              'time.sleep(.2)')
    srv = Server(tmp_path, command=[sys.executable, '-c', parent])
    try:
        assert srv.request('start')['ok']
        result = srv.response()
        assert result['remaining_pids'] == []
        assert not Path('/proc/'+marker.read_text()).exists()
    finally:
        srv.close()


def test_idle_rejection_never_launches_or_sets_time(tmp_path):
    srv = Server(tmp_path, idle='lambda: (_ for _ in ()).throw(RuntimeError("old stack"))')
    try:
        result = srv.request('clock')
        assert result['error'] == 'old stack'
        assert 'launch_pid' not in result
        assert result['remaining_pids'] == []
    finally:
        srv.close()


@pytest.mark.parametrize('op', ['clock', 'set_clock', 'start'])
def test_reject_clock_change_or_duplicate_start_after_launch(tmp_path, op):
    srv = Server(tmp_path)
    try:
        assert srv.request('start')['ok']
        result = srv.request(op, epoch=time.time())
        assert not result['ok'] and result['remaining_pids'] == []
        assert 'Không chỉnh giờ/start' in result['error']
    finally:
        srv.close()


def test_sudo_authorization_and_clock_command_not_persisting_password(tmp_path):
    capture = tmp_path/'calls.jsonl'
    extra = '''
def fake_run(cmd, **kwargs):
    # Record command only; test supplies a sentinel password in memory.
    with open(CAPTURE, 'a') as f: f.write(json.dumps(cmd)+'\\n')
    return type('Result', (), {'returncode': 0})()
subprocess.run = fake_run
'''.replace('CAPTURE', repr(str(capture)))
    srv = Server(tmp_path, extra=extra)
    try:
        assert srv.request('authorize', password='SENTINEL_SECRET')['authorized']
        assert srv.request('set_clock', epoch=1792230000)['ok']
        result = srv.request('stop')
        commands = [json.loads(line) for line in capture.read_text().splitlines()]
        assert commands == [['sudo', '-S', '-p', '', '-v'],
                            ['sudo', '-n', 'date', '-u', '--set=@1792230000.000000']]
        assert 'SENTINEL_SECRET' not in (Path(result['log_dir'])/'result.json').read_text()
    finally:
        srv.close()


def make_client(tmp_path):
    events = []
    client = pi.PiSession('unused', tmp_path, lambda *a, **kw: events.append((a, kw)))
    return client, events


def test_aligned_clock_does_not_need_sudo(tmp_path, monkeypatch):
    client, events = make_client(tmp_path)
    monkeypatch.setattr(client, 'measure_clock', lambda: (.002, .010))
    monkeypatch.setattr(client, 'request', lambda *a, **kw: pytest.fail('unexpected sudo'))
    client.synchronize()
    assert events[-1][0] == ('PI_CLOCK_VERIFIED',)


def test_sync_uses_fresh_time_after_password_prompt(tmp_path, monkeypatch):
    client, events = make_client(tmp_path)
    measurements = iter([(86400, .01), (.003, .01)])
    monkeypatch.setattr(client, 'measure_clock', lambda: next(measurements))
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    clock = [1792230000.0]
    monkeypatch.setattr(pi.time, 'time', lambda: clock[0])
    def prompt(*args):
        clock[0] += 120
        return 'SENTINEL_SECRET'
    monkeypatch.setattr(pi.getpass, 'getpass', prompt)
    calls = []
    def rpc(op, **kw):
        calls.append((op, kw))
        return {'authorized': 'password' in kw} if op == 'authorize' else {'ok': True}
    monkeypatch.setattr(client, 'request', rpc)
    client.synchronize()
    assert calls[-1] == ('set_clock', {'epoch': clock[0]+.005})
    assert 'SENTINEL_SECRET' not in repr(events)


def test_failed_clock_verification_blocks_start(tmp_path, monkeypatch):
    client, _ = make_client(tmp_path)
    monkeypatch.setattr(client, 'connect', lambda: None)
    monkeypatch.setattr(client, 'measure_clock', lambda: (2, .01))
    calls = []
    def rpc(op, **kw):
        calls.append(op)
        return {'authorized': True}
    monkeypatch.setattr(client, 'request', rpc)
    with pytest.raises(RuntimeError, match='Giờ Pi còn lệch'):
        client.start()
    assert 'start' not in calls


def test_clock_measurement_rejects_network_uncertainty(tmp_path, monkeypatch):
    client, _ = make_client(tmp_path)
    wall = iter([100.,100.2,101.,101.2,102.,102.2])
    mono = iter([10.,10.2,11.,11.2,12.,12.2])
    monkeypatch.setattr(pi.time, 'time', lambda: next(wall))
    monkeypatch.setattr(pi.time, 'monotonic', lambda: next(mono))
    monkeypatch.setattr(client, 'request', lambda *a: {'time':100, 'log_dir':'/tmp/mock'})
    with pytest.raises(RuntimeError, match='RTT'):
        client.measure_clock()


def test_clock_rejects_local_time_step(tmp_path, monkeypatch):
    client, _ = make_client(tmp_path)
    wall, mono = iter([100.,200.]), iter([10.,10.01])
    monkeypatch.setattr(pi.time, 'time', lambda: next(wall))
    monkeypatch.setattr(pi.time, 'monotonic', lambda: next(mono))
    monkeypatch.setattr(client, 'request', lambda *a: {'time':100, 'log_dir':'/tmp/mock'})
    with pytest.raises(RuntimeError, match='laptop thay đổi'):
        client.measure_clock()


def test_client_close_handles_late_reply_from_interrupted_rpc(tmp_path):
    import io
    client, events = make_client(tmp_path)
    class Process:
        stdin = io.StringIO()
        def wait(self, timeout): return 0
    client.process = Process()
    client.sequence = 2
    client.responses.put({'ok': True, 'request_id': 2, 'pid': 123})
    client.responses.put({'ok': True, 'request_id': 3,
                          'stopped': True, 'remaining_pids': []})
    assert client.close()
    assert events[-1][0] == ('PI_SESSION_STOPPED',)


def test_client_does_not_claim_cleanup_on_link_failure(tmp_path):
    import io
    client, events = make_client(tmp_path)
    class Process:
        stdin = io.StringIO()
        def wait(self, timeout): return 0
    client.process = Process()
    client.failure = 'SSH disconnected'
    assert not client.close()
    assert events[-1][0] == ('PI_CLEANUP_UNCONFIRMED',)


def test_client_lifecycle_with_real_dummy_remote(tmp_path, monkeypatch):
    srv = Server(tmp_path)
    client, events = make_client(tmp_path)
    def attach():
        client.process = srv.proc
        client.responses = srv.lines
    monkeypatch.setattr(client, 'connect', attach)
    monkeypatch.setattr(client, 'synchronize', lambda: None)
    try:
        client.start()
        client.assert_alive()
        assert client.request('ping')['running']
        assert client.close()
        assert events[-1][0] == ('PI_SESSION_STOPPED',)
    finally:
        srv.close()


def test_local_close_cleans_orphaned_group_and_ignores_repeat_ctrl_c(tmp_path):
    """Run the real Session in a child so subreaper cannot adopt pytest jobs."""
    scripts = str(Path(pi.__file__).parent)
    child_code = 'import time; time.sleep(30)'
    parent_code = ('import subprocess,sys; '
                   f'subprocess.Popen([sys.executable,"-c",{child_code!r}],start_new_session=True)')
    source = '''
import os,sys,time,signal,threading
sys.path.insert(0, SCRIPTS)
from fire_robot_session import Session
s=Session(manage_pi=True)
p=s.start('fixture',[sys.executable,'-c',PARENT],optional=True)
time.sleep(.1)
# Repeated Ctrl+C arrives during cleanup, as with impatient operator input.
threading.Timer(.01,lambda: os.kill(os.getpid(),signal.SIGINT)).start()
s.close()
print('CLEAN_DONE',flush=True)
'''.replace('SCRIPTS',repr(scripts)).replace('PARENT',repr(parent_code))
    result = subprocess.run([sys.executable,'-c',source],capture_output=True,text=True,timeout=16)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'SESSION_STOPPED' in result.stdout and 'CLEAN_DONE' in result.stdout


def test_demo_registers_cleanup_before_pi_start_failure(monkeypatch):
    import fire_robot_session as session
    from types import SimpleNamespace
    calls = []
    class MockSession:
        def __init__(self, **kw):
            assert kw == {'manage_pi': True}
        def start_pi(self):
            calls.append('start')
            raise session.SessionError('clock sync failed')
        def event(self, name, **kw): calls.append((name, kw))
        def close(self): calls.append('close')
    monkeypatch.setattr(session, 'validate_map', lambda value: value)
    monkeypatch.setattr(session, 'no_external_owners', lambda: None)
    monkeypatch.setattr(session, 'Session', MockSession)
    with pytest.raises(session.SessionError, match='clock sync failed'):
        session.demo(SimpleNamespace(live=True,map='test.yaml',align_only=False))
    assert calls == [
        'start',
        ('SESSION_FAILED', {'phase': 'pi_start',
                            'error_type': 'SessionError',
                            'error': 'clock sync failed'}),
        'close',
    ]


def test_remote_lock_prevents_second_controller(tmp_path):
    first = Server(tmp_path)
    second = None
    try:
        assert first.request('start')['ok']
        second = Server(tmp_path)
        second.proc.wait(timeout=3)
        assert second.proc.returncode != 0
        assert 'Resource temporarily unavailable' in second.proc.stderr.read()
        assert first.request('ping')['running']
        assert first.request('stop')['remaining_pids'] == []
    finally:
        first.close()
        if second:
            second.proc.stdin.close()


def test_sudo_denial_never_sets_clock_or_launches(tmp_path, monkeypatch):
    client, _ = make_client(tmp_path)
    monkeypatch.setattr(client, 'connect', lambda: None)
    monkeypatch.setattr(client, 'measure_clock', lambda: (500, .01))
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr(pi.getpass, 'getpass', lambda *a: 'wrong')
    calls = []
    def rpc(op, **kw):
        calls.append(op)
        return {'authorized': False}
    monkeypatch.setattr(client, 'request', rpc)
    with pytest.raises(RuntimeError, match='Sudo Pi không được cấp quyền'):
        client.start()
    assert calls == ['authorize', 'authorize']


def test_cleanup_receipt_after_rpc_failure_is_still_verified(tmp_path):
    import io
    client, events = make_client(tmp_path)
    class Process:
        stdin = io.StringIO()
        def wait(self, timeout):
            client.responses.put({'ok': True, 'stopped': True, 'remaining_pids': []})
    client.process = Process()
    client.failure = 'response timeout'
    assert client.close()
    assert events[-1][0] == ('PI_SESSION_STOPPED',)


def test_idle_checks_actual_pi_executables_without_matching_shell_text(tmp_path):
    proc = tmp_path/'123';proc.mkdir()
    (proc/'cmdline').write_bytes(b'bash\x00-c\x00echo ros2 launch serial_bridge_node\x00')
    pi.require_idle(tmp_path)
    (proc/'cmdline').write_bytes(b'/usr/bin/python3\x00/ros/lib/scan_qos_relay\x00')
    with pytest.raises(RuntimeError, match='123'):
        pi.require_idle(tmp_path)
