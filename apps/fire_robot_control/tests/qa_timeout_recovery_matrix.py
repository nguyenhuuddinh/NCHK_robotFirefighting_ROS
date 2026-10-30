"""Timeout/retry QA with temporary receipts, fake ROS and no external processes.

PASS means the observed branch matches its stated outcome, including documented
BLOCKED cases. It does not mean every failure can currently be recovered.
"""
import copy
import json
import os
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace as Obj
from unittest.mock import patch
import uuid

import pytest
import rclpy
import operational_entry as entry
import services.app_peer
from services import mapping_runner, recovery, recovery_probe as probe
from services import stop_recovery_runner as stop
from services.operator_bootstrap import create_bootstrap_file
from services.operator_runner import OperatorBootstrap
from services.supervisor import SessionController
from proof_fixture import clean_local, clean_remote
import test_stop_recovery as stop_fixture


@pytest.fixture(autouse=True)
def no_external_processes(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('QA must never run SSH, ROS commands or external processes')
    monkeypatch.setattr('subprocess.run', forbidden)
    monkeypatch.setattr('subprocess.Popen', forbidden)


@pytest.mark.parametrize('point,verify', [('io', False), ('seal', False),
                                        ('slam', False), ('io', True)])
def test_actual_mapping_timeout_drains_laptop_before_pi(tmp_path, monkeypatch, point, verify):
    """Execute the production runner through a real timeout, without ROS I/O."""
    clock = Obj(now=0.)
    def advance(dt=1.):
        clock.now += max(dt, 1.)
    monkeypatch.setattr(mapping_runner, 'time', Obj(monotonic=lambda: clock.now, sleep=advance))
    calls = []
    packet = 'a' * 64
    maps = tmp_path/'maps'; maps.mkdir()
    pairing = tmp_path/'pairing.json'; pairing.write_text(json.dumps(dict(packet=packet)))
    boot = OperatorBootstrap(str(uuid.uuid4()), 'b'*32, 'c'*32, packet, tmp_path/'ui.sock')
    class Peer:
        def __init__(self, *_): pass
        def startup(self, cmd):
            calls.append(('peer', cmd))
            if cmd == 'STATE' and not any(c == ('peer', 'SEAL') for c in calls):
                return dict(phase='WAIT_LAPTOP')
            return dict(phase='IO_STARTING' if point == 'seal' else 'RUNNING_OBSERVE',
                        sealed=point != 'seal', health=dict(io=point != 'seal'))
        def stop(self): calls.append(('pi_stop',)); return clean_remote(dict(packet=packet))
    class Local:
        def __init__(self, *_args, **_kwargs):
            self.processes = {}
            self.stream = tempfile.TemporaryFile()
            self.stream.write(b'MAPPING_MONITOR_READY\n'); self.stream.flush()
            self.registry = Obj(logs={'operator': Obj(file=self.stream)})
        def start(self, role, argv, **_):
            calls.append(('start', role))
            self.processes[role] = Obj(done=False, tick=lambda _: None,
                                       report=lambda: dict(group_exit_confirmed=True))
        def check(self, _): pass
        def drain(self, **_): calls.append(('local_drain',)); return clean_local()
        def close_logs(self): self.stream.close()
    monkeypatch.setattr(entry, 'packet_digest', lambda: packet)
    monkeypatch.setattr(entry, 'LocalProcesses', Local)
    monkeypatch.setattr(entry, 'run_clock_gate', lambda *_: None)
    monkeypatch.setattr(entry, 'read_health', lambda *_: dict(checks=dict(io=point != 'io'), stable_s=dict(io=2.)))
    monkeypatch.setattr('operational_preflight.preflight', lambda *_: None)
    monkeypatch.setattr(services.app_peer, 'AppPeer', Peer)
    monkeypatch.setattr(mapping_runner, 'PtyBackend', lambda: Obj())
    monkeypatch.setattr('services.monitor_log.start_monitor', lambda local, argv: local.start('operator', argv))
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    monkeypatch.setattr(rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(rclpy, 'create_node', lambda *_: Obj(
        create_subscription=lambda *_: None, get_node_names_and_namespaces=lambda: [],
        get_publishers_info_by_topic=lambda *_: [], destroy_node=lambda: None))
    monkeypatch.setattr(rclpy, 'spin_once', lambda *_, **__: advance(5.))
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with create_bootstrap_file(boot, tmp_path) as fd:
            args = ['--bootstrap-fd', str(os.dup(fd.fileno())), '--pairing', str(pairing),
                    '--remote-script', 'fixture-only', '--out', str(tmp_path/'laptop'), '--map-root', str(maps)]
            if verify: args.append('--verify-stop')
            rc = mapping_runner.run(args)
    finally:
        for sig, handler in handlers.items(): signal.signal(sig, handler)
    expected = {'io': 'MAPPING_PI_IO_TIMEOUT', 'seal': 'MAPPING_PI_SEAL_TIMEOUT', 'slam': 'MAPPING_SLAM_TIMEOUT'}[point]
    failure = json.loads((tmp_path/'laptop/failure.json').read_text())
    row = json.loads((tmp_path/'laptop/receipt.json').read_text())
    assert rc == 2 and expected in failure['error'], (failure, calls)
    assert row['local']['stop_confirmed'] and row['local']['clean']
    assert calls.index(('local_drain',)) < calls.index(('pi_stop',))
    assert ('start', 'rviz') not in calls
    if point != 'slam': assert ('start', 'nav') not in calls
    assert row['profile'] == ('DESKTOP_STOP_RECOVERY' if verify else 'DESKTOP_MAPPING')


@pytest.mark.parametrize('stage', ['PI_IO', 'CAMERA', 'SEAL_PI', 'POSE', 'YOLO', 'CONSUMERS'])
def test_fire_stage_timeout_requests_stop_and_late_data_does_not_restart(stage):
    s = entry.StartupSequence(0., live=True)
    observations = [dict(clock_ok=True), dict(io=True), dict(io=True, camera=True),
        dict(io=True, camera=True, pi_sealed=True), dict(io=True, camera=True, pose=True),
        dict(io=True, camera=True, pose=True, yolo=True)]
    for at, observation in enumerate(observations):
        s.step(float(at), observation)
        if s.stage == stage: break
    assert s.stage == stage
    when = s.since + s.LIMITS[stage] + .01
    effects = s.step(when, {})
    if stage == 'CAMERA':
        assert effects == ['RETRY_PI', 'RESTART_HEALTH'] and s.retry == 1
        s.step(when+.1, dict(retry_ready=True))
        s.step(when+.2, dict(io=True))
        assert s.stage == 'CAMERA'
        when = s.since + s.LIMITS['CAMERA'] + .01
        effects = s.step(when, {})
    assert effects == ['STOP']
    assert s.done and s.error == 'STARTUP_TIMEOUT:' + stage
    late = dict(io=True, camera=True, pi_sealed=True, pose=True, yolo=True, operator_present=True)
    assert s.step(when+1., late) == [] and s.stage == stage


def fake_pi_idle(binding, cleanup, current_boot, *, missing_done=False, strict=False):
    """Exercise actual same/new boot probe code; every OS/network input is fake."""
    pairing = dict(binding, secret='fixture-not-a-real-secret')
    done = dict(session=binding['session'], phase='DONE', live=False, cleanup=cleanup, error='STARTUP_TIMEOUT')
    def read(path):
        if path.name == 'pairing.json': return pairing, 'c'*64
        if missing_done: raise FileNotFoundError('synthetic missing DONE')
        assert path.name == 'DONE.json'
        return done, 'd'*64
    with patch.object(Path, 'is_symlink', return_value=False), patch.object(Path, 'resolve', lambda p: p), \
         patch.object(Path, 'read_text', return_value=current_boot), patch.object(probe, 'read_object', side_effect=read), \
         patch.object(probe, 'lock_existing', return_value=123), patch.object(probe.os, 'close'), \
         patch.object(probe, 'absent'), patch.object(probe, 'no_robot_processes'):
        return probe.probe_pi(binding) if strict else probe.probe_pi_idle(binding, expected_boot=current_boot)


CASES = ['clean', 'no_old_stop', 'pi_tf_child_failed', 'pi_missing_done',
         'local_forced', 'missing_local_receipt', 'pi_failed_after_reboot']


@pytest.mark.parametrize('profile', ['DESKTOP_MAPPING', 'R5_AUTOMATIC'])
@pytest.mark.parametrize('case', CASES)
def test_timeout_retry_matrix_with_actual_gate_code(profile, case):
    fixture = stop_fixture.StopRecoveryTests()
    fixture.setUp()
    try:
        row = fixture.row
        row.update(profile=profile, rc=2, local=clean_local())
        binding = copy.deepcopy(row['remote']['binding'])
        row['remote'] = clean_remote(binding)
        fixture.desktop.update(robot_stop_confirmed=True, cleanup_clean=True, session_was_started=True)
        if case == 'no_old_stop':
            row['local'].update(stop_confirmed=False, clean=False)
            fixture.desktop['robot_stop_confirmed'] = False
        if case in ('pi_tf_child_failed', 'pi_failed_after_reboot'):
            c = row['remote']['receipt']; c['children_clean'] = False
            c['child_audit']['clean'] = False
            c['child_audit']['roles']['fixture'].update(clean=False, errors=['CHILD_DIED:odom_to_tf_broadcaster-8:1'])
        if case == 'local_forced': row['local']['processes']['operator']['forced'] = True
        (fixture.old/'desktop_receipt.json').write_text(json.dumps(fixture.desktop))
        receipt_path = fixture.old/'laptop/receipt.json'; receipt_path.write_text(json.dumps(row))
        if case == 'missing_local_receipt': receipt_path.unlink()
        historical = {p: p.read_bytes() for p in fixture.old.rglob('*.json')}
        current = str(uuid.uuid4()) if case == 'pi_failed_after_reboot' else binding['boot_id']
        def idle(b, **_):
            return fake_pi_idle(b, row['remote']['receipt'], current, missing_done=case == 'pi_missing_done')
        def strict(b):
            return fake_pi_idle(b, row['remote']['receipt'], current,
                                missing_done=case == 'pi_missing_done', strict=True)
        with patch.object(probe, 'absent'), patch.object(probe, 'no_robot_processes'):
            # Read-only RECHECK can clear a timeout with valid old STOP/cleanup.
            recheck_ok = case == 'clean'
            if recheck_ok:
                result = recovery.audit(fixture.root, remote=strict, lock_path=fixture.lock)
                recovery.archive_verified(fixture.root, result, '6'*32, lock_path=fixture.lock)
                assert not (fixture.root/'pending.json').exists()
                return
            with pytest.raises((ValueError, FileNotFoundError)):
                recovery.audit(fixture.root, remote=strict, lock_path=fixture.lock)
            allowed = case in ('no_old_stop', 'pi_failed_after_reboot', 'pi_tf_child_failed', 'pi_missing_done')
            if not allowed:
                with pytest.raises((ValueError, FileNotFoundError)):
                    stop.prepare(fixture.root, remote=idle, lock_path=fixture.lock)
                assert (fixture.root/'pending.json').read_bytes() == fixture.raw
                return
            prepared = stop.prepare(fixture.root, remote=idle, lock_path=fixture.lock)
            assert prepared['marker_sha256'] == fixture.digest
            assert (fixture.root/'pending.json').read_bytes() == fixture.raw
            new = fixture.new_proof()
            new_row = json.loads((new/'laptop/receipt.json').read_text())
            new_row['remote']['binding']['boot_id'] = current
            (new/'laptop/receipt.json').write_text(json.dumps(new_row))
            desktop = json.loads((new/'desktop_receipt.json').read_text()); desktop['recovery_boot_id'] = current
            (new/'desktop_receipt.json').write_text(json.dumps(desktop))
            result = stop.audit_new_stop(fixture.root, new,
                                        remote=lambda b: fake_pi_idle(b, new_row['remote']['receipt'], current),
                                        idle_remote=idle, lock_path=fixture.lock)
            recovery.archive_verified(fixture.root, result, '7'*32, lock_path=fixture.lock)
            assert not (fixture.root/'pending.json').exists()
            assert all(p.read_bytes() == data for p, data in historical.items())
    finally:
        fixture.tearDown()


@pytest.mark.parametrize('action', ['SCAN', 'FIRE'])
@pytest.mark.parametrize('started,stopped,clean,expected', [
    (False, None, True, 'IDLE'), (True, True, True, 'IDLE'),
    (True, False, False, 'BLOCKED'), (True, True, False, 'BLOCKED')])
def test_timeout_controller_can_restart_only_after_proven_result(tmp_path, action, started, stopped, clean, expected):
    """Real button controller, injected worker result; no child is created."""
    for name in ('maps', 'runtime', 'state'): (tmp_path/name).mkdir()
    maps = tmp_path/'maps'
    (maps/'demo.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xff\x00')
    (maps/'demo.yaml').write_text('image: demo.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    workers = []
    class Worker:
        def __init__(self, **kw):
            self.session = str(uuid.uuid4()); self.kind = kw['kind']; self.out = kw['out']
        def start(self): workers.append(self)
        def heartbeat(self, *_): pass
        def send(self, *_): pass
    c = SessionController(candidate=tmp_path, packet='a'*64, state_dir=tmp_path/'state',
        runtime=tmp_path/'runtime', map_root=maps, map_roots=[maps], session_factory=Worker)
    c.action('SELECT', str(maps/'demo.yaml'))
    c.action(action)
    assert len(workers) == 1
    worker = c.session
    error = 'STARTUP_TIMEOUT:SEAL_PI' if action == 'FIRE' else 'MAPPING_PI_IO_TIMEOUT'
    c.inbox.put(dict(event='FINISHED', session=worker.session, robot_stop_confirmed=stopped,
                    cleanup_clean=clean, session_was_started=started, error=error))
    c.tick()
    assert c.phase == expected and c.session is None and c.error.startswith(error)
    assert (tmp_path/'state/pending.json').exists() is (expected == 'BLOCKED')
    c.action(action)
    assert len(workers) == (2 if expected == 'IDLE' else 1)
