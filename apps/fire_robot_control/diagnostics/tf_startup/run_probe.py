"""Bounded synthetic TF experiment; protects R5 and uses localhost domain213."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from common import DOMAIN, summarize, verify_source


def require_no_robot(run=subprocess.run):
    pattern = ('[r]5_pi_session.py|[r]5_laptop_session.py|[s]ervices.mapping_runner|'
               '[s]erial_bridge_node|[n]ative_mjpeg_camera|[b]t_navigator|'
               '[s]afety_watchdog|[f]ire_robot_operator|[r]5_startup_health')
    row = run(['pgrep', '-af', pattern], stdout=subprocess.DEVNULL,
              stderr=subprocess.DEVNULL, timeout=3, check=False)
    if row.returncode != 1:
        raise RuntimeError('DIAGNOSTIC_REFUSES_ACTIVE_R5_OR_UNKNOWN_PROCESS_SNAPSHOT')


def experiment(source, output, duration, sampling, readers, delay_ms):
    require_no_robot()
    output.mkdir(mode=0o700)
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp', RMW_FASTRTPS_USE_QOS_FROM_XML='0')
    # A test must not inherit custom transport configuration aimed at hardware.
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI',
                'RMW_FASTRTPS_PUBLICATION_MODE'):
        env.pop(key, None)
    children, logs = [], []
    roles = ['fixture', 'fast', 'health'] + (['readers'] if readers == 'early' else [])
    interrupted = [False]
    previous = {}
    next_process_check = 0.
    def watch_robot():
        nonlocal next_process_check
        if time.monotonic() >= next_process_check:
            require_no_robot()
            next_process_check = time.monotonic() + .5
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: interrupted.__setitem__(0, True))
    try:
        for role in roles:
            log = (output / (role + '.log')).open('xb'); logs.append(log)
            argv = [sys.executable, '-B', str(Path(__file__).with_name('worker.py')), '--role', role,
                '--source', str(source), '--output', str(output), '--duration', str(duration + 16),
                '--sampling', sampling, '--delay-ms', str(delay_ms)]
            worker_env = dict(env)
            if role == 'fixture':
                worker_env['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'
            child = subprocess.Popen(argv, env=worker_env, cwd=output,
                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            children.append(child)
        ready_deadline = time.monotonic() + 12.
        while not all((output / (role + '.ready')).exists() for role in roles):
            watch_robot()
            if interrupted[0] or any(p.poll() is not None for p in children) or time.monotonic() >= ready_deadline:
                raise RuntimeError('DIAGNOSTIC_WORKER_NOT_READY_OR_CANCELLED')
            time.sleep(.05)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            watch_robot()
            if interrupted[0] or any(p.poll() is not None for p in children):
                raise RuntimeError('DIAGNOSTIC_WORKER_EXITED_OR_CANCELLED')
            time.sleep(.05)
    finally:
        # Only processes created in this function are signalled. No pgrep/pkill,
        # sessions, SSH runner or physical device is ever controlled here.
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGTERM)
        for child in children:
            try:
                child.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=2.)
        for log in logs:
            log.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if any(child.returncode != 0 for child in children):
        raise RuntimeError('DIAGNOSTIC_WORKER_FAILED; see private output logs')
    result = summarize(output)
    if any(result['trace_dropped'].values()):
        raise RuntimeError('DIAGNOSTIC_TRACE_CAPACITY_EXCEEDED')
    result.update(duration_s=duration, sampling=sampling, readers=readers, injected_delay_ms=delay_ms)
    if not all(result['metrics'][key]['count'] for key in (
            'tf_send_source_age_ms', 'fast_callback_source_age_ms', 'health_callback_source_age_ms', 'health_lookup_source_age_ms')):
        raise RuntimeError('DIAGNOSTIC_INSUFFICIENT_TF_EVIDENCE')
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration', type=float, default=10.)
    p.add_argument('--sampling', choices=['callback', 'timer'], default='callback')
    p.add_argument('--readers', choices=['none', 'early'], default='none')
    p.add_argument('--delay-ms', type=float, default=0.)
    p.add_argument('--matrix', action='store_true')
    a = p.parse_args()
    if not 5 <= a.duration <= 25 or not 0 <= a.delay_ms <= 30:
        p.error('duration5..25 seconds; synthetic delay0..30ms')
    source, output = a.source.resolve(strict=True), a.output.resolve()
    if output.is_relative_to(source) or a.output.exists():
        p.error('Use a new output folder outside protected R5 source')
    before = verify_source(source)
    require_no_robot()
    output.mkdir(mode=0o700)
    result = dict(scope='SYNTHETIC_ONLY_NO_ROBOT', source_before=before, cases={})
    try:
        cases = [('original_no_readers', 'callback', 'none'), ('original_early_readers', 'callback', 'early'),
                 ('sampled_early_readers', 'timer', 'early')] if a.matrix else [('case', a.sampling, a.readers)]
        for name, sampling, readers in cases:
            print('DIAGNOSTIC_CASE=' + name, flush=True)
            result['cases'][name] = experiment(source, output / name, a.duration, sampling, readers, a.delay_ms)
    finally:
        result['source_after'] = verify_source(source)
        result['r5_unchanged'] = result['source_after'] == before
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        files = sorted(p for p in output.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
        (output / 'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.relative_to(output).as_posix() + '\n' for p in files))
    print('DIAGNOSTIC_RESULT=' + str(output / 'result.json'), flush=True)


if __name__ == '__main__':
    main()
