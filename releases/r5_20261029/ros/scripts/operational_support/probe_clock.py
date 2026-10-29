"""Read clocks over one ready SSH stream; never set either clock."""
import argparse
import json
import os
from pathlib import Path
import selectors
import shlex
import subprocess
import time

REMOTE = r'''
import json,sys,time
print(json.dumps({'ready': True}), flush=True)
for line in sys.stdin:
    receive_wall, receive_mono = time.time_ns(), time.monotonic_ns()
    request = json.loads(line)
    send_wall, send_mono = time.time_ns(), time.monotonic_ns()
    print(json.dumps(dict(id=request['id'], receive_wall_ns=receive_wall,
        receive_mono_ns=receive_mono, send_wall_ns=send_wall,
        send_mono_ns=send_mono)), flush=True)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer', choices=['10.0.0.1', '10.42.0.185'], required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    report = dict(protocol='persistent_ssh_four_timestamps_v2', probes=[], narrowest=None,
                  note='Peer-minus-local interval bounds; no symmetric-link assumption. '
                       'Ready/handshake excluded, transport/executor queueing retained. '
                       'Snapshot only, not UTC accuracy or drift guarantee.')
    child = None
    error_path = args.out.with_suffix('.stderr.log')
    try:
        with error_path.open('w') as errors, selectors.DefaultSelector() as selector:
            bootstrap = time.monotonic_ns()
            child = subprocess.Popen(
                ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                 '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
                 'pi@' + args.peer, 'python3 -u -c ' + shlex.quote(REMOTE)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errors)
            selector.register(child.stdout, selectors.EVENT_READ)
            buffer = bytearray()

            def receive(seconds):
                deadline = time.monotonic() + seconds
                while b'\n' not in buffer:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not selector.select(remaining):
                        raise TimeoutError('SSH response deadline expired')
                    chunk = os.read(child.stdout.fileno(), 4096)
                    if not chunk:
                        raise RuntimeError('SSH EOF before response')
                    buffer.extend(chunk)
                    if len(buffer) > 32768:
                        raise ValueError('SSH response exceeds bound')
                line, _, tail = buffer.partition(b'\n')
                buffer[:] = tail
                return json.loads(line)

            if receive(10) != {'ready': True}:
                raise ValueError('Missing peer ready marker')
            report['bootstrap_ms'] = (time.monotonic_ns() - bootstrap) / 1e6
            for request_id in range(8):
                request = json.dumps({'id': request_id}).encode() + b'\n'
                before, mono_before = time.time_ns(), time.monotonic_ns()
                child.stdin.write(request)
                child.stdin.flush()
                response = receive(3)
                after, mono_after = time.time_ns(), time.monotonic_ns()
                if response['id'] != request_id:
                    raise ValueError('Response request id mismatch')
                peer_receive, peer_send = response['receive_wall_ns'], response['send_wall_ns']
                peer_mono = response['send_mono_ns'] - response['receive_mono_ns']
                lower, upper = (peer_send - after) / 1e6, (peer_receive - before) / 1e6
                local_delta = ((after - before) - (mono_after - mono_before)) / 1e6
                peer_delta = ((peer_send - peer_receive) - peer_mono) / 1e6
                valid = (after >= before and mono_after >= mono_before and
                         peer_send >= peer_receive and peer_mono >= 0 and
                         lower <= upper and abs(local_delta) < 10 and abs(peer_delta) < 10)
                report['probes'].append(dict(
                    id=request_id, local_before_ns=before, local_after_ns=after,
                    peer_receive_ns=peer_receive, peer_send_ns=peer_send,
                    rtt_ms=(mono_after - mono_before) / 1e6,
                    peer_minus_local_lower_ms=lower, peer_minus_local_upper_ms=upper,
                    width_ms=upper - lower, local_wall_vs_mono_delta_ms=local_delta,
                    peer_wall_vs_mono_delta_ms=peer_delta, valid=valid))
            good = [r for r in report['probes'] if r['valid']]
            if good:
                report['narrowest'] = min(good, key=lambda row: row['width_ms'])
                report['status'] = 'MEASURED'
            else:
                report['status'] = 'BLOCKED_NO_VALID_CLOCK_PROBE'
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError) as exc:
        report['status'] = 'BLOCKED_PROBE_ERROR'
        report['error'] = str(exc)
        # Partial probes remain evidence, but must not yield PASS after protocol failure.
        report['narrowest'] = None
    finally:
        if child is not None:
            if child.stdin:
                try:
                    child.stdin.close()
                except OSError as exc:
                    report['close_stdin_error'] = str(exc)
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=2)
            if child.stdout:
                child.stdout.close()
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print('CLOCK_PROBE_STATUS=' + report['status'])
    return 0 if report['status'] == 'MEASURED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
