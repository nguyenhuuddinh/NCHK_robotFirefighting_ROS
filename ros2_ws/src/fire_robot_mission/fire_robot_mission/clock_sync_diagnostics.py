"""Measure laptop-to-Pi wall-clock offset over a read-only SSH session."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import statistics
import subprocess
import tempfile
import time


def summarize_clock_samples(samples, max_rtt_ms, max_offset_ms):
    """Summarize midpoint clock samples and apply quality gates."""
    accepted = [
        sample for sample in samples
        if sample['rtt_ms'] <= max_rtt_ms
    ]
    report = {
        'samples_total': len(samples),
        'samples_accepted': len(accepted),
        'max_rtt_ms': float(max_rtt_ms),
        'max_offset_ms': float(max_offset_ms),
        'status': 'FAIL',
        'reason': '',
        'rtt_ms': None,
        'pi_minus_laptop_ms': None,
        'samples': samples,
    }
    if len(accepted) < 3:
        report['reason'] = 'fewer than 3 samples passed the RTT gate'
        return report
    rtts = [sample['rtt_ms'] for sample in accepted]
    offsets = [sample['pi_minus_laptop_ms'] for sample in accepted]
    median_offset = statistics.median(offsets)
    report['rtt_ms'] = {
        'min': round(min(rtts), 3),
        'median': round(statistics.median(rtts), 3),
        'max': round(max(rtts), 3),
    }
    report['pi_minus_laptop_ms'] = {
        'min': round(min(offsets), 3),
        'median': round(median_offset, 3),
        'max': round(max(offsets), 3),
    }
    if abs(median_offset) > max_offset_ms:
        report['reason'] = 'median clock offset exceeds configured limit'
        return report
    report['status'] = 'PASS'
    report['reason'] = 'RTT and median clock offset are within limits'
    return report


def _run_command(command, timeout, check=True):
    return subprocess.run(
        command,
        check=check,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def measure_peer_clock(
        peer, samples, interval_s, connect_timeout_s, sample_timeout_s):
    """Collect midpoint offset samples through one temporary SSH master."""
    results = []
    with tempfile.TemporaryDirectory(prefix='fire_robot_clock_') as temp_dir:
        socket_path = str(Path(temp_dir) / 'ssh_mux')
        common = [
            'ssh',
            '-S', socket_path,
            '-o', 'BatchMode=yes',
            '-o', f'ConnectTimeout={connect_timeout_s:g}',
        ]
        start = [
            'ssh', '-M', '-S', socket_path,
            '-o', 'BatchMode=yes',
            '-o', f'ConnectTimeout={connect_timeout_s:g}',
            '-fnNT', peer,
        ]
        _run_command(start, connect_timeout_s + 2.0)
        try:
            # Discard the first request because the control socket may still
            # be completing setup and would bias the RTT distribution.
            _run_command(
                common + [peer, 'date', '+%s%N'], sample_timeout_s)
            for index in range(samples):
                local_before = time.time_ns()
                completed = _run_command(
                    common + [peer, 'date', '+%s%N'], sample_timeout_s)
                local_after = time.time_ns()
                remote_ns = int(completed.stdout.strip())
                midpoint = (local_before + local_after) / 2.0
                results.append({
                    'index': index + 1,
                    'rtt_ms': round(
                        (local_after - local_before) / 1_000_000.0, 3),
                    'pi_minus_laptop_ms': round(
                        (remote_ns - midpoint) / 1_000_000.0, 3),
                })
                if index + 1 < samples:
                    time.sleep(interval_s)
        finally:
            _run_command(
                common + ['-O', 'exit', peer],
                sample_timeout_s,
                check=False,
            )
    return results


def _argument_parser():
    parser = argparse.ArgumentParser(
        description='Read-only laptop/Pi clock offset diagnostic over SSH')
    parser.add_argument(
        '--peer', default='pi@10.42.0.185',
        help='SSH destination for the Raspberry Pi')
    parser.add_argument(
        '--samples', type=int, default=10,
        help='number of measured samples after one warm-up')
    parser.add_argument(
        '--interval', type=float, default=0.10,
        help='seconds between samples')
    parser.add_argument(
        '--connect-timeout', type=float, default=5.0,
        help='SSH connection timeout in seconds')
    parser.add_argument(
        '--sample-timeout', type=float, default=3.0,
        help='timeout for each remote timestamp request')
    parser.add_argument(
        '--max-rtt-ms', type=float, default=50.0,
        help='discard samples slower than this RTT')
    parser.add_argument(
        '--max-offset-ms', type=float, default=20.0,
        help='absolute median offset limit for PASS')
    parser.add_argument(
        '--output', default='',
        help='optional path for the final JSON report')
    return parser


def main(args=None):
    """Run the bounded read-only SSH clock diagnostic."""
    parser = _argument_parser()
    options = parser.parse_args(args)
    if options.samples < 3:
        parser.error('--samples must be at least 3')
    positive = (
        options.interval,
        options.connect_timeout,
        options.sample_timeout,
        options.max_rtt_ms,
        options.max_offset_ms,
    )
    if any(value <= 0.0 for value in positive):
        parser.error('interval, timeouts and limits must be positive')

    print('=== FIRE ROBOT CLOCK DIAGNOSTICS START ===', flush=True)
    print(
        f'peer={shlex.quote(options.peer)} samples={options.samples} '
        'mode=READ_ONLY_SSH',
        flush=True,
    )
    try:
        samples = measure_peer_clock(
            options.peer,
            options.samples,
            options.interval,
            options.connect_timeout,
            options.sample_timeout,
        )
        report = summarize_clock_samples(
            samples, options.max_rtt_ms, options.max_offset_ms)
    except (
            OSError, ValueError, subprocess.SubprocessError) as exc:
        report = {
            'status': 'ERROR',
            'reason': f'{type(exc).__name__}: {exc}',
            'samples_total': 0,
            'samples_accepted': 0,
            'samples': [],
        }
    report['peer'] = options.peer
    report['generated_utc'] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(report, indent=2, sort_keys=True))
    if options.output:
        output_path = Path(options.output).expanduser()
        output_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n')
        print(f'REPORT_FILE {output_path}')
    print('=== FIRE ROBOT CLOCK DIAGNOSTICS END ===')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
