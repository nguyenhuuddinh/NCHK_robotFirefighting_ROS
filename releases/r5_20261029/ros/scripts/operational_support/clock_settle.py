"""Bounded, read-only wait for existing clock checks; never changes either clock."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

PASS = 'CLOCK_SETTLED'
WAIT = 'WAIT_CLOCK_SETTLE'
RETRY = {'BLOCKED_CLOCK_OFFSET', 'INCONCLUSIVE_SSH_BOUNDS_TOO_WIDE',
         'WAIT_OR_BLOCKED_WIFI_NTP_SOURCE_NOT_SELECTED',
         'BLOCKED_NTP_FLAG_NOT_CONFIRMED'}


def read_observation(folder, rc):
    """Only diagnosed convergence can be retried, not script/protocol errors."""
    try:
        result = json.loads((folder / 'result.json').read_text())
        best = result['narrowest']
        bounds = [best['peer_minus_local_lower_ms'], best['peer_minus_local_upper_ms']]
        if (result['measurement_budget_ms'] != 50 or best['valid'] is not True
                or not all(type(v) in (int, float) and math.isfinite(v) for v in bounds)
                or bounds[0] > bounds[1]):
            raise ValueError('Invalid interval or changed budget')
        reason = result['verdict']
        if rc == 2 and reason in RETRY:
            return dict(state='WAIT', reason=reason, bounds_ms=bounds)
        freshness = json.loads((folder / 'freshness.json').read_text())
        if rc == 2 and freshness['status'] == 'BLOCKED_STALE_OR_FUTURE_REFERENCE':
            return dict(state='WAIT', reason=freshness['status'], bounds_ms=bounds)
        if (rc == 0 and reason == 'EXISTING_CHRONY_WIFI_PAIR_OK'
                and -50 <= bounds[0] <= bounds[1] <= 50
                and result['pi_selected_laptop_wifi_source'] is True
                and result['ntp_flags'] == {'laptop': True, 'pi': True}
                and freshness['status'] == 'PASS'):
            return dict(state='GOOD', reason=reason, bounds_ms=bounds)
        raise ValueError('Check failed without a retryable convergence result')
    except (OSError, ValueError, TypeError, KeyError) as exc:
        return dict(state='FATAL', reason='BLOCKED_CLOCK_EVIDENCE', error=str(exc), rc=rc)


class SettleWindow:
    def __init__(self, start, limit=120.0, separation=5.0):
        self.start = self.last = start
        self.limit, self.separation = limit, separation
        self.good_since = None

    def observe(self, now, observation):
        if not math.isfinite(now) or now < self.last:
            return 'BLOCKED_MONOTONIC_CLOCK'
        self.last = now
        if now - self.start >= self.limit:
            return 'BLOCKED_CLOCK_SETTLE_TIMEOUT'
        if observation['state'] == 'FATAL':
            return observation['reason']
        if observation['state'] != 'GOOD':
            self.good_since = None
            return WAIT
        if self.good_since is None:
            self.good_since = now
        elif now - self.good_since >= self.separation:
            return PASS
        return WAIT


def stop_group(child):
    # Only our clock probe session and its SSH children.
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=1)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    child.wait(timeout=2)


def run_attempt(command, log, seconds):
    with log.open('x') as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            return child.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            stop_group(child)
            return 124
        except BaseException:
            stop_group(child)
            raise


def settle(out, peer):
    out.mkdir(mode=0o700)
    root = Path(__file__).resolve().parent
    start = time.monotonic()
    window = SettleWindow(start)
    attempts = []
    error = None
    status = 'BLOCKED_CLOCK_SETTLE_TIMEOUT'
    print('CLOCK_WAIT: at most120s; two fresh passes >=5s apart; budget remains50ms.', flush=True)
    try:
        while time.monotonic() - start < 120:
            index = len(attempts) + 1
            folder = out / f'attempt_{index:02d}'
            remaining = 120 - (time.monotonic() - start)
            if remaining <= 0:
                break
            print(f'CLOCK_PROBE={index} remaining_s={remaining:.1f}', flush=True)
            rc = run_attempt(['bash', str(root / 'clock_once.sh'), str(folder), peer],
                             out / f'attempt_{index:02d}.log', min(40, remaining))
            observation = read_observation(folder, rc)
            now = time.monotonic()
            status = window.observe(now, observation)
            attempts.append({**observation, 'attempt':index, 'elapsed_s':now-start, 'rc':rc})
            print(f'CLOCK_CHECK={index} reason={observation["reason"]} '
                  f'offset_bounds_ms={observation.get("bounds_ms")} elapsed_s={now-start:.1f}', flush=True)
            if status != WAIT:
                break
            time.sleep(min(5, max(0, 120 - (time.monotonic() - start))))
        if status == WAIT:
            status = 'BLOCKED_CLOCK_SETTLE_TIMEOUT'
        if status == PASS:
            for name in ['clock_pair.json', 'result.json', 'freshness.json',
                         'laptop_clock.log', 'pi_clock.log', 'pi_sources_after.log']:
                shutil.copy2(folder / name, out / name)
    except (KeyboardInterrupt, InterruptedError):
        status = 'BLOCKED_CLOCK_INTERRUPTED'
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        status = 'BLOCKED_CLOCK_TOOL_ERROR'
        error = str(exc)
    finally:
        report = dict(status=status, error=error, limit_s=120, elapsed_s=time.monotonic()-start,
                      measurement_budget_ms=50, pass_separation_s=5, attempts=attempts,
                      scope='Pre-ROS pair alignment only; not loaded drift or robot acceptance.')
        (out / 'settle.json').write_text(json.dumps(report, indent=2) + '\n')
        print('CLOCK_FINAL=' + status, flush=True)
    return 0 if status == PASS else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out', type=Path)
    parser.add_argument('peer', choices=['10.0.0.253'])
    args = parser.parse_args()

    def interrupt(_signum, _frame):
        raise InterruptedError('operator signal')

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    return settle(args.out, args.peer)


if __name__ == '__main__':
    raise SystemExit(main())
