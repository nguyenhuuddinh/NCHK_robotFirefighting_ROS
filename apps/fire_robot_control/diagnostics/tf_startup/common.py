"""Pure diagnostic helpers. No ROS, networking, devices or source writes."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

DOMAIN = '213'
R5_PACKET = '0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0'


def require_isolation():
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('DIAGNOSTIC_REQUIRES_LOCALHOST_DOMAIN_213')


def verify_source(root):
    root = Path(root).resolve(strict=True)
    raw = (root / 'operational_packet.sha256').read_bytes()
    if hashlib.sha256(raw).hexdigest() != R5_PACKET:
        raise ValueError('DIAGNOSTIC_REQUIRES_PINNED_R5_SOURCE')
    seen = set()
    for line in raw.decode().splitlines():
        digest, name = line.split('  ', 1)
        path = (root / name).resolve(strict=True)
        if (name in seen or not path.is_relative_to(root)
                or hashlib.sha256(path.read_bytes()).hexdigest() != digest):
            raise ValueError('R5_SOURCE_CHANGED:' + name)
        seen.add(name)
    return dict(packet=R5_PACKET, pins=len(seen))


def stats(values):
    values = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not values:
        return dict(count=0, p50=None, p95=None, p99=None, maximum=None)
    def q(fraction):
        return values[max(0, math.ceil(len(values) * fraction) - 1)]
    return dict(count=len(values), p50=q(.5), p95=q(.95), p99=q(.99), maximum=values[-1])


def source_age_ms(stamp_ns, wall_ns):
    # R5 stamp = Pi odom receive time + 100 ms. Undo only this pinned profile.
    return (wall_ns - stamp_ns) / 1e6 + 100.


def stamp_ns(transform):
    stamp = transform.header.stamp
    return stamp.sec * 10**9 + stamp.nanosec


class Trace:
    def __init__(self, limit=20000):
        self.events, self.limit, self.dropped = [], limit, 0

    def add(self, event, **fields):
        if len(self.events) >= self.limit:
            self.dropped += 1
            return
        self.events.append(dict(event=event, mono_ns=time.monotonic_ns(), wall_ns=time.time_ns(), **fields))

    def save(self, output, role):
        output = Path(output)
        with (output / (role + '.json')).open('x') as stream:
            json.dump(dict(scope='SYNTHETIC_LOCALHOST_NO_ROBOT', role=role,
                trace_dropped=self.dropped, events=self.events), stream, allow_nan=False)
            stream.write('\n')


def summarize(folder):
    folder = Path(folder)
    streams = {role: json.loads((folder / (role + '.json')).read_text())
               for role in ('fixture', 'health', 'fast')}
    # Ignore each worker's first two seconds of discovery; retain raw events.
    def values(role, event, key):
        rows = streams[role]['events']
        start = rows[0]['mono_ns'] if rows else 0
        return [row[key] for row in rows if row['event'] == event and key in row
                and row['mono_ns'] >= start + 2 * 10**9 and row[key] is not None]
    metrics = {}
    for label, role, event, key in (
        ('tf_send_source_age_ms', 'fixture', 'tf_send', 'source_age_ms'),
        ('fast_callback_source_age_ms', 'fast', 'tf_callback', 'source_age_ms'),
        ('health_callback_source_age_ms', 'health', 'tf_callback', 'source_age_ms'),
        ('health_lookup_source_age_ms', 'health', 'sample', 'tf_source_age_ms'),
        ('sample_cost_ms', 'health', 'sample', 'sample_cost_ms'),
        ('write_cost_ms', 'health', 'sample', 'write_cost_ms')):
        metrics[label] = stats(values(role, event, key))
    return dict(scope='SYNTHETIC_ONLY_NOT_ROBOT_ROOT_CAUSE_PROOF', metrics=metrics,
        trace_dropped={role: row['trace_dropped'] for role, row in streams.items()},
        unique_tf_stamp_assumption=False,
        note='TF stamps repeat at 20Hz with 10Hz odom; source ages are measured, not exact per-message transit.')
