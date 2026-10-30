"""Fire-app-only instrumentation around unchanged R5 health decisions/schedule."""
import argparse
from pathlib import Path
import os
import sys
import time

from .diagnostic_stream import DiagnosticStream


def health_command(*args):
    return [sys.executable, '-B', '-m', 'services.fire_health_entry', *map(str, args)]


def instrument(original, stream, session, packet):
    class Observed(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            report = self.core.report
            self._app_detail = {}
            def observed(now, **fields):
                row = report(now, **fields)
                # Values passed to the actual policy, no new graph/TF queries.
                value, at = self.core.inputs.get('pose', (None, None))
                initial = self.core.initial_pose_at
                self._app_detail = dict(
                    pose_source=fields['sources'].get('pose') is True,
                    pose_valid=value is True, initial_pose_seen=initial is not None,
                    pose_after_initial=at is not None and initial is not None and at >= initial,
                    map_tf=fields.get('tf_ok') is True,
                    action_ready=fields.get('action_ready') is True,
                    lifecycle=fields.get('lifecycle_ok') is True,
                    identity_present=row.get('identity_key') is not None)
                return row
            self.core.report = observed

        def sample(self):
            start = time.monotonic()
            row = super().sample()
            finished = time.monotonic()
            if stream.due():
                detail = self._app_detail
                stream.emit(dict(schema_version=1, scope='PERFORMANCE_ONLY_NOT_READINESS_OR_STOP_PROOF',
                    session=session, packet=packet, sample_mono=finished, sample_ms=(finished-start)*1000,
                    policy_sample_age_ms=(finished-row['sample_mono'])*1000,
                    localization=detail, localization_blockers=[k for k,v in detail.items() if not v],
                    checks=row['checks'], stable_s=row['stable_s'],
                    lifecycle_services=row.get('io_diagnostics', {}).get('detail', {}).get('control', {}),
                    runtime_fault=row.get('runtime_recovery', {}).get('operator', {}).get('fault')))
            return row
    return Observed


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--packet', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--session', required=True)
    args, _ = parser.parse_known_args(argv)
    values = list(sys.argv[1:] if argv is None else argv)
    index = values.index('--packet')
    del values[index:index+2]
    from fire_robot_mission import startup_health as original
    old = original.StartupHealthNode
    stream = DiagnosticStream(args.out.with_name('health_diag_'+str(os.getpid())+'.jsonl'))
    original.StartupHealthNode = instrument(old, stream, args.session, args.packet)
    try:
        return original.main(values)
    finally:
        original.StartupHealthNode = old
        stream.close()


if __name__ == '__main__':
    raise SystemExit(main())
