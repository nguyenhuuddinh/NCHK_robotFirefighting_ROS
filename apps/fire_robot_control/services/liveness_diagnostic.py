"""Small session-bound diagnostic, never a physical STOP receipt."""
import json
import os
from pathlib import Path


def write_diagnostic(path, bootstrap, lease, now):
    path = Path(path)
    row = dict(schema_version=1, session=bootstrap.session, packet=bootstrap.packet,
        generation=bootstrap.generation, sample_mono=now, reason=lease.reason,
        last_heartbeat_mono=lease.last_heartbeat, sequence=lease.sequence,
        last_sent_mono=getattr(lease, 'last_sent_mono', None),
        max_delivery_delay_s=getattr(lease, 'max_delivery_delay_s', None),
        max_poll_gap_s=getattr(lease, 'max_poll_gap_s', None),
        scope='UI_LIVENESS_ONLY_NOT_STOP_PROOF')
    temp = path.with_name(path.name+'.tmp')
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(row, stream, allow_nan=False)
    os.replace(temp, path)
