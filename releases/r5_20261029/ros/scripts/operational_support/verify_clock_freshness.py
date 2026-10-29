"""Assess saved Pi NTP reference age, including date -Ins on Python 3.10."""
import datetime
import json
from pathlib import Path
import re
import sys

EPOCH = datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
TIMESTAMP = re.compile(
    r"(?P<seconds>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})"
    r"(?:[.,](?P<fraction>\d{1,9}))?(?P<zone>Z|[+-]\d{2}:\d{2})")


def epoch_ns(value):
    """Require an aware ISO timestamp; avoid float epochs and 3.11-only parsing."""
    match = TIMESTAMP.fullmatch(value.strip())
    if match is None:
        raise ValueError('Expected ISO timestamp with timezone and 0-9 fractional digits')
    zone = match.group('zone')
    if zone != 'Z' and (int(zone[1:3]) > 23 or int(zone[4:6]) > 59):
        raise ValueError('Invalid timezone offset')
    # Python3.10 handles the whole-second portion with an explicit offset.
    whole = datetime.datetime.fromisoformat(
        match.group('seconds') + ('+00:00' if zone == 'Z' else zone))
    delta = whole - EPOCH
    fraction = int((match.group('fraction') or '').ljust(9, '0'))
    return (delta.days * 86400 + delta.seconds) * 1_000_000_000 + fraction


def assess_text(text):
    wall = next((line.strip() for line in text.splitlines()
                 if re.match(r'^\d{4}-\d{2}-\d{2}T', line)), None)
    if wall is None:
        raise ValueError('Pi wall timestamp unavailable')
    match = re.search(r'^Ref time \(UTC\)\s*:\s*(.+)$', text, re.M)
    if match is None:
        raise ValueError('Pi chrony reference time unavailable')
    reference = datetime.datetime.strptime(
        match.group(1).strip(), '%a %b %d %H:%M:%S %Y').replace(
            tzinfo=datetime.timezone.utc)
    delta = reference - EPOCH
    reference_ns = (delta.days * 86400 + delta.seconds) * 1_000_000_000
    age_ns = epoch_ns(wall) - reference_ns
    valid = -5_000_000_000 <= age_ns <= 120_000_000_000
    return dict(reference_age_ns=age_ns, reference_age_s=age_ns / 1_000_000_000,
                reference_max_age_s=120, reference_future_allowance_s=5,
                status='PASS' if valid else 'BLOCKED_STALE_OR_FUTURE_REFERENCE',
                scope='Before ROS snapshot only; loaded clock stability must be reviewed')


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        raise SystemExit('Usage: verify_clock_freshness.py SAVED_CLOCK_DIRECTORY')
    out = Path(args[0])
    try:
        result = assess_text((out / 'pi_clock.log').read_text())
    except (OSError, ValueError, OverflowError) as exc:
        result = dict(status='BLOCKED_INVALID_CLOCK_EVIDENCE', error=str(exc))
    (out / 'freshness.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
