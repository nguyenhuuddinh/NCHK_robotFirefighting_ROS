"""Assess saved time/source evidence; no networking or state changes."""
import json
from pathlib import Path
import re
import sys


def main():
    out = Path(sys.argv[1])
    probe = json.loads((out / 'clock_pair.json').read_text())
    best = probe.get('narrowest')
    source = (out / 'pi_sources_after.log').read_text()
    selected = any(re.match(r'^\^\*\s+10\.0\.0\.253\s', line)
                   for line in source.splitlines())
    flags = {host: 'NTPSynchronized=yes' in (out / (host + '_clock.log')).read_text().splitlines()
             for host in ['laptop', 'pi']}
    if best is None:
        alignment = 'BLOCKED_NO_VALID_PROBE'
    elif best['peer_minus_local_upper_ms'] < -50 or best['peer_minus_local_lower_ms'] > 50:
        alignment = 'BLOCKED_CLOCK_OFFSET'
    elif best['peer_minus_local_lower_ms'] < -50 or best['peer_minus_local_upper_ms'] > 50:
        alignment = 'INCONCLUSIVE_SSH_BOUNDS_TOO_WIDE'
    else:
        alignment = 'CLOCK_ALIGNMENT_WITHIN_BUDGET'
    if alignment != 'CLOCK_ALIGNMENT_WITHIN_BUDGET':
        verdict = alignment
    elif not selected:
        verdict = 'WAIT_OR_BLOCKED_WIFI_NTP_SOURCE_NOT_SELECTED'
    elif not all(flags.values()):
        verdict = 'BLOCKED_NTP_FLAG_NOT_CONFIRMED'
    else:
        verdict = 'EXISTING_CHRONY_WIFI_PAIR_OK'
    result = dict(verdict=verdict, clock_alignment=alignment,
                  pi_selected_laptop_wifi_source=selected, ntp_flags=flags,
                  narrowest=best, measurement_budget_ms=50,
                  note='Alignment and source selection are separate checks. '
                       'No robot300ms/ACT350ms guard change; not loaded WiFi acceptance.')
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if verdict == 'EXISTING_CHRONY_WIFI_PAIR_OK' else 2


if __name__ == '__main__':
    raise SystemExit(main())
