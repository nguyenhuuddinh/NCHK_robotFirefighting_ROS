"""Read-only eligibility for one pre-ROS WiFi measurement-cache reset.

Chrony4.2 111 111 1101 means the delay-deviation filter rejected the sample.
Reset does not accept that packet: a new selected reference must still pass the
unchanged fresh_ntp_sample and laptop clock gates. Never change chrony options.
"""
import json
from pathlib import Path
import re
import sys
from verify_clock_freshness import assess_text
from fresh_ntp_sample import reference


def field(text, name):
    match = re.search(r'^' + re.escape(name) + r'\s*:\s*(.+)$', text, re.M)
    if not match:
        raise ValueError('Missing ' + name)
    return match.group(1).strip()


def decision(tracking, sources, before_data, after_data, baseline_tracking=None):
    try:
        age = assess_text(tracking)['reference_age_s']
        wifi = bool(re.search(r'^Reference ID\s*:.*\(10\.0\.0\.253\)\s*$', tracking, re.M))
        source = re.search(r'^\^-\s+10\.0\.0\.253\s+\d+\s+\d+\s+377\s+(\d+)\s+', sources, re.M)
        counts = [int(field(s, 'Total valid RX')) for s in (before_data, after_data)]
        exact_peer = all(field(s, 'Remote address').split()[0]=='10.0.0.253'
                         and field(s, 'Local address').split()[0]=='10.0.0.1'
                         and field(s, 'Leap status')=='Normal'
                         and field(s, 'Mode')=='Server' for s in (before_data, after_data))
        tests = [field(s, 'NTP tests').split() for s in (before_data, after_data)]
        rejected = ['111', '111', '1101']
        both_rejected = all(row == rejected for row in tests)
        recent_stuck = False
        if baseline_tracking is not None:
            # A previous accepted WiFi reference is still recent, but burst
            # replies can already be rejected against its old minimum delay.
            # Recovery changes no acceptance gate: after reset a NEW selected
            # reference <=15s old must pass, followed by the paired clock gate.
            recent_stuck = (
                reference(tracking) == reference(baseline_tracking)
                and field(baseline_tracking, 'Reference ID') ==
                field(tracking, 'Reference ID')
                and field(baseline_tracking, 'Leap status') == 'Normal'
                and age >= 8 and source is not None
                and int(source.group(1)) >= 8
                and tests[0] in (['111', '111', '1111'], rejected)
                and tests[1] == rejected and counts[1] - counts[0] >= 2)
        stale_stuck = (age > 120 and source is not None
                       and int(source.group(1)) > 120 and both_rejected)
        eligible = (wifi and field(tracking, 'Leap status') == 'Normal'
                    and exact_peer and 0 < counts[0] < counts[1]
                    and (stale_stuck or recent_stuck))
        return dict(eligible=bool(eligible), reference_age_s=age,
                    new_valid_replies=counts[1]-counts[0],
                    delay_filter_rejected=tests[1] == rejected,
                    before_delay_filter_rejected=tests[0] == rejected,
                    reason=('RECENT_WIFI_REFERENCE_NOT_ADVANCING' if eligible and recent_stuck
                            else 'STALE_WIFI_DELAY_FILTER' if eligible else 'NO_MATCHING_RECOVERY_EVIDENCE'),
                    action='ONE_PRE_ROS_CACHE_RESET' if eligible else 'NO_WIFI_RESET',
                    permission='No ROS or clock permission granted by this policy')
    except (ValueError, TypeError, OverflowError) as exc:
        return dict(eligible=False, action='NO_WIFI_RESET', error=str(exc))


if __name__=='__main__':
    if len(sys.argv) not in (5, 6):
        raise SystemExit('Expected tracking/sources/before_ntpdata/after_ntpdata [baseline_tracking] files')
    result=decision(*(Path(x).read_text() for x in sys.argv[1:]))
    print(json.dumps(result, allow_nan=False))
    raise SystemExit(0 if result['eligible'] else 2)
