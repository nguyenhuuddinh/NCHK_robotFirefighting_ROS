"""Require a new Pi chrony reference after bounded pre-ROS refresh; no I/O to clock."""
import datetime
import json
from pathlib import Path
import re
import sys
from verify_clock_freshness import assess_text


def reference(text):
    match=re.search(r'^Ref time \(UTC\)\s*:\s*(.+)$',text,re.M)
    if not match:raise ValueError('Missing chrony reference')
    return datetime.datetime.strptime(match.group(1).strip(),'%a %b %d %H:%M:%S %Y').replace(tzinfo=datetime.timezone.utc)


def assess(before,after):
    try:
        info=assess_text(after)
        advanced=reference(after)>reference(before)
        selected=bool(re.search(r'^Reference ID\s*:.*\(10\.0\.0\.253\)\s*$',after,re.M))
        normal=bool(re.search(r'^Leap status\s*:\s*Normal\s*$',after,re.M))
        ok=advanced and selected and normal and 0<=info['reference_age_s']<=15
        return dict(status='FRESH_WIFI_REFERENCE' if ok else 'WAIT_NEW_WIFI_REFERENCE',
            advanced=advanced,wifi_reference=selected,leap_normal=normal,reference_age_s=info['reference_age_s'],
            required_max_age_s=15,scope='Pre-ROS refresh only; downstream two-sample50ms/120s gate unchanged')
    except (ValueError,OverflowError,TypeError) as exc:
        return dict(status='INVALID_REFERENCE_EVIDENCE',error=str(exc))


if __name__=='__main__':
    result=assess(Path(sys.argv[1]).read_text(),Path(sys.argv[2]).read_text())
    print(json.dumps(result))
    raise SystemExit(0 if result['status']=='FRESH_WIFI_REFERENCE' else 2)
