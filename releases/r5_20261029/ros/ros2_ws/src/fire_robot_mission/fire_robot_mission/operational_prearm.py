"""R4 pre-arm IR continuity, kept separate from permission freshness.

Port of qa_r4d_ir_owner_tools_20261026/r4d_prearm.py. The raw bridge sample
qualifies standby for 600 ms; an explicit A grants only with a valid sample
received within 350 ms. Live approach/pulse still use their strict preview
checks. No commands, subscriptions or timers are created here.
"""
import math


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def ir_readiness(latest, now):
    payload, received = latest.get('ir_raw', ({}, None))
    age = now - received if number(now) and number(received) else None
    valid = (isinstance(payload, dict) and type(payload.get('schema_version')) is int
             and payload.get('schema_version') == 1
             and payload.get('valid') is True
             and all(type(payload.get(k)) is int and 0 <= payload[k] <= 32767
                     for k in ('left', 'center', 'right'))
             and all(number(payload.get(k)) and 0 <= payload[k] <= 300
                     for k in ('sample_age_ms', 'receive_age_ms')))
    continuity = bool(valid and number(age) and 0 <= age <= .6)
    fresh = bool(continuity and age <= .35)
    return dict(valid=bool(valid), continuity=continuity, grant_fresh=fresh,
                receive_age_ms=age * 1000 if number(age) else None)
