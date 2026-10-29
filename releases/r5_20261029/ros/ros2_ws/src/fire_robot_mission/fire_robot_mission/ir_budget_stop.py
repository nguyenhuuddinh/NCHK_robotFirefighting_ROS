"""A capped IR cycle may rotate/confirm range, never regain translation."""
import math

CAP_REASON = 'APPROACH_TOTAL_DISTANCE_LIMIT'
CAP_POLICY = 'ZERO_RECENTER_SPRAY_ONLY_V1'
COAST_ALLOWANCE_M = .03


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def cap_report_valid(report):
    if not isinstance(report, dict):return False
    limit = report.get('approach_distance_limit_m')
    distance = report.get('approach_distance_m')
    return bool(report.get('schema_version') == 1 and
        report.get('enabled') is True and report.get('one_shot') is True and
        report.get('integrated_handoff') is True and report.get('continuous_approach') is True and
        report.get('cap_stop_policy') == CAP_POLICY and
        report.get('forward_motion_inhibited') is True and
        report.get('approach_budget_exhausted') is True and
        number(limit) and 0 < limit <= 2. and number(distance) and
        limit <= distance <= limit + COAST_ALLOWANCE_M + .000001 and
        report.get('state') in ('STOPPING', 'COMPLETE') and
        type(report.get('step_count')) is int and
        report['step_count'] >= (1 if report['state'] == 'COMPLETE' else 0) and
        report.get('stop_reason') in (CAP_REASON, 'SPRAY_RANGE_REACHED') and
        report.get('command') == {'linear_x_m_s': 0., 'angular_z_rad_s': 0.} and
        (report.get('safety') or {}).get('pump_command_publisher') is False)
