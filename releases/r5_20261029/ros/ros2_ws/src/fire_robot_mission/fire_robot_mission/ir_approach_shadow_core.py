"""Read-only, fail-closed preview of a short IR-guided approach step."""

import math


def _fresh(received, now, timeout):
    return received is not None and 0.0 <= now - received <= timeout


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


VISION_ONLY_BLOCKERS = frozenset({
    'YOLO_MISSING_OR_STALE', 'YOLO_INVALID', 'YOLO_FIRE_INVALID',
    'YOLO_SOURCE_STALE_OR_INVALID', 'FIRE_NOT_CONFIRMED',
    'ALIGNMENT_MISSING_OR_STALE', 'FIRE_NOT_CENTERED',
    'ALIGNMENT_SAFETY_CONTRACT',
})


def validated_ir_band(payload):
    """Check the provisional IR-only band, ignoring only vision blockers."""
    if not isinstance(payload, dict):
        return None
    safety = payload.get('safety')
    counts = payload.get('filtered_counts')
    blockers = payload.get('blockers')
    if (payload.get('schema_version') != 1 or
            payload.get('mode') != 'OBSERVE_ONLY' or
            payload.get('observe_only') is not True or
            payload.get('calibration') != 'PROVISIONAL_SINGLE_FLAME' or
            payload.get('validated_for_actuation') is not False or
            not isinstance(safety, dict) or
            safety.get('motion_command_publisher') is not False or
            safety.get('actuator_command_publisher') is not False or
            not isinstance(blockers, list) or
            any(item not in VISION_ONLY_BLOCKERS for item in blockers) or
            not _finite(payload.get('ir_receive_age_ms')) or
            not 0 <= payload['ir_receive_age_ms'] <= 500.0 or
            type(payload.get('stable_samples')) is not int or
            payload['stable_samples'] < 3 or
            not isinstance(counts, dict) or
            any(type(counts.get(key)) is not int or
                not 0 <= counts[key] <= 32767
                for key in ('left', 'center', 'right'))):
        return None
    band = payload.get('ir_band')
    if band not in ('TOO_CLOSE', 'SPRAY_RANGE', 'APPROACH', 'TOO_FAR'):
        return None
    return band


def sector_clearance(scan, center_angle_rad, half_angle_rad, required_m):
    """Require two-sided sector coverage and enough finite obstacle returns."""
    if not isinstance(scan, tuple) or len(scan) != 5:
        return None, 'SCAN_INVALID'
    angle_min, increment, range_min, range_max, ranges = scan
    if (not all(_finite(v) for v in scan[:4]) or increment == 0 or
            range_min < 0 or range_max <= range_min or
            not isinstance(ranges, tuple) or len(ranges) < 90):
        return None, 'SCAN_INVALID'
    selected = []
    selected_angles = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * increment - center_angle_rad
        signed_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(signed_angle) <= half_angle_rad:
            selected.append(value)
            selected_angles.append(signed_angle)
    required_side = half_angle_rad - abs(increment) * 1.5
    if (len(selected) < 10 or required_side <= 0 or
            min(selected_angles, default=0) > -required_side or
            max(selected_angles, default=0) < required_side):
        return None, 'SCAN_SECTOR_COVERAGE_LOW'
    finite = [float(value) for value in selected
              if _finite(value) and range_min <= value <= range_max]
    invalid = [value for value in selected
               if not (_finite(value) and range_min <= value <= range_max)
               and value != math.inf]
    if len(finite) < 10 or len(invalid) > len(selected) // 5:
        return None, 'SCAN_SECTOR_UNOBSERVED'
    clearance = min(finite)
    if clearance < required_m:
        return clearance, 'OBSTACLE_OR_INSUFFICIENT_MARGIN'
    return clearance, None


class IrApproachShadowCore:
    """Suggest a bounded step; never authorize or produce a motor command."""

    def __init__(self, *, step_m=0.05, front_half_angle_rad=0.35,
                 front_margin_m=0.45, timeout_s=0.35):
        values = (step_m, front_half_angle_rad, front_margin_m, timeout_s)
        if any(not _finite(value) or value <= 0 for value in values):
            raise ValueError('approach parameters must be finite and positive')
        if (step_m > 0.05 or front_half_angle_rad < 0.30 or
                front_margin_m < 0.40 or timeout_s > 0.50):
            raise ValueError('approach hard safety bound exceeded')
        self.step_m = float(step_m)
        self.front_half_angle_rad = float(front_half_angle_rad)
        self.front_margin_m = float(front_margin_m)
        self.timeout_s = float(timeout_s)
        self.inputs = {}

    def update(self, name, value, now):
        self.inputs[name] = (value, float(now))

    def _input(self, name, now):
        value, received = self.inputs.get(name, (None, None))
        if not _fresh(received, now, self.timeout_s):
            return None
        return value

    def _front_clearance(self, scan):
        clearance, error = sector_clearance(
            scan, 0.0, self.front_half_angle_rad,
            self.front_margin_m + self.step_m)
        names = {
            'SCAN_SECTOR_COVERAGE_LOW': 'SCAN_FRONT_COVERAGE_LOW',
            'SCAN_SECTOR_UNOBSERVED': 'SCAN_FRONT_UNOBSERVED',
            'OBSTACLE_OR_INSUFFICIENT_MARGIN':
                'FRONT_OBSTACLE_OR_INSUFFICIENT_MARGIN',
        }
        return clearance, names.get(error, error)

    def snapshot(self, now, counts):
        """Return sensor-only telemetry; not a real-motion admission proof."""
        now = float(now)
        blockers = []
        for key in ('ir', 'scan', 'odom', 'cmd'):
            if counts.get(key) != 1:
                blockers.append(key.upper() + '_PUBLISHER_INVALID')
        if counts.get('raw') != 0:
            blockers.append('RAW_MOTION_OWNER_ACTIVE')

        ir = self._input('ir', now)
        band = validated_ir_band(ir)
        if band is None:
            blockers.append('IR_NOT_TRUSTWORTHY_OR_STALE')
        if band not in ('APPROACH', 'TOO_FAR'):
            blockers.append('IR_ZONE_NOT_APPROACH')

        odom = self._input('odom', now)
        if (not isinstance(odom, tuple) or len(odom) != 6 or
                not all(_finite(v) for v in odom) or
                any(abs(v) > 0.02 for v in odom[:3]) or
                any(abs(v) > 0.05 for v in odom[3:])):
            blockers.append('ODOM_NOT_FRESH_STATIONARY')
        cmd = self._input('cmd', now)
        if (not isinstance(cmd, tuple) or len(cmd) != 6 or
                not all(_finite(v) for v in cmd) or
                any(abs(v) > 0.001 for v in cmd)):
            blockers.append('CMD_VEL_NOT_FRESH_ZERO')

        scan = self._input('scan', now)
        clearance, scan_error = self._front_clearance(scan)
        if scan_error:
            blockers.append(scan_error)

        return {
            'schema_version': 1,
            'mode': 'SHADOW_ONLY',
            'state': 'STEP_CANDIDATE' if not blockers else 'HOLD',
            'ir_zone': band if band is not None else 'UNKNOWN',
            'vision_zone': ir.get('zone') if isinstance(ir, dict) else None,
            'suggested_step_m': self.step_m if not blockers else 0.0,
            'front_clearance_m': clearance,
            'blockers': blockers,
            'validated_for_actuation': False,
            'note': 'Sensor preview only; no Nav2 cancel or motion authority',
            'safety': {
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
            },
        }
