"""Fresh OFF telemetry contract, extracted from the accepted R3/R4 gate."""
import math

def off_fields(payload):
    return (isinstance(payload, dict)
            and payload.get('pump_pwm_commanded_on') is False
            and payload.get('servo_phase') == 'IDLE'
            and payload.get('servo_pwm_commanded_on') is False)


def fresh_off(payload, received, now):
    # Count source age at Pi publication as well as laptop receipt age.
    age = payload.get('receive_age_ms') if isinstance(payload, dict) else None
    return (off_fields(payload) and isinstance(age, (int, float))
            and not isinstance(age, bool) and math.isfinite(age) and age >= 0
            and 0 <= now-received and (now-received)*1000+age <= 350)


def fresh_motion_off(payload, received, now):
    """A current stationary actuator is enough for navigation, not for spray.

    LOCKED is accepted only with both PWM outputs OFF. RETURNING_HOME,
    SWEEPING, unknown phase or stale telemetry cannot release the drive.
    """
    if (not isinstance(payload, dict) or payload.get('schema_version') != 1
            or payload.get('mode') != 'COMMAND_TELEMETRY_ONLY'
            or payload.get('servo_phase') not in ('IDLE', 'LOCKED')):
        return False
    p = dict(payload, servo_phase='IDLE')
    return received is not None and fresh_off(p, received, now)
