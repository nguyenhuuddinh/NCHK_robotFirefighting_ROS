"""Pure classification for paired ESP uptime samples around one USB fault."""

ESP_MILLIS_MODULUS = 1 << 32
UPTIME_TOLERANCE_MS = 3000.0


def classify_esp_uptime(before, after):
    """Return evidence label; clock agreement alone never proves root cause."""
    if before is None or after is None:
        return 'missing_sample'
    old = before.get('esp_ms')
    new = after.get('esp_ms')
    old_mono = before.get('receive_mono_ns')
    new_mono = after.get('receive_mono_ns')
    if (type(old) is not int or type(new) is not int
            or not 0 <= old < ESP_MILLIS_MODULUS
            or not 0 <= new < ESP_MILLIS_MODULUS
            or type(old_mono) is not int
            or type(new_mono) is not int
            or new_mono < old_mono):
        return 'invalid_sample'
    pi_gap_ms = (new_mono - old_mono) / 1_000_000.0
    if pi_gap_ms >= ESP_MILLIS_MODULUS:
        return 'ambiguous_long_gap'
    uptime_gap_ms = (new - old) % ESP_MILLIS_MODULUS
    if abs(uptime_gap_ms - pi_gap_ms) <= UPTIME_TOLERANCE_MS:
        return 'wrap_consistent' if new < old else 'continuous_consistent'
    if new < old:
        return 'reset_suspected'
    return 'uptime_inconsistent'
