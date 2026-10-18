"""Protocol V2 parser and encoders."""
import math
import re

UINT_PATTERN = re.compile(r"^[0-9]+$")
FLOAT_PATTERN = re.compile(r"^-?[0-9]+\.[0-9]+$")
ELF_SHA256_PATTERN = re.compile(r"^[0-9a-fA-F]{64}$")


def is_strict_uint(s: str) -> bool:
    """Check if string is a strict unsigned integer."""
    return bool(UINT_PATTERN.match(s))


def is_strict_float(s: str) -> bool:
    """Check if string is a strict decimal float."""
    return bool(FLOAT_PATTERN.match(s))


def crc16_ccitt_false(data: bytes) -> int:
    """Calculate CRC16 CCITT FALSE."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SerialProtocolV2:
    """Raw Serial V2 Protocol Parser and Encoder."""

    def __init__(self):
        """Initialize protocol state."""
        self.buffer = bytearray()
        self.tx_seq = 0
        self.rx_seq = -1
        self.gap_count = 0
        self.dup_count = 0
        self.crc_fail_count = 0
        self.parse_fail_count = 0
        self.overflow_count = 0
        self.valid_count = 0

    def _commit_tx(self, prefix: str, suffix: str) -> bytes:
        """Commit TX frame if valid length."""
        next_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        payload = f"{prefix},{next_seq},{suffix}"
        crc = crc16_ccitt_false(payload.encode('ascii', errors='strict'))
        frame_str = f"@{payload}*{crc:04X}\n"
        frame_bytes = frame_str.encode('ascii')
        if len(frame_bytes) > 192:
            return None
        self.tx_seq = next_seq
        return frame_bytes

    def generate_cmd(self, vx: float, wz: float) -> bytes:
        """Generate CMD frame."""
        if math.isnan(vx) or math.isinf(vx) or math.isnan(wz) or math.isinf(wz):
            return None
        return self._commit_tx("CMD,2", f"{vx:.3f},{wz:.3f}")

    def generate_fire(self, x: float, y: float) -> bytes:
        """Generate FIRE frame."""
        if math.isnan(x) or math.isinf(x) or math.isnan(y) or math.isinf(y):
            return None
        if not (-1.0 <= x <= 1.0) or not (-1.0 <= y <= 1.0):
            return None
        return self._commit_tx("FIRE,2", f"{x:.3f},{y:.3f}")

    def generate_pump(self, on: int) -> bytes:
        """Generate PUMP frame."""
        if on not in (0, 1):
            return None
        return self._commit_tx("PUMP,2", f"{on}")

    def reset_parser(self):
        """Reset parser state."""
        self.buffer.clear()
        self.rx_seq = -1

    def parse_chunk(self, chunk: bytes):
        """Parse incoming chunk of bytes."""
        self.buffer.extend(chunk)

        while True:
            at_idx = self.buffer.find(b'@')
            if at_idx > 0:
                self.parse_fail_count += 1
                self.buffer = self.buffer[at_idx:]
            elif at_idx == -1:
                if len(self.buffer) > 0:
                    self.parse_fail_count += 1
                    self.buffer.clear()
                break

            lf_idx = self.buffer.find(b'\n')
            if lf_idx == -1:
                last_at = self.buffer.rfind(b'@')
                if last_at > 0:
                    self.parse_fail_count += 1
                    self.buffer = self.buffer[last_at:]
                    continue
                if len(self.buffer) > 192:
                    self.overflow_count += 1
                    self.buffer = self.buffer[1:]
                    continue
                break

            frame_with_nl = self.buffer[:lf_idx + 1]
            self.buffer = self.buffer[lf_idx + 1:]

            last_at = frame_with_nl.rfind(b'@')
            if last_at > 0:
                self.parse_fail_count += 1
                frame_with_nl = frame_with_nl[last_at:]

            if len(frame_with_nl) > 192:
                self.overflow_count += 1
                continue

            frame_bytes = frame_with_nl[:-1]
            if frame_bytes and frame_bytes[-1] == ord(b'\r'):
                frame_bytes = frame_bytes[:-1]

            frame_bytes = frame_bytes[1:]

            star_idx = frame_bytes.rfind(b'*')
            if star_idx == -1 or star_idx + 5 != len(frame_bytes):
                self.parse_fail_count += 1
                continue

            payload = frame_bytes[:star_idx]
            crc_str_bytes = frame_bytes[star_idx + 1:]

            try:
                crc_str = crc_str_bytes.decode('ascii', errors='strict')
            except ValueError:
                self.parse_fail_count += 1
                continue

            if len(crc_str) != 4 or not all(c in '0123456789ABCDEF' for c in crc_str):
                self.parse_fail_count += 1
                continue

            try:
                expected_crc = int(crc_str, 16)
            except ValueError:
                self.parse_fail_count += 1
                continue

            actual_crc = crc16_ccitt_false(payload)
            if actual_crc != expected_crc:
                self.crc_fail_count += 1
                continue

            try:
                payload_str = payload.decode('ascii', errors='strict')
            except ValueError:
                self.parse_fail_count += 1
                continue

            parts = payload_str.split(',')
            if len(parts) < 3:
                self.parse_fail_count += 1
                continue

            msg_type = parts[0]
            if not is_strict_uint(parts[1]) or not is_strict_uint(parts[2]):
                self.parse_fail_count += 1
                continue

            version = int(parts[1])
            seq = int(parts[2])

            if version != 2 or not (0 <= seq <= 0xFFFFFFFF):
                self.parse_fail_count += 1
                continue

            parsed = None
            if msg_type == 'STATE':
                if len(parts) == 10:
                    if not is_strict_uint(parts[3]):
                        self.parse_fail_count += 1
                        continue
                    if not all(is_strict_float(f) for f in parts[4:]):
                        self.parse_fail_count += 1
                        continue

                    esp_ms = int(parts[3])
                    x = float(parts[4])
                    y = float(parts[5])
                    yaw = float(parts[6])
                    vx = float(parts[7])
                    wz = float(parts[8])
                    gyro_z = float(parts[9])

                    if not (0 <= esp_ms <= 0xFFFFFFFF):
                        self.parse_fail_count += 1
                        continue

                    valid_floats = True
                    for val in [x, y, yaw, vx, wz, gyro_z]:
                        if math.isnan(val) or math.isinf(val):
                            valid_floats = False
                            break

                    if not valid_floats:
                        self.parse_fail_count += 1
                        continue

                    parsed = {
                        'type': 'STATE',
                        'seq': seq,
                        'esp_ms': esp_ms,
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'vx': vx,
                        'wz': wz,
                        'gyro_z': gyro_z
                    }
                else:
                    self.parse_fail_count += 1
                    continue
            elif msg_type == 'ENV':
                if len(parts) == 8:
                    if not is_strict_uint(parts[3]) or not is_strict_uint(parts[7]):
                        self.parse_fail_count += 1
                        continue
                    if not all(is_strict_float(f) for f in parts[4:7]):
                        self.parse_fail_count += 1
                        continue

                    fire_flags = int(parts[3])
                    gas_ppm = float(parts[4])
                    temp_c = float(parts[5])
                    batt_v = float(parts[6])
                    valid = int(parts[7])

                    if not (0 <= fire_flags <= 7):
                        self.parse_fail_count += 1
                        continue
                    if valid not in (0, 1):
                        self.parse_fail_count += 1
                        continue

                    valid_floats = True
                    for val in [gas_ppm, temp_c, batt_v]:
                        if math.isnan(val) or math.isinf(val):
                            valid_floats = False
                            break

                    if not valid_floats:
                        self.parse_fail_count += 1
                        continue

                    parsed = {
                        'type': 'ENV',
                        'seq': seq,
                        'fire_flags': fire_flags,
                        'gas_ppm': gas_ppm,
                        'temp_c': temp_c,
                        'batt_v': batt_v,
                        'valid': valid
                    }
                else:
                    self.parse_fail_count += 1
                    continue
            elif msg_type == 'IR':
                # IR,2,seq,sample_age_ms,left,center,right,valid
                if (len(parts) != 8 or
                        not all(is_strict_uint(f) for f in parts[3:])):
                    self.parse_fail_count += 1
                    continue
                age_ms, left, center, right, valid = map(int, parts[3:])
                if (age_ms > 0xFFFFFFFF or valid not in (0, 1) or
                        any(raw > 32767 for raw in (left, center, right)) or
                        (valid == 1 and age_ms > 300) or
                        (valid == 0 and any((left, center, right)))):
                    self.parse_fail_count += 1
                    continue
                parsed = {
                    'type': 'IR', 'seq': seq, 'sample_age_ms': age_ms,
                    'left': left, 'center': center, 'right': right,
                    'valid': valid
                }
            elif msg_type == 'BOOT':
                # One-shot diagnostic, never substitutes for STATE freshness.
                if (len(parts) != 6 or
                        not is_strict_uint(parts[3]) or
                        not is_strict_uint(parts[4]) or
                        (parts[5] != 'unknown' and
                         not ELF_SHA256_PATTERN.fullmatch(parts[5]))):
                    self.parse_fail_count += 1
                    continue
                esp_ms = int(parts[3])
                reset_reason = int(parts[4])
                if esp_ms > 0xFFFFFFFF or reset_reason > 0xFFFFFFFF:
                    self.parse_fail_count += 1
                    continue
                parsed = {
                    'type': 'BOOT', 'seq': seq, 'esp_ms': esp_ms,
                    'reset_reason': reset_reason, 'elf_sha256': parts[5],
                }
            elif msg_type == 'USB':
                # Diagnostic-only recovery snapshot; never a STATE substitute.
                # D5 adds five fields after short_write. Accept older 17-field
                # frames so Pi can be updated before the ESP firmware.
                names = (
                    'esp_ms', 'recoveries', 'recovery_ms', 'reason', 'offset',
                    'length', 'available', 'late_ms', 'task_gap_ms',
                    'tx_event_age_ms', 'tx_event_seen', 'deadline_expired',
                    'deadline_blocked', 'short_write',
                )
                if len(parts) not in (17, 22) or not all(
                        is_strict_uint(f) for f in parts[3:]):
                    self.parse_fail_count += 1
                    continue
                values = list(map(int, parts[3:]))
                fields = dict(zip(names, values[:len(names)]))
                if (any(v > 0xFFFFFFFF for v in values) or
                        fields['reason'] not in (0, 1, 2) or
                        fields['tx_event_seen'] not in (0, 1) or
                        not 0 <= fields['offset'] <= fields['length'] <= 192):
                    self.parse_fail_count += 1
                    continue
                if len(parts) == 22:
                    (snapshot_valid, cdc_connected, cdc_line_state,
                     raw_available, cached_dtr) = values[len(names):]
                    if (snapshot_valid not in (0, 1) or
                            cdc_connected not in (0, 1) or
                            cached_dtr not in (0, 1) or
                            cdc_line_state > 3 or
                            (not snapshot_valid and
                             (cdc_connected or cdc_line_state or
                              raw_available or cached_dtr))):
                        self.parse_fail_count += 1
                        continue
                    fields.update({
                        'fault_snapshot_valid': snapshot_valid,
                        'fault_cdc_connected': cdc_connected,
                        'fault_cdc_line_state': cdc_line_state,
                        'fault_raw_write_available': raw_available,
                        'fault_cached_dtr': cached_dtr,
                    })
                parsed = {'type': 'USB', 'seq': seq, **fields}
            elif msg_type == 'ACT':
                # Commanded PWM state only; no flow or nozzle feedback.
                # ACT,2,seq,esp_ms,pump_pwm,phase,angle,servo_pwm,home_count,pump_ready
                if len(parts) != 10 or not all(
                        is_strict_uint(f) for f in parts[3:]):
                    self.parse_fail_count += 1
                    continue
                (esp_ms, pump_pwm, phase, angle, servo_pwm,
                 home_count, pump_ready) = map(int, parts[3:])
                if (esp_ms > 0xFFFFFFFF or home_count > 0xFFFFFFFF or
                        pump_pwm not in (0, 1) or
                        servo_pwm not in (0, 1) or
                        pump_ready not in (0, 1) or phase not in (0, 1, 2, 3) or
                        (servo_pwm == 0 and angle != 255) or
                        (servo_pwm == 1 and not 0 <= angle <= 180) or
                        (pump_pwm == 1 and (phase != 1 or pump_ready != 1)) or
                        (phase == 1 and pump_pwm != 1) or
                        (phase == 2 and (pump_pwm != 0 or servo_pwm != 1)) or
                        (phase == 3 and (pump_pwm != 0 or servo_pwm != 0))):
                    self.parse_fail_count += 1
                    continue
                parsed = {
                    'type': 'ACT', 'seq': seq, 'esp_ms': esp_ms,
                    'pump_pwm': pump_pwm, 'servo_phase': phase,
                    'servo_angle_deg': angle, 'servo_pwm': servo_pwm,
                    'home_count': home_count, 'pump_ready': pump_ready,
                }
            else:
                self.parse_fail_count += 1
                continue

            if self.rx_seq != -1:
                diff = (seq - self.rx_seq) & 0xFFFFFFFF
                if diff == 0:
                    self.dup_count += 1
                    continue
                elif diff > 0x80000000:
                    self.dup_count += 1
                    continue
                else:
                    if diff > 1:
                        self.gap_count += (diff - 1)

            self.rx_seq = seq
            self.valid_count += 1
            yield parsed
