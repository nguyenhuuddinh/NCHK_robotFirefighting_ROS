"""Protocol V2 parser and encoders."""
import math
import re

UINT_PATTERN = re.compile(r"^[0-9]+$")
FLOAT_PATTERN = re.compile(r"^-?[0-9]+\.[0-9]+$")


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
