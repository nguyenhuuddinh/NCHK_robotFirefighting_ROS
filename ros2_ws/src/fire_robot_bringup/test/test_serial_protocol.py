"""Protocol V2 parser and encoders."""
from fire_robot_bringup.serial_protocol import SerialProtocolV2, crc16_ccitt_false


def test_crc_vectors():
    """Test known CRC vectors."""
    assert crc16_ccitt_false(b"CMD,2,1,0.000,0.000") == 0xB006
    assert crc16_ccitt_false(b"FIRE,2,2,0.000,0.000") == 0xE53B
    assert crc16_ccitt_false(b"PUMP,2,3,0") == 0xD1AA
    assert crc16_ccitt_false(b"STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000") == 0xB1C7
    assert crc16_ccitt_false(b"ENV,2,2,0,0.0,0.0,0.0,0") == 0x5A2E


def test_usb_recovery_diagnostics_fragmented_bounded_and_crc_checked():
    payload = b'USB,2,2,5000,1,4500,1,64,100,0,20,120,300,1,3,3,0'
    wire = _create_bad_frame(payload)
    for chunk_size in (1, 7, 64, 192):
        parser = SerialProtocolV2()
        frames = []
        for i in range(0, len(wire), chunk_size):
            frames.extend(parser.parse_chunk(wire[i:i + chunk_size]))
        assert len(frames) == 1
        assert frames[0]['recoveries'] == 1
        assert frames[0]['recovery_ms'] == 4500
        assert frames[0]['deadline_blocked'] == 3
        assert frames[0]['type'] == 'USB'
        next_frame = _create_bad_frame(
            b'STATE,2,3,5100,0.000,0.000,0.000,0.000,0.000,0.000')
        assert list(parser.parse_chunk(next_frame))[0]['type'] == 'STATE'
        assert parser.gap_count == 0

    parser = SerialProtocolV2()
    values = payload.decode().split(',')
    for index, bad_value in ((3, '-1'), (4, '4294967296'), (6, '3'),
                             (7, '101'), (8, '193'), (13, '2')):
        bad = values.copy()
        bad[index] = bad_value
        assert list(parser.parse_chunk(_create_bad_frame(','.join(bad).encode()))) == []
    assert list(parser.parse_chunk(_create_bad_frame(b','.join(payload.split(b',')[:-1])))) == []
    corrupt = bytearray(wire)
    corrupt[12] ^= 1
    assert list(parser.parse_chunk(corrupt)) == []
    assert parser.crc_fail_count == 1
    assert parser.rx_seq == -1
    assert list(parser.parse_chunk(wire))[0]['reason'] == 1


def test_boot_identity_is_strict_diagnostic_and_keeps_sequence():
    sha = b'a' * 64
    payload = b'BOOT,2,10,12345,9,' + sha
    wire = _create_bad_frame(payload)
    parser = SerialProtocolV2()
    frames = []
    for i in range(0, len(wire), 7):
        frames.extend(parser.parse_chunk(wire[i:i + 7]))
    assert frames == [{
        'type': 'BOOT', 'seq': 10, 'esp_ms': 12345,
        'reset_reason': 9, 'elf_sha256': sha.decode(),
    }]
    next_state = _create_bad_frame(
        b'STATE,2,11,12400,0.000,0.000,0.000,0.000,0.000,0.000')
    assert list(parser.parse_chunk(next_state))[0]['type'] == 'STATE'
    assert parser.gap_count == 0

    for bad in (b'BOOT,2,12,1,9,xyz',
                b'BOOT,2,12,1,9,unknown,extra',
                b'BOOT,2,12,1,-1,unknown',
                b'BOOT,2,12,4294967296,9,unknown'):
        assert list(parser.parse_chunk(_create_bad_frame(bad))) == []
    assert list(parser.parse_chunk(_create_bad_frame(
        b'BOOT,2,12,1,0,unknown')))[0]['elf_sha256'] == 'unknown'


def test_encode_frames_and_tx_invalid():
    """Test generating frames and rejecting invalid bounds/types."""
    p = SerialProtocolV2()
    assert p.generate_cmd(0.0, 0.0) == b"@CMD,2,1,0.000,0.000*B006\n"
    assert p.tx_seq == 1
    assert p.generate_cmd(float('nan'), 0.0) is None
    assert p.tx_seq == 1
    assert p.generate_fire(2.0, -3.0) is None
    assert p.tx_seq == 1
    assert p.generate_fire(0.0, 0.0) == b"@FIRE,2,2,0.000,0.000*E53B\n"
    assert p.tx_seq == 2
    assert p.generate_pump(2) is None
    assert p.tx_seq == 2

    # Oversize TX test
    # A huge float will be rejected by length check since we formatted it.
    p.tx_seq = 1
    frame = p.generate_cmd(1e250, 0.0)
    assert frame is None
    assert p.tx_seq == 1


def test_parser_incremental_and_crlf():
    """Test incremental parsing and CRLF."""
    p = SerialProtocolV2()
    frames = list(p.parse_chunk(b"@STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000*B1C7\r\n"))
    assert len(frames) == 1
    frames = list(p.parse_chunk(b"@ENV,2,2,0,"))
    assert len(frames) == 0
    frames = list(p.parse_chunk(b"0.0,0.0,0.0,0*5A2E\n"))
    assert len(frames) == 1


def test_parser_garbage_and_mid_frame_at():
    """Test parsing with garbage and resync on @."""
    p = SerialProtocolV2()
    frames = list(p.parse_chunk(
        b"garbage@BROKEN@STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000*B1C7\n"
    ))
    assert len(frames) == 1
    assert p.valid_count == 1
    assert p.parse_fail_count > 0


def test_multiple_frames_in_large_chunk():
    """Test parsing multiple frames in a single chunk >192 bytes."""
    p = SerialProtocolV2()
    p1 = b"STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000"
    p2 = b"ENV,2,2,0,0.0,0.0,0.0,0"
    p3 = b"STATE,2,3,1234,0.000,0.000,0.000,0.000,0.000,0.000"
    p4 = b"ENV,2,4,0,0.0,0.0,0.0,0"
    p5 = b"STATE,2,5,1234,0.000,0.000,0.000,0.000,0.000,0.000"

    chunk = (
        f"@{p1.decode()}*{crc16_ccitt_false(p1):04X}\n"
        f"@{p2.decode()}*{crc16_ccitt_false(p2):04X}\n"
        f"@{p3.decode()}*{crc16_ccitt_false(p3):04X}\n"
        f"@{p4.decode()}*{crc16_ccitt_false(p4):04X}\n"
        f"@{p5.decode()}*{crc16_ccitt_false(p5):04X}\n"
    ).encode()
    assert len(chunk) > 192
    frames = list(p.parse_chunk(chunk))
    assert len(frames) == 5


def test_overflow_and_recovery():
    """Test per-frame overflow dropping and subsequent recovery."""
    p = SerialProtocolV2()
    frames1 = list(p.parse_chunk(b"@STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000*B1C7\n"))
    assert len(frames1) == 1
    long_junk = b"@" + b"X" * 200 + b"\n"
    frames2 = list(p.parse_chunk(long_junk))
    assert len(frames2) == 0
    assert p.overflow_count == 1
    frames3 = list(p.parse_chunk(b"@ENV,2,2,0,0.0,0.0,0.0,0*5A2E\n"))
    assert len(frames3) == 1


def _create_bad_frame(payload: bytes) -> bytes:
    """Wrap a bad payload with correct CRC."""
    return f"@{payload.decode()}*{crc16_ccitt_false(payload):04X}\n".encode()


def test_ir_frames_strict_and_shared_sequence():
    p = SerialProtocolV2()
    state = b"STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000"
    ir = b"IR,2,2,25,1100,1040,760,1"
    invalid_ir = b"IR,2,3,4294967295,0,0,0,0"
    env = b"ENV,2,4,0,0.0,0.0,0.0,0"
    frames = list(p.parse_chunk(b''.join(
        map(_create_bad_frame, (state, ir, invalid_ir, env)))))
    assert [frame['type'] for frame in frames] == ['STATE', 'IR', 'IR', 'ENV']
    assert frames[1]['left'] == 1100
    assert frames[1]['sample_age_ms'] == 25
    assert frames[2]['valid'] == 0
    assert p.gap_count == 0

    bad = (
        b"IR,2,5,301,1,2,3,1",        # stale marked valid
        b"IR,2,5,0,1,0,0,0",          # invalid with stale raw data
        b"IR,2,5,0,32768,0,0,1",      # ADS1115 signed max
        b"IR,2,5,-1,1,2,3,1",         # signed age
        b"IR,2,5,4294967296,1,2,3,0",  # age overflow
        b"IR,2,5,0,1,2,3,2",          # invalid boolean
        b"IR,2,5,0,1,2,3",            # missing field
    )
    for payload in bad:
        assert list(p.parse_chunk(_create_bad_frame(payload))) == []
    assert p.parse_fail_count >= len(bad)
    accepted = list(p.parse_chunk(_create_bad_frame(b"IR,2,5,0,1,2,3,1")))
    assert accepted[0]['seq'] == 5

    corrupted = bytearray(_create_bad_frame(b"IR,2,6,0,1,2,3,1"))
    corrupted[12] ^= 1
    assert list(p.parse_chunk(bytes(corrupted))) == []
    assert p.crc_fail_count >= 1


def test_actuator_command_telemetry_strict_and_shared_sequence():
    p = SerialProtocolV2()
    state = b'STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000'
    act_on = b'ACT,2,2,1300,1,1,70,1,0,1'
    act_home = b'ACT,2,3,1800,0,2,90,1,0,1'
    act_idle = b'ACT,2,4,2300,0,0,255,0,1,1'
    frames = list(p.parse_chunk(b''.join(map(
        _create_bad_frame, (state, act_on, act_home, act_idle)))))
    assert [frame['type'] for frame in frames] == [
        'STATE', 'ACT', 'ACT', 'ACT']
    assert frames[-1]['home_count'] == 1
    assert frames[1]['pump_pwm'] == 1
    assert p.gap_count == 0

    bad = (
        b'ACT,2,5,3000,1,2,90,1,1,1',  # pump ON while returning
        b'ACT,2,5,3000,1,1,90,1,1,0',  # pump not ready
        b'ACT,2,5,3000,0,3,90,1,1,1',  # locked but servo energized
        b'ACT,2,5,3000,0,0,90,0,1,1',  # detached angle not sentinel
        b'ACT,2,5,3000,0,0,255,1,1,1',  # active PWM missing angle
        b'ACT,2,5,3000,0,0,255,0,1',  # truncated
        b'ACT,2,5,-1,0,0,255,0,1,1',  # signed timestamp
        b'ACT,2,5,3000,0,0,255,0,4294967296,1',  # count overflow
    )
    for payload in bad:
        assert list(p.parse_chunk(_create_bad_frame(payload))) == []
    assert p.parse_fail_count >= len(bad)

    corrupted = bytearray(_create_bad_frame(b'ACT,2,5,3000,0,0,255,0,1,1'))
    corrupted[15] ^= 1
    assert list(p.parse_chunk(bytes(corrupted))) == []
    assert p.crc_fail_count >= 1


def test_parser_rejections():
    """Test strict lexical parsing and rejection of invalid fields."""
    p = SerialProtocolV2()

    # lowercase CRC is rejected because it doesn't match the strict 4-char hex format.
    # We will test CRC case manually because _create_bad_frame uses uppercase.
    frames = list(p.parse_chunk(b"@STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000*b1c7\n"))
    assert len(frames) == 0

    # whitespace version
    frames = list(p.parse_chunk(
        _create_bad_frame(b"STATE, 2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000")
    ))
    assert len(frames) == 0

    # sign + in seq
    frames = list(p.parse_chunk(
        _create_bad_frame(b"STATE,2,+1,1234,0.000,0.000,0.000,0.000,0.000,0.000")
    ))
    assert len(frames) == 0

    # whitespace float
    frames = list(p.parse_chunk(
        _create_bad_frame(b"STATE,2,1,1234, 0.000,0.000,0.000,0.000,0.000,0.000")
    ))
    assert len(frames) == 0

    # exponent float
    frames = list(p.parse_chunk(
        _create_bad_frame(b"STATE,2,1,1234,1e3,0.000,0.000,0.000,0.000,0.000")
    ))
    assert len(frames) == 0

    # esp_ms outside uint32
    frames = list(p.parse_chunk(
        _create_bad_frame(b"STATE,2,1,-1,0.000,0.000,0.000,0.000,0.000,0.000")
    ))
    assert len(frames) == 0

    # fire_flags outside 0..7
    frames = list(p.parse_chunk(_create_bad_frame(b"ENV,2,1,8,0.0,0.0,0.0,0")))
    assert len(frames) == 0


def test_sequence_tracking():
    """Test duplicate drop, gap count, and out-of-order sequence."""
    p = SerialProtocolV2()
    p1 = b"STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000"
    p0 = b"STATE,2,0,1234,0.000,0.000,0.000,0.000,0.000,0.000"
    p3 = b"STATE,2,3,1234,0.000,0.000,0.000,0.000,0.000,0.000"

    f1 = list(p.parse_chunk(_create_bad_frame(p1)))
    assert len(f1) == 1
    f2 = list(p.parse_chunk(_create_bad_frame(p1)))
    assert len(f2) == 0
    assert p.dup_count == 1

    f3 = list(p.parse_chunk(_create_bad_frame(p0)))
    assert len(f3) == 0
    assert p.dup_count == 2

    f4 = list(p.parse_chunk(_create_bad_frame(p3)))
    assert len(f4) == 1
    assert p.gap_count == 1

    p.rx_seq = 4294967295
    f5 = list(p.parse_chunk(_create_bad_frame(p0)))
    assert len(f5) == 1
    assert p.gap_count == 1
