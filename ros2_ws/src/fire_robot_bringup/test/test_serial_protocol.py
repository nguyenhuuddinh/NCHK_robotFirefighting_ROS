"""Protocol V2 parser and encoders."""
from fire_robot_bringup.serial_protocol import SerialProtocolV2, crc16_ccitt_false


def test_crc_vectors():
    """Test known CRC vectors."""
    assert crc16_ccitt_false(b"CMD,2,1,0.000,0.000") == 0xB006
    assert crc16_ccitt_false(b"FIRE,2,2,0.000,0.000") == 0xE53B
    assert crc16_ccitt_false(b"PUMP,2,3,0") == 0xD1AA
    assert crc16_ccitt_false(b"STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000") == 0xB1C7
    assert crc16_ccitt_false(b"ENV,2,2,0,0.0,0.0,0.0,0") == 0x5A2E


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
