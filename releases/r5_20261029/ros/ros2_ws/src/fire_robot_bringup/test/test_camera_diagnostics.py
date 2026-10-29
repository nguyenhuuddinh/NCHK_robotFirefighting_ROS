"""Offline checks for the bounded camera diagnostics command."""

from fire_robot_bringup.camera_diagnostics import TopicStats
from sensor_msgs.msg import CompressedImage


def test_empty_topic_has_copyable_summary():
    """Absent camera publishers must report zero instead of hanging."""
    assert TopicStats(image=True).summary(10.0).startswith(
        'frames=0 rx_hz=0.00 max_gap_s=0.000 avg_jpeg_bytes=0'
    )


def test_image_stats_do_not_keep_payloads():
    """Diagnostics count bytes, stamps and gaps but retain no frames."""
    stats = TopicStats(image=True)
    msg = CompressedImage()
    msg.header.frame_id = 'camera_frame'
    msg.header.stamp.sec = 42
    msg.data = [0xff, 0xd8, 3, 4, 0xff, 0xd9]
    stats.record(msg, now=1.0, wall_time=42.1)
    stats.record(msg, now=1.2, wall_time=42.2)

    summary = stats.summary(2.0)
    assert 'frames=2' in summary
    assert 'rx_hz=5.00' in summary
    assert 'max_gap_s=0.200' in summary
    assert 'avg_jpeg_bytes=6' in summary
    assert 'payload_Bps=6' in summary
    assert 'valid_jpeg=2/2' in summary
    assert 'wall_age_ms_avg=150.0' in summary
    assert 'wall_age_ms_max=200.0' in summary
    assert 'frame_id=camera_frame' in summary
    assert 'last_stamp=42.000000000' in summary
    assert not hasattr(stats, 'messages')


def test_invalid_jpeg_is_counted_without_retaining_payload():
    """A malformed camera message remains visible in the summary."""
    stats = TopicStats(image=True)
    msg = CompressedImage()
    msg.data = [0xff, 0xd8, 1, 2]
    stats.record(msg, now=1.0)

    assert 'valid_jpeg=0/1' in stats.summary(1.0)
