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
    msg.data = [1, 2, 3, 4]
    stats.record(msg, now=1.0)
    stats.record(msg, now=1.2)

    summary = stats.summary(2.0)
    assert 'frames=2' in summary
    assert 'rx_hz=5.00' in summary
    assert 'max_gap_s=0.200' in summary
    assert 'avg_jpeg_bytes=4' in summary
    assert 'payload_Bps=4' in summary
    assert 'frame_id=camera_frame' in summary
    assert 'last_stamp=42.000000000' in summary
    assert not hasattr(stats, 'messages')
