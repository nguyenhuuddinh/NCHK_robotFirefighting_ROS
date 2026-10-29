"""Unit tests for clock sample quality gates."""

from fire_robot_mission.clock_sync_diagnostics import (
    summarize_clock_samples,
)


def sample(index, rtt, offset):
    """Build one clock sample."""
    return {
        'index': index,
        'rtt_ms': rtt,
        'pi_minus_laptop_ms': offset,
    }


def test_clock_summary_passes_good_samples_and_discards_slow_one():
    report = summarize_clock_samples([
        sample(1, 10.0, 2.0),
        sample(2, 12.0, 3.0),
        sample(3, 11.0, 4.0),
        sample(4, 90.0, 200.0),
    ], max_rtt_ms=50.0, max_offset_ms=20.0)
    assert report['status'] == 'PASS'
    assert report['samples_accepted'] == 3
    assert report['pi_minus_laptop_ms']['median'] == 3.0


def test_clock_summary_fails_large_offset():
    report = summarize_clock_samples([
        sample(1, 10.0, -70.0),
        sample(2, 11.0, -69.0),
        sample(3, 12.0, -71.0),
    ], max_rtt_ms=50.0, max_offset_ms=20.0)
    assert report['status'] == 'FAIL'
    assert 'offset' in report['reason']


def test_clock_summary_fails_when_rtt_gate_leaves_too_few_samples():
    report = summarize_clock_samples([
        sample(1, 70.0, 0.0),
        sample(2, 80.0, 0.0),
        sample(3, 10.0, 0.0),
    ], max_rtt_ms=50.0, max_offset_ms=20.0)
    assert report['status'] == 'FAIL'
    assert report['samples_accepted'] == 1
