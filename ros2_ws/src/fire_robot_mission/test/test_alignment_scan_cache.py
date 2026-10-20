"""Scan reuse must preserve obstacle rejection and expire with the sample."""

import math

import pytest

from fire_robot_mission.live_alignment_takeover import LiveAlignmentCore


def scan(core, ranges=None, *, at=10.0):
    core.update_scan(-3.14, 3.14, [1.0] * 360 if ranges is None else ranges,
                     at, range_max=8.0)


def test_repeated_checks_do_not_rescan_or_extend_freshness(monkeypatch):
    core = LiveAlignmentCore()
    scan(core)
    geometry = core._scan_geometry_clear
    calls = []

    def counted():
        calls.append(True)
        return geometry()

    monkeypatch.setattr(core, '_scan_geometry_clear', counted)
    assert core._scan_clear(10.01) is True
    assert core._scan_clear(10.10) is True
    assert core._scan_clear(10.20) is True
    assert len(calls) == 1
    # A cached clear result cannot survive an expired or future-dated scan.
    assert core._scan_clear(10.36) is False
    assert core._scan_clear(9.99) is False
    assert len(calls) == 1


def test_new_obstacle_invalidates_clear_result_and_recovery_uses_new_scan():
    core = LiveAlignmentCore()
    scan(core)
    assert core._scan_clear(10.01) is True
    scan(core, [0.2] + [1.0] * 359, at=10.1)
    assert core._scan_clear(10.11) is False
    assert core._scan_clear(10.12) is False
    scan(core, at=10.2)
    assert core._scan_clear(10.21) is True


def test_clearance_threshold_change_invalidates_both_cached_outcomes():
    core = LiveAlignmentCore()
    scan(core, [0.4] * 360)
    assert core._scan_clear(10.01) is True
    core.min_clearance = 0.5
    assert core._scan_clear(10.02) is False
    core.min_clearance = 0.3
    assert core._scan_clear(10.03) is True


def test_scan_owns_immutable_copy_of_received_ranges():
    core = LiveAlignmentCore()
    values = [1.0] * 360
    scan(core, values)
    assert core._scan_clear(10.01) is True
    values[0] = 0.1
    assert core._scan_clear(10.02) is True
    scan(core, values, at=10.1)
    assert core._scan_clear(10.11) is False


@pytest.mark.parametrize('ranges,expected', [
    ([math.inf] * 360, False),  # Still require finite returns.
    ([1.0] * 10 + [math.inf] * 350, True),
    ([1.0] * 9 + [math.inf] * 351, False),
    ([1.0] * 288 + [math.nan] * 72, True),
    ([1.0] * 287 + [math.nan] * 73, False),
    ([9.0] * 360, False),  # Finite but beyond the sensor range.
    ([0.0] * 360, False),
])
def test_cached_result_keeps_scan_coverage_guards(ranges, expected):
    core = LiveAlignmentCore()
    scan(core, ranges)
    assert core._scan_clear(10.01) is expected
    assert core._scan_clear(10.02) is expected


def test_narrow_or_nonfinite_geometry_cannot_reuse_prior_clearance():
    core = LiveAlignmentCore()
    scan(core)
    assert core._scan_clear(10.01) is True
    core.update_scan(-1.0, 1.0, [1.0] * 360, 10.1)
    assert core._scan_clear(10.11) is False
    core.update_scan(-3.14, math.nan, [1.0] * 360, 10.2)
    assert core._scan_clear(10.21) is False
