"""Source-age admission boundaries, ordering and clock epoch recovery."""
from types import SimpleNamespace

import pytest

from fire_robot_safety.odom_freshness import OdomStampGuard


def stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000,
                           nanosec=ns % 1_000_000_000)


@pytest.mark.parametrize('age, accepted', [
    (0, True), (300_000_000, True), (300_000_001, False), (-1, False)])
def test_age_boundary(age, accepted):
    g = OdomStampGuard()
    result = g.admit(stamp(10_000_000_000 - age), 10_000_000_000, 50., .3)
    if accepted:
        assert result == pytest.approx(50. - age / 1e9)
    else:
        assert result is None


@pytest.mark.parametrize('sec,nsec', [(0, 0), (-1, 0), (1, 1_000_000_000)])
def test_invalid_stamp(sec, nsec):
    g = OdomStampGuard()
    assert g.admit(SimpleNamespace(sec=sec, nanosec=nsec),
                   10_000_000_000, 50., .3) is None


def test_reordered_duplicate_future_and_stale_cannot_poison_recovery():
    g = OdomStampGuard()
    assert g.admit(stamp(10_000_000_000), 10_100_000_000, 50., .3) == 49.9
    for ns in (10_000_000_000, 9_950_000_000, 9_000_000_000, 15_000_000_000):
        assert g.admit(stamp(ns), 10_150_000_000, 50.05, .3) is None
    assert g.admit(stamp(10_200_000_000), 10_250_000_000, 50.15, .3) == 50.1


def test_clock_rollback_rejects_once_then_accepts_new_epoch():
    g = OdomStampGuard()
    assert g.admit(stamp(10_000_000_000), 10_000_000_000, 50., .3) == 50.
    assert g.admit(stamp(5_000_000_000), 5_000_000_000, 50.1, .3) is None
    assert g.reason == 'ROS_CLOCK_BACKWARD'
    assert g.admit(stamp(5_100_000_000), 5_100_000_000, 50.2, .3) == 50.2


def test_forward_jump_does_not_accept_old_stamp():
    g = OdomStampGuard()
    g.admit(stamp(10_000_000_000), 10_000_000_000, 50., .3)
    assert g.admit(stamp(10_100_000_000), 30_000_000_000, 50.1, .3) is None
    assert g.admit(stamp(30_100_000_000), 30_100_000_000, 50.2, .3) == 50.2
