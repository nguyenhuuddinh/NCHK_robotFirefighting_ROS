"""Compare the optimized clearance algorithm with its pre-optimization result."""

import math
import random

import pytest

from fire_robot_mission import ir_approach_shadow_core as module


def reference_clearance(scan, center, half, required):
    """Frozen algorithm from 0de7c12, independent of the geometry cache."""
    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    if not isinstance(scan, tuple) or len(scan) != 5:
        return None, 'SCAN_INVALID'
    angle_min, increment, range_min, range_max, ranges = scan
    if (not all(finite(value) for value in scan[:4]) or increment == 0 or
            range_min < 0 or range_max <= range_min or
            not isinstance(ranges, tuple) or len(ranges) < 90):
        return None, 'SCAN_INVALID'
    selected, angles = [], []
    for index, value in enumerate(ranges):
        angle = angle_min + index * increment - center
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= half:
            selected.append(value)
            angles.append(angle)
    side = half - abs(increment) * 1.5
    if (len(selected) < 10 or side <= 0 or
            min(angles, default=0) > -side or
            max(angles, default=0) < side):
        return None, 'SCAN_SECTOR_COVERAGE_LOW'
    valid = [float(value) for value in selected
             if finite(value) and range_min <= value <= range_max]
    invalid = [value for value in selected
               if not (finite(value) and range_min <= value <= range_max)
               and value != math.inf]
    if len(valid) < 10 or len(invalid) > len(selected) // 5:
        return None, 'SCAN_SECTOR_UNOBSERVED'
    distance = min(valid)
    if distance < required:
        return distance, 'OBSTACLE_OR_INSUFFICIENT_MARGIN'
    return distance, None


@pytest.fixture(autouse=True)
def clear_geometry_cache():
    module._sector_indices.cache_clear()
    yield
    module._sector_indices.cache_clear()


GEOMETRIES = (
    (-math.pi, 2 * math.pi / 359, 360),
    (math.pi, -2 * math.pi / 359, 360),
    (0.0, -2 * math.pi / 400, 400),
    (-math.pi, 2 * math.pi / 720, 720),
    (0.0, -2 * math.pi / 359, 129),  # partial, one-sided front
    (2 * math.pi, -2 * math.pi / 399, 400),  # wrapped rear
    (-0.7, 0.01, 140),
)


@pytest.mark.parametrize('geometry', GEOMETRIES)
@pytest.mark.parametrize('center', [0.0, math.pi, -math.pi])
def test_differential_geometry_distances_and_invalid_returns(geometry, center):
    rng = random.Random(1835)
    start, increment, count = geometry
    populations = (
        [1.0], [math.inf], [math.nan], [0.49, 0.50, 0.51, 1.0],
        [0.0, -1.0, -math.inf, math.inf, math.nan, True, None, 'bad', 1.0],
        [1.0] * 20 + [0.04, 8.01, math.nan, math.inf],
    )
    for half in (0.01, 0.35, 0.60):
        for population in populations:
            ranges = tuple(rng.choice(population) for _ in range(count))
            scan = (start, increment, 0.05, 8.0, ranges)
            assert module.sector_clearance(scan, center, half, 0.5) == \
                reference_clearance(scan, center, half, 0.5)


@pytest.mark.parametrize('scan', [
    None, (), (0.0, 0.01, 0.05, 8.0, [1.0] * 400),
    (0.0, 0.0, 0.05, 8.0, (1.0,) * 400),
    (math.nan, 0.01, 0.05, 8.0, (1.0,) * 400),
    (0.0, 0.01, 8.0, 0.05, (1.0,) * 400),
    (0.0, 0.01, -0.1, 8.0, (1.0,) * 400),
    (0.0, 0.01, 0.05, 8.0, (1.0,) * 89),
])
def test_invalid_scan_still_fails_closed(scan):
    assert module.sector_clearance(scan, 0.0, 0.35, 0.5) == \
        (None, 'SCAN_INVALID')


def scan_with(values):
    return (-math.pi, 2 * math.pi / 400, 0.05, 8.0, tuple(values))


def test_cache_reuses_angles_but_reads_new_obstacles_and_limits(monkeypatch):
    scan = scan_with([1.0] * 400)
    assert module.sector_clearance(scan, 0.0, 0.35, 0.5) == (1.0, None)

    def unnecessary_trigonometry(_angle):
        raise AssertionError('unchanged geometry recomputed')

    monkeypatch.setattr(module.math, 'sin', unnecessary_trigonometry)
    monkeypatch.setattr(module.math, 'cos', unnecessary_trigonometry)
    assert module.sector_clearance(scan, 0.0, 0.35, 1.1) == \
        (1.0, 'OBSTACLE_OR_INSUFFICIENT_MARGIN')
    closer = [1.0] * 400
    closer[200] = 0.25
    assert module.sector_clearance(scan_with(closer), 0.0, 0.35, 0.5) == \
        (0.25, 'OBSTACLE_OR_INSUFFICIENT_MARGIN')
    invalid_limits = (scan[0], scan[1], 0.05, 0.9, scan[4])
    assert module.sector_clearance(invalid_limits, 0.0, 0.35, 0.5) == \
        (None, 'SCAN_SECTOR_UNOBSERVED')
    assert module.sector_clearance(scan_with([math.inf] * 400),
                                   0.0, 0.35, 0.5) == \
        (None, 'SCAN_SECTOR_UNOBSERVED')


def test_new_geometry_and_sector_select_new_obstacle_indices():
    ranges = [1.0] * 400
    ranges[200] = 0.25
    scan = scan_with(ranges)
    assert module.sector_clearance(scan, 0.0, 0.35, 0.5)[1] == \
        'OBSTACLE_OR_INSUFFICIENT_MARGIN'
    assert module.sector_clearance(scan, math.pi, 0.35, 0.5) == (1.0, None)
    shifted = (0.0, scan[1], scan[2], scan[3], scan[4])
    assert module.sector_clearance(shifted, 0.0, 0.35, 0.5) == (1.0, None)
    assert module.sector_clearance(shifted, math.pi, 0.35, 0.5)[1] == \
        'OBSTACLE_OR_INSUFFICIENT_MARGIN'


def test_invalid_fraction_boundary_and_ten_finite_returns():
    scan = scan_with([1.0] * 400)
    indices, _ = module._sector_indices(scan[0], scan[1], 400, 0.0, 0.35)
    for invalid_count in (len(indices) // 5, len(indices) // 5 + 1):
        ranges = list(scan[4])
        for index in indices[:invalid_count]:
            ranges[index] = math.nan
        changed = scan_with(ranges)
        assert module.sector_clearance(changed, 0.0, 0.35, 0.5) == \
            reference_clearance(changed, 0.0, 0.35, 0.5)
    for finite_count in (9, 10):
        ranges = [math.inf] * 400
        for index in indices[:finite_count]:
            ranges[index] = 1.0
        expected = (1.0, None) if finite_count == 10 else \
            (None, 'SCAN_SECTOR_UNOBSERVED')
        assert module.sector_clearance(
            scan_with(ranges), 0.0, 0.35, 0.5) == expected


def test_geometry_cache_is_bounded():
    for count in range(390, 410):
        scan = (-math.pi, 2 * math.pi / count, 0.05, 8.0, (1.0,) * count)
        assert module.sector_clearance(scan, 0.0, 0.35, 0.5) == (1.0, None)
    info = module._sector_indices.cache_info()
    assert info.maxsize == 8
    assert info.currsize == 8


def test_geometry_cache_does_not_make_a_stale_scan_fresh():
    core = module.IrApproachShadowCore()
    core.update('scan', scan_with([1.0] * 400), 1.0)
    assert core._front_clearance(core._input('scan', 1.1)) == (1.0, None)
    assert core._front_clearance(core._input('scan', 1.6)) == \
        (None, 'SCAN_INVALID')
