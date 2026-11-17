import math
import os

import pytest

from hw4_planning.safest import plan_safest
from hw4_planning.shortest import plan_shortest
from hw4_planning.world import clearance, load_world, segment_free

WORLD_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4_world.yaml')
FT = 0.3048


@pytest.fixture(scope='module')
def world():
    return load_world(WORLD_FILE)


def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def min_clearance(world, path, step=0.01):
    low = float('inf')
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.dist(a, b) / step))
        for k in range(n + 1):
            p = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            low = min(low, clearance(world, p))
    return low


def test_world_file(world):
    assert world.arena == pytest.approx(10 * FT)
    assert sorted(world.landmarks) == list(range(12))
    assert world.obstacle == pytest.approx((4.5 * FT, 4.5 * FT, 5.5 * FT, 5.5 * FT))
    assert world.start_heading == pytest.approx(math.radians(90))


def test_clearance(world):
    assert clearance(world, (1.0, 0.5)) == pytest.approx(0.5)  # nearest edge is the bottom wall
    assert clearance(world, (1.2, 1.5)) == pytest.approx(world.obstacle[0] - 1.2)  # beside the left face
    assert clearance(world, (1.524, 1.524)) < 0  # inside the box


def test_segment_free(world):
    assert not segment_free(world, world.start, world.goal, 0.0)  # runs through the box
    assert segment_free(world, (0.5, 0.5), (2.5, 0.5), 0.1)  # along the bottom, clear of everything
    assert not segment_free(world, (0.5, 0.5), (2.5, 0.5), 0.6)  # too fat for the wall


def test_shortest_without_inflation_is_ten_feet(world):
    path = plan_shortest(world, radius=0.0)
    assert path[0] == pytest.approx(world.start) and path[-1] == pytest.approx(world.goal)
    assert len(path) == 3
    assert length(path) == pytest.approx(10 * FT, abs=1e-3)


@pytest.mark.parametrize('prefer, corner', [('right', (5.5 * FT, 5.5 * FT)), ('left', (4.5 * FT, 4.5 * FT))])
def test_shortest_takes_the_preferred_side(world, prefer, corner):
    path = plan_shortest(world, radius=0.0, prefer=prefer)
    assert path[1] == pytest.approx(corner)


def test_shortest_keeps_its_distance(world):
    radius = 0.18
    path = plan_shortest(world, radius=radius)
    assert length(path) > 10 * FT
    assert min_clearance(world, path) >= radius - 1e-6


ROBOT_RADIUS = 0.1115  # half-diagonal of the 16.5 x 15 cm robot


def points_along(path, step=0.01):
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.dist(a, b) / step))
        for k in range(n + 1):
            yield (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)


def side(world, p):
    """Negative = right of the direction of travel from start to goal, positive = left."""
    (sx, sy), (gx, gy) = world.start, world.goal
    return (gx - sx) * (p[1] - sy) - (gy - sy) * (p[0] - sx)


def test_safest_stays_clear_of_everything(world):
    path = plan_safest(world, radius=ROBOT_RADIUS)
    assert path[0] == pytest.approx(world.start) and path[-1] == pytest.approx(world.goal)
    assert min_clearance(world, path) >= ROBOT_RADIUS - 1e-6


def test_safest_is_safer_but_longer_than_shortest(world):
    fast = plan_shortest(world, radius=ROBOT_RADIUS + 0.05)
    safe = plan_safest(world, radius=ROBOT_RADIUS)
    assert min_clearance(world, safe) > 0.40
    assert min_clearance(world, safe) > min_clearance(world, fast) + 0.1
    assert length(safe) > length(fast)


def test_safest_has_few_waypoints(world):
    assert len(plan_safest(world, radius=ROBOT_RADIUS)) <= 6


@pytest.mark.parametrize('prefer, sign', [('right', -1), ('left', 1)])
def test_safest_takes_the_preferred_side(world, prefer, sign):
    path = plan_safest(world, radius=ROBOT_RADIUS, prefer=prefer)
    assert all(sign * side(world, p) >= -1e-6 for p in points_along(path))


def test_safest_fails_when_the_robot_does_not_fit(world):
    with pytest.raises(ValueError):
        plan_safest(world, radius=1.0)
