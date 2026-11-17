import math
import os

import pytest
import yaml

from hw4_planning.export import (MARGIN, ROBOT_RADIUS, estimate_time, from_start_frame, plan_both, to_start_frame,
                                 waypoints, write_localization_yaml)
from hw4_planning.shortest import plan_shortest
from hw4_planning.world import load_world

WORLD_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4_world.yaml')


@pytest.fixture(scope='module')
def world():
    return load_world(WORLD_FILE)


def test_start_frame_puts_the_start_at_the_origin_facing_the_top_wall(world):
    assert to_start_frame(world, world.start) == pytest.approx((0.0, 0.0), abs=1e-9)
    gx, gy = to_start_frame(world, world.goal)
    assert gx == pytest.approx(world.goal[1] - world.start[1], abs=1e-3)  # forward is floor +y
    assert gy == pytest.approx(world.start[0] - world.goal[0], abs=1e-3)  # left is floor -x
    assert (gx, gy) == pytest.approx((2.134, 2.134), abs=1e-3)


def test_from_start_frame_undoes_to_start_frame(world):
    for p in world.landmarks.values():
        assert from_start_frame(world, to_start_frame(world, p)) == pytest.approx(p, abs=1e-9)


def test_start_frame_keeps_distances(world):
    a, b = world.landmarks[0], world.landmarks[5]
    assert math.dist(to_start_frame(world, a), to_start_frame(world, b)) == pytest.approx(math.dist(a, b))


def test_waypoints_face_the_next_leg(world):
    path = [world.start, (1.9, 1.9), world.goal]
    wps = waypoints(world, path)
    pts = [to_start_frame(world, p) for p in path]
    legs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
    assert len(wps) == len(path)
    for (x, y, theta), p, want in zip(wps, pts, legs + legs[-1:]):
        assert (x, y) == pytest.approx(p, abs=1e-9)
        assert theta == pytest.approx(want)


def test_estimate_time_matches_the_velocity_control_model():
    assert estimate_time([(1.0, 0.0, 0.0)]) == pytest.approx(1 / 0.55 + 1.0)
    assert estimate_time([(0.0, 1.0, math.pi / 2)]) == pytest.approx(math.pi / 2 / 3.49 + 1.0 + 1 / 0.55 + 1.0)


def test_plan_both_pads_only_the_fast_path(world):
    paths = plan_both(world)
    fast = plan_shortest(world, ROBOT_RADIUS + MARGIN)
    assert MARGIN == pytest.approx(0.05)
    assert len(paths['fast']) == len(fast)
    assert all(math.dist(a, b) < 1e-9 for a, b in zip(paths['fast'], fast))
    assert paths['safe'][0] == pytest.approx(world.start) and paths['safe'][-1] == pytest.approx(world.goal)


def test_localization_yaml_holds_the_tags_in_the_start_frame(world, tmp_path):
    out = tmp_path / 'loc.yaml'
    write_localization_yaml(str(out), world)
    params = yaml.safe_load(out.read_text())['/**']['ros__parameters']
    tags = params['tags']
    assert tags['ids'] == list(range(8))
    assert len(tags['z']) == len(tags['yaw']) == 8
    for i, tag_id in enumerate(tags['ids']):
        assert (tags['x'][i], tags['y'][i]) == pytest.approx(to_start_frame(world, world.landmarks[tag_id]), abs=5e-5)
    assert params['camera']['x'] == pytest.approx(0.01)
    assert params['min_turn_cmd'] == pytest.approx(0.38)


def test_localization_yaml_loosens_the_ekf_gate(world, tmp_path):
    out = tmp_path / 'loc.yaml'
    write_localization_yaml(str(out), world)
    params = yaml.safe_load(out.read_text())['/**']['ros__parameters']
    assert params['ekf']['gate'] == 1000.0
    assert params['ekf']['gate'] >= 100
