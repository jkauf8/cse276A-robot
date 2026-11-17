import math
from dataclasses import dataclass

import yaml

EPS = 1e-9


@dataclass(frozen=True)
class World:
    arena: float
    landmarks: dict
    obstacle: tuple  # (x_min, y_min, x_max, y_max)
    start: tuple
    goal: tuple
    start_heading: float


def load_world(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    return World(
        arena=float(d['arena']),
        landmarks={int(k): tuple(map(float, v)) for k, v in d['landmarks'].items()},
        obstacle=tuple(map(float, d['obstacle']['rect'])),
        start=tuple(map(float, d['start'])),
        goal=tuple(map(float, d['goal'])),
        start_heading=math.radians(d['start_heading_deg']),
    )


def _wall_distance(world, p):
    return min(p[0], p[1], world.arena - p[0], world.arena - p[1])


def _box_distance(box, p):
    x0, y0, x1, y1 = box
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    if dx > 0 or dy > 0:
        return math.hypot(dx, dy)
    # inside the box: minus the depth to the nearest face
    return -min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1])


def clearance(world, p):
    return min(_wall_distance(world, p), _box_distance(world.obstacle, p))


def _enters_interior(box, a, b):
    # Liang-Barsky clip against the box shrunk by EPS, so touching an edge or corner is not interior
    x0, y0, x1, y1 = box[0] + EPS, box[1] + EPS, box[2] - EPS, box[3] - EPS
    t0, t1 = 0.0, 1.0
    for p, q in ((-(b[0] - a[0]), a[0] - x0), (b[0] - a[0], x1 - a[0]),
                 (-(b[1] - a[1]), a[1] - y0), (b[1] - a[1], y1 - a[1])):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def _point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def segment_free(world, a, b, radius):
    if _wall_distance(world, a) < radius - EPS or _wall_distance(world, b) < radius - EPS:
        return False
    box = world.obstacle
    if _enters_interior(box, a, b):
        return False
    x0, y0, x1, y1 = box
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    near = min([_box_distance(box, a), _box_distance(box, b)]
               + [_point_segment_distance(c, a, b) for c in corners])
    return near >= radius - EPS
