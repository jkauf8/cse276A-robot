import heapq
import math

from hw4_planning.world import clearance, segment_free

TIE_PENALTY = 1e-6
WEIGHT = 0.5
STEPS = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]


def plan_safest(world, radius, resolution=0.02, weight=WEIGHT, prefer='right'):
    for p in (world.start, world.goal):
        if clearance(world, p) < radius:
            raise ValueError('start or goal is too close to an obstacle for this radius')

    # nudge the spacing so the grid ends on the wall, which keeps it mirror symmetric
    n = math.ceil(world.arena / resolution - 1e-9)
    resolution = world.arena / n
    clear = {}

    def clr(c):
        if c not in clear:
            clear[c] = clearance(world, (c[0] * resolution, c[1] * resolution))
        return clear[c]

    def snap(p):
        return (round(p[0] / resolution), round(p[1] / resolution))

    start, goal = snap(world.start), snap(world.goal)
    if clr(start) < radius or clr(goal) < radius:
        raise ValueError('start or goal is too close to an obstacle for this radius')

    gx, gy = world.goal[0] - world.start[0], world.goal[1] - world.start[1]
    sign = -1 if prefer == 'right' else 1

    def tie(c):
        cross = gx * (c[1] * resolution - world.start[1]) - gy * (c[0] * resolution - world.start[0])
        return 0.0 if cross * sign >= 0 else TIE_PENALTY

    def h(c):
        return resolution * math.dist(c, goal)

    best = {start: 0.0}
    prev = {}
    heap = [(h(start), start)]
    while heap:
        _, c = heapq.heappop(heap)
        if c == goal:
            break
        for dx, dy in STEPS:
            nb = (c[0] + dx, c[1] + dy)
            if not (0 <= nb[0] <= n and 0 <= nb[1] <= n) or clr(nb) < radius:
                continue
            step = resolution * math.hypot(dx, dy)
            nd = best[c] + step * (1 + weight / clr(nb)) + tie(nb)
            if nd < best.get(nb, math.inf):
                best[nb], prev[nb] = nd, c
                heapq.heappush(heap, (nd + h(nb), nb))
    if goal not in best:
        raise ValueError('no path from start to goal')

    cells = [goal]
    while cells[-1] != start:
        cells.append(prev[cells[-1]])
    raw = [(c[0] * resolution, c[1] * resolution) for c in cells[::-1]]
    raw[0], raw[-1] = world.start, world.goal

    floor = max(radius, 0.9 * min(clearance(world, p) for p in raw))
    path, i = [raw[0]], 0
    while i < len(raw) - 1:
        j = next((j for j in range(len(raw) - 1, i, -1)
                  if segment_free(world, raw[i], raw[j], floor)), i + 1)
        path.append(raw[j])
        i = j
    return path
