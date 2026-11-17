import heapq
import math

from hw4_planning.world import segment_free

TIE_PENALTY = 1e-6


def visibility_graph(world, radius):
    x0, y0, x1, y1 = world.obstacle
    r = radius
    corners = [(x0 - r, y0 - r), (x1 + r, y0 - r), (x1 + r, y1 + r), (x0 - r, y1 + r)]
    nodes = [world.start, world.goal] + corners
    edges = [(i, j) for i in range(len(nodes)) for j in range(i + 1, len(nodes))
             if segment_free(world, nodes[i], nodes[j], radius)]
    return nodes, edges


def plan_shortest(world, radius, prefer='right'):
    nodes, edges = visibility_graph(world, radius)
    corners = nodes[2:]
    neighbors = {i: [] for i in range(len(nodes))}
    for i, j in edges:
        neighbors[i].append(j)
        neighbors[j].append(i)

    gx, gy = world.goal[0] - world.start[0], world.goal[1] - world.start[1]
    sign = -1 if prefer == 'right' else 1
    penalty = [0.0, 0.0]
    for n in corners:
        cross = gx * (n[1] - world.start[1]) - gy * (n[0] - world.start[0])
        penalty.append(0.0 if cross * sign >= 0 else TIE_PENALTY)

    best = {0: 0.0}
    prev = {}
    heap = [(0.0, 0)]
    while heap:
        d, i = heapq.heappop(heap)
        if d > best.get(i, math.inf):
            continue
        if i == 1:
            break
        for j in neighbors[i]:
            nd = d + math.dist(nodes[i], nodes[j]) + penalty[j]
            if nd < best.get(j, math.inf):
                best[j], prev[j] = nd, i
                heapq.heappush(heap, (nd, j))
    if 1 not in best:
        raise ValueError('no path from start to goal')

    path, i = [], 1
    while i != 0:
        path.append(nodes[i])
        i = prev[i]
    path.append(nodes[0])
    return path[::-1]
