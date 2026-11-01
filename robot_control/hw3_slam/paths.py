"""Waypoint lists for the HW3 paths, relative to the start pose (0, 0, 0 = where the robot is placed, facing +x).
Each waypoint's heading is the direction of the next leg, so the robot is already facing it when the next move starts."""
import math
import os

from hw2_localization.ekf import wrap


def polygon(sides, length, loops=1):
    """Counter-clockwise regular polygon with the given side length (m), driven `loops` times."""
    step = 2 * math.pi / sides
    x = y = 0.0
    waypoints = [(0.0, 0.0, 0.0)]
    for k in range(sides * loops):
        x += length * math.cos((k % sides) * step)
        y += length * math.sin((k % sides) * step)
        waypoints.append((x, y, wrap(((k + 1) % sides) * step)))
    return waypoints


def square(n, loops=1):
    return polygon(4, n, loops)


def octagon(m, loops=1):
    return polygon(8, m, loops)


def write(path, waypoints):
    with open(path, 'w') as f:
        f.write('# x, y, theta per line (m, m, rad), relative to the start pose\n')
        f.writelines(f'{x:.4f}, {y:.4f}, {th:.5f}\n' for x, y, th in waypoints)


if __name__ == '__main__':
    config = os.path.join(os.path.dirname(__file__), '..', 'config')
    write(os.path.join(config, 'hw3_square.txt'), square(0.9144, loops=3))  # 3 ft
    write(os.path.join(config, 'hw3_octagon.txt'), octagon(0.4572, loops=3))  # 1.5 ft
