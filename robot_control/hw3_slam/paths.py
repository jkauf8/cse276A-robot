"""Waypoint lists for the HW3 paths, relative to the start pose (0, 0, 0 = where the robot is placed, facing +x).
Each waypoint's heading is the direction of the next leg, so the robot is already facing it when the next move starts.

    python3 -m hw3_slam.paths      # from robot_control/: rewrite config/hw3_square.txt and config/hw3_octagon.txt
"""
import math
import os

from hw2_localization.ekf import wrap

FT = 0.3048  # m per ft
CORNERS = {'square': 4, 'octagon': 8}  # waypoints per loop
SIDE = {'square': 0.9144, 'octagon': 0.4572}  # side length of each path (m): 3 ft and 1.5 ft
LOOPS = 3


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


def write(path, waypoints):
    with open(path, 'w') as f:
        f.write('# x, y, theta per line (m, m, rad), relative to the start pose\n')
        f.writelines(f'{x:.4f}, {y:.4f}, {th:.5f}\n' for x, y, th in waypoints)


if __name__ == '__main__':
    config = os.path.join(os.path.dirname(__file__), '..', 'config')
    for shape, corners in CORNERS.items():
        write(os.path.join(config, f'hw3_{shape}.txt'), polygon(corners, SIDE[shape], loops=LOOPS))
