"""Plot the robot's path from a run log and print the error at each waypoint.
Runs on a laptop, not on the robot. Needs pandas and matplotlib."""
import argparse
import math
import os

import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from . import ekf as E


def print_table(log, waypoints):
    print('\nerror at each waypoint when the controller finished it (cm / deg)')
    print(f"{'wp':>2} {'goal (x, y, th)':>20} {'reached at':>10} | {'estimate vs goal':>18}")
    for i, (gx, gy, gth) in enumerate(waypoints):
        goal = f'({gx:.2f}, {gy:.2f}, {math.degrees(gth):.0f})'
        done = log[log.done_wp >= i]  # rows after the controller finished waypoint i
        if done.empty:
            print(f'{i:>2} {goal:>20} {"never":>10}')
            continue
        r = done.iloc[0]
        err_cm = math.hypot(r.x - gx, r.y - gy) * 100
        err_deg = math.degrees(abs(E.wrap(r.theta - gth)))
        print(f'{i:>2} {goal:>20} {r.t:9.1f}s | {err_cm:9.1f} {err_deg:8.1f}')
    path_length = np.hypot(log.x.diff(), log.y.diff()).sum()
    print(f'\nrun time {log.t.iloc[-1]:.1f} s, estimated path {path_length:.2f} m')
    print(f'a tag was used in {(log.n_tags > 0).mean() * 100:.0f}% of ticks')


def plot(log, waypoints, path):
    fig = Figure(figsize=(8, 8))
    ax = fig.subplots()
    ax.plot(log.x, log.y, color='tab:blue', lw=1, label='EKF estimate')
    dots = log[(log.t // 0.5).diff().fillna(1) != 0]  # first row of every 0.5 s
    ax.plot(dots.x, dots.y, 'o', color='tab:blue', ms=4, label='estimate, dot every 0.5 s')
    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], 'k*', ms=13)
        ax.annotate(f'wp{i}', wp[:2], textcoords='offset points', xytext=(8, 6))
    for tag_id, (tx, ty, tz, face) in E.TAG_MAP.items():
        ax.plot(tx, ty, 's', color='tab:green', ms=8)
        ax.arrow(tx, ty, 0.35 * math.cos(face), 0.35 * math.sin(face), color='tab:green', head_width=0.07)
        ax.annotate(f'tag {tag_id}', (tx, ty), textcoords='offset points', xytext=(8, -12), color='tab:green')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)'), ax.set_ylabel('y (m)')
    ax.set_title('Trajectory (world frame, origin = start pose)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f'plot saved to {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', help='run log (localization_*.csv)')
    ap.add_argument('--waypoints', required=True, help='waypoint file used in the run')
    ap.add_argument('--out', default=None, help='PNG path (default: next to the log)')
    args = ap.parse_args()

    log = pd.read_csv(args.log)
    waypoints = np.loadtxt(args.waypoints, delimiter=',', ndmin=2)
    print_table(log, waypoints)
    plot(log, waypoints, args.out or os.path.splitext(args.log)[0] + '.png')


if __name__ == '__main__':
    main()
