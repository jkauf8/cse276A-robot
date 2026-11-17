"""Figures for the HW4 report."""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from hw4_planning.export import (CONFIG, MARGIN, ROBOT_RADIUS, estimate_time, from_start_frame, min_clearance,  # noqa: E402
                                 path_length, plan_both, waypoints)
from hw4_planning.shortest import visibility_graph  # noqa: E402
from hw4_planning.world import clearance, load_world  # noqa: E402

COLORS = {'fast': '#d95f02', 'safe': '#1b9e77'}
LOG_COLOR = '#7570b3'


def _label_offset(world, x, y):
    x0, y0, x1, y1 = world.obstacle
    if x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6:
        # box tags: push the label straight out of its face so the four never overlap
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        return (16 * (x - cx) / abs(x - cx), 0) if abs(x - cx) > abs(y - cy) else (0, 16 * (y - cy) / abs(y - cy))
    a = world.arena
    return (11 if x < a / 2 else -11, 11 if y < a / 2 else -11)


def draw_arena(ax, world):
    a = world.arena
    ax.plot([0, a, a, 0, 0], [0, 0, a, a, 0], 'k-', lw=2)
    x0, y0, x1, y1 = world.obstacle
    ax.fill([x0, x1, x1, x0], [y0, y0, y1, y1], color='0.35', zorder=3)
    for i, (x, y) in world.landmarks.items():
        ax.plot(x, y, 's', color='k', ms=5)
        ax.annotate(str(i), (x, y), textcoords='offset points', ha='center', va='center', fontsize=9, zorder=5,
                    bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.7),
                    xytext=_label_offset(world, x, y))
    ax.plot(*world.start, 'o', color='tab:blue', ms=10, zorder=6, label='start')
    ax.plot(*world.goal, '*', color='tab:red', ms=16, zorder=6, label='goal')
    ax.set_aspect('equal')
    ax.set_xlim(-0.12, a + 0.12)
    ax.set_ylim(-0.12, a + 0.12)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')


def draw_path(ax, path, name, color=None):
    ax.plot(*zip(*path), '-o', color=color or COLORS[name], lw=3, ms=5, zorder=4, label=f'{name} path')


def read_log(world, csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    return [from_start_frame(world, (float(r['x']), float(r['y']))) for r in rows]


def draw_log(ax, world, log_csv):
    run = read_log(world, log_csv)
    ax.plot(*zip(*run), '-', color=LOG_COLOR, lw=1, zorder=5, label='executed')
    return math.dist(run[-1], world.goal)


def title(world, path, name, log_error):
    text = (f'{name}: length {path_length(path):.2f} m, min clearance {min_clearance(world, path):.2f} m, '
            f'est. time {estimate_time(waypoints(world, path)):.1f} s')
    return text + (f'\nfinal error to goal {log_error * 100:.1f} cm' if log_error is not None else '')


def plot_fast(world, path, out, log_csv=None):
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_arena(ax, world)
    nodes, edges = visibility_graph(world, ROBOT_RADIUS + MARGIN)
    for i, j in edges:
        ax.plot(*zip(nodes[i], nodes[j]), '-', color='0.8', lw=1, zorder=1)
    ax.plot(*zip(*nodes), '.', color='0.5', ms=6, zorder=2)
    r = ROBOT_RADIUS + MARGIN
    x0, y0, x1, y1 = world.obstacle
    ax.plot([x0 - r, x1 + r, x1 + r, x0 - r, x0 - r], [y0 - r, y0 - r, y1 + r, y1 + r, y0 - r], '--', color='tab:red',
            lw=1, label=f'box inflated by {r:.3f} m')
    draw_path(ax, path, 'fast')
    err = draw_log(ax, world, log_csv) if log_csv else None
    ax.set_title(title(world, path, 'fast', err))
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5, fontsize=8)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_safe(world, path, out, log_csv=None):
    fig, ax = plt.subplots(figsize=(7, 7))
    xs = np.linspace(0, world.arena, round(world.arena / 0.02) + 1)
    grid = np.array([[clearance(world, (x, y)) for x in xs] for y in xs])
    grid = np.ma.masked_less(grid, ROBOT_RADIUS)
    im = ax.imshow(grid, origin='lower', extent=(0, world.arena, 0, world.arena), cmap='viridis', zorder=0)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label='clearance (m), cells under the robot radius blanked')
    draw_arena(ax, world)
    draw_path(ax, path, 'safe', color='#e7298a')  # magenta stands out on viridis
    err = draw_log(ax, world, log_csv) if log_csv else None
    ax.set_title(title(world, path, 'safe', err))
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5, fontsize=8)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_both(world, paths, out, logs):
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_arena(ax, world)
    for name, path in paths.items():
        draw_path(ax, path, name)
        if logs.get(name):
            ax.plot(*zip(*read_log(world, logs[name])), '-', color=LOG_COLOR, lw=1, zorder=5)
    ax.set_title('fast vs safe')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5, fontsize=8)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)


def main():
    here = os.path.join(os.path.dirname(__file__), '..')
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(here, '..', 'logs', 'hw4'))
    ap.add_argument('--fast-log')
    ap.add_argument('--safe-log')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    world = load_world(os.path.join(CONFIG, 'hw4_world.yaml'))
    paths = plan_both(world)
    plot_fast(world, paths['fast'], os.path.join(args.out, 'fast.png'), args.fast_log)
    plot_safe(world, paths['safe'], os.path.join(args.out, 'safe.png'), args.safe_log)
    plot_both(world, paths, os.path.join(args.out, 'plans.png'), {'fast': args.fast_log, 'safe': args.safe_log})


if __name__ == '__main__':
    main()
