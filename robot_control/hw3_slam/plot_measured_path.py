"""Plot where the tags say the robot was, next to the path the filter believed and the ideal path.
The logged x, y is the filter's belief (the controller steers it onto each corner). Whenever two or more tags are seen at
once, the known tag positions give the robot's real pose. Runs on a laptop. Needs pandas, matplotlib, pyyaml and scipy.

    python3 -m hw3_slam.plot_measured_path slam_X.csv slam_X_detections.csv --truth config/hw3_ground_truth.yaml --shape octagon
"""
import argparse
import math
import os

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from scipy.optimize import least_squares

from hw3_slam import paths
from hw3_slam.plot_slam import load_truth, rot

START_GUESS = (1.7, 0.9, math.pi / 2)  # x, y (m), heading (rad): where the start fit begins (near the start, facing the top wall)
MOMENT = 0.1  # s; detections in the same bin of this width count as seen at the same moment
MAX_RESIDUAL = 0.03  # keep a pose fit only if every range (m) and bearing (rad) residual is below this...
MAX_JUMP = 1.0  # ...and it lies within this distance (m) of where the filter believed the robot was


def solve_pose(rows, truth, cam_x, guess):
    """Robot pose (x, y, theta) that best explains the measured range and bearing to the tags in rows."""
    def residual(p):
        x, y, th = p
        cx, cy = x + cam_x * math.cos(th), y + cam_x * math.sin(th)
        out = []
        for r in rows.itertuples():
            dx, dy = truth[r.tag_id][0] - cx, truth[r.tag_id][1] - cy
            out += [math.hypot(dx, dy) - r.range, (math.atan2(dy, dx) - th - r.bearing + math.pi) % (2 * math.pi) - math.pi]
        return np.array(out)

    fit = least_squares(residual, guess)
    return fit.x, np.abs(fit.fun).max()


def distance_to_outline(p, corners):
    """Distance from point p to the closed polygon through corners."""
    best = np.inf
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        ab = b - a
        best = min(best, np.linalg.norm(p - (a + np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1) * ab)))
    return best


def find_start(log, det, truth, cam_x):
    """The real start pose (x, y, theta in the truth frame), from the tags seen before the robot first moved."""
    moved = log.t[(log.left != 0) | (log.right != 0)].iloc[0]
    early = det[det.t < moved].groupby('tag_id').agg(range=('range', 'median'), bearing=('bearing', 'median')).reset_index()
    return solve_pose(early, truth, cam_x, START_GUESS)[0]


def pose_fixes(log, det, truth, cam_x, start):
    """(time, x, y) in the truth frame of the robot whenever two or more tags are seen at the same moment."""
    R, t0 = rot(start[2]), start[:2]
    det = det.assign(moment=(det.t / MOMENT).round().astype(int))
    poses = []
    for _, g in det.groupby('moment'):
        g = g.drop_duplicates('tag_id')
        if len(g) < 2:
            continue
        i = (log.t - g.t.mean()).abs().idxmin()
        believed = R @ [log.x[i], log.y[i]] + t0
        p, worst = solve_pose(g, truth, cam_x, [*believed, log.theta[i] + start[2]])
        if worst < MAX_RESIDUAL and np.linalg.norm(p[:2] - believed) < MAX_JUMP:
            poses.append((g.t.mean(), *p[:2]))
    return np.array(poses)


def draw(shape, truth, ideal, believed, poses):
    fig = Figure(figsize=(8, 8))
    ax = fig.subplots()
    for tag_id, (x, y) in truth.items():
        ax.plot(x, y, 's', color='tab:green', ms=8)
        ax.annotate(str(tag_id), (x, y), xytext=(6, 6), textcoords='offset points', color='tab:green')
    ax.plot(*np.vstack([ideal, ideal[:1]]).T, '--', color='gray', lw=1.5, label=f'ideal {shape} from the real start')
    ax.plot(believed[:, 0], believed[:, 1], color='tab:blue', lw=0.8, alpha=0.6,
            label='what the filter believed (the logged x, y)')
    sc = ax.scatter(poses[:, 1], poses[:, 2], c=poses[:, 0], cmap='viridis', s=14, zorder=3,
                    label='where the tags say the robot was (colour = time)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)'), ax.set_ylabel('y (m)')
    ax.set_title(f'{shape.capitalize()}: believed path vs path measured from the tags')
    ax.legend(fontsize=7, loc='upper right')
    fig.colorbar(sc, ax=ax, shrink=0.6, label='time (s)')
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='trajectory log (slam_*.csv)')
    ap.add_argument('detections', help='detection log (slam_*_detections.csv)')
    ap.add_argument('--truth', required=True, help='yaml with the measured landmark positions')
    ap.add_argument('--shape', choices=paths.CORNERS, required=True)
    ap.add_argument('--side-ft', type=float, default=None, help='side length in ft (default 3 square, 1.5 octagon)')
    ap.add_argument('--camera-x', type=float, default=0.01,
                    help='camera distance ahead of the wheel midpoint (m); keep equal to camera.x in slam_full.yaml')
    ap.add_argument('--out', default=None, help='PNG path (default: next to the log)')
    args = ap.parse_args()

    truth = load_truth(args.truth)
    log, det = pd.read_csv(args.log), pd.read_csv(args.detections)
    det = det[det.status.isin(['used', 'rejected', 'new', 'waiting']) & det.tag_id.isin(truth)]

    start = find_start(log, det, truth, args.camera_x)
    print(f'real start from the first detections: ({start[0] / 0.0254:.1f} in, {start[1] / 0.0254:.1f} in), '
          f'heading {math.degrees(start[2]):.0f} deg')
    R, t0 = rot(start[2]), start[:2]
    poses = pose_fixes(log, det, truth, args.camera_x, start)

    corners = paths.CORNERS[args.shape]
    side = args.side_ft * paths.FT if args.side_ft else paths.SIDE[args.shape]
    ideal = (R @ np.array([w[:2] for w in paths.polygon(corners, side)][:corners]).T).T + t0
    believed = (R @ log[['x', 'y']].to_numpy().T).T + t0
    off = np.array([distance_to_outline(p, ideal) for p in poses[:, 1:]]) * 100
    print(f'{len(poses)} poses from sightings of 2+ tags; they are {np.median(off):.0f} cm (median), '
          f'{np.percentile(off, 90):.0f} cm (90th percentile), {off.max():.0f} cm (worst) from the ideal outline')

    out = args.out or os.path.splitext(args.log)[0] + '_measured_path.png'
    draw(args.shape, truth, ideal, believed, poses).savefig(out, dpi=130)
    print(f'plot saved to {out}')


if __name__ == '__main__':
    main()
