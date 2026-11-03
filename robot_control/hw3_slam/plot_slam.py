"""Plot the SLAM trajectory and landmark map with their covariance, and score the map against the measured ground truth.
Runs on a laptop, not on the robot. Needs pandas, matplotlib and pyyaml.

    python3 -m hw3_slam.plot_slam slam_X.csv slam_X_landmarks.csv --truth config/hw3_ground_truth.yaml --start 1.2 0.8 25
    python3 -m hw3_slam.plot_slam --truth config/hw3_ground_truth.yaml      # only the ground-truth sketch
"""
import argparse
import math
import os

import numpy as np
import pandas as pd
import yaml
from matplotlib.figure import Figure
from matplotlib.patches import Ellipse


def rot(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def load_truth(path):
    with open(path) as f:
        return {int(i): np.array(p, dtype=float) for i, p in yaml.safe_load(f)['landmarks'].items()}


def fit_error(est, true):
    """Mean error (cm) after the best rotation and shift of the estimated map onto the true one. This leaves out the
    error from a start pose that was placed or measured a little differently from what we used."""
    est, true = np.asarray(est), np.asarray(true)
    ce, ct = est.mean(axis=0), true.mean(axis=0)
    U, _, Vt = np.linalg.svd((est - ce).T @ (true - ct))
    R = (U @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ Vt).T
    return np.linalg.norm((R @ (est - ce).T).T + ct - true, axis=1).mean() * 100


def print_scores(landmarks, start, truth, sides):
    """Error of each landmark after every finished loop. The estimate is in the start frame: rotate and shift it."""
    R, t0 = rot(start[2]), np.array(start[:2])
    print(f"{'after loop':>10} {'landmarks':>10} {'mean error (cm)':>16} {'worst (cm)':>11} {'after best fit (cm)':>20} "
          f"{'mean sigma (cm)':>16}")
    loops = sorted(set(landmarks.done_wp[landmarks.done_wp % sides == 0]) - {0})
    last = landmarks.done_wp.max()
    for wp in loops + ([last] if last not in loops else []):
        snap = landmarks[landmarks.done_wp == wp]
        snap = snap[snap.t == snap.t.max()].drop_duplicates('tag_id', keep='last')
        rows = [r for r in snap.itertuples() if r.tag_id in truth]
        err = [np.linalg.norm(R @ [r.x, r.y] + t0 - truth[r.tag_id]) * 100 for r in rows]
        sigma = [math.sqrt((r.cov_xx + r.cov_yy) / 2) * 100 for r in rows]  # per-axis 1-sigma of the landmark estimate
        label = f'{wp // sides}' if wp in loops else 'end'
        if err:
            fit = fit_error([[r.x, r.y] for r in rows], [truth[r.tag_id] for r in rows]) if len(rows) > 2 else float('nan')
            print(f'{label:>10} {len(err):>10} {np.mean(err):>16.1f} {max(err):>11.1f} {fit:>20.1f} {np.mean(sigma):>16.1f}')
    print('\nfinal map: tag, estimate (x, y), true (x, y), error')
    for r in snap.itertuples():
        est = R @ [r.x, r.y] + t0
        true = truth[r.tag_id]
        print(f'{r.tag_id:>3}  ({est[0]:.3f}, {est[1]:.3f})  ({true[0]:.3f}, {true[1]:.3f})  {np.linalg.norm(est - true) * 100:.1f} cm')
    missing = sorted(set(truth) - set(snap.tag_id))
    if missing:
        print('not in the map:', missing)
    return snap


def plot(path, truth, log=None, snap=None, start=None):
    fig = Figure(figsize=(8, 8))
    ax = fig.subplots()
    for tag_id, (x, y) in truth.items():
        ax.plot(x, y, 's', color='tab:green', ms=8, label='measured landmark' if tag_id == min(truth) else None)
        ax.annotate(f'{tag_id}', (x, y), textcoords='offset points', xytext=(6, 6), color='tab:green')
    if log is not None:
        R, t0 = rot(start[2]), np.array(start[:2])
        xy = (R @ log[['x', 'y']].to_numpy().T).T + t0
        ax.plot(xy[:, 0], xy[:, 1], color='tab:blue', lw=1, label='estimated trajectory')
        ax.plot(*xy[0], 'o', color='tab:blue', ms=7, label='start')
        for r in snap.itertuples():
            est = R @ [r.x, r.y] + t0
            cov = R @ np.array([[r.cov_xx, r.cov_xy], [r.cov_xy, r.cov_yy]]) @ R.T
            vals, vecs = np.linalg.eigh(cov)
            ax.add_patch(Ellipse(est, 4 * math.sqrt(vals[1]), 4 * math.sqrt(vals[0]),
                                 angle=math.degrees(math.atan2(vecs[1, 1], vecs[0, 1])),
                                 fill=False, color='tab:red'))  # 2-sigma ellipse
            ax.plot(*est, 'x', color='tab:red', ms=7, label='estimated landmark (2-sigma)' if r.Index == snap.index[0] else None)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)'), ax.set_ylabel('y (m)')
    ax.set_title('SLAM map and trajectory (ground-truth frame)' if log is not None else 'Ground-truth map (top view)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f'plot saved to {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?', help='trajectory log (slam_*.csv)')
    ap.add_argument('landmarks', nargs='?', help='landmark log (slam_*_landmarks.csv)')
    ap.add_argument('--truth', required=True, help='yaml with the measured landmark positions')
    ap.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'DEG'), help='measured start pose in the truth frame')
    ap.add_argument('--sides', type=int, default=4, help='waypoints per loop (4 square, 8 octagon)')
    ap.add_argument('--out', default=None, help='PNG path (default: next to the log)')
    args = ap.parse_args()

    truth = load_truth(args.truth)
    if not args.log:
        plot(args.out or 'ground_truth.png', truth)
        return
    start = (args.start[0], args.start[1], math.radians(args.start[2]))
    log, landmarks = pd.read_csv(args.log), pd.read_csv(args.landmarks)
    snap = print_scores(landmarks, start, truth, args.sides)
    plot(args.out or os.path.splitext(args.log)[0] + '.png', truth, log, snap, start)


if __name__ == '__main__':
    main()
