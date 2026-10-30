"""Plot the robot's path from a run log and print the error at each waypoint"""
import argparse
import csv
import math

import yaml
from matplotlib.figure import Figure

from . import ekf as E


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def load_waypoints(path):
    with open(path) as f:
        return [tuple(map(float, ln.replace(',', ' ').split())) for ln in f if ln.strip() and not ln.startswith('#')]


def load_tag_map(path):
    if not path:
        return E.TAG_MAP
    with open(path) as f:
        t = yaml.safe_load(f)['/**']['ros__parameters']['tags']
    return {int(i): (t['x'][n], t['y'][n], t['z'][n], t['yaw'][n]) for n, i in enumerate(t['ids'])}


def pose_err(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) * 100, math.degrees(abs(E.wrap(a[2] - b[2])))


def waypoint_rows(rows, waypoints):
    """For each waypoint: the first log row after the controller finished it (None if it never did)."""
    out = []
    for i in range(len(waypoints)):
        out.append(next((r for r in rows if int(r['done_wp']) >= i), None))
    return out


def print_table(rows, waypoints):
    print('\nerror at each waypoint when the controller finished it (cm / deg)')
    print(f"{'wp':>2} {'goal (x, y, th)':>20} {'reached at':>10} | {'estimate vs goal':>18}")
    for i, (wp, r) in enumerate(zip(waypoints, waypoint_rows(rows, waypoints))):
        gtxt = f'({wp[0]:.2f}, {wp[1]:.2f}, {math.degrees(wp[2]):.0f})'
        if r is None:
            print(f'{i:>2} {gtxt:>20} {"never":>10}')
            continue
        est = (float(r['x']), float(r['y']), float(r['theta']))
        err = pose_err(est, wp)
        print(f"{i:>2} {gtxt:>20} {float(r['t']):9.1f}s | {err[0]:9.1f} {err[1]:8.1f}")
    xy = [(float(r['x']), float(r['y'])) for r in rows]
    dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:]))
    seen = sum(1 for r in rows if int(r['n_tags'])) / len(rows)
    print(f"\nrun time {float(rows[-1]['t']):.1f} s, estimated path {dist:.2f} m")
    print(f"a tag was used in {seen * 100:.0f}% of ticks")


def plot(rows, waypoints, tag_map, path):
    fig = Figure(figsize=(8, 8))
    ax = fig.subplots()
    ax.plot([float(r['x']) for r in rows], [float(r['y']) for r in rows], color='tab:blue', lw=1, label='EKF estimate')
    dots = [r for k, r in enumerate(rows) if k == 0 or int(float(r['t']) / 0.5) != int(float(rows[k - 1]['t']) / 0.5)]
    ax.plot([float(r['x']) for r in dots], [float(r['y']) for r in dots], 'o', color='tab:blue', ms=4,
            label='estimate, dot every 0.5 s')
    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], 'k*', ms=13)
        ax.annotate(f'wp{i}', wp[:2], textcoords='offset points', xytext=(8, 6))
    for tag_id, (tx, ty, tz, face) in tag_map.items():
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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='run log (localization_*.csv)')
    ap.add_argument('--waypoints', required=True, help='waypoint file used in the run')
    ap.add_argument('--params', help='params yaml, to draw the tag map')
    ap.add_argument('--out', default=None, help='PNG path (default: next to the log)')
    args = ap.parse_args()

    rows = load_csv(args.log)
    waypoints = load_waypoints(args.waypoints)
    print_table(rows, waypoints)
    out = args.out or args.log.rsplit('.', 1)[0] + '.png'
    plot(rows, waypoints, load_tag_map(args.params), out)


if __name__ == '__main__':
    main()
