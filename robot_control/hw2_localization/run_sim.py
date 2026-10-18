"""Run the simulation (open-loop plan vs closed-loop on the EKF), print the report, save the plot.

    python3 -m hw2_localization.run_sim            # from robot_control/; the plot needs matplotlib
    python3 -m hw2_localization.run_sim --seed 3 --runs 50 --out hw2_localization/out/sim.png
"""
import argparse
import math
import os
import random
import statistics

from . import ekf as E
from . import sim


def pose_err(est, ref):
    """(translation error m, heading error deg) of est relative to ref."""
    return math.hypot(est[0] - ref[0], est[1] - ref[1]), math.degrees(abs(E.wrap(est[2] - ref[2])))


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def waypoint_table(open_r, closed_r):
    print('\nTRUE error at each waypoint, robot vs goal (cm / deg), when the controller considers the waypoint reached')
    print(f"{'wp':>2} {'goal (x, y, th)':>18} | {'open-loop':>14} | {'closed-loop':>14}")
    for i, wp in enumerate(open_r['route']):
        o, c = (pose_err(r['snaps'][i]['true'], wp) for r in (open_r, closed_r))
        gtxt = f'({wp[0]:.1f}, {wp[1]:.1f}, {math.degrees(wp[2]):.0f})'
        print(f"{i:>2} {gtxt:>18} | {o[0] * 100:6.1f} {o[1]:7.1f} | {c[0] * 100:6.1f} {c[1]:7.1f}")
    print(f"\n{'':<34} {'open-loop':>10} {'closed-loop':>12}")
    rows = (('time to reach the last waypoint (s)', 'duration', '{:.1f}'),
            ('translation path length (m)', 'true_distance', '{:.2f}'),
            ('total turning (deg)', 'rotation', None),
            ('wheel travel incl. rotation (m)', 'wheel_travel', '{:.2f}'))
    for label, key, fmt in rows:
        vals = [r[key] for r in (open_r, closed_r)]
        if key == 'rotation':
            vals = [math.degrees(v) for v in vals]
            fmt = '{:.0f}'
        print(f"{label:<34} {fmt.format(vals[0]):>10} {fmt.format(vals[1]):>12}")
    print(f"closed-loop: {closed_r['reaims']} re-aims, {closed_r['timeouts']} timeouts, "
          f"detections used {closed_r['accepted']}, rejected {closed_r['rejected']}")


def estimator_summary(r, name):
    h = r['hist']
    e = [pose_err(a, b) for a, b in zip(h['ekf'], h['true'])]
    d = [pose_err(a, b) for a, b in zip(h['dr'], h['true'])]
    print(f"{name}: estimate error vs true, RMS pos EKF {rms([x[0] for x in e]) * 100:.1f} cm "
          f"(dead reckoning of the same commands {rms([x[0] for x in d]) * 100:.1f} cm), "
          f"RMS heading EKF {rms([x[1] for x in e]):.1f} deg")


def monte_carlo(runs, seed0):
    """Random 1-5% wheel and track-width errors per run; the same errors and noise seed for both modes."""
    stats = {m: {'pos': [], 'head': [], 'time': [], 'wheel': [], 'fail': 0, 'timeouts': 0} for m in ('open', 'closed')}
    for k in range(runs):
        g = random.Random(1000 + seed0 + k)
        sign = lambda: g.choice((-1, 1)) * g.uniform(0.01, 0.05)
        gains = dict(gain_l=1 + sign(), gain_r=1 + sign(), track_scale=1 + sign())
        for m in stats:
            r = sim.simulate(seed=seed0 + k, mode=m, **gains)
            s = stats[m]
            if not r['finished']:
                s['fail'] += 1
                continue
            errs = [pose_err(r['snaps'][i]['true'], wp) for i, wp in enumerate(r['route'])]
            s['pos'] += [e[0] for e in errs[1:]]  # waypoint 0 is the start, error is 0 by definition
            s['head'] += [e[1] for e in errs[1:]]
            s['time'].append(r['duration']), s['wheel'].append(r['wheel_travel'])
            s['timeouts'] += r['timeouts']
    print(f'\nMonte Carlo, {runs} runs, random 1-5% wheel and track-width errors; '
          f'error = true robot vs goal over waypoints 1-3')
    print(f"{'':<12} {'mean pos (cm)':>14} {'worst pos (cm)':>15} {'mean head (deg)':>16} {'worst head (deg)':>17} "
          f"{'mean time (s)':>14} {'mean wheel travel (m)':>22} {'unfinished':>11} {'timeouts':>9}")
    for m, s in stats.items():
        print(f"{m + '-loop':<12} {statistics.mean(s['pos']) * 100:14.1f} {max(s['pos']) * 100:15.1f} "
              f"{statistics.mean(s['head']):16.1f} {max(s['head']):17.1f} {statistics.mean(s['time']):14.1f} "
              f"{statistics.mean(s['wheel']):22.2f} {s['fail']:11d} {s['timeouts']:9d}")


def plot(open_r, closed_r, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h, ho = closed_r['hist'], open_r['hist']
    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(1, 2, 1)
    xy = lambda hist, key: ([p[0] for p in hist[key]], [p[1] for p in hist[key]])
    ax.plot(*xy(ho, 'true'), color='0.6', lw=1.5, ls='--', label='true path, open-loop')
    ax.plot(*xy(h, 'true'), color='tab:orange', lw=2.5, label='true path, closed-loop')
    ax.plot(*xy(h, 'ekf'), color='tab:blue', lw=1, label='EKF estimate (closed-loop run)')
    step = int(round(0.5 / sim.DT))
    ax.plot([p[0] for p in h['ekf'][::step]], [p[1] for p in h['ekf'][::step]], 'o', color='tab:blue', ms=4,
            label='EKF, dot every 0.5 s')
    for i, wp in enumerate(closed_r['route']):
        ax.plot(wp[0], wp[1], 'k*', ms=13)
        ax.annotate(f'wp{i}', (wp[0], wp[1]), textcoords='offset points', xytext=(8, 6))
    for tag_id, (tx, ty, tz, face) in E.TAG_MAP.items():
        ax.plot(tx, ty, 's', color='tab:green', ms=8)
        ax.arrow(tx, ty, 0.35 * math.cos(face), 0.35 * math.sin(face), color='tab:green', head_width=0.07)
        ax.annotate(f'tag {tag_id}', (tx, ty), textcoords='offset points', xytext=(8, -12), color='tab:green')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x (m)'), ax.set_ylabel('y (m)')
    ax.set_title('Closed-loop on the EKF (green arrows = direction each tag faces)')
    ax.legend(loc='best', fontsize=8)

    t = h['t']
    for n, (idx, label) in enumerate(((0, 'position error (cm)'), (1, 'heading error (deg)'))):
        a = fig.add_subplot(2, 2, 2 + 2 * n)
        scale = 100 if idx == 0 else 1
        a.plot(t, [pose_err(p, q)[idx] * scale for p, q in zip(h['dr'], h['true'])], 'r--',
               label='dead reckoning of the same commands')
        a.plot(t, [pose_err(p, q)[idx] * scale for p, q in zip(h['ekf'], h['true'])], color='tab:blue', label='EKF')
        a.set_ylabel(label), a.grid(alpha=0.3)
        if n == 0:
            a.legend(fontsize=8)
            a.set_title('Estimate error vs true pose (closed-loop run)')
        else:
            a.set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f'\nplot saved to {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--runs', type=int, default=30, help='Monte Carlo runs (0 to skip)')
    ap.add_argument('--out', default='hw2_localization/out/sim.png')
    args = ap.parse_args()

    open_r = sim.simulate(seed=args.seed, mode='open')
    closed_r = sim.simulate(seed=args.seed, mode='closed')
    print(f'Route {[tuple(round(a, 2) for a in w) for w in open_r["route"]]}, seed {args.seed}')
    waypoint_table(open_r, closed_r)
    print()
    estimator_summary(open_r, 'open-loop run  ')
    estimator_summary(closed_r, 'closed-loop run')
    if args.runs:
        monte_carlo(args.runs, args.seed)
    try:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        plot(open_r, closed_r, args.out)
    except ImportError:
        print('\nmatplotlib not installed here, plot skipped (run it in the ros_dev container)')


if __name__ == '__main__':
    main()
