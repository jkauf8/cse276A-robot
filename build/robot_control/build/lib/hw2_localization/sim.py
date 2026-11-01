"""Simulator: a robot following the HW1 open-loop plan with calibration error, fake AprilTag detections,
and two estimators run side by side (EKF with tag updates, and dead reckoning = same EKF, predict only).

The plan and wheel commands come straight from robot_control/velocity_control.py (loaded from the file with
ROS stubbed out, so it stays unmodified and the numbers can't drift apart from the real node).
"""
import bisect
import importlib
import importlib.util
import math
import random
import sys
import types
from pathlib import Path

from . import ekf as E
from .controller import Controller

ROUTE = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 2.0, math.pi), (0.0, 0.0, 0.0)]
DT = 0.05  # control tick, same as velocity_control (20 Hz)
DETECT_EVERY = 2  # ticks, so detections arrive at 10 Hz
MAX_TIME = 180.0  # s, hard stop for a closed-loop run

FOV = math.radians(60)  # horizontal field of view
MAX_RANGE = 3.5
MAX_INCIDENCE = math.radians(70)  # tag seen more edge-on than this is not detected
DROPOUT = 0.05  # chance a visible tag is missed in a given frame
# real noise on the fake detections (the filter's belief in ekf.py is deliberately a bit larger)
NOISE_R_ABS, NOISE_R_REL, NOISE_BEARING, NOISE_Z = 0.01, 0.02, 0.015, 0.01

# source tree and installed package both have robot_control/ next to hw2_localization/
VC_PATH = Path(__file__).resolve().parent.parent / 'robot_control' / 'velocity_control.py'


def load_velocity_control():
    """Import velocity_control.py without ROS: stub rclpy & co. only if they are missing."""
    stubs = {
        'rclpy': {}, 'rclpy.node': {'Node': object}, 'rclpy.signals': {'SignalHandlerOptions': object},
        'ament_index_python': {}, 'ament_index_python.packages': {'get_package_share_directory': lambda n: ''},
        'std_msgs': {}, 'std_msgs.msg': {'Float32MultiArray': object},
    }
    added = []
    for name, attrs in stubs.items():
        try:
            importlib.import_module(name)
        except ImportError:
            mod = types.ModuleType(name)
            mod.__dict__.update(attrs)
            sys.modules[name] = mod
            added.append(name)
    try:
        spec = importlib.util.spec_from_file_location('velocity_control_for_sim', VC_PATH)
        vc = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vc)
    finally:
        for name in added:
            del sys.modules[name]
    return vc


class TrueRobot:
    """The 'real' robot: same commands, but wheels and track width differ from the calibration."""

    def __init__(self, rng, gain_l, gain_r, track_scale, slip, deadband=0.10):
        self.rng, self.gain_l, self.gain_r, self.track_scale, self.slip = rng, gain_l, gain_r, track_scale, slip
        self.deadband = deadband  # wheel commands smaller than this do not move the wheel (ASSUMED)
        self.x = [0.0, 0.0, 0.0]
        self.distance = 0.0  # translation path length
        self.rotation = 0.0  # total |turning|, rad
        self.wheel_travel = 0.0  # mean of the two wheels' path lengths (counts rotation too)

    def _wheel(self, cmd, scale, gain):
        if abs(cmd) < self.deadband:
            return 0.0
        return cmd / (E.CMD_PER_MPS * scale) * gain * (1 + self.rng.gauss(0, self.slip))

    def step(self, left, right, dt):
        v_l = self._wheel(left, E.LEFT_SCALE, self.gain_l)
        v_r = self._wheel(right, E.RIGHT_SCALE, self.gain_r)
        v, w = (v_l + v_r) / 2.0, (v_r - v_l) / (E.TRACK_WIDTH * self.track_scale)
        x, y, th = self.x
        mid = th + 0.5 * w * dt
        self.x = [x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), E.wrap(th + w * dt)]
        self.distance += abs(v) * dt
        self.rotation += abs(w) * dt
        self.wheel_travel += (abs(v_l) + abs(v_r)) / 2.0 * dt


def detect(pose, rng, tag_map=None, camera=None):
    """Fake AprilTag detections from the true pose: tags in view, facing the camera, in range.
    camera = (x, y, z, pitch) mount as in ekf.CAMERA; the camera is assumed level."""
    tag_map = E.TAG_MAP if tag_map is None else tag_map
    cx, cy, cz = (E.CAMERA if camera is None else camera)[:3]
    x, y, th = pose
    cam_x = x + cx * math.cos(th) - cy * math.sin(th)
    cam_y = y + cx * math.sin(th) + cy * math.cos(th)
    out = []
    for tag_id, (tx, ty, tz, face) in tag_map.items():
        dx, dy = tx - cam_x, ty - cam_y
        r = math.hypot(dx, dy)
        bearing = E.wrap(math.atan2(dy, dx) - th)
        if r > MAX_RANGE or abs(bearing) > FOV / 2:
            continue
        if -(math.cos(face) * dx + math.sin(face) * dy) < r * math.cos(MAX_INCIDENCE):
            continue  # tag points away from the camera
        if rng.random() < DROPOUT:
            continue
        r_n = r + rng.gauss(0, NOISE_R_ABS + NOISE_R_REL * r)
        b_n = bearing + rng.gauss(0, NOISE_BEARING)
        up = (tz - cz) + rng.gauss(0, NOISE_Z)  # tag height above the camera
        # vehicle-axes vector (forward, left, up) -> camera frame (right, down, forward)
        out.append(E.Detection(tag_id, (-r_n * math.sin(b_n), -up, r_n * math.cos(b_n))))
    return out


def simulate(seed=1, gain_l=0.985, gain_r=1.015, track_scale=0.97, slip=0.01, route=ROUTE, mode='open'):
    """Run the route. mode 'open' = the fixed-duration plan from velocity_control.py,
    'closed' = controller.py steering on the EKF estimate. Returns per-tick history and per-waypoint snapshots."""
    vc = load_velocity_control()
    E.check_constants(vc)
    rng = random.Random(seed)
    robot = TrueRobot(rng, gain_l, gain_r, track_scale, slip)
    ekf, dr = E.EKF(), E.EKF()
    ctrl = None
    if mode == 'open':
        segs = vc.plan(route)
        ends, acc = [], 0.0
        for s in segs:
            acc += s[2]
            ends.append(acc)
        # tick each waypoint is reached: end of its last move (the plan for the first i+1 waypoints ends in a pause)
        wp_ticks = [int(round((sum(s[2] for s in vc.plan(route[:i + 1])) - vc.PAUSE) / DT)) for i in range(len(route))]
        max_ticks = int((acc + 1.0) / DT) + 1
    else:
        ctrl = Controller(route, vc.wheel_cmds, vc.CRUISE_SPEED, vc.TURN_RATE, vc.PAUSE, vc.TURN_SIGN)
        max_ticks = int(MAX_TIME / DT)

    hist = {k: [] for k in ('t', 'true', 'ekf', 'dr', 'seen')}
    snaps = {}
    ekf_path = 0.0
    prev = list(ekf.x)
    cmd_distance = 0.0
    end_time = None
    for k in range(max_ticks):
        t = k * DT
        seen = detect(robot.x, rng) if k % DETECT_EVERY == 0 else []
        for d in seen:
            ekf.update(d)  # dr never gets updates
        ekf_path += math.hypot(ekf.x[0] - prev[0], ekf.x[1] - prev[1])
        prev = list(ekf.x)
        hist['t'].append(t)
        hist['true'].append(list(robot.x))
        hist['ekf'].append(list(ekf.x))
        hist['dr'].append(list(dr.x))
        hist['seen'].append(len(seen))
        if mode == 'open':
            for i, wk in enumerate(wp_ticks):
                if wk == k:
                    snaps[i] = {'true': list(robot.x), 'ekf': list(ekf.x), 'dr': list(dr.x)}
            i = bisect.bisect_right(ends, t)  # move active at time t, same rule as the node's tick()
            cmd = vc.wheel_cmds(*segs[i][:2]) if i < len(segs) else (0.0, 0.0)
            if k == wp_ticks[-1]:
                end_time = t
        else:
            cmd = ctrl.step(ekf.x, t)  # the controller only sees the estimate
            if ctrl.done_wp >= 0 and ctrl.done_wp not in snaps:
                snaps[ctrl.done_wp] = {'true': list(robot.x), 'ekf': list(ekf.x), 'dr': list(dr.x)}
            if ctrl.done:
                end_time = t
                break
        v, w = E.cmds_to_vw(*cmd)  # estimators only know the commands
        cmd_distance += abs(v) * DT
        robot.step(cmd[0], cmd[1], DT)
        ekf.predict(v, w, DT)
        dr.predict(v, w, DT)
    return {'hist': hist, 'snaps': snaps, 'route': route, 'mode': mode, 'true_distance': robot.distance,
            'rotation': robot.rotation, 'wheel_travel': robot.wheel_travel, 'ekf_path': ekf_path,
            'dr_distance': cmd_distance, 'accepted': ekf.accepted, 'rejected': ekf.rejected,
            'duration': end_time, 'finished': end_time is not None and len(snaps) == len(route),
            'timeouts': ctrl.timeouts if ctrl else 0, 'reaims': ctrl.reaims if ctrl else 0}
