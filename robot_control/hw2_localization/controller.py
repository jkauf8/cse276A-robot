"""Closed-loop stop-turn-go on the EKF pose estimate. ROS-free: step(pose, t) returns the [left, right] command.

Same structure as velocity_control.py (turn to face the waypoint, drive, turn to the final heading, stop between
moves), but each move ends when the ESTIMATE says it is done instead of when a timer runs out.
"""
import math

from .ekf import wrap

MIN_CMD = 0.12  # smallest wheel command used while moving (ASSUMED above the motors' deadband; measure on the robot)
TURN_TOL = math.radians(2)  # a turn is done when the heading error is below this
REAIM_TOL = math.radians(5)  # while driving, stop and re-aim if the heading error grows past this
POS_TOL = 0.03  # a drive is done when the distance to the waypoint is below this (m)
NEAR = 0.15  # closer than this the bearing is too noisy to steer by: no steering, no re-aim (m)
KP_TURN = 3.0  # turn rate (rad/s) per rad of heading error
KP_DRIVE = 2.0  # forward speed (m/s) per m of distance left
KP_HEADING = 1.5  # steering (rad/s) per rad of heading error while driving
MAX_STEER = 1.0  # rad/s
WP_TIMEOUT = 60.0  # give up on a waypoint after this long (s)


class Controller:
    def __init__(self, waypoints, wheel_cmds, cruise_speed, turn_rate, pause, turn_sign=1,
                 min_cmd=MIN_CMD, deadband=0.0, turn_tol=TURN_TOL, reaim_tol=REAIM_TOL, pos_tol=POS_TOL):
        self.wps, self.wheel_cmds = waypoints, wheel_cmds
        self.cruise, self.turn_rate, self.pause, self.turn_sign = cruise_speed, turn_rate, pause, turn_sign
        self.turn_tol, self.reaim_tol, self.pos_tol = turn_tol, reaim_tol, pos_tol
        floor = max(min_cmd, deadband)  # a wheel command below the deadband does nothing, so never ask for one
        # slowest body speeds that still give a wheel command of `floor`
        self.v_min = floor / (wheel_cmds(0.1, 0.0)[0] / 0.1)
        self.w_min = floor / (wheel_cmds(0.0, 0.1)[1] / 0.1)
        self.i = 0  # current waypoint
        self.phase = 'face'  # face -> drive -> final -> (next waypoint); 'settle' sits between moves
        self.after = None  # phase to start when 'settle' ends
        self.t0 = self.wp_t0 = 0.0
        self.limit = None  # time allowed for the current move
        self.done = False
        self.done_wp = -1  # index of the last waypoint finished
        self.timeouts = 0
        self.reaims = 0
        self.started = False

    # ---- helpers ----
    def _cmd(self, v, w):
        return self.wheel_cmds(v, self.turn_sign * w)

    def _begin(self, phase, t, pose):
        gx, gy, gth = self.wps[self.i]
        self.phase, self.t0 = phase, t
        if phase == 'drive':
            self.limit = 3 * math.hypot(gx - pose[0], gy - pose[1]) / self.cruise + 3.0
        else:
            target = math.atan2(gy - pose[1], gx - pose[0]) if phase == 'face' else gth
            self.limit = 3 * abs(wrap(target - pose[2])) / self.turn_rate + 2.0

    def _stop_then(self, nxt, t):
        self.phase, self.after, self.t0 = 'settle', nxt, t
        return (0.0, 0.0)

    def _finish_waypoint(self, t):
        self.done_wp = self.i
        self.i += 1
        self.wp_t0 = t  # timeout clock for the next waypoint
        if self.i >= len(self.wps):
            self.done = True
            return (0.0, 0.0)
        return self._stop_then('face', t)

    def _turn(self, err):
        w = math.copysign(min(self.turn_rate, max(self.w_min, KP_TURN * abs(err))), err)
        return self._cmd(0.0, w)

    # ---- one control tick ----
    def step(self, pose, t):
        if self.done:
            return (0.0, 0.0)
        if not self.started:
            self.started = True
            self.wp_t0 = t
            self._begin('face', t, pose)
        if self.phase == 'settle':
            if t - self.t0 < self.pause:
                return (0.0, 0.0)
            self._begin(self.after, t, pose)

        gx, gy, gth = self.wps[self.i]
        dist = math.hypot(gx - pose[0], gy - pose[1])
        timed_out = t - self.t0 > self.limit or t - self.wp_t0 > WP_TIMEOUT
        if timed_out:
            self.timeouts += 1
            if t - self.wp_t0 > WP_TIMEOUT:
                return self._finish_waypoint(t)  # abandon this waypoint

        if self.phase == 'face':
            if dist < self.pos_tol:  # already there, nothing to aim at
                return self._stop_then('final', t)
            err = wrap(math.atan2(gy - pose[1], gx - pose[0]) - pose[2])
            if abs(err) < self.turn_tol or timed_out:
                return self._stop_then('drive', t)
            return self._turn(err)

        if self.phase == 'drive':
            herr = wrap(math.atan2(gy - pose[1], gx - pose[0]) - pose[2])
            if dist < self.pos_tol or timed_out or (dist < 2 * NEAR and abs(herr) > math.pi / 2):  # arrived or passed it
                return self._stop_then('final', t)
            if dist > NEAR and abs(herr) > self.reaim_tol:
                self.reaims += 1
                return self._stop_then('face', t)
            v = min(self.cruise, max(self.v_min, KP_DRIVE * dist))
            steer = 0.0 if dist < NEAR else max(-MAX_STEER, min(MAX_STEER, KP_HEADING * herr))
            return self._cmd(v, steer)

        # phase == 'final'
        err = wrap(gth - pose[2])
        if abs(err) < self.turn_tol or timed_out:
            return self._finish_waypoint(t)
        return self._turn(err)
