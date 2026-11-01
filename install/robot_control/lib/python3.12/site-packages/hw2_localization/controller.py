"""Stop-turn-go controller that uses the EKF pose. step(pose, t) returns the [left, right] wheel command.

Same moves as velocity_control.py, but a move ends when the estimated pose says so, not when a timer runs out.
"""
import math

from .ekf import wrap

MIN_CMD = 0.12  # smallest wheel command while moving (the yaml sets the real value)
TURN_TOL = math.radians(2)  # turn is done below this heading error
REAIM_TOL = math.radians(5)  # while driving, stop and re-aim above this heading error
POS_TOL = 0.03  # drive is done below this distance to the waypoint (m)
NEAR = 0.15  # closer than this: no steering, no re-aim (m)
KP_TURN = 3.0  # turn rate per rad of heading error
KP_DRIVE = 2.0  # speed per m of distance left
KP_HEADING = 1.5  # steering per rad of heading error while driving
MAX_STEER = 1.0  # rad/s
WP_TIMEOUT = 60.0  # give up on a waypoint after this long (s)
FINAL_CHECKS = 2  # heading re-checks after the last turn


class Controller:
    def __init__(self, waypoints, wheel_cmds, cruise_speed, turn_rate, pause, turn_sign=1,
                 min_cmd=MIN_CMD, deadband=0.0, turn_tol=TURN_TOL, reaim_tol=REAIM_TOL, pos_tol=POS_TOL,
                 final_checks=FINAL_CHECKS):
        self.waypoints, self.wheel_cmds = waypoints, wheel_cmds
        self.cruise, self.turn_rate, self.pause, self.turn_sign = cruise_speed, turn_rate, pause, turn_sign
        self.turn_tol, self.reaim_tol, self.pos_tol = turn_tol, reaim_tol, pos_tol
        self.final_checks, self.checks = final_checks, 0  # re-checks allowed / done
        floor = max(min_cmd, deadband)  # commands below the deadband do nothing
        # slowest speeds that still give a wheel command of floor
        self.v_min = floor / (wheel_cmds(0.1, 0.0)[0] / 0.1)
        self.w_min = floor / (wheel_cmds(0.0, 0.1)[1] / 0.1)
        self.i = 0  # current waypoint
        self.phase = 'face'  # face -> drive -> final; 'settle' is the pause between moves
        self.after = None  # phase to start after the pause
        self.phase_start = self.waypoint_start = 0.0
        self.limit = None  # time allowed for this move
        self.done = False
        self.done_wp = -1  # last waypoint finished
        self.timeouts = 0
        self.reaims = 0
        self.started = False

    def _cmd(self, v, w):
        return self.wheel_cmds(v, self.turn_sign * w)

    def _begin(self, phase, t, pose):
        gx, gy, goal_theta = self.waypoints[self.i]
        self.phase, self.phase_start = phase, t
        if phase == 'drive':
            self.limit = 3 * math.hypot(gx - pose[0], gy - pose[1]) / self.cruise + 3.0
        else:
            target = math.atan2(gy - pose[1], gx - pose[0]) if phase == 'face' else goal_theta
            self.limit = 3 * abs(wrap(target - pose[2])) / self.turn_rate + 2.0

    def _stop_then(self, nxt, t):
        self.phase, self.after, self.phase_start = 'settle', nxt, t
        return (0.0, 0.0)

    def _finish_waypoint(self, t):
        self.done_wp = self.i
        self.i += 1
        self.waypoint_start = t  # timeout clock for the next waypoint
        if self.i >= len(self.waypoints):
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
            self.waypoint_start = t
            self._begin('face', t, pose)
        if self.phase == 'settle':
            if t - self.phase_start < self.pause:
                return (0.0, 0.0)
            self._begin(self.after, t, pose)

        gx, gy, goal_theta = self.waypoints[self.i]
        dist = math.hypot(gx - pose[0], gy - pose[1])
        timed_out = t - self.phase_start > self.limit or t - self.waypoint_start > WP_TIMEOUT
        if timed_out:
            self.timeouts += 1
            if t - self.waypoint_start > WP_TIMEOUT:
                return self._finish_waypoint(t)  # abandon this waypoint

        if self.phase == 'face':
            if dist < self.pos_tol:  # already there
                return self._stop_then('final', t)
            err = wrap(math.atan2(gy - pose[1], gx - pose[0]) - pose[2])
            if abs(err) < self.turn_tol or timed_out:
                return self._stop_then('drive', t)
            return self._turn(err)

        if self.phase == 'drive':
            heading_err = wrap(math.atan2(gy - pose[1], gx - pose[0]) - pose[2])
            passed = dist < 2 * NEAR and abs(heading_err) > math.pi / 2  # waypoint is behind the robot
            if dist < self.pos_tol or timed_out or passed:
                return self._stop_then('final', t)
            if dist > NEAR and abs(heading_err) > self.reaim_tol:
                self.reaims += 1
                return self._stop_then('face', t)
            v = min(self.cruise, max(self.v_min, KP_DRIVE * dist))
            steer = 0.0 if dist < NEAR else max(-MAX_STEER, min(MAX_STEER, KP_HEADING * heading_err))
            return self._cmd(v, steer)

        err = wrap(goal_theta - pose[2])
        if self.phase == 'verify':  # pause after the last turn is over, check heading
            if abs(err) < self.turn_tol:
                return self._finish_waypoint(t)
            self._begin('final', t, pose)
            return self._turn(err)

        # phase == 'final'
        if abs(err) < self.turn_tol or timed_out:
            if self.i == len(self.waypoints) - 1 and self.checks < self.final_checks:
                self.checks += 1
                return self._stop_then('verify', t)
            return self._finish_waypoint(t)
        return self._turn(err)
