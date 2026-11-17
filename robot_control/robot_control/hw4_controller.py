"""HW2 controller that re-drives the last leg if a late tag correction shows the robot is still short of the goal."""
import math

from hw2_localization.controller import Controller
from hw2_localization.ekf import wrap


class ArriveController(Controller):
    def __init__(self, *args, arrive_tol=0.05, max_redrives=2, **kwargs):
        super().__init__(*args, **kwargs)
        self.arrive_tol, self.max_redrives, self.redrives = arrive_tol, max_redrives, 0
        self.pose = None

    def step(self, pose, t):
        self.pose = pose
        return super().step(pose, t)

    def _finish_waypoint(self, t):
        gx, gy, _ = self.waypoints[self.i]
        # the corner tags only correct the pose at the final heading check, so re-check position after it
        # only re-drive when the goal is ahead: turning around for a few cm of overshoot costs more than it fixes
        ahead = abs(wrap(math.atan2(gy - self.pose[1], gx - self.pose[0]) - self.pose[2])) < math.pi / 2
        if (self.i == len(self.waypoints) - 1 and self.redrives < self.max_redrives
                and math.hypot(gx - self.pose[0], gy - self.pose[1]) > self.arrive_tol and ahead):
            self.redrives += 1
            self.checks = 0  # verify the heading again after the re-drive
            return self._stop_then('face', t)
        return super()._finish_waypoint(t)
