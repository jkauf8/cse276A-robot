"""EKF-SLAM. State is [x, y, theta, m1x, m1y, m2x, m2y, ...]: the robot pose followed by every landmark in the map.
World frame: origin = start pose, x = initial facing, y = left (the robot does not know where it started).

predict: moves the robot with the last wheel command. Landmarks do not move.
update: corrects the robot and the landmarks with the range and bearing to a tag (matched by its id).
A tag joins the map after it has been seen INIT_COUNT times in agreement.
"""
import math

import numpy as np

from hw2_localization import ekf as E

INIT_COUNT = 3  # detections that must agree before a new tag is added to the map
INIT_SPREAD = 0.30  # they agree if each landmark guess is within this distance of their mean (m)


class EKFSlam:
    def __init__(self, pose=(0.0, 0.0, 0.0), sigma0=(0.01, 0.01, 0.01), camera=E.CAMERA, noise=None,
                 init_count=INIT_COUNT, init_spread=INIT_SPREAD):
        self.x = np.array(pose, dtype=float)
        self.P = np.diag(np.square(sigma0))
        self.index = {}  # tag id -> where its x sits in the state (3, 5, 7, ...)
        self.cam_x, self.cam_y, self.cam_z, pitch = camera
        self.r_vc = E.rotation_vc(pitch)
        self.n = {**E.NOISE, **(noise or {})}
        self.init_count, self.init_spread = init_count, init_spread
        self.guesses = {}  # tag id -> landmark guesses from the last detections, tag not in the map yet
        self.accepted = 0
        self.rejected = 0
        self.last = None  # what happened to the latest detection: (range, bearing, NIS, status), for logging

    @property
    def pose(self):
        return [float(v) for v in self.x[:3]]

    def predict(self, v, w, dt):
        x, y, theta = self.x[:3]
        mid = theta + 0.5 * w * dt
        self.x[:3] = [x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), E.wrap(theta + w * dt)]
        F = np.array([[1.0, 0.0, -v * dt * math.sin(mid)],
                      [0.0, 1.0, v * dt * math.cos(mid)],
                      [0.0, 0.0, 1.0]])
        G = np.array([[dt * math.cos(mid), -0.5 * v * dt * dt * math.sin(mid)],  # d(pose)/d(v, w)
                      [dt * math.sin(mid), 0.5 * v * dt * dt * math.cos(mid)],
                      [0.0, dt]])
        sigma_v = self.n['sig_v_rel'] * abs(v) + self.n['sig_v_abs']
        sigma_w = self.n['sig_w_rel'] * abs(w) + self.n['sig_w_abs']
        Q = G @ np.diag([sigma_v ** 2, sigma_w ** 2]) @ G.T
        self.P[:3, :3] = F @ self.P[:3, :3] @ F.T + Q
        self.P[:3, 3:] = F @ self.P[:3, 3:]  # landmarks do not move, only their link to the robot changes
        self.P[3:, :3] = self.P[:3, 3:].T

    def expected(self, pose, m):
        """Range and bearing from the camera to landmark m, and the 2x3 Jacobian with respect to the robot pose."""
        x, y, theta = pose
        c, s = math.cos(theta), math.sin(theta)
        dx = m[0] - (x + self.cam_x * c - self.cam_y * s)
        dy = m[1] - (y + self.cam_x * s + self.cam_y * c)
        r2 = dx * dx + dy * dy
        r = math.sqrt(r2)
        ddx_dth = self.cam_x * s + self.cam_y * c  # d(dx)/d(theta)
        ddy_dth = -self.cam_x * c + self.cam_y * s  # d(dy)/d(theta)
        H = np.array([[-dx / r, -dy / r, (dx * ddx_dth + dy * ddy_dth) / r],
                      [dy / r2, -dx / r2, (-dy * ddx_dth + dx * ddy_dth) / r2 - 1.0]])
        return r, E.wrap(math.atan2(dy, dx) - theta), H

    def landmark_guess(self, pose, z_r, z_b):
        """Landmark position (x, y) from a measured range and bearing seen from this robot pose."""
        x, y, theta = pose
        cam_x = x + self.cam_x * math.cos(theta) - self.cam_y * math.sin(theta)
        cam_y = y + self.cam_x * math.sin(theta) + self.cam_y * math.cos(theta)
        return np.array([cam_x + z_r * math.cos(theta + z_b), cam_y + z_r * math.sin(theta + z_b)])

    def update(self, det, at=None):
        """Use one detection. Returns True if it changed the state (correction or new landmark).
        at: robot pose from when the image was taken, used for the expected measurement if the detection is late."""
        z_r, z_b = E.camera_range_bearing(det.t_cam, self.r_vc)
        if z_r < 0.05:
            self.last = (z_r, z_b, float('nan'), 'too close')
            return False
        pose = self.pose if at is None else list(at)
        R = np.diag([(self.n['sig_r_abs'] + self.n['sig_r_rel'] * z_r) ** 2, self.n['sig_bearing'] ** 2])
        if det.tag_id not in self.index:
            added = self._new_landmark(det.tag_id, pose, z_r, z_b, R)
            self.last = (z_r, z_b, float('nan'), 'new' if added else 'waiting')
            return added

        i = self.index[det.tag_id]
        h_r, h_b, H_pose = self.expected(pose, self.x[i:i + 2])
        H = np.zeros((2, len(self.x)))
        H[:, :3] = H_pose
        H[:, i:i + 2] = -H_pose[:, :2]  # moving the landmark moves the expected measurement the opposite way
        innovation = np.array([z_r - h_r, E.wrap(z_b - h_b)])
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        nis = innovation @ S_inv @ innovation  # normalized innovation squared
        if nis > self.n['gate']:
            self.rejected += 1
            self.last = (z_r, z_b, nis, 'rejected')
            return False
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        self.x[2] = E.wrap(self.x[2])
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T  # Joseph form
        self.accepted += 1
        self.last = (z_r, z_b, nis, 'used')
        return True

    def _new_landmark(self, tag_id, pose, z_r, z_b, R):
        """Keep the last few guesses for an unmapped tag. Add it once they agree (a lone misread id never gets in)."""
        guesses = self.guesses.setdefault(tag_id, [])
        guesses.append(self.landmark_guess(pose, z_r, z_b))
        del guesses[:-self.init_count]
        if len(guesses) < self.init_count:
            return False
        mean = np.mean(guesses, axis=0)
        if max(np.linalg.norm(g - mean) for g in guesses) > self.init_spread:
            return False

        x, y, theta = pose
        c, s = math.cos(theta), math.sin(theta)
        phi = theta + z_b  # direction to the tag in the world frame
        Gx = np.array([[1.0, 0.0, -(self.cam_x * s + self.cam_y * c) - z_r * math.sin(phi)],  # d(landmark)/d(pose)
                       [0.0, 1.0, (self.cam_x * c - self.cam_y * s) + z_r * math.cos(phi)]])
        Gz = np.array([[math.cos(phi), -z_r * math.sin(phi)],  # d(landmark)/d(range, bearing)
                       [math.sin(phi), z_r * math.cos(phi)]])
        n = len(self.x)
        P_mr = Gx @ self.P[:3, :]  # new landmark vs everything already in the state
        P = np.zeros((n + 2, n + 2))
        P[:n, :n] = self.P
        P[n:, :n] = P_mr
        P[:n, n:] = P_mr.T
        P[n:, n:] = Gx @ self.P[:3, :3] @ Gx.T + Gz @ R @ Gz.T
        self.P = P
        self.x = np.concatenate([self.x, self.landmark_guess(pose, z_r, z_b)])
        self.index[tag_id] = n
        del self.guesses[tag_id]
        return True

    def landmarks(self):
        """Map as {tag id: (x, y, 2x2 covariance)}."""
        return {t: (float(self.x[i]), float(self.x[i + 1]), self.P[i:i + 2, i:i + 2].copy())
                for t, i in self.index.items()}
