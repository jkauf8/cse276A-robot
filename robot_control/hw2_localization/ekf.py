"""EKF pose estimator, state [x, y, theta] in the world frame (origin = start pose, x = initial facing,
y = left, theta CCW). Pure Python (stdlib only), no ROS, so the ROS 2 node and the simulator share it.

predict(v, w, dt):  same differential-drive model as robot_control/velocity_control.py (cmds_to_vw inverts
                    its wheel_cmds), plus process noise.
update(detection):  AprilTag position in the camera frame -> range and bearing from the CAMERA to a tag at a
                    known map position. The camera sits cam_x ahead of the robot center, so h() uses the
                    camera position, not the robot position.
"""
import math
from dataclasses import dataclass

# ---- Model constants, copied from velocity_control.py (check_constants() verifies they still match) ----
CMD_PER_MPS = 0.74
TRACK_WIDTH = 0.37
LEFT_SCALE = 0.9975
RIGHT_SCALE = 0.9970

# ---- Defaults for the camera mount (vehicle frame: x forward, y left, z up): (x, y, z, pitch down in rad) ----
CAMERA = (0.085, 0.0, 0.125, 0.0)
# camera frame (OpenCV: x right, y down, z forward) -> vehicle frame, camera looking straight ahead
R_VC_LEVEL = ((0.0, 0.0, 1.0),
              (-1.0, 0.0, 0.0),
              (0.0, -1.0, 0.0))

# ---- Default tag map: id -> (x, y, z, facing yaw of the tag's normal), meters / radians. First pass, re-measure. ----
TAG_MAP = {
    0: (2.5, 0.0, 0.12, math.pi),
    1: (1.0, 3.0, 0.12, -math.pi / 2),
    2: (-1.5, 2.0, 0.12, 0.0),
    3: (-0.5, -1.5, 0.12, math.radians(63)),
}

# ---- Default noise (the filter's belief; sim.py draws the real noise separately) ----
NOISE = {
    'sig_v_rel': 0.20, 'sig_v_abs': 0.01,  # forward speed noise: 20% of |v| plus 1 cm/s (large on purpose:
    'sig_w_rel': 0.20, 'sig_w_abs': 0.15,  # open-loop bias acts like noise); turn rate: 20% of |w| plus 0.15 rad/s
    'sig_r_abs': 0.02, 'sig_r_rel': 0.03,  # range noise: 2 cm + 3% of range
    'sig_bearing': 0.03,  # bearing noise, rad (about 1.7 degrees)
    'gate': 20.0,  # reject a detection whose normalized innovation squared (2 dof) is above this (about 99.99%)
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- tiny matrix helpers (lists of lists) ----
def mm(A, B):
    return [[sum(a * b for a, b in zip(row, col)) for col in zip(*B)] for row in A]


def tr(A):
    return [list(r) for r in zip(*A)]


def madd(A, B):
    return [[a + b for a, b in zip(ra, rb)] for ra, rb in zip(A, B)]


def msub(A, B):
    return [[a - b for a, b in zip(ra, rb)] for ra, rb in zip(A, B)]


def inv2(S):
    (a, b), (c, d) = S
    det = a * d - b * c
    return [[d / det, -b / det], [-c / det, a / det]]


def cmds_to_vw(left, right):
    """Invert velocity_control.wheel_cmds: motor commands [L, R] -> (v m/s, w rad/s)."""
    v_l = left / (CMD_PER_MPS * LEFT_SCALE)
    v_r = right / (CMD_PER_MPS * RIGHT_SCALE)
    return (v_l + v_r) / 2.0, (v_r - v_l) / TRACK_WIDTH


def check_constants(vc):
    """The copies above must equal the constants velocity_control.py actually uses (vc = that module)."""
    for name in ('CMD_PER_MPS', 'TRACK_WIDTH', 'LEFT_SCALE', 'RIGHT_SCALE'):
        if getattr(vc, name) != globals()[name]:
            raise ValueError(f'{name}: velocity_control.py has {getattr(vc, name)}, ekf.py has {globals()[name]}')


def rotation_vc(pitch=0.0):
    """Vehicle <- camera rotation for a camera tilted down by `pitch` rad (0 = level)."""
    c, s = math.cos(pitch), math.sin(pitch)
    tilt = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return tuple(tuple(r) for r in mm(tilt, R_VC_LEVEL))


@dataclass
class Detection:
    tag_id: int
    t_cam: tuple  # tag center in the camera frame (x right, y down, z forward), meters


def camera_range_bearing(t_cam, r_vc=R_VC_LEVEL):
    """Tag position in the camera frame -> horizontal range and bearing (left positive) from the camera."""
    rel = [sum(r_vc[i][j] * t_cam[j] for j in range(3)) for i in range(3)]  # same vector, vehicle axes
    return math.hypot(rel[0], rel[1]), math.atan2(rel[1], rel[0])


class EKF:
    def __init__(self, pose=(0.0, 0.0, 0.0), sigma0=(0.01, 0.01, 0.01), tag_map=None, camera=CAMERA, noise=None):
        self.x = list(pose)
        self.P = [[sigma0[i] ** 2 if i == j else 0.0 for j in range(3)] for i in range(3)]
        self.tag_map = TAG_MAP if tag_map is None else tag_map
        self.cam_x, self.cam_y, self.cam_z, pitch = camera
        self.r_vc = rotation_vc(pitch)
        self.n = {**NOISE, **(noise or {})}
        self.accepted = 0
        self.rejected = 0

    def predict(self, v, w, dt):
        x, y, th = self.x
        mid = th + 0.5 * w * dt
        self.x = [x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), wrap(th + w * dt)]
        F = [[1.0, 0.0, -v * dt * math.sin(mid)],
             [0.0, 1.0, v * dt * math.cos(mid)],
             [0.0, 0.0, 1.0]]
        G = [[dt * math.cos(mid), -0.5 * v * dt * dt * math.sin(mid)],  # d(state)/d(v, w)
             [dt * math.sin(mid), 0.5 * v * dt * dt * math.cos(mid)],
             [0.0, dt]]
        sv = self.n['sig_v_rel'] * abs(v) + self.n['sig_v_abs']
        sw = self.n['sig_w_rel'] * abs(w) + self.n['sig_w_abs']
        Q = mm(mm(G, [[sv * sv, 0.0], [0.0, sw * sw]]), tr(G))
        self.P = madd(mm(mm(F, self.P), tr(F)), Q)

    def h(self, tag, pose=None):
        """Expected (range, bearing) from the camera to the tag, and its Jacobian wrt [x, y, theta]."""
        x, y, th = self.x if pose is None else pose
        tx, ty = tag[0], tag[1]
        c, s = math.cos(th), math.sin(th)
        dx = tx - (x + self.cam_x * c - self.cam_y * s)
        dy = ty - (y + self.cam_x * s + self.cam_y * c)
        r2 = dx * dx + dy * dy
        r = math.sqrt(r2)
        ddx_dth = self.cam_x * s + self.cam_y * c  # d(dx)/d(theta)
        ddy_dth = -self.cam_x * c + self.cam_y * s  # d(dy)/d(theta)
        H = [[-dx / r, -dy / r, (dx * ddx_dth + dy * ddy_dth) / r],
             [dy / r2, -dx / r2, (-dy * ddx_dth + dx * ddy_dth) / r2 - 1.0]]
        return (r, wrap(math.atan2(dy, dx) - th)), H

    def update(self, det, at=None):
        """Fuse one detection. Returns True if used, False if unknown tag or rejected by the gate.
        at: the pose estimate from when the image was taken (for a late detection). The expected measurement
        is computed there and the correction is applied to the current state."""
        tag = self.tag_map.get(det.tag_id)
        if tag is None:
            return False
        z_r, z_b = camera_range_bearing(det.t_cam, self.r_vc)
        if z_r < 0.05:
            return False
        (h_r, h_b), H = self.h(tag, at)
        innov = [[z_r - h_r], [wrap(z_b - h_b)]]
        R = [[(self.n['sig_r_abs'] + self.n['sig_r_rel'] * z_r) ** 2, 0.0], [0.0, self.n['sig_bearing'] ** 2]]
        S = madd(mm(mm(H, self.P), tr(H)), R)
        Si = inv2(S)
        nis = mm(mm(tr(innov), Si), innov)[0][0]
        if nis > self.n['gate']:
            self.rejected += 1
            return False
        K = mm(mm(self.P, tr(H)), Si)
        dxs = mm(K, innov)
        self.x = [self.x[0] + dxs[0][0], self.x[1] + dxs[1][0], wrap(self.x[2] + dxs[2][0])]
        IKH = msub([[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)], mm(K, H))
        self.P = madd(mm(mm(IKH, self.P), tr(IKH)), mm(mm(K, R), tr(K)))  # Joseph form
        self.accepted += 1
        return True
