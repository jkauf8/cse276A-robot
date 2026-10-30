"""EKF pose estimator. State is [x, y, theta] in the world frame (origin = start pose, x = initial facing, y = left).

predict: moves the pose using the last wheel command (same model as velocity_control.py).
update: corrects the pose with the range and bearing to a tag at a known position.
"""
import math
from dataclasses import dataclass

# Model constants, copied from velocity_control.py
CMD_PER_MPS = 0.74
TRACK_WIDTH = 0.37
LEFT_SCALE = 0.9975
RIGHT_SCALE = 0.9970

# Camera mount in the vehicle frame: (x forward, y left, z up, pitch down in rad)
CAMERA = (0.085, 0.0, 0.125, 0.0)
# camera frame (x right, y down, z forward) -> vehicle frame, camera level
R_VC_LEVEL = ((0.0, 0.0, 1.0),
              (-1.0, 0.0, 0.0),
              (0.0, -1.0, 0.0))

# Tag map: id -> (x, y, z, yaw the tag faces), m and rad. Same as localization_full.yaml, which the robot reads.
TAG_MAP = {
    0: (2.0, 0.0, 0.12, 3.14159),
    1: (1.5, 3.0, 0.12, -2.0344),
    2: (0.5, 3.0, 0.12, -1.1071),
    3: (0.0, 2.0, 0.12, 0.0),
    4: (-0.5, -0.5, 0.12, 0.7854),
}

# Noise values
NOISE = {
    'sig_v_rel': 0.20, 'sig_v_abs': 0.01,  # speed noise: 20% of |v| + 1 cm/s (large on purpose)
    'sig_w_rel': 0.20, 'sig_w_abs': 0.15,  # turn noise: 20% of |w| + 0.15 rad/s
    'sig_r_abs': 0.02, 'sig_r_rel': 0.03,  # range noise: 2 cm + 3% of range
    'sig_bearing': 0.03,  # bearing noise (rad)
    'gate': 20.0,  # reject a detection if its normalized innovation squared is above this
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- tiny matrix helpers (lists of lists) ----
def matmul(A, B):
    return [[sum(a * b for a, b in zip(row, col)) for col in zip(*B)] for row in A]


def transpose(A):
    return [list(r) for r in zip(*A)]


def mat_add(A, B):
    return [[a + b for a, b in zip(ra, rb)] for ra, rb in zip(A, B)]


def mat_sub(A, B):
    return [[a - b for a, b in zip(ra, rb)] for ra, rb in zip(A, B)]


def inv2(S):
    (a, b), (c, d) = S
    det = a * d - b * c
    return [[d / det, -b / det], [-c / det, a / det]]


def cmds_to_vw(left, right):
    """Wheel commands [L, R] -> forward speed v and turn rate w (inverse of wheel_cmds in velocity_control.py)."""
    v_l = left / (CMD_PER_MPS * LEFT_SCALE)
    v_r = right / (CMD_PER_MPS * RIGHT_SCALE)
    return (v_l + v_r) / 2.0, (v_r - v_l) / TRACK_WIDTH


def check_constants(vc):
    """Check the constants above match velocity_control.py (vc)."""
    for name in ('CMD_PER_MPS', 'TRACK_WIDTH', 'LEFT_SCALE', 'RIGHT_SCALE'):
        if getattr(vc, name) != globals()[name]:
            raise ValueError(f'{name}: velocity_control.py has {getattr(vc, name)}, ekf.py has {globals()[name]}')


def rotation_vc(pitch=0.0):
    """Rotation from camera to vehicle axes for a camera tilted down by pitch (rad)."""
    c, s = math.cos(pitch), math.sin(pitch)
    tilt = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return tuple(tuple(r) for r in matmul(tilt, R_VC_LEVEL))


@dataclass
class Detection:
    tag_id: int
    t_cam: tuple  # tag center in the camera frame (x right, y down, z forward), m


def camera_range_bearing(t_cam, r_vc=R_VC_LEVEL):
    """Tag position in the camera frame -> range and bearing (left positive) from the camera."""
    rel = [sum(r_vc[i][j] * t_cam[j] for j in range(3)) for i in range(3)]  # same vector in vehicle axes
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
        x, y, theta = self.x
        mid = theta + 0.5 * w * dt
        self.x = [x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), wrap(theta + w * dt)]
        F = [[1.0, 0.0, -v * dt * math.sin(mid)],
             [0.0, 1.0, v * dt * math.cos(mid)],
             [0.0, 0.0, 1.0]]
        G = [[dt * math.cos(mid), -0.5 * v * dt * dt * math.sin(mid)],  # d(state)/d(v, w)
             [dt * math.sin(mid), 0.5 * v * dt * dt * math.cos(mid)],
             [0.0, dt]]
        sigma_v = self.n['sig_v_rel'] * abs(v) + self.n['sig_v_abs']
        sigma_w = self.n['sig_w_rel'] * abs(w) + self.n['sig_w_abs']
        Q = matmul(matmul(G, [[sigma_v * sigma_v, 0.0], [0.0, sigma_w * sigma_w]]), transpose(G))
        self.P = mat_add(matmul(matmul(F, self.P), transpose(F)), Q)

    def h(self, tag, pose=None):
        """Expected (range, bearing) from the camera to the tag, and its Jacobian."""
        x, y, theta = self.x if pose is None else pose
        tx, ty = tag[0], tag[1]
        c, s = math.cos(theta), math.sin(theta)
        dx = tx - (x + self.cam_x * c - self.cam_y * s)
        dy = ty - (y + self.cam_x * s + self.cam_y * c)
        r2 = dx * dx + dy * dy
        r = math.sqrt(r2)
        ddx_dth = self.cam_x * s + self.cam_y * c  # d(dx)/d(theta)
        ddy_dth = -self.cam_x * c + self.cam_y * s  # d(dy)/d(theta)
        H = [[-dx / r, -dy / r, (dx * ddx_dth + dy * ddy_dth) / r],
             [dy / r2, -dx / r2, (-dy * ddx_dth + dx * ddy_dth) / r2 - 1.0]]
        return (r, wrap(math.atan2(dy, dx) - theta)), H

    def update(self, det, at=None):
        """Correct the pose with one detection. Returns False if the tag is unknown or the gate rejects it.
        at: pose from when the image was taken, used for the expected measurement if the detection is late."""
        tag = self.tag_map.get(det.tag_id)
        if tag is None:
            return False
        z_r, z_b = camera_range_bearing(det.t_cam, self.r_vc)
        if z_r < 0.05:
            return False
        (h_r, h_b), H = self.h(tag, at)
        innovation = [[z_r - h_r], [wrap(z_b - h_b)]]
        R = [[(self.n['sig_r_abs'] + self.n['sig_r_rel'] * z_r) ** 2, 0.0], [0.0, self.n['sig_bearing'] ** 2]]
        S = mat_add(matmul(matmul(H, self.P), transpose(H)), R)
        S_inv = inv2(S)
        nis = matmul(matmul(transpose(innovation), S_inv), innovation)[0][0]
        if nis > self.n['gate']:
            self.rejected += 1
            return False
        K = matmul(matmul(self.P, transpose(H)), S_inv)
        correction = matmul(K, innovation)
        self.x = [self.x[0] + correction[0][0], self.x[1] + correction[1][0], wrap(self.x[2] + correction[2][0])]
        I_KH = mat_sub([[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)], matmul(K, H))
        self.P = mat_add(matmul(matmul(I_KH, self.P), transpose(I_KH)), matmul(matmul(K, R), transpose(K)))  # Joseph form
        self.accepted += 1
        return True
