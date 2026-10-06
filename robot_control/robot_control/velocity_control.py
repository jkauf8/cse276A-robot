"""Stop-turn-go waypoint follower (open-loop, timed): turn to face each waypoint, drive straight,
turn to the final heading. No position sensor, so accuracy comes from calibrating the constants."""
import math
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

# ---- Calibration constants ----
CMD_PER_MPS = 1.0    # TODO: calibrate on real robot: motor command per m/s (straight-run test)
TRACK_WIDTH = 0.20   # TODO: calibrate on real robot: effective wheel spacing in m (spin test)
LEFT_SCALE = 1.0     # TODO: calibrate on real robot: >1 boosts the left wheel if it curves right
RIGHT_SCALE = 1.0    # TODO: calibrate on real robot: >1 boosts the right wheel if it curves left
CRUISE_SPEED = 0.15  # m/s, kept low for first runs
TURN_RATE = 1.0      # rad/s, kept low for first runs
TURN_SIGN = 1        # TODO: flip to -1 if positive omega (L-, R+) turns the robot right
PAUSE = 0.3          # s of zeros between moves
RATE = 20.0          # Hz; motor_control stops the wheels after 0.15 s without a message
CMD_LIMIT = 0.5      # motor_control clips each wheel to +/-0.5


def wrap(a):
    """Wrap an angle to [-pi, pi] so every turn takes the shortest way."""
    return math.atan2(math.sin(a), math.cos(a))


def wheel_cmds(v, w):
    """Kinematic model, differential drive (Lecture 4): v_l = v - wL/2, v_r = v + wL/2 [m/s],
    then motor command = wheel speed * CMD_PER_MPS, clipped to +/-0.5."""
    v_l = v - w * TRACK_WIDTH / 2.0
    v_r = v + w * TRACK_WIDTH / 2.0
    clip = lambda c: max(-CMD_LIMIT, min(CMD_LIMIT, c))  # noqa: E731
    return clip(v_l * CMD_PER_MPS * LEFT_SCALE), clip(v_r * CMD_PER_MPS * RIGHT_SCALE)


def plan(waypoints):
    """Moves as (v, w, seconds). Believed pose starts at (0, 0, 0), assuming each move is exact."""
    segs, x, y, th = [], 0.0, 0.0, 0.0

    def turn_to(target):  # in-place turn, duration = |angle| / TURN_RATE
        nonlocal th
        angle = wrap(target - th)
        if abs(angle) > 0.02:
            segs.extend([(0.0, TURN_SIGN * math.copysign(TURN_RATE, angle), abs(angle) / TURN_RATE),
                         (0.0, 0.0, PAUSE)])
            th = target

    for gx, gy, gth in waypoints:
        dist = math.hypot(gx - x, gy - y)
        if dist > 0.02:  # tiny distance (e.g. waypoint 0 at the origin): skip the turn and drive
            turn_to(math.atan2(gy - y, gx - x))
            segs.extend([(CRUISE_SPEED, 0.0, dist / CRUISE_SPEED), (0.0, 0.0, PAUSE)])
            x, y = gx, gy
        turn_to(gth)
    return segs


class WaypointNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')
        self.declare_parameter('waypoints_file',
                               get_package_share_directory('robot_control') + '/config/waypoints.txt')
        with open(self.get_parameter('waypoints_file').value) as f:
            wps = [tuple(map(float, ln.replace(',', ' ').split()))
                   for ln in f if ln.strip() and not ln.startswith('#')]
        self.segs, self.i, self.t0, self.done = plan(wps), 0, time.monotonic(), False
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(f'{len(wps)} waypoints -> {len(self.segs)} moves')

    def send(self, left, right):
        self.pub.publish(Float32MultiArray(data=[float(left), float(right)]))

    def tick(self):
        while self.i < len(self.segs) and time.monotonic() - self.t0 >= self.segs[self.i][2]:
            self.t0 += self.segs[self.i][2]  # carry the remainder so moves don't lose time
            self.i += 1
        if self.i >= len(self.segs):
            self.done = True
            return
        self.send(*wheel_cmds(*self.segs[self.i][:2]))


def main(args=None):
    # NO: rclpy's own SIGINT handler would shut the context down before our stop is sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:  # zeros when finished and on Ctrl-C; a second SIGINT must not skip the stop
        for _ in range(5):
            try:
                node.send(0.0, 0.0)
                time.sleep(0.02)
            except KeyboardInterrupt:
                continue
            except Exception:
                break
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
