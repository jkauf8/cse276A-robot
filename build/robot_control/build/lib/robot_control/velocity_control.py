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
# Geisel carpet
CMD_PER_MPS = 0.74  # motor command per m/s of wheel speed
TRACK_WIDTH = 0.37  # wheel spacing, which sets the turn angle
LEFT_SCALE = 0.9975  # multiplier for left wheel motor command
RIGHT_SCALE = 0.9970  # multiplier for right wheel motor command
CRUISE_SPEED = 0.55  # driving speed in m/s
TURN_RATE = 2.25  # spin speed in rad/s
TURN_SIGN = 1  # turn direction (flip to -1 if the robot turns the wrong way)
PAUSE = 1  # seconds stopped between moves
RATE = 20.0  # Hz publish rate; motor_control stops the wheels after 0.15 s of silence
CMD_LIMIT = 0.5  # motor_control clips each wheel to +/-0.5


# justin carpet at home
# CMD_PER_MPS = 0.74
# TRACK_WIDTH = 0.69
# LEFT_SCALE = 0.9975
# RIGHT_SCALE = 1.0
# CRUISE_SPEED = 0.55
# TURN_RATE = 1.57
# TURN_SIGN = 1
# PAUSE = 0.3
# RATE = 20.0
# CMD_LIMIT = 0.5


def wrap_angle(a):
    """Wrap an angle to [-pi, pi] so every turn takes the shortest way."""
    return math.atan2(math.sin(a), math.cos(a))


def wheel_cmds(v, w):
    """Kinematic model, differential drive (Lecture 4): v_l = v - wL/2, v_r = v + wL/2 [m/s],
    then motor command = wheel speed * CMD_PER_MPS, clipped to +/-0.5."""
    v_l = v - w * TRACK_WIDTH / 2.0
    v_r = v + w * TRACK_WIDTH / 2.0
    cmds = (v_l * CMD_PER_MPS * LEFT_SCALE, v_r * CMD_PER_MPS * RIGHT_SCALE)
    return tuple(max(-CMD_LIMIT, min(CMD_LIMIT, c)) for c in cmds)


def add_turn(segments, theta, target):
    """Add an in-place turn from heading theta to target (duration = |angle| / TURN_RATE).
    Returns the new heading."""
    angle = wrap_angle(target - theta)
    if abs(angle) <= 0.02:  # already facing that way, nothing to turn
        return theta
    segments.extend([
        (0.0, TURN_SIGN * math.copysign(TURN_RATE, angle), abs(angle) / TURN_RATE),
        (0.0, 0.0, PAUSE)])
    return target


def plan(waypoints):
    """Moves as (v, w, seconds). Believed pose starts at (0, 0, 0), assuming each move is exact."""
    segments, x, y, theta = [], 0.0, 0.0, 0.0

    # for each waypoint: turn to face it, drive there, then turn to its final heading.
    for gx, gy, gtheta in waypoints:
        dist = math.hypot(gx - x, gy - y)
        theta = add_turn(segments, theta, math.atan2(gy - y, gx - x))
        segments.extend([(CRUISE_SPEED, 0.0, dist / CRUISE_SPEED), (0.0, 0.0, PAUSE)])
        x, y = gx, gy
        theta = add_turn(segments, theta, gtheta)
    return segments


class WaypointNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')
        self.declare_parameter('waypoints_file',
                               get_package_share_directory('robot_control') + '/config/waypoints.txt')
        with open(self.get_parameter('waypoints_file').value) as f:
            wps = [tuple(map(float, ln.replace(',', ' ').split()))
                   for ln in f if ln.strip() and not ln.startswith('#')]
        self.segments, self.index, self.t0, self.done = plan(wps), 0, time.monotonic(), False
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info(f'{len(wps)} waypoints -> {len(self.segments)} moves')

    def send(self, left, right):
        self.pub.publish(Float32MultiArray(data=[float(left), float(right)]))

    def tick(self):
        # advance if current move over (checks every 1/RATE (50ms atm)), stop when moves run out, otherwise send current move 
        while (self.index < len(self.segments)
               and time.monotonic() - self.t0 >= self.segments[self.index][2]):
            self.t0 += self.segments[self.index][2]  # carry the remainder so moves don't lose time
            self.index += 1
        if self.index >= len(self.segments):
            self.done = True
            return
        self.send(*wheel_cmds(*self.segments[self.index][:2]))


def main(args=None):
    # Run the plan; always stop the wheels at the end, even on Ctrl-C (ROS's own handler is off).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:  # tell the wheels to stop, whether the plan finished or Ctrl-C was pressed
        node.send(0.0, 0.0)
        time.sleep(0.05)  # give the stop message time to go out before ROS shuts down
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
