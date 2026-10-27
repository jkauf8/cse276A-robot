"""EKF localization + closed-loop waypoint controller (ROS 2 wrapper around hw2_localization).

Every tick (20 Hz): EKF predict from the last command, fuse tag detections, run the controller on the
estimate, publish [left, right] on motor_commands. A tag detection is the tag pose in the camera frame, read from
TF (apriltag_node publishes camera_frame -> tag_<id>) at the detection's timestamp.
"""
import csv
import math
import os
import time
from collections import deque

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from hw2_localization import ekf as E
from hw2_localization.controller import Controller
from robot_control import velocity_control as vc
from robot_control.localization_params import declare_map_params, load_waypoints, read_camera, read_tag_map

NOISE_KEYS = tuple(E.NOISE)
HISTORY = 2.0  # seconds of pose estimates kept, to place late detections
LOG_COLUMNS = ['ros_time', 't', 'x', 'y', 'theta', 'left', 'right', 'n_tags', 'tag_ids', 'wp', 'done_wp', 'phase',
               'sigma_xy', 'accepted', 'rejected']


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_control')
        declare_map_params(self)
        self.declare_parameter('waypoints_file', '')  # '' = installed config/hw2_waypoints.txt
        self.declare_parameter('rate', 20.0)  # Hz; motor_control stops the wheels after 0.15 s of silence
        self.declare_parameter('start_delay', 1.0)  # s of zero commands first, so motor_control is listening
        self.declare_parameter('max_blind_time', 10.0)  # s without any usable tag detection before stopping
        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('commands_topic', 'motor_commands')
        self.declare_parameter('tf_max_age', 0.3)  # s; newest-TF fallback is rejected if older than this
        self.declare_parameter('min_cmd', 0.12)  # TO MEASURE on carpet: smallest wheel command used while moving
        self.declare_parameter('deadband', 0.10)  # TO MEASURE on carpet: wheel commands below this do nothing
        self.declare_parameter('turn_tol_deg', 2.0)
        self.declare_parameter('reaim_tol_deg', 5.0)
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('final_checks', 2)  # after the last turn: pause for a tag fix, re-check the heading this many times
        self.declare_parameter('log_dir', os.path.expanduser('~/hw2_logs'))
        for key, val in E.NOISE.items():
            self.declare_parameter(f'ekf.{key}', float(val))

        P = lambda name: self.get_parameter(name).value
        E.check_constants(vc)
        self.rate = P('rate')
        self.camera_frame, self.prefix = P('camera_frame'), P('tag_frame_prefix')
        self.max_blind, self.start_delay, self.tf_max_age = P('max_blind_time'), P('start_delay'), P('tf_max_age')
        tag_map = read_tag_map(self)
        self.ekf = E.EKF(tag_map=tag_map, camera=read_camera(self), noise={k: P(f'ekf.{k}') for k in NOISE_KEYS})
        waypoints = load_waypoints(P('waypoints_file'))
        self.ctrl = Controller(waypoints, vc.wheel_cmds, vc.CRUISE_SPEED, vc.TURN_RATE, vc.PAUSE, vc.TURN_SIGN,
                               min_cmd=P('min_cmd'), deadband=P('deadband'),
                               turn_tol=math.radians(P('turn_tol_deg')), reaim_tol=math.radians(P('reaim_tol_deg')),
                               pos_tol=P('pos_tol'), final_checks=P('final_checks'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Float32MultiArray, P('commands_topic'), 10)
        self.create_subscription(AprilTagDetectionArray, P('detections_topic'), self.on_detections, 10)
        self.pending = []  # (image stamp, tag id) waiting for their TF
        self.history = deque()  # (ros time s, x, y, theta) of recent estimates
        self.cmd = (0.0, 0.0)
        self.finished = False
        self.stop_reason = ''
        self.fallbacks = 0
        self.dropped = 0
        self.t_start = self.t_prev = self.last_seen = time.monotonic()

        os.makedirs(P('log_dir'), exist_ok=True)
        self.log_path = os.path.join(P('log_dir'), time.strftime('localization_%Y%m%d_%H%M%S.csv'))
        self.log_file = open(self.log_path, 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(LOG_COLUMNS)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'{len(waypoints)} waypoints, {len(tag_map)} mapped tags {sorted(tag_map)}; '
                               f'logging to {self.log_path}')

    # ---- detections ----
    def on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        self.pending += [(stamp, d.id) for d in msg.detections if d.id in self.ekf.tag_map]

    def tag_in_camera(self, stamp, tag_id):
        """Tag position in the camera frame from TF at the image stamp, else the newest TF if it is recent enough.
        Returns None if TF is not there yet (the caller may retry)."""
        frame = f'{self.prefix}{tag_id}'
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, frame, stamp)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.camera_frame, frame, Time())
            except TransformException:
                return None
            if abs((Time.from_msg(tf.header.stamp) - stamp).nanoseconds) * 1e-9 > self.tf_max_age:
                return None
            self.fallbacks += 1
        p = tf.transform.translation
        return (p.x, p.y, p.z)

    def pose_at(self, ros_s):
        """Estimated pose at ros_s from the recent history (linear in x, y; shortest way for theta)."""
        h = self.history
        if not h or ros_s <= h[0][0]:
            return list(self.ekf.x) if not h else list(h[0][1:])
        if ros_s >= h[-1][0]:
            return list(self.ekf.x)
        for (t0, *a), (t1, *b) in zip(h, list(h)[1:]):
            if t0 <= ros_s <= t1:
                f = (ros_s - t0) / (t1 - t0) if t1 > t0 else 0.0
                return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), E.wrap(a[2] + f * E.wrap(b[2] - a[2]))]
        return list(self.ekf.x)

    def fuse_detections(self):
        """Apply every pending detection whose TF is available. Returns the tag IDs that were used."""
        now = self.get_clock().now()
        seen, keep = [], []
        for stamp, tag_id in self.pending:
            t_cam = self.tag_in_camera(stamp, tag_id)
            if t_cam is None:
                if (now - stamp).nanoseconds * 1e-9 < 0.5:
                    keep.append((stamp, tag_id))  # TF may still be on its way
                else:
                    self.dropped += 1
                continue
            self.ekf.update(E.Detection(tag_id, t_cam), at=self.pose_at(stamp.nanoseconds * 1e-9))
            seen.append(tag_id)
        self.pending = keep
        return seen

    # ---- control loop ----
    def send(self, left, right):
        self.pub.publish(Float32MultiArray(data=[float(left), float(right)]))

    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        dt, self.t_prev = min(now - self.t_prev, 0.2), now
        self.ekf.predict(*E.cmds_to_vw(*self.cmd), dt)  # the command sent last tick acted during dt
        ros_s = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((ros_s, *self.ekf.x))
        while self.history and ros_s - self.history[0][0] > HISTORY:
            self.history.popleft()

        seen = self.fuse_detections()
        if seen:
            self.last_seen = now
        t = now - self.t_start
        if t < self.start_delay:
            self.last_seen = now  # the blind clock starts when the robot starts moving
            cmd = (0.0, 0.0)
        elif now - self.last_seen > self.max_blind:
            self.get_logger().warning(f'no tag seen for {self.max_blind:.0f} s, stopping')
            self.stop_reason, self.finished, cmd = 'blind', True, (0.0, 0.0)
        else:
            cmd = self.ctrl.step(self.ekf.x, t - self.start_delay)
            if self.ctrl.done:
                self.stop_reason, self.finished = 'done', True
        self.cmd = cmd
        self.send(*cmd)
        P = self.ekf.P
        self.log.writerow([f'{ros_s:.3f}', f'{t:.3f}', *(f'{v:.4f}' for v in self.ekf.x), f'{cmd[0]:.4f}', f'{cmd[1]:.4f}',
                           len(seen), '|'.join(map(str, seen)), self.ctrl.i, self.ctrl.done_wp, self.ctrl.phase,
                           f'{(P[0][0] + P[1][1]) ** 0.5:.4f}', self.ekf.accepted, self.ekf.rejected])
        self.log_file.flush()

    def close(self):
        self.send(0.0, 0.0)
        time.sleep(0.05)  # give the stop message time to go out before ROS shuts down
        self.log_file.close()
        self.get_logger().info(
            f'finished ({self.stop_reason or "interrupted"}): {self.ekf.accepted} detections used, '
            f'{self.ekf.rejected} gated out, {self.fallbacks} newest-TF fallbacks, {self.dropped} dropped, '
            f'{self.ctrl.timeouts} timeouts, {self.ctrl.reaims} re-aims; pose '
            f'({self.ekf.x[0]:.3f}, {self.ekf.x[1]:.3f}, {self.ekf.x[2]:.3f}); log {self.log_path}')


def main(args=None):
    # Run until the controller finishes; always stop the wheels at the end, even on Ctrl-C (ROS's own handler is off).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
