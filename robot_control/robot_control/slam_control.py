"""ROS node for HW3 (SLAM). Every tick (20 Hz) it predicts the robot with the EKF-SLAM filter, fuses the tag
detections (new tags are added to the map), runs the controller on the estimated pose, and publishes the wheel
commands on motor_commands. Tag poses come from TF (camera_frame -> tag_<id>).
"""
import csv
import math
import os
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from hw2_localization import ekf as E
from hw2_localization.controller import Controller
from hw3_slam.slam import EKFSlam
from robot_control import velocity_control as vc
from robot_control.localization_params import load_waypoints, read_camera

HISTORY = 2.0  # seconds of past poses kept, for late detections
TF_MAX_AGE = 0.3  # s; ignore a fallback TF older than this
LOG_COLUMNS = ['ros_time', 't', 'x', 'y', 'theta', 'left', 'right', 'n_tags', 'tag_ids', 'wp', 'done_wp', 'phase',
               'sigma_xy', 'n_landmarks', 'accepted', 'rejected']
LANDMARK_COLUMNS = ['done_wp', 't', 'tag_id', 'x', 'y', 'cov_xx', 'cov_xy', 'cov_yy']
DETECTION_COLUMNS = ['t', 'tag_id', 'range', 'bearing', 'nis', 'status', 'x', 'y', 'theta', 'phase']


def open_csv(path, columns):
    """Open path for writing, write the header row, and return (file, csv writer)."""
    f = open(path, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(columns)
    return f, writer


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_control')
        for key, val in (('x', 0.01), ('y', 0.0), ('z', 0.125), ('pitch_deg', 0.0)):
            self.declare_parameter(f'camera.{key}', val)  # camera mount in the vehicle frame
        self.declare_parameter('camera_frame', 'camera_frame')  # frame name on the camera images
        self.declare_parameter('tag_frame_prefix', 'tag_')  # tag frames are named <prefix><id>
        self.declare_parameter('waypoints_file', '')  # '' = installed config/hw3_square.txt
        self.declare_parameter('rate', 20.0)  # Hz (motor_control stops the wheels after 0.15 s of silence)
        self.declare_parameter('start_delay', 1.0)  # s of zero commands before moving
        self.declare_parameter('start_landmarks', 2)  # tags that must be in the map before the robot moves
        self.declare_parameter('start_wait', 15.0)  # s to wait for them at most
        self.declare_parameter('max_blind_time', 30.0)  # stop after this many s with no tag
        self.declare_parameter('min_cmd', 0.30)  # smallest wheel command while moving
        self.declare_parameter('deadband', 0.10)  # wheel commands below this do nothing
        self.declare_parameter('min_turn_cmd', 0.38)  # smallest wheel command while spinning (90 deg turn calibrated at 0.38)
        self.declare_parameter('turn_gain', 1.0)  # real turn / commanded turn (1 = the turn calibration is exact)
        self.declare_parameter('turn_tol_deg', 5.0)  # a turn is done below this heading error
        self.declare_parameter('reaim_tol_deg', 12.0)  # must stay above turn_tol_deg
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('final_checks', 2)  # heading re-checks after the last turn
        self.declare_parameter('init_count', 3)  # detections that must agree before a tag joins the map
        self.declare_parameter('init_spread', 0.30)  # how far apart (m) those detections may be
        self.declare_parameter('log_dir', os.path.expanduser('~/hw3_logs'))

        def param(name):
            return self.get_parameter(name).value

        E.check_constants(vc)
        self.rate = param('rate')
        self.camera_frame, self.prefix = param('camera_frame'), param('tag_frame_prefix')
        self.max_blind, self.start_delay = param('max_blind_time'), param('start_delay')
        self.start_landmarks, self.start_wait = param('start_landmarks'), param('start_wait')
        self.turn_gain = param('turn_gain')
        self.slam = EKFSlam(camera=read_camera(self), init_count=param('init_count'), init_spread=param('init_spread'))
        waypoints = load_waypoints(param('waypoints_file') or os.path.join(
            get_package_share_directory('robot_control'), 'config', 'hw3_square.txt'))
        self.ctrl = Controller(waypoints, vc.wheel_cmds, vc.CRUISE_SPEED, vc.TURN_RATE, vc.PAUSE, vc.TURN_SIGN,
                               min_cmd=param('min_cmd'), deadband=param('deadband'),
                               turn_tol=math.radians(param('turn_tol_deg')), reaim_tol=math.radians(param('reaim_tol_deg')),
                               pos_tol=param('pos_tol'), final_checks=param('final_checks'),
                               min_turn_cmd=param('min_turn_cmd'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(AprilTagDetectionArray, 'detections', self.on_detections, 10)
        self.pending = []  # (image stamp, tag id) waiting for their TF
        self.history = deque()  # (time, x, y, theta) of recent poses
        self.cmd = (0.0, 0.0)
        self.finished = False
        self.stop_reason = ''
        self.logged_wp = -2  # last waypoint whose map was saved
        self.t_start = self.t_prev = self.last_seen = time.monotonic()

        # three logs per run: the trajectory, the map after each waypoint, and every detection
        os.makedirs(param('log_dir'), exist_ok=True)
        stem = os.path.join(param('log_dir'), time.strftime('slam_%Y%m%d_%H%M%S'))
        self.log_path = f'{stem}.csv'
        self.log_file, self.log = open_csv(self.log_path, LOG_COLUMNS)
        self.landmark_file, self.landmark_log = open_csv(f'{stem}_landmarks.csv', LANDMARK_COLUMNS)
        self.detection_file, self.detection_log = open_csv(f'{stem}_detections.csv', DETECTION_COLUMNS)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'{len(waypoints)} waypoints; logging to {self.log_path}')

    # ---- detections ----
    def on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        self.pending += [(stamp, d.id) for d in msg.detections]  # any id: the map is not known in advance

    def tag_in_camera(self, stamp, tag_id):
        """Tag position in the camera frame, from TF at the image time (or the newest TF if recent). None if missing."""
        frame = f'{self.prefix}{tag_id}'
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, frame, stamp)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.camera_frame, frame, Time())
            except TransformException:
                return None
            if abs((Time.from_msg(tf.header.stamp) - stamp).nanoseconds) * 1e-9 > TF_MAX_AGE:
                return None
        p = tf.transform.translation
        return (p.x, p.y, p.z)

    def pose_at(self, ros_s):
        """Pose estimate at time ros_s, interpolated from the history."""
        h = self.history
        if not h or ros_s <= h[0][0]:
            return self.slam.pose if not h else list(h[0][1:])
        if ros_s >= h[-1][0]:
            return self.slam.pose
        for (t0, *a), (t1, *b) in zip(h, list(h)[1:]):
            if t0 <= ros_s <= t1:
                f = (ros_s - t0) / (t1 - t0) if t1 > t0 else 0.0
                return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), E.wrap(a[2] + f * E.wrap(b[2] - a[2]))]
        return self.slam.pose

    def fuse_detections(self):
        """Fuse every pending detection whose TF is available. Returns the tag ids seen."""
        now = self.get_clock().now()
        seen, keep = [], []
        for stamp, tag_id in self.pending:
            t_cam = self.tag_in_camera(stamp, tag_id)
            if t_cam is None:
                if (now - stamp).nanoseconds * 1e-9 < 0.5:
                    keep.append((stamp, tag_id))  # TF may still arrive
                continue
            self.slam.update(E.Detection(tag_id, t_cam), at=self.pose_at(stamp.nanoseconds * 1e-9))
            r, b, nis, status = self.slam.last
            x, y, theta = self.slam.pose
            self.detection_log.writerow([f'{time.monotonic() - self.t_start:.3f}', tag_id, f'{r:.3f}', f'{b:.4f}',
                                         f'{nis:.2f}', status, f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}', self.ctrl.phase])
            seen.append(tag_id)
        self.pending = keep
        return seen

    # ---- control loop ----
    def send(self, left, right):
        self.pub.publish(Float32MultiArray(data=[float(left), float(right)]))

    def save_map(self, t):
        """Write the current landmark estimates, tagged with the last finished waypoint."""
        self.logged_wp = self.ctrl.done_wp
        for tag_id, (x, y, cov) in sorted(self.slam.landmarks().items()):
            self.landmark_log.writerow([self.ctrl.done_wp, f'{t:.3f}', tag_id, f'{x:.4f}', f'{y:.4f}',
                                        f'{cov[0][0]:.6f}', f'{cov[0][1]:.6f}', f'{cov[1][1]:.6f}'])
        self.landmark_file.flush()

    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        dt, self.t_prev = min(now - self.t_prev, 0.2), now
        v, w = E.cmds_to_vw(*self.cmd)  # last tick's command acted during dt
        self.slam.predict(v, w * self.turn_gain, dt)  # scale the commanded turn by turn_gain
        ros_s = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((ros_s, *self.slam.pose))
        while self.history and ros_s - self.history[0][0] > HISTORY:
            self.history.popleft()

        seen = self.fuse_detections()
        if seen:
            self.last_seen = now
        t = now - self.t_start
        # stay still until a few tags are mapped from the start pose (the origin), so the map has an anchor
        waiting = t < self.start_delay or (len(self.slam.index) < self.start_landmarks and t < self.start_delay + self.start_wait)
        if waiting:
            self.last_seen = now  # the no-tag timer starts when the robot starts moving
            cmd = (0.0, 0.0)
        elif now - self.last_seen > self.max_blind:
            self.get_logger().warning(f'no tag seen for {self.max_blind:.0f} s, stopping')
            self.stop_reason, self.finished, cmd = 'blind', True, (0.0, 0.0)
        else:
            cmd = self.ctrl.step(self.slam.pose, t - self.start_delay)
            if self.ctrl.done:
                self.stop_reason, self.finished = 'done', True
        self.cmd = cmd
        self.send(*cmd)
        if self.ctrl.done_wp != self.logged_wp:
            self.save_map(t)
        pose, cov = self.slam.pose, self.slam.P
        self.log.writerow([f'{ros_s:.3f}', f'{t:.3f}', *(f'{p:.4f}' for p in pose), f'{cmd[0]:.4f}', f'{cmd[1]:.4f}',
                           len(seen), '|'.join(map(str, seen)), self.ctrl.i, self.ctrl.done_wp, self.ctrl.phase,
                           f'{(cov[0][0] + cov[1][1]) ** 0.5:.4f}', len(self.slam.index),
                           self.slam.accepted, self.slam.rejected])
        self.log_file.flush()

    def close(self):
        self.send(0.0, 0.0)
        time.sleep(0.05)  # let the stop message go out
        self.save_map(time.monotonic() - self.t_start)
        self.log_file.close()
        self.landmark_file.close()
        self.detection_file.close()
        x, y, theta = self.slam.pose
        self.get_logger().info(
            f'finished ({self.stop_reason or "interrupted"}): {len(self.slam.index)} landmarks {sorted(self.slam.index)}, '
            f'{self.slam.accepted} detections used, {self.slam.rejected} gated out, {self.ctrl.timeouts} timeouts, '
            f'{self.ctrl.reaims} re-aims; pose ({x:.3f}, {y:.3f}, {theta:.3f}); log {self.log_path}')


def main(args=None):
    # run until the controller finishes; always stop the wheels at the end, even on Ctrl-C
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SlamNode()
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
