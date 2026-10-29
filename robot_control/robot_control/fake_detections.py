"""TEST ONLY: stands in for the camera + apriltag_node. Simulates the robot (open-loop calibration error
from the simulator) driven by the commands on motor_commands, and publishes what apriltag_node would:
  - /tf   camera_frame -> tag_<id> (tag position in the camera frame; rotation is identity, NOT meaningful)
  - detections (AprilTagDetectionArray with id/family/centre; homography and corners are left zero)
Also publishes the true pose on fake_truth ([x, y, theta]) and writes it to a CSV for the error report.
"""
import csv
import os
import random
import time
from collections import deque

import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from hw2_localization import sim
from robot_control.localization_params import declare_map_params, read_camera, read_tag_map

FX, FY, CX, CY = 491.0, 492.0, 639.0, 357.0  # approximate intrinsics, only used for the fake detection's pixel centre


class FakeDetections(Node):
    def __init__(self):
        super().__init__('fake_detections')
        declare_map_params(self)
        for name, val in (('seed', 1), ('gain_l', 0.985), ('gain_r', 1.015), ('track_scale', 0.97), ('slip', 0.01),
                          ('deadband', 0.10), ('latency', 0.08), ('tf_stamp_offset', 0.0), ('detect_rate', 10.0), ('cmd_timeout', 0.15),
                          ('truth_log', os.path.expanduser('~/hw2_logs/fake_truth.csv'))):
            self.declare_parameter(name, val)
        P = lambda name: self.get_parameter(name).value
        self.tag_map, self.camera = read_tag_map(self), read_camera(self)
        self.camera_frame, self.prefix = P('camera_frame'), P('tag_frame_prefix')
        self.rng = random.Random(P('seed'))
        self.robot = sim.TrueRobot(self.rng, P('gain_l'), P('gain_r'), P('track_scale'), P('slip'), P('deadband'))
        self.latency, self.cmd_timeout = P('latency'), P('cmd_timeout')
        self.tf_offset = P('tf_stamp_offset')  # test knob: TF stamp differs from the detection stamp (forces the newest-TF fallback)
        self.cmd, self.cmd_time = (0.0, 0.0), 0.0
        self.t_prev = time.monotonic()
        self.history = deque()  # (ros time s, true pose), so detections can be made from the pose at their stamp
        self.n_ticks = 0

        self.create_subscription(Float32MultiArray, 'motor_commands', self.on_cmd, 10)
        self.pub_det = self.create_publisher(AprilTagDetectionArray, 'detections', 1)
        self.pub_truth = self.create_publisher(Float32MultiArray, 'fake_truth', 10)
        self.tf = TransformBroadcaster(self)
        os.makedirs(os.path.dirname(P('truth_log')), exist_ok=True)
        self.truth_file = open(P('truth_log'), 'w', newline='')
        self.truth = csv.writer(self.truth_file)
        self.truth.writerow(['ros_time', 'x', 'y', 'theta'])
        self.create_timer(1.0 / 20.0, self.step)
        self.create_timer(1.0 / P('detect_rate'), self.detect)
        self.get_logger().info(f'fake robot + detections for tags {sorted(self.tag_map)}, latency {self.latency} s')

    def on_cmd(self, msg):
        # motor_control clips each wheel to +/-0.5 and stops the wheels if it hears nothing for 0.15 s
        self.cmd = tuple(max(-0.5, min(0.5, float(c))) for c in msg.data[:2])
        self.cmd_time = time.monotonic()

    def step(self):
        now = time.monotonic()
        dt, self.t_prev = min(now - self.t_prev, 0.2), now
        left, right = self.cmd if now - self.cmd_time < self.cmd_timeout else (0.0, 0.0)
        self.robot.step(left, right, dt)
        ros_s = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((ros_s, list(self.robot.x)))
        while self.history and ros_s - self.history[0][0] > 2.0:
            self.history.popleft()
        self.truth.writerow([f'{ros_s:.3f}', *(f'{v:.5f}' for v in self.robot.x)])
        self.pub_truth.publish(Float32MultiArray(data=[float(v) for v in self.robot.x]))

    def pose_at(self, ros_s):
        for t, pose in reversed(self.history):
            if t <= ros_s:
                return pose
        return self.history[0][1] if self.history else self.robot.x

    def detect(self):
        stamp_s = self.get_clock().now().nanoseconds * 1e-9 - self.latency
        dets = sim.detect(self.pose_at(stamp_s), self.rng, self.tag_map, self.camera)
        stamp = Time(nanoseconds=int(stamp_s * 1e9)).to_msg()
        msg = AprilTagDetectionArray()
        msg.header.stamp, msg.header.frame_id = stamp, self.camera_frame
        tfs = []
        for d in dets:
            x, y, z = d.t_cam
            det = AprilTagDetection()
            det.family, det.id, det.hamming, det.decision_margin = 'tag36h11', d.tag_id, 0, 100.0
            det.centre.x, det.centre.y = CX + FX * x / z, CY + FY * y / z
            msg.detections.append(det)
            t = TransformStamped()
            t.header.stamp = Time(nanoseconds=int((stamp_s + self.tf_offset) * 1e9)).to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f'{self.prefix}{d.tag_id}'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.pub_det.publish(msg)  # apriltag_node publishes detections first, then TF
        if tfs:
            self.tf.sendTransform(tfs)

    def close(self):
        self.truth_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetections()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
