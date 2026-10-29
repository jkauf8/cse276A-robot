"""Parameter helpers shared by localization_control and fake_detections: the tag map and camera mount
come from ROS parameters (config/localization_full.yaml), nothing is hard-coded in the nodes."""
import math
import os

from ament_index_python.packages import get_package_share_directory

from hw2_localization import ekf as E


def declare_map_params(node):
    """Declare the tag map and camera mount parameters (defaults = TAG_MAP and CAMERA in ekf.py)."""
    ids = sorted(E.TAG_MAP)
    node.declare_parameter('tags.ids', ids)
    for i, key in enumerate(('x', 'y', 'z', 'yaw')):
        node.declare_parameter(f'tags.{key}', [float(E.TAG_MAP[t][i]) for t in ids])
    node.declare_parameter('camera.x', E.CAMERA[0])
    node.declare_parameter('camera.y', E.CAMERA[1])
    node.declare_parameter('camera.z', E.CAMERA[2])
    node.declare_parameter('camera.pitch_deg', math.degrees(E.CAMERA[3]))
    node.declare_parameter('camera_frame', 'camera_frame')  # frame_id the camera node puts on its images
    node.declare_parameter('tag_frame_prefix', 'tag_')  # apriltag_node names the tag frame <prefix><id>


def read_tag_map(node):
    p = lambda name: node.get_parameter(name).value
    ids = [int(i) for i in p('tags.ids')]
    cols = [list(p(f'tags.{k}')) for k in ('x', 'y', 'z', 'yaw')]
    if any(len(c) != len(ids) for c in cols):
        raise ValueError('tags.ids, tags.x, tags.y, tags.z and tags.yaw must all have the same length')
    return {i: tuple(float(c[n]) for c in cols) for n, i in enumerate(ids)}


def read_camera(node):
    p = lambda name: node.get_parameter(name).value
    return (p('camera.x'), p('camera.y'), p('camera.z'), math.radians(p('camera.pitch_deg')))


def load_waypoints(path):
    """One 'x, y, theta' per line (m, m, rad), same format as waypoints.txt. Empty path = installed hw2 file."""
    path = path or os.path.join(get_package_share_directory('robot_control'), 'config', 'hw2_waypoints.txt')
    with open(path) as f:
        return [tuple(map(float, ln.replace(',', ' ').split())) for ln in f if ln.strip() and not ln.startswith('#')]
