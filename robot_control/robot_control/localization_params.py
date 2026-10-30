"""Read the tag map, camera mount and waypoints for localization_control (values come from localization_full.yaml)."""
import math
import os

from ament_index_python.packages import get_package_share_directory

from hw2_localization import ekf as E


def declare_map_params(node):
    """Declare the tag map and camera parameters (defaults are in ekf.py)."""
    ids = sorted(E.TAG_MAP)
    node.declare_parameter('tags.ids', ids)
    for i, key in enumerate(('x', 'y', 'z', 'yaw')):
        node.declare_parameter(f'tags.{key}', [float(E.TAG_MAP[t][i]) for t in ids])
    node.declare_parameter('camera.x', E.CAMERA[0])
    node.declare_parameter('camera.y', E.CAMERA[1])
    node.declare_parameter('camera.z', E.CAMERA[2])
    node.declare_parameter('camera.pitch_deg', math.degrees(E.CAMERA[3]))
    node.declare_parameter('camera_frame', 'camera_frame')  # frame name on the camera images
    node.declare_parameter('tag_frame_prefix', 'tag_')  # tag frames are named <prefix><id>


def read_tag_map(node):
    ids = [int(i) for i in node.get_parameter('tags.ids').value]
    cols = [list(node.get_parameter(f'tags.{k}').value) for k in ('x', 'y', 'z', 'yaw')]
    if any(len(c) != len(ids) for c in cols):
        raise ValueError('tags.ids, tags.x, tags.y, tags.z and tags.yaw must all have the same length')
    return {i: tuple(float(c[n]) for c in cols) for n, i in enumerate(ids)}


def read_camera(node):
    x, y, z, pitch = (node.get_parameter(f'camera.{k}').value for k in ('x', 'y', 'z', 'pitch_deg'))
    return (x, y, z, math.radians(pitch))


def load_waypoints(path):
    """Read 'x, y, theta' lines (m, m, rad). Empty path = the installed hw2_waypoints.txt."""
    path = path or os.path.join(get_package_share_directory('robot_control'), 'config', 'hw2_waypoints.txt')
    with open(path) as f:
        return [tuple(map(float, ln.replace(',', ' ').split())) for ln in f if ln.strip() and not ln.startswith('#')]
