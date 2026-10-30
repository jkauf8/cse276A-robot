#!/usr/bin/env python3
"""Camera calibration from checkerboard JPEGs (needs opencv-python and numpy). Writes camera_parameter.yaml.

    python3 calibrate.py frames/ --cols 8 --rows 6 --square 0.025 --out camera_parameter.yaml
cols and rows are inner corners (a 9x7 board has 8x6), square is the square size in meters.
"""
import argparse
import glob
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('images')
ap.add_argument('--cols', type=int, required=True)
ap.add_argument('--rows', type=int, required=True)
ap.add_argument('--square', type=float, required=True)
ap.add_argument('--out', default='camera_parameter.yaml')
ap.add_argument('--width', type=int, default=1280)
ap.add_argument('--height', type=int, default=720)
a = ap.parse_args()

pattern = (a.cols, a.rows)
objp = np.zeros((a.cols * a.rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)

obj, img, used = [], [], []
for path in sorted(glob.glob(os.path.join(a.images, '*.jpg'))):
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.shape != (a.height, a.width):
        print(f'skip {path}: size {None if gray is None else gray.shape[::-1]}, expected {(a.width, a.height)}')
        continue
    ok, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        print(f'no board found in {path}')
        continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    obj.append(objp)
    img.append(corners)
    used.append(path)
print(f'{len(used)} usable images')
if len(used) < 10:
    raise SystemExit('need at least 10 usable images')

size = (a.width, a.height)
rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None)
K[0, 1] = 0.0
print(f'RMS reprojection error: {rms:.3f} px')
print('K =\n', K, '\nD =', D.ravel())
for p, o, i, r, t in zip(used, obj, img, rvecs, tvecs):
    proj, _ = cv2.projectPoints(o, r, t, K, D)
    e = np.sqrt(np.mean(np.sum((i.reshape(-1, 2) - proj.reshape(-1, 2)) ** 2, axis=1)))
    if e > 1.5 * rms:
        print(f'  high error {e:.2f} px: {p}')

P = np.hstack([K, np.zeros((3, 1))])
fs = cv2.FileStorage(a.out, cv2.FILE_STORAGE_WRITE)
fs.write('image_width', a.width)
fs.write('image_height', a.height)
fs.write('camera_name', 'camera')
fs.write('camera_matrix', K)
fs.write('distortion_model', 'plumb_bob')
fs.write('distortion_coefficients', D.reshape(1, -1)[:, :5])
fs.write('rectification_matrix', np.eye(3))
fs.write('projection_matrix', P)
fs.release()
print('wrote', a.out)
