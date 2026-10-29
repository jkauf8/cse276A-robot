# hw2_localization

AprilTag-based localization (EKF) and a closed-loop waypoint controller for HW2, plus a simulator and a ROS 2 wrapper.
The code in this folder is **stdlib-only and ROS-free**, so the simulator and the ROS node share it.
(Run the Python commands below from the `robot_control/` folder, the parent of this one.)

## Files

| File | Role |
|---|---|
| `ekf.py` | EKF, state `[x, y, theta]` in the world frame (origin = start pose, x = initial facing, y = left, theta CCW) |
| `controller.py` | Closed-loop stop-turn-go on the EKF estimate: `step(pose, t)` returns `[left, right]` |
| `sim.py` | Simulated robot with HW1-style open-loop error, fake tag detections, open-loop and closed-loop runs |
| `run_sim.py` | Runs the simulator, prints the reports, saves `out/sim.png` |
| `plot_log.py` | Turns a `localization_control` CSV log into the trajectory plot and the waypoint error table |
| `out/sim.png` | Closed-loop simulation plot |

ROS side (in `robot_control/`): `robot_control/localization_control.py` (the node), `robot_control/fake_detections.py` (test stand-in for the camera),
`robot_control/localization_params.py`, `config/localization_full.yaml`, `config/apriltag_hw2.yaml`, `config/hw2_waypoints.txt`,
`launch/tag_detection_launch.py`, `launch/localization_launch.py`. The repo-root `run_hw2.sh` starts everything on the robot.

## EKF (`ekf.py`)
- **Predict:** the same differential-drive model as `velocity_control.py`. `cmds_to_vw` inverts `wheel_cmds` to get `(v, w)` from the
  commands. Process noise is large on purpose (open-loop bias behaves like noise).
- **Update:** a detection is the tag position in the camera frame. It is converted to range and bearing **from the camera**, and compared
  with the known tag map. The camera is 0.085 m ahead of the robot center, and `h()` uses the camera's world position. Tag orientation is not used.
- Gating on the normalized innovation, Joseph-form covariance update, and `update(det, at=pose)` for late detections
  (expected measurement computed at the pose from when the image was taken, correction applied to the current state).
- Camera mount, tag map and noise values are constructor arguments. The defaults live at the top of the file.
- `check_constants()` verifies that the copied model constants still equal the ones in `velocity_control.py`.

## Controller (`controller.py`)
Same structure as `velocity_control.py` (face the waypoint, drive, turn to the final heading, stop between moves), but each move ends on the **estimate**:
- turn until the heading error is under 2 deg
- drive until the distance is under 3 cm, with proportional speed and steering. Stop and re-aim if the heading error exceeds 5 deg
  (not within 0.15 m of the goal, where the bearing is too noisy)
- proportional control clipped to `CRUISE_SPEED` / `TURN_RATE`, never below `max(min_cmd, deadband)`
- after the last waypoint's final turn: pause, re-check the heading against the (now tag-corrected) estimate, and turn again if it is
  still off, up to `final_checks` times (default 2, `0` = off)
- timeout on every move and 60 s per waypoint, so it cannot loop forever
- zero command at the end of every move, then the 1 s pause from `velocity_control.PAUSE`

## Simulator (`sim.py`)
- Uses the **real** `velocity_control.py` plan and `wheel_cmds` (loaded from the file with ROS stubbed out, so it can't drift).
- True robot: wheel gain error, track-width error, slip noise and an (assumed) wheel deadband.
- Fake detections at 10 Hz: 60 deg horizontal field of view, 3.5 m max range, tags seen more than 70 deg edge-on are dropped, 5% random dropout,
  noise on range and bearing.
- Default route `(0,0,0) -> (1,0,0) -> (1,2,pi) -> (0,0,0)`, tag map ID0 (2.5, 0) facing -x, ID1 (1, 3) facing -y, ID2 (-1.5, 2) facing +x, ID3 (-0.5, -1.5) facing 63 deg.
  (This is the simulator's own layout, not the course layout in `config/localization_full.yaml`.)

Run: `python3 -m hw2_localization.run_sim` (the plot needs matplotlib; `--runs N` sets the Monte Carlo size).

### Simulation results (seed 1, 3% wheel mismatch, 3% track error)
True error, robot vs goal (cm / deg):

| Waypoint | Open-loop | Closed-loop |
|---|---|---|
| 1 (1,0,0) | 4.4 / 4.9 | 2.2 / 0.5 |
| 2 (1,2,180) | 43.5 / 14.0 | 2.9 / 0.3 |
| 3 (0,0,0) | 47.6 / 24.3 | 1.7 / 0.3 |

Run time to the last waypoint: 19.3 s open-loop, 32.1 s closed-loop.

30 runs with random 1-5% errors: open-loop mean 33.6 cm / 17.8 deg (worst 164.7 cm), closed-loop mean 2.7 cm / 0.9 deg (worst 5.1 cm, 3.3 deg).

## ROS 2 node (`robot_control/localization_control.py`)
- 20 Hz tick: EKF predict from the last command, fuse pending detections, run the controller, publish `[L, R]` on `motor_commands`.
- Detections: subscribes to `detections`, then reads the tag pose from TF (`camera_frame -> tag_<id>`) at the detection's header stamp.
  If that TF is missing it falls back to the newest TF, but only if it is under 0.3 s old.
- Failsafe: no usable tag for `max_blind_time` means stop and log a warning. Otherwise it runs predict-only.
- Always sends a final `[0, 0]` on finish or Ctrl-C (same pattern as `velocity_control.py`).
- Writes a CSV log to `~/hw2_logs/` (time, pose, wheels, tags in view and their IDs, waypoint index, phase, uncertainty).
- Everything is a parameter (`config/localization_full.yaml`): tag map, camera mount, waypoints, `min_cmd`, `deadband`, tolerances, noise.

### What the camera/AprilTag code provides
- `apriltag_node` publishes **poses only as TF** (header = image header, child = `tag_<id>`). `/detections` has id, corners, centre and homography, no pose.
  It publishes `/detections` first and TF second.
- Images and `camera_info` use frame `camera_frame`, which uses the OpenCV axes (x right, y down, z forward).
- `image_proc` rectify: `/camera/image_raw` + `camera_info` -> `/camera/image_rect`, which apriltag reads.
- The camera node's own `image_rectify` option publishes a mismatched `camera_info`, so keep it off.
- The intrinsics in `robot_vision/config/camera_parameter.yaml` come from `robot_vision/calibration/` (`capture_frames.py` on the robot, `calibrate.py` on a laptop).

## Running it
On the robot, `./run_hw2.sh` (from the repo root) starts the camera, tag detection, motor control and the run, each in a tmux window; the run waits for Enter.
Without the script:
```
ros2 launch robot_vision_camera robot_vision_camera.launch.py
ros2 launch robot_control tag_detection_launch.py tag_size:=0.16   # rectify + apriltag_node (family 36h11, ids 0-5)
ros2 run robot_control motor_control
ros2 launch robot_control localization_launch.py                    # localization_control with config/localization_full.yaml
```

After a run, plot a log with
`python3 -m hw2_localization.plot_log LOG.csv --waypoints config/hw2_waypoints.txt [--truth fake_truth.csv]`.
On the real robot there is no ground truth, so the table compares the estimated pose with each goal; measure the real final position with a tape.

## Fake-detection tests (no camera)
`fake_detections` simulates the robot and publishes the same TF and detection messages. It runs against `motor_control` with the fake serial module.

| Test | Result |
|---|---|
| Full route | Finished in 30.2 s. True error vs goal 2.1 / 3.1 / 2.6 cm and 0.9 / 1.1 / 0.3 deg at waypoints 1 / 2 / 3. 258 detections used, 0 rejected. |
| Blind failsafe | Stopped 3 s after moving began, with the warning. |
| Ctrl-C mid-turn | Final zero command reached the serial output 52 ms after the last turn command. |
| Newest-TF fallback (TF stamp offset 20 ms) | Route finished, 3 fallbacks used. |
| Real `apriltag_node` on a synthetic tag image | A tag placed at (0, 0.005, 2.415) m was reported at (0.003, 0.009, 2.398) m. Our node used 59 detections. |
