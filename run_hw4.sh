#!/usr/bin/env bash
# Starts the HW4 (planned path) nodes on the robot, each in its own tmux window (camera, tags, params, motors, run).
# Put the robot on the start mark, wheel midpoint on the mark, facing the goal (heading 135 deg).
#   ./run_hw4.sh          start the fast plan; the run window waits for Enter
#   PLAN=safe ./run_hw4.sh   same with the safe plan
#   ./run_hw4.sh build    git pull and colcon build, then start
#   ./run_hw4.sh stop     stop the robot and close everything
#   ./run_hw4.sh look     count tag detections for 8 s (use it with the robot on the start mark)
# A session that is already running is stopped and replaced.
# tmux: Ctrl-b then 0-4 switches windows, Ctrl-b d leaves, "tmux attach -t hw4" comes back
WS=${WS:-$HOME/ros2_ws}                       # ROS workspace
REPO=$(dirname "$(readlink -f "$0")")         # this repo
CONFIG=${CONFIG:-hw4_localization.yaml}  # settings file in robot_control/config
TAG_SIZE=${TAG_SIZE:-0.16}                    # tag size (black square), m
PLAN=${PLAN:-fast}                          # fast or safe (waypoint file hw4_<plan>.txt)
DECIMATE=${DECIMATE:-1.0}                   # set live because an old installed apriltag config may still say 2.0
SESSION=hw4
SRC="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"

if [ "$1" = "stop" ]; then
  tmux send-keys -t $SESSION:run C-c 2>/dev/null
  sleep 1
  tmux kill-session -t $SESSION 2>/dev/null && echo "stopped" || echo "no $SESSION session running"
  exit 0
fi

if [ "$1" = "look" ]; then
  eval "$SRC"
  timeout -s INT 8 ros2 topic echo /detections > /tmp/d.txt
  grep -c ' id:' /tmp/d.txt
  grep ' id:' /tmp/d.txt | sort | uniq -c
  exit 0
fi

if [ "$1" = "build" ]; then
  (cd "$REPO" && git pull --ff-only) || exit 1
  (source /opt/ros/jazzy/setup.bash && cd "$WS" && colcon build --packages-select robot_control) || exit 1
fi

if tmux has-session -t $SESSION 2>/dev/null; then
  tmux send-keys -t $SESSION:run C-c
  sleep 1
  tmux kill-session -t $SESSION
  echo "replaced the old $SESSION session"
fi

tmux new-session -d -s $SESSION -n camera
tmux send-keys -t $SESSION:camera "$SRC && ros2 launch robot_vision_camera robot_vision_camera.launch.py" Enter
sleep 4

tmux new-window -t $SESSION -n tags
tmux send-keys -t $SESSION:tags "$SRC && ros2 launch robot_control tag_detection_launch.py tag_size:=$TAG_SIZE apriltag_params:=\$(ros2 pkg prefix robot_control)/share/robot_control/config/apriltag_hw4.yaml" Enter
sleep 3

tmux new-window -t $SESSION -n params
tmux send-keys -t $SESSION:params "$SRC && for i in \$(seq 30); do ros2 param set /apriltag detector.decimate $DECIMATE && break; sleep 1; done; ros2 param get /apriltag detector.decimate" Enter

tmux new-window -t $SESSION -n motors
tmux send-keys -t $SESSION:motors "$SRC && ros2 run robot_control motor_control" Enter
sleep 2

# the run command is typed but not started: press Enter when the robot is on the start marker
tmux new-window -t $SESSION -n run
tmux send-keys -t $SESSION:run "$SRC" Enter
tmux send-keys -t $SESSION:run 'ros2 launch robot_control hw4_launch.py params_file:=$(ros2 pkg prefix robot_control)/share/robot_control/config/'"$CONFIG"' waypoints_file:=$(ros2 pkg prefix robot_control)/share/robot_control/config/hw4_'"$PLAN"'.txt log_dir:=$HOME/hw4_logs/'"$PLAN"

echo "camera, tags and motors started. In the run window (Ctrl-b w lists windows), press Enter to start."
tmux select-window -t $SESSION:run
tmux attach -t $SESSION
