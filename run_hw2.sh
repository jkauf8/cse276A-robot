#!/usr/bin/env bash
# HW2 on the robot: starts the camera, tag detection and motor nodes, and the run itself, each in its own tmux window.
#   ./run_hw2.sh          start. Windows 1-3 start by themselves; window 4 has the run command typed but waits for Enter.
#   ./run_hw2.sh build    git pull + colcon build first, then start
#   ./run_hw2.sh stop     stop the robot (Ctrl-C in the run window) and close everything
# Move between windows with Ctrl-b then 0/1/2/3, leave tmux running with Ctrl-b d, come back with: tmux attach -t hw2
WS=${WS:-$HOME/ros2_ws}                       # workspace on the robot
REPO=$(dirname "$(readlink -f "$0")")         # this repo (the script lives in its root)
CONFIG=${CONFIG:-localization_full.yaml}      # params file in robot_control/config
TAG_SIZE=${TAG_SIZE:-0.16}                    # black square of the printed tags, m
SESSION=hw2
SRC="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"

if [ "$1" = "stop" ]; then
  tmux send-keys -t $SESSION:run C-c 2>/dev/null
  sleep 1
  tmux kill-session -t $SESSION 2>/dev/null && echo "stopped" || echo "no $SESSION session running"
  exit 0
fi

if [ "$1" = "build" ]; then
  (cd "$REPO" && git pull --ff-only) || exit 1
  (source /opt/ros/jazzy/setup.bash && cd "$WS" && colcon build --packages-select robot_control) || exit 1
fi

if tmux has-session -t $SESSION 2>/dev/null; then
  echo "session '$SESSION' is already running: attach with 'tmux attach -t $SESSION' or stop it with './run_hw2.sh stop'"
  exit 1
fi

tmux new-session -d -s $SESSION -n camera
tmux send-keys -t $SESSION:camera "$SRC && ros2 launch robot_vision_camera robot_vision_camera.launch.py" Enter
sleep 4

tmux new-window -t $SESSION -n tags
tmux send-keys -t $SESSION:tags "$SRC && ros2 launch robot_control tag_detection_launch.py tag_size:=$TAG_SIZE" Enter
sleep 3

tmux new-window -t $SESSION -n motors
tmux send-keys -t $SESSION:motors "$SRC && ros2 run robot_control motor_control" Enter
sleep 2

# the run: typed but NOT started, so the robot only moves when you press Enter (robot on the start marker, facing +x)
tmux new-window -t $SESSION -n run
tmux send-keys -t $SESSION:run "$SRC" Enter
tmux send-keys -t $SESSION:run 'ros2 launch robot_control localization_launch.py params_file:=$(ros2 pkg prefix robot_control)/share/robot_control/config/'"$CONFIG"

echo "camera, tags and motors started. In the 'run' window (Ctrl-b 3), press Enter when the robot is on the start marker."
tmux select-window -t $SESSION:run
tmux attach -t $SESSION
