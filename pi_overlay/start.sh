#!/bin/bash

SESSION="ros"
WINDOW="ros"
CONTAINER="ros-noetic"
WORKSPACE="/root/ros_ws/cleaning_robot/pi_ws"

DELAY_DOCKER=1
DELAY_BRINGUP=3
DELAY_ROS=2

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux attach-session -t "$SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"
tmux set-option -t "$SESSION" mouse on

PANE0=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux split-window -h -t "$PANE0"
PANE1=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux split-window -v -t "$PANE1"
PANE2=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux split-window -v -t "$PANE0"
PANE3=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux split-window -v -t "$PANE1"
PANE4=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux split-window -v -t "$PANE0"
PANE5=$(tmux display-message -p -t "$SESSION:$WINDOW" '#{pane_id}')
tmux select-layout -t "$SESSION:$WINDOW" tiled

for pane in "$PANE0" "$PANE1" "$PANE2" "$PANE3" "$PANE4" "$PANE5"; do
    tmux send-keys -t "$pane" "docker exec -it $CONTAINER bash" C-m
    sleep "$DELAY_DOCKER"
done

ROS_ENV="cd $WORKSPACE && source /opt/ros/noetic/setup.bash && source devel/setup.bash && source rosmaster_pi.sh"

# The Pi is the ROS master. Do not depend on the old developer machine's IP.
tmux send-keys -t "$PANE0" \
    "$ROS_ENV && (rosnode list >/dev/null 2>&1 && echo 'Using existing local ROS master' || roscore)" C-m
sleep "$DELAY_BRINGUP"

tmux send-keys -t "$PANE1" "$ROS_ENV && bringup" C-m
sleep "$DELAY_ROS"

tmux send-keys -t "$PANE2" \
    "$ROS_ENV && roslaunch localize_manager localize_manager.launch" C-m
sleep "$DELAY_ROS"

tmux send-keys -t "$PANE3" \
    "$ROS_ENV && roslaunch moving_server moving_server.launch" C-m

tmux send-keys -t "$PANE4" \
    "$ROS_ENV && cd /root/ros_ws/cleaning_robot/pi_ws/src/nox/scripts && python3 sensor.py" C-m

tmux send-keys -t "$PANE5" 'echo "No extra command configured for this pane"' C-m

tmux select-pane -t "$PANE0"
tmux attach-session -t "$SESSION"
