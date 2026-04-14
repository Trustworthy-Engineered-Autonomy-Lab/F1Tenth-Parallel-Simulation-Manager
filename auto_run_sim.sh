#!/usr/bin/env bash
set -e

WS_DIR="/sim_ws"
SESSION_ID="${SESSION_ID:-1}"
MAX_LAPS="${MAX_LAPS:-3}"
RESULTS_DIR="${RESULTS_DIR:-/sim_ws/results/session_${SESSION_ID}}"
TMUX_SESSION="f1sim_${SESSION_ID}"
ATTACH_TMUX="${ATTACH_TMUX:-0}"

cd "$WS_DIR"

echo "Using workspace: $WS_DIR"
echo "SESSION_ID: $SESSION_ID"
echo "MAX_LAPS: $MAX_LAPS"
echo "RESULTS_DIR: $RESULTS_DIR"

mkdir -p "$RESULTS_DIR"
mkdir -p /sim_ws/src

# Flatten nested ROS packages into sibling packages in /sim_ws/src
if [ -d /sim_ws/src/f1tenth_gym_ros/src/sim_ftg ] && [ ! -d /sim_ws/src/sim_ftg ]; then
    cp -r /sim_ws/src/f1tenth_gym_ros/src/sim_ftg /sim_ws/src/sim_ftg
fi

if [ -d /sim_ws/src/f1tenth_gym_ros/src/f1tenth_env_manager ] && [ ! -d /sim_ws/src/f1tenth_env_manager ]; then
    cp -r /sim_ws/src/f1tenth_gym_ros/src/f1tenth_env_manager /sim_ws/src/f1tenth_env_manager
fi

tmux has-session -t "$TMUX_SESSION" 2>/dev/null && tmux kill-session -t "$TMUX_SESSION"
tmux new-session -d -s "$TMUX_SESSION"

# Pane 0: build + simulator
tmux send-keys -t "$TMUX_SESSION":0.0 "
cd $WS_DIR
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build
source install/local_setup.bash
echo 'Starting simulator for session $SESSION_ID...'
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
" C-m

# Pane 1: FTG
tmux split-window -h -t "$TMUX_SESSION":0
tmux send-keys -t "$TMUX_SESSION":0.1 "
cd $WS_DIR
sleep 8
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build
source install/local_setup.bash
echo 'Starting FTG controller for session $SESSION_ID...'
ros2 run sim_ftg both
" C-m

# Pane 2: manager
tmux split-window -v -t "$TMUX_SESSION":0.1
tmux send-keys -t "$TMUX_SESSION":0.2 "
cd $WS_DIR
sleep 10
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build
source install/local_setup.bash
echo 'Starting env manager for session $SESSION_ID...'
SESSION_ID=$SESSION_ID MAX_LAPS=$MAX_LAPS RESULTS_DIR=$RESULTS_DIR ros2 run env_manager main
" C-m

tmux select-layout -t "$TMUX_SESSION" tiled

if [ \"$ATTACH_TMUX\" = \"1\" ]; then
    tmux attach -t "$TMUX_SESSION"
else
    echo "Started tmux session $TMUX_SESSION in detached mode."
    echo "To inspect: tmux attach -t $TMUX_SESSION"
    tail -f /dev/null
fi