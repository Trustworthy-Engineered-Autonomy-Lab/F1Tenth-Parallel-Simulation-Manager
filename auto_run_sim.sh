#!/usr/bin/env bash
set -e

# --- 1. Configuration & Environment Setup ---
WS_DIR="/sim_ws"
SESSION_ID="${SESSION_ID:-1}"
MAX_LAPS="${MAX_LAPS:-3}"
RESULTS_DIR="${RESULTS_DIR:-/sim_ws/results/session_${SESSION_ID}}"
TMUX_SESSION="f1sim_${SESSION_ID}"
ATTACH_TMUX="${ATTACH_TMUX:-0}"

echo "=========================================="
echo " Starting F1TENTH Session: $SESSION_ID"
echo " Workspace: $WS_DIR"
echo " Domain ID: $ROS_DOMAIN_ID"
echo "=========================================="

cd "$WS_DIR"
mkdir -p "$RESULTS_DIR"
mkdir -p "$WS_DIR/src"

# --- 2. Workspace Flattening ---
# Moves nested packages to /src so colcon sees them as siblings
if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/sim_ftg" ] && [ ! -d "$WS_DIR/src/sim_ftg" ]; then
    echo "Flattening sim_ftg..."
    cp -r "$WS_DIR/src/f1tenth_gym_ros/src/sim_ftg" "$WS_DIR/src/sim_ftg"
fi

if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/f1tenth_env_manager" ] && [ ! -d "$WS_DIR/src/f1tenth_env_manager" ]; then
    echo "Flattening env_manager..."
    cp -r "$WS_DIR/src/f1tenth_gym_ros/src/f1tenth_env_manager" "$WS_DIR/src/f1tenth_env_manager"
fi

if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/frenet_frame_conv" ] && [ ! -d "$WS_DIR/src/frenet_frame_conv" ]; then
    echo "Flattening frenet_frame_conv..."
    cp -r "$WS_DIR/src/f1tenth_gym_ros/src/frenet_frame_conv" "$WS_DIR/src/frenet_frame_conv"
fi

if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/data_logger" ] && [ ! -d "$WS_DIR/src/data_logger" ]; then
    echo "Flattening data_logger..."
    cp -r "$WS_DIR/src/f1tenth_gym_ros/src/data_logger" "$WS_DIR/src/data_logger"
fi

# DOESNTR WORK FOR NWO
# if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/lidar_processing" ] && [ ! -d "$WS_DIR/src/lidar_processing" ]; then
#     echo "Flattening lidar_processing..."
#     cp -r "$WS_DIR/src/f1tenth_gym_ros/src/lidar_processing" "$WS_DIR/src/lidar_processing"
# fi

if [ -d "$WS_DIR/src/f1tenth_gym_ros/src/overtake_controller" ] && [ ! -d "$WS_DIR/src/overtake_controller" ]; then
    echo "Flattening overtake_controller..."
    cp -r "$WS_DIR/src/f1tenth_gym_ros/src/overtake_controller" "$WS_DIR/src/overtake_controller"
fi


# --- 3. The "Single Build" Fix ---
# Build once here so the panes don't fight over the install/ folder
echo "Performing global colcon build..."
source /opt/ros/foxy/setup.bash
colcon build --symlink-install

# Final source to ensure the environment knows about the new build
source install/local_setup.bash

# --- 4. tmux Orchestration ---
echo "Initializing tmux session: $TMUX_SESSION"
tmux has-session -t "$TMUX_SESSION" 2>/dev/null && tmux kill-session -t "$TMUX_SESSION"
tmux new-session -d -s "$TMUX_SESSION"

# Pane 0: Simulator Bridge
tmux send-keys -t "$TMUX_SESSION":0.0 "
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
echo 'Launching Simulator Bridge...'
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
" C-m

# Create pane 1 (split horizontally)
tmux split-window -h -t "$TMUX_SESSION":0

# Pane 1: All other nodes in sequence
tmux send-keys -t "$TMUX_SESSION":0.1 "
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
sleep 5
echo 'Launching Ego FTG (slower, from overtake package)...'
ros2 run overtake_controller ego_ftg &
sleep 2
echo 'Launching IMM Filter (for opponent)...'
ros2 run overtake_controller imm_filter &
sleep 2
echo 'Launching Opponent Overtaking Controller...'
ros2 run overtake_controller overtake &
sleep 2
echo 'Launching Frenet Node...'
ros2 run frenet_frame_conv frenet_node &
sleep 3
echo 'Launching Env Manager...'
SESSION_ID=$SESSION_ID MAX_LAPS=$MAX_LAPS RESULTS_DIR=$RESULTS_DIR ros2 run env_manager main &
sleep 3
echo 'Launching Data Logger...'
SESSION_ID=$SESSION_ID RESULTS_DIR=$RESULTS_DIR ros2 run data_logger data_logger
" C-m

tmux select-layout -t "$TMUX_SESSION" even-horizontal

# --- 5. Attach or Keep Alive ---
if [ "$ATTACH_TMUX" = "1" ]; then
    tmux attach -t "$TMUX_SESSION"
else
    echo "------------------------------------------"
    echo "Tmux session $TMUX_SESSION started in background."
    echo "Run this to inspect: tmux attach -t $TMUX_SESSION"
    echo "------------------------------------------"
    # Keep container alive
    tail -f /dev/null
fi