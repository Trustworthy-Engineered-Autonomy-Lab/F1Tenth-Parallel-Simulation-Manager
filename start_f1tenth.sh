#!/usr/bin/env bash
set -e

IMAGE_NAME="f1tenth_gym_ros"
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_ROOT="$REPO_ROOT/results"

mkdir -p "$RESULTS_ROOT"

docker build -t "$IMAGE_NAME" "$REPO_ROOT"

CONFIG_VALUES="$(
  docker run --rm --entrypoint python3 -v "$REPO_ROOT:/workspace:ro" "$IMAGE_NAME" \
    /workspace/src/lidar_processing/lidar_processing/manager_config.py \
    /workspace/config/manager.yaml --shell
)"
read -r CONFIG_SESSIONS CONFIG_LAPS CONFIG_SIMULATIONS CONFIG_MARGIN CONFIG_HOLD <<< "$CONFIG_VALUES"
NUM_SESSIONS="${NUM_SESSIONS:-$CONFIG_SESSIONS}"
MAX_LAPS="${MAX_LAPS:-$CONFIG_LAPS}"
NUM_SIMULATIONS="${NUM_SIMULATIONS:-$CONFIG_SIMULATIONS}"
OVERTAKE_MARGIN="${OVERTAKE_MARGIN:-$CONFIG_MARGIN}"
SCENARIO_RESET_HOLD_SECONDS="${SCENARIO_RESET_HOLD_SECONDS:-$CONFIG_HOLD}"

docker ps -aq --filter "name=f1tenth_session_" | xargs -r docker rm -f
docker ps -aq --filter "name=novnc_session_" | xargs -r docker rm -f

for i in $(seq 1 "$NUM_SESSIONS"); do
  CONTAINER_NAME="f1tenth_session_$i"
  NOVNC_NAME="novnc_session_$i"
  VNC_PORT=$((8080 + i))
  SESSION_RESULTS="$RESULTS_ROOT/session_$i"

  mkdir -p "$SESSION_RESULTS"

  # Copy overtake settings to results
  cp "$REPO_ROOT/config/opp_overtake_settings.yaml" "$SESSION_RESULTS/opp_overtake_settings.yaml"
  echo "Copied overtake settings to $SESSION_RESULTS"

  docker rm -f "$NOVNC_NAME" "$CONTAINER_NAME" 2>/dev/null || true

  docker run -d \
    --name "$NOVNC_NAME" \
    -p "${VNC_PORT}:8080" \
    theasp/novnc:latest

  docker run -d \
    --name "$CONTAINER_NAME" \
    --link "${NOVNC_NAME}:novnc" \
    -e DISPLAY=novnc:0.0 \
    -e SESSION_ID="$i" \
    -e ROS_DOMAIN_ID="$i" \
    -e MAX_LAPS="$MAX_LAPS" \
    -e NUM_SIMULATIONS="$NUM_SIMULATIONS" \
    -e OVERTAKE_MARGIN="$OVERTAKE_MARGIN" \
    -e CONFIG_FILE=/sim_ws/src/f1tenth_gym_ros/config/manager.yaml \
    -e SCENARIO_RESET_HOLD_SECONDS="$SCENARIO_RESET_HOLD_SECONDS" \
    -e RESULTS_DIR="/sim_ws/results/session_$i" \
    -e ATTACH_TMUX=0 \
    -v "$REPO_ROOT:/sim_ws/src/f1tenth_gym_ros" \
    -v "$SESSION_RESULTS:/sim_ws/results/session_$i" \
    "$IMAGE_NAME" \
    /sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh
done

echo "All sessions launched."
echo "Open dashboard.html in this folder."
