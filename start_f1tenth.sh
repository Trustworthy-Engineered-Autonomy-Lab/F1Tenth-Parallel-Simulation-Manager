#!/usr/bin/env bash
set -e

IMAGE_NAME="f1tenth_gym_ros"
REPO_ROOT="$(pwd)"
RESULTS_ROOT="$REPO_ROOT/results"
CONFIG_FILE="$REPO_ROOT/config/manager.yaml"

# default values if the YAML keys are missing
DEFAULT_SESSIONS=3
DEFAULT_LAPS=3
DEFAULT_SIMULATIONS=3
DEFAULT_OVERTAKE_MARGIN=3.0

# Read the YAML, falling back on the defaults
if [[ -f "$CONFIG_FILE" ]]; then
    NUM_SESSIONS=$(yq e '.num_sessions' "$CONFIG_FILE")
    MAX_LAPS=$(yq e '.max_laps' "$CONFIG_FILE")
    NUM_SIMULATIONS=$(yq e '.num_simulations' "$CONFIG_FILE")
    OVERTAKE_MARGIN=$(yq e '.overtake_margin' "$CONFIG_FILE")
    # Empty strings mean the key is not present
    [[ -z $NUM_SESSIONS ]] && NUM_SESSIONS=$DEFAULT_SESSIONS
    [[ -z $MAX_LAPS ]] && MAX_LAPS=$DEFAULT_LAPS
    [[ -z $NUM_SIMULATIONS ]] && NUM_SIMULATIONS=$DEFAULT_SIMULATIONS
    [[ -z $OVERTAKE_MARGIN ]] && OVERTAKE_MARGIN=$DEFAULT_OVERTAKE_MARGIN
else
    NUM_SESSIONS=$DEFAULT_SESSIONS
    MAX_LAPS=$DEFAULT_LAPS
    NUM_SIMULATIONS=$DEFAULT_SIMULATIONS
    OVERTAKE_MARGIN=$DEFAULT_OVERTAKE_MARGIN
fi

mkdir -p "$RESULTS_ROOT"

docker build -t "$IMAGE_NAME" .

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
    -e CONFIG_FILE="/sim_ws/src/f1tenth_gym_ros/config/manager.yaml" \
    -e RESULTS_DIR="/sim_ws/results/session_$i" \
    -e ATTACH_TMUX=0 \
    -v "$REPO_ROOT:/sim_ws/src/f1tenth_gym_ros" \
    -v "$SESSION_RESULTS:/sim_ws/results/session_$i" \
    "$IMAGE_NAME" \
    /sim_ws/src/f1tenth_gym_ros/auto_run_sim.sh
done

echo "All sessions launched."
echo "Open dashboard.html in this folder."
