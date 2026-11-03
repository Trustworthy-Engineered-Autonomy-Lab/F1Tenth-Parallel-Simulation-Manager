#!/usr/bin/env bash
set -e
exec 2>&1
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
python3 -u /sim_ws/src/f1tenth_gym_ros/test/check_experiment_protocol_live.py "$@"
