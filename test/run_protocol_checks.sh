#!/usr/bin/env bash
set -eo pipefail
exec 2>&1
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
root=/sim_ws/src/f1tenth_gym_ros
python3 -m compileall -q "$root/src" "$root/f1tenth_gym_ros" "$root/launch"
python3 "$root/test/test_combined_workflow.py"
python3 "$root/test/test_scenario_lifecycle.py"
python3 "$root/test/test_maneuver_regressions.py"
python3 "$root/test/test_experiment_protocol.py"
