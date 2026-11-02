#!/usr/bin/env python3
"""Exercise the real two-car simulation and report behavior, not just live nodes.

Run inside the built container after sourcing /sim_ws/install/setup.bash:
  python3 /sim_ws/src/f1tenth_gym_ros/test/check_maneuvers.py --duration 90
Uses a separate ROS_DOMAIN_ID and /tmp output; existing results are preserved.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from racetrack_utilities.racetrack_utilities import RacetrackUtilities


class Monitor(Node):
    def __init__(self):
        super().__init__('maneuver_check')
        self.track = RacetrackUtilities('/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv')
        self.poses = {}
        self.history = {'ego': [], 'opp': []}
        self.states = set()
        self.defense_paths = 0
        self.lateral_defenses = 0
        self.overtake_paths = 0
        self.max_defense_shift = 0.0
        self.current_state = 'RACELINE'
        self.divert_entry_d = None
        self.max_driven_defense_shift = 0.0
        self.gaps = []
        self.outside_track = {'ego': 0, 'opp': 0}
        self.collisions = {'ego': 0, 'opp': 0}
        self.collision_messages = {'ego': 0, 'opp': 0}
        for car, topic in [('ego', '/ego_racecar/odom'), ('opp', '/opp_racecar/odom')]:
            self.create_subscription(Odometry, topic, lambda m, c=car: self.odom(c, m), 1)
            self.create_subscription(Bool, topic.rsplit('/', 1)[0] + '/collision',
                                     lambda m, c=car: self.collision(c, m), 10)
        self.create_subscription(String, '/mppi_state', self.state, 10)
        self.create_subscription(MarkerArray, '/ego_best_defensive_spline', self.defense, 10)
        self.create_subscription(RosPath, '/overtake_spline', self.overtake, 10)

    def odom(self, car, msg):
        xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        sd = self.track.convert_to_frenet(*xy)
        self.poses[car] = sd
        self.history[car].append([*xy, *sd])
        self.outside_track[car] += int(not self.track.in_bounds_frenet(*sd))
        if car == 'ego' and self.current_state in ('DIVERT', 'RETURN') and self.divert_entry_d is not None:
            self.max_driven_defense_shift = max(
                self.max_driven_defense_shift, abs(float(sd[1]) - self.divert_entry_d))
        if car == 'opp' and len(self.poses) == 2:
            gap = self.track.unwrap_s(sd[0], self.poses['ego'][0]) - self.poses['ego'][0]
            self.gaps.append(float(gap))

    def state(self, msg):
        if msg.data == 'DIVERT' and self.current_state != 'DIVERT' and 'ego' in self.poses:
            self.divert_entry_d = float(self.poses['ego'][1])
        self.current_state = msg.data
        self.states.add(msg.data)

    def collision(self, car, msg):
        self.collision_messages[car] += 1
        self.collisions[car] += int(msg.data)

    def defense(self, msg):
        for marker in msg.markers:
            if len(marker.points) >= 3:
                sd = self.track.convert_to_frenet_arr([[p.x, p.y] for p in marker.points])
                shift = float(np.max(np.abs(sd[:, 1] - sd[0, 1])))
                self.defense_paths += 1
                self.lateral_defenses += int(shift > 0.15)
                self.max_defense_shift = max(self.max_defense_shift, shift)

    def overtake(self, msg):
        self.overtake_paths += int(len(msg.poses) > 5)

    def report(self):
        movement = {}
        for car, rows in self.history.items():
            xy = np.array(rows)[:, :2] if rows else np.empty((0, 2))
            movement[car] = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        return {
            'distance_m': movement,
            'outside_track_samples': self.outside_track,
            'simulator_collision_samples': self.collisions,
            'collision_messages_received': self.collision_messages,
            'mppi_states': sorted(self.states),
            'defense_paths': self.defense_paths,
            'lateral_defenses': self.lateral_defenses,
            'max_planned_defense_shift_m': self.max_defense_shift,
            'max_driven_defense_shift_m': self.max_driven_defense_shift,
            'overtake_paths': self.overtake_paths,
            'opponent_passed': bool(self.gaps and min(self.gaps) < -0.5 and max(self.gaps) > 0.5),
            'relative_s_range_m': [min(self.gaps), max(self.gaps)] if self.gaps else [],
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=90)
    parser.add_argument('--controller', choices=['overtake', 'overtake_aggressive'], default='overtake')
    parser.add_argument('--output', default='/tmp/maneuver-check')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('CHECK_ROS_DOMAIN_ID', '91')
    os.environ['RESULTS_DIR'] = args.output
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    processes, logs = [], []
    rclpy.init()
    node = Monitor()
    try:
        commands = [
            ['ros2', 'run', 'f1tenth_gym_ros', 'gym_bridge', '--ros-args',
             '-r', '__node:=bridge', '--params-file', '/sim_ws/src/f1tenth_gym_ros/config/sim.yaml'],
            ['ros2', 'launch', 'f1tenth_gym_ros', 'maneuver_launch.py',
             'scenario_manager:=false', 'controller:=' + args.controller],
        ]
        for name, command in zip(['bridge', 'pipeline'], commands):
            log = (output / (name + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(p.poll() is not None for p in processes):
                break
        report = node.report()
        report['controller'] = args.controller
        report['duration_s'] = args.duration
        report['processes_alive'] = all(p.poll() is None for p in processes)
        (output / 'report.json').write_text(json.dumps(report, indent=2))
        np.savez_compressed(str(output / 'poses.npz'), **{c: np.asarray(h) for c, h in node.history.items()})
        print(json.dumps(report, indent=2), flush=True)
        passed = (report['processes_alive'] and min(report['distance_m'].values()) > 5
                  and max(report['outside_track_samples'].values()) == 0
                  and max(report['simulator_collision_samples'].values()) == 0
                  and min(report['collision_messages_received'].values()) > 0
                  and report['lateral_defenses'] > 0 and report['max_driven_defense_shift_m'] > 0.1
                  and 'DIVERT' in report['mppi_states']
                  and report['overtake_paths'] > 0 and report['opponent_passed'])
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
