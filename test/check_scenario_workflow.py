#!/usr/bin/env python3
"""Run the configured scenario sequence naturally and audit resets and CSV output.

Run in the sourced simulator image. Results are isolated from ordinary sessions:
  python3 /sim_ws/src/f1tenth_gym_ros/test/check_scenario_workflow.py --output /verification
Natural mode never injects lap/overtake messages. --controlled-transitions instead
tests all reset/logging transitions quickly using a manager with isolated inputs;
the cars still drive normally, but the completion events are synthetic.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import MarkerArray
from lidar_processing.experiment_protocol import protocol_qos

ROOT = Path('/sim_ws/src/f1tenth_gym_ros')


class WorkflowProbe(Node):
    def __init__(self, config, output, controlled=False):
        super().__init__('scenario_workflow_probe')
        self.config, self.output = config, output
        self.scenario = None
        self.scenarios = {}
        self.finished = False
        self.final_speeds = {}
        self.previous = {}
        self.started = time.monotonic()
        self.controlled = controlled
        self.control_ready = False
        self.experiment_phase = None
        self.active_since = None
        self.last_seed = 0
        self.trigger_steps = {}
        self.ego_test_pub = self.create_publisher(Pose2D, '/workflow_test/ego_frenet', 10)
        self.opp_test_pub = self.create_publisher(Pose2D, '/workflow_test/opp_frenet', 10)
        self.odom_test_pub = self.create_publisher(Odometry, '/workflow_test/ego_odom', 10)
        self.create_subscription(PoseArray, '/env_manager/reset_scenario', self.reset, 10)
        self.create_subscription(Bool, '/env_manager/shutdown', self.shutdown, 10)
        self.create_subscription(String, '/env_manager/experiment_state', self.experiment_state, protocol_qos())
        self.create_subscription(String, '/mppi_state', self.state, 10)
        self.create_subscription(MarkerArray, '/ego_best_defensive_spline', self.defense, 10)
        for car in ('ego', 'opp'):
            prefix = '/' + car + '_racecar'
            self.create_subscription(Odometry, prefix + '/odom', lambda m, c=car: self.odom(c, m), 1)
            self.create_subscription(Bool, prefix + '/collision', lambda m, c=car: self.collision(c, m), 10)

    def reset(self, msg):
        if len(msg.poses) != 2:
            return
        pairs = [(p.position.x, p.position.y) for p in msg.poses]
        found = None
        for index in range(1, self.config['num_simulations'] + 1):
            expected = self.config['sim_start'][f's_{index}']
            if all(math.dist(pairs[i], expected[c][:2]) < 1e-5 for i, c in enumerate(('ego', 'opp'))):
                found = index
                break
        if found is None:
            raise AssertionError(f'Unexpected starting pair: {pairs}')
        self.scenario = found
        self.experiment_phase = None
        self.active_since = None
        self.previous.clear()
        self.scenarios[found] = {
            'started_at': time.monotonic(), 'elapsed_s': 0,
            'spawn_error_m': {'ego': None, 'opp': None},
            'distance_m': {'ego': 0.0, 'opp': 0.0},
            'collision_samples': {'ego': 0, 'opp': 0},
            'collision_messages': {'ego': 0, 'opp': 0},
            'states': [], 'defensive_paths': 0,
        }
        self.save_progress()
        print(f'Started scenario {found}/{self.config["num_simulations"]}', flush=True)

    def shutdown(self, msg):
        if msg.data:
            self.finished = True

    def experiment_state(self, message):
        state = json.loads(message.data)
        if state['scenario'] == self.scenario:
            self.experiment_phase = state['phase']
            if state['phase'] == 'RUNNING' and self.active_since is None:
                self.active_since = time.monotonic()

    def state(self, msg):
        self.control_ready = True
        if self.scenario:
            states = self.scenarios[self.scenario]['states']
            if msg.data not in states:
                states.append(msg.data)

    def defense(self, msg):
        if self.scenario and any(len(m.points) >= 3 for m in msg.markers):
            self.scenarios[self.scenario]['defensive_paths'] += 1

    def collision(self, car, msg):
        if self.scenario:
            row = self.scenarios[self.scenario]
            row['collision_samples'][car] += int(msg.data)
            row['collision_messages'][car] += 1

    def odom(self, car, msg):
        self.final_speeds[car] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if not self.scenario:
            return
        row = self.scenarios[self.scenario]
        now = time.monotonic()
        age = now - row['started_at']
        row['elapsed_s'] = age
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if 0.2 <= age <= 1.5:
            target = self.config['sim_start'][f's_{self.scenario}'][car][:2]
            error = math.dist(xy, target)
            old = row['spawn_error_m'][car]
            row['spawn_error_m'][car] = error if old is None else min(old, error)
        if age > 2.1:
            previous = self.previous.get(car)
            if previous is not None:
                step = math.dist(previous, xy)
                if step < 0.5:
                    row['distance_m'][car] += step
            self.previous[car] = xy

    def save_progress(self):
        rows = {str(k): {a: b for a, b in v.items() if a != 'started_at'} for k, v in self.scenarios.items()}
        report = {'finished': self.finished, 'current_scenario': self.scenario,
                  'verification_mode': 'controlled_transitions' if self.controlled else 'natural',
                  'elapsed_s': time.monotonic() - self.started,
                  'final_speeds': self.final_speeds, 'scenarios': rows}
        (self.output / 'progress.json').write_text(json.dumps(report, indent=2))
        return report

    def drive_manager_check(self):
        if not self.controlled or self.finished:
            return
        if not self.scenario:
            if time.monotonic() - self.last_seed > 0.25:
                self.last_seed = time.monotonic()
                self.odom_test_pub.publish(Odometry())
                self.ego_test_pub.publish(Pose2D(x=100.0))
                self.opp_test_pub.publish(Pose2D(x=99.0))
            return
        if self.experiment_phase != 'RUNNING':
            if time.monotonic() - self.last_seed > 0.25:
                self.last_seed = time.monotonic()
                self.ego_test_pub.publish(Pose2D(x=100.0))
                self.opp_test_pub.publish(Pose2D(x=99.0))
            return
        age = time.monotonic() - self.active_since
        step = min(2, int((age - 0.5) / 0.25))
        if age < 0.5 or self.trigger_steps.get(self.scenario) == step:
            return
        self.trigger_steps[self.scenario] = step
        if self.scenario % 2:
            self.ego_test_pub.publish(Pose2D(x=100.0))
            offset = (-1.0, self.config['overtake_margin'] - 0.1,
                      self.config['overtake_margin'] + 0.1)[step]
            self.opp_test_pub.publish(Pose2D(x=100.0 + offset))
        else:
            message = Odometry()
            scenario = self.config['sim_start'][f's_{self.scenario}']
            center = scenario.get('lap_center', scenario['ego'][:2])
            message.pose.pose.position.x = float(center[0])
            message.pose.pose.position.y = float(center[1])
            if step == 1:
                message.pose.pose.position.x += self.config['lap_detection']['bubble_radius'] + 1.0
            self.odom_test_pub.publish(message)
            if step == 1:
                # Only this explicit controlled mode advances isolated manager inputs.
                # A full unwrapped lap is now required in addition to the bubble crossing.
                length = self.config['lap_detection']['track_length']
                for point in range(1, 13):
                    self.ego_test_pub.publish(Pose2D(x=(100.0 + length * point / 12) % length))
                    time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/scenario-workflow-check')
    parser.add_argument('--timeout', type=float, default=10800)
    parser.add_argument('--scenario-timeout', type=float, default=700)
    parser.add_argument('--controlled-transitions', action='store_true')
    args = parser.parse_args()
    config_path = ROOT / 'config/manager.yaml'
    config = yaml.safe_load(config_path.read_text())
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='112', SESSION_ID='1', RESULTS_DIR=str(output),
                      CONFIG_FILE=str(config_path), NUM_SIMULATIONS=str(config['num_simulations']),
                      MAX_LAPS=str(config['max_laps']), OVERTAKE_MARGIN=str(config['overtake_margin']),
                      SCENARIO_RESET_HOLD_SECONDS=str(config.get('reset_settle_seconds', 2.0)))
    rclpy.init()
    probe = WorkflowProbe(config, output, args.controlled_transitions)
    processes, logs = [], []
    error = None
    try:
        commands = [
            ['ros2', 'run', 'f1tenth_gym_ros', 'gym_bridge', '--ros-args', '-r', '__node:=bridge',
             '--params-file', str(ROOT / 'config/sim.yaml')],
            ['ros2', 'launch', 'f1tenth_gym_ros', 'maneuver_launch.py', 'scenario_manager:=false'],
        ]
        manager_command = ['ros2', 'run', 'env_manager', 'main']
        if args.controlled_transitions:
            manager_command.extend(['--ros-args',
                             '-r', '/ego_racecar/odom:=/workflow_test/ego_odom',
                             '-r', '/ego_racecar/frenet:=/workflow_test/ego_frenet',
                             '-r', '/opp_racecar/frenet:=/workflow_test/opp_frenet'])
        for name, command in zip(('bridge', 'pipeline', 'manager'), commands):
            log = (output / (name + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        ready_deadline = time.monotonic() + 45
        while not (probe.control_ready and len(probe.final_speeds) == 2) and time.monotonic() < ready_deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        assert probe.control_ready, 'Controllers did not become ready'
        assert 'env_manager' not in probe.get_node_names(), 'A second scenario manager is already running'
        log = (output / 'manager.log').open('w')
        logs.append(log)
        processes.append(subprocess.Popen(manager_command, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))
        deadline, next_progress = time.monotonic() + args.timeout, time.monotonic() + 20
        while not probe.finished and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            probe.drive_manager_check()
            if any(p.poll() is not None for p in processes):
                raise AssertionError('A simulation process exited before session completion')
            if probe.scenario and probe.scenarios[probe.scenario]['elapsed_s'] > args.scenario_timeout:
                raise AssertionError(f'Scenario {probe.scenario} did not finish within {args.scenario_timeout}s')
            if time.monotonic() >= next_progress:
                probe.save_progress()
                print(f'Scenario {probe.scenario}: {probe.scenarios.get(probe.scenario, {})}', flush=True)
                next_progress = time.monotonic() + 20
        assert probe.finished, 'The configured sequence did not finish before timeout'
        settle_deadline = time.monotonic() + 3
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        assert set(probe.scenarios) == set(range(1, config['num_simulations'] + 1)), 'Missing scenarios'
        assert all(v < 0.05 for v in probe.final_speeds.values()), 'A car kept moving after shutdown'
        for index, row in probe.scenarios.items():
            assert all(v is not None and v < 0.3 for v in row['spawn_error_m'].values()), f'Spawn mismatch: {index}'
            assert min(row['collision_messages'].values()) > 0, f'Missing collision telemetry: {index}'
            assert min(row['distance_m'].values()) > 0.1, f'A car did not resume driving: {index}'
        csv_reports = {}
        for index in range(1, config['num_simulations'] + 1):
            file = output / f'simulation_{index}.csv'
            with file.open() as stream:
                reader = csv.DictReader(stream)
                assert {'imm_trajectory', 'imm_active', 'mppi_state', 'driving_mode'} <= set(reader.fieldnames), file
                rows = list(reader)
                assert rows and all(None not in row for row in rows), f'Empty or malformed CSV: {file}'
            csv_reports[index] = len(rows)
        probe.save_progress()
        (output / 'csv_audit.json').write_text(json.dumps(csv_reports, indent=2))
    except Exception as exception:
        error = str(exception)
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
        report = probe.save_progress()
        report['passed'] = error is None
        report['error'] = error
        (output / 'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        probe.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)


if __name__ == '__main__':
    main()
