"""Exercise real warmup, driving and scenario completion without synthetic events."""
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
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from lidar_processing.experiment_protocol import protocol_qos

ROOT = Path('/sim_ws/src/f1tenth_gym_ros')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--scenarios', type=int, default=3)
    parser.add_argument('--controller', default='overtake')
    parser.add_argument('--timeout', type=float, default=240)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((ROOT / 'config/manager.yaml').read_text())
    config['num_simulations'] = args.scenarios
    config_path = output / 'test_manager.yaml'
    config_path.write_text(yaml.safe_dump(config))
    os.environ.update(SESSION_ID='protocol_validation', RESULTS_DIR=str(output),
                      CONFIG_FILE=str(config_path), NUM_SIMULATIONS=str(args.scenarios),
                      MAX_LAPS='1', OVERTAKE_MARGIN='3.0', SCENARIO_RESET_HOLD_SECONDS='2.0')
    rclpy.init()
    observer = Node('experiment_protocol_probe')
    transitions = []
    collisions = {'ego': 0, 'opp': 0}
    finished = [False]
    final_speeds = {}
    initial_speeds = {}

    def odom(car, message):
        speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        final_speeds[car] = speed
        if transitions and transitions[-1]['phase'] == 'WARMUP' and speed > 0.05:
            initial_speeds.setdefault(transitions[-1]['scenario'], {}).setdefault(car, speed)

    def state(message):
        value = json.loads(message.data)
        key = (value['scenario'], value['phase'])
        if not transitions or key != (transitions[-1]['scenario'], transitions[-1]['phase']):
            transitions.append(value)
            print(f"Scenario {value['scenario']}: {value['phase']} (ego={value['ego_speed']}, opp={value['opp_speed']})", flush=True)

    observer.create_subscription(String, '/env_manager/experiment_state', state, protocol_qos())
    observer.create_subscription(Bool, '/env_manager/shutdown', lambda m: finished.__setitem__(0, m.data), 10)
    for car in ('ego', 'opp'):
        observer.create_subscription(Odometry, '/' + car + '_racecar/odom', lambda m, c=car: odom(c, m), 10)
        observer.create_subscription(Bool, '/' + car + '_racecar/collision',
            lambda m, c=car: collisions.__setitem__(c, collisions[c] + int(m.data)), 10)
    processes, streams = [], []
    error = None
    audits = {}
    try:
        commands = [
            ['ros2', 'run', 'f1tenth_gym_ros', 'gym_bridge', '--ros-args', '-r', '__node:=bridge',
             '--params-file', str(ROOT / 'config/sim.yaml')],
            ['ros2', 'launch', 'f1tenth_gym_ros', 'maneuver_launch.py',
             'scenario_manager:=false', 'controller:=' + args.controller],
            ['ros2', 'run', 'env_manager', 'main'],
        ]
        for name, command in zip(('bridge', 'pipeline', 'manager'), commands):
            stream = (output / (name + '.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        started, next_report = time.monotonic(), time.monotonic() + 20
        while not finished[0] and time.monotonic() - started < args.timeout:
            rclpy.spin_once(observer, timeout_sec=0.1)
            if any(process.poll() is not None for process in processes):
                raise AssertionError('A process exited before experiment completion; inspect logs')
            if time.monotonic() >= next_report:
                print(f"Still running: {transitions[-1] if transitions else 'awaiting bridge initialization'}", flush=True)
                next_report += 20
        assert finished[0], 'Natural run did not finish within verification timeout'
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert all(final_speeds.get(car, float('inf')) < 0.05 for car in ('ego', 'opp')), final_speeds
        # The durable sidecar records even short FINISHED phases between two resets.
        transitions = [json.loads(line) for line in (output / 'scenario_events.jsonl').read_text().splitlines()]
        for number in range(1, args.scenarios + 1):
            phases = [s for s in transitions if s['scenario'] == number]
            names = [s['phase'] for s in phases]
            assert all(p in names for p in ('SETTLING', 'WARMUP', 'RUNNING', 'FINISHED')), names
            warmup = next(s for s in phases if s['phase'] == 'WARMUP')
            running = next(s for s in phases if s['phase'] == 'RUNNING')
            warmup_seconds = (running['event_stamp_ns'] - warmup['event_stamp_ns']) / 1e9
            assert warmup_seconds >= 1.5, warmup_seconds
            with (output / f'simulation_{number}.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            assert rows and all(None not in row and all(v is not None for v in row.values()) for row in rows)
            assert {'WARMUP', 'RUNNING'} <= {r['experiment_phase'] for r in rows}
            assert all(r['mppi_state'] == 'RACELINE' for r in rows if r['experiment_phase'] == 'WARMUP')
            assert {'rel_s_unwrapped', 'opponent_faster', 'opp_speed_limited'} <= rows[0].keys()
            for car in ('ego', 'opp'):
                assert abs(initial_speeds[number][car] - warmup[car + '_speed']) < 0.2, initial_speeds
            active = [r for r in rows if r['experiment_phase'] == 'RUNNING']
            assert all(float(r['opp_target_speed']) > float(r['ego_target_speed']) for r in rows)
            assert any(r['defensive_imm_ready'] == 'True' for r in active)
            assert any(r['defensive_imm_trajectory'] != '[]' for r in active)
            assert any(r['mppi_state'] == 'DIVERT' for r in active), 'Defense never activated'
            for before, after in zip(rows, rows[1:]):
                for car in ('ego', 'opp'):
                    jump = math.hypot(float(after[car+'_x']) - float(before[car+'_x']),
                                      float(after[car+'_y']) - float(before[car+'_y']))
                    assert jump < 1.0, f'Reset contamination in scenario {number}: {jump}'
            audits[number] = {'rows': len(rows), 'active_rows': len(active),
                              'warmup_seconds': warmup_seconds,
                              'ego_target_speed': warmup['ego_speed'], 'opp_target_speed': warmup['opp_speed'],
                              'initial_measured_speeds': initial_speeds[number],
                              'defense_samples': sum(r['mppi_state'] == 'DIVERT' for r in active),
                              'completion': phases[-1]['reason']}
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
        for stream in streams:
            stream.close()
        report = {'passed': error is None, 'error': error, 'mode': 'natural',
                  'synthetic_completion_events': False, 'controller': args.controller,
                  'scenarios': audits, 'transitions': transitions, 'collision_samples': collisions,
                  'final_measured_speeds': final_speeds}
        (output / 'protocol_verification.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k:v for k,v in report.items() if k != 'transitions'}, indent=2), flush=True)
        observer.destroy_node()
        rclpy.shutdown()
    raise SystemExit(1 if error else 0)


if __name__ == '__main__':
    main()
