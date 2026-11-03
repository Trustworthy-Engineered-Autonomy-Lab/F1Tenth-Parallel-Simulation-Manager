"""Regression checks for experiment timing, velocity configuration and data boundaries."""
import csv
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/lidar_processing/scripts'))
from lidar_processing.experiment_protocol import TrackProgress, scenario_speeds
from lidar_processing.scenario_lifecycle import ScenarioLifecycle
from env_manager.manager import EnvManager
from data_logger.logger import DataLogger
from f1tenth_gym_ros.gym_bridge import GymBridge

spec = importlib.util.spec_from_file_location('defensive_imm_protocol', ROOT / 'src/lidar_processing/scripts/imm.py')
imm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imm)


class ProtocolChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.environment = patch.dict(os.environ, {
            'RESULTS_DIR': self.tmp.name, 'CONFIG_FILE': str(ROOT / 'config/manager.yaml'),
            'NUM_SIMULATIONS': '2', 'MAX_LAPS': '1', 'OVERTAKE_MARGIN': '3.0'})
        self.environment.start()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            if getattr(node, 'csv_file', None) and not node.csv_file.closed:
                node.csv_file.close()
            node.destroy_node()
        self.environment.stop()
        self.tmp.cleanup()

    def node(self, cls):
        node = cls()
        self.nodes.append(node)
        return node

    def test_stationary_opponent_cannot_pass_at_half_a_track(self):
        progress = TrackProgress(360)
        progress.update('ego', 100)
        progress.update('opp', 98)
        for step in range(1, 362):
            progress.update('ego', (100 + step) % 360)
            self.assertLess(progress.gap, 0)
        self.assertAlmostEqual(progress.distance['ego'], 361)

    def test_true_pass_across_coordinate_origin(self):
        progress = TrackProgress(360)
        progress.update('ego', 358)
        progress.update('opp', 356)
        progress.update('ego', 1)
        progress.update('opp', 4.1)
        self.assertAlmostEqual(progress.gap, 3.1)

    def test_two_lap_limit_requires_two_distinct_laps(self):
        manager = self.node(EnvManager)
        manager.max_laps_per_sim = 2
        manager.reset_in_progress = False
        manager.progress.distance['ego'] = manager.track_length
        manager._detect_lap_with_frenet()
        manager._detect_lap_with_frenet()
        self.assertEqual(manager.sim_lap_count, 1)
        self.assertEqual(manager.current_sim, 1)
        manager.progress.distance['ego'] = 2 * manager.track_length
        manager._detect_lap_with_frenet()
        self.assertEqual(manager.current_sim, 2)

    def test_invalid_speed_order_and_nonfinite_speeds_rejected(self):
        for ego, opp in ((1, 1), (2, 1), (0, 1), (1, float('nan')), (True, 2)):
            with self.subTest(speeds=(ego, opp)), self.assertRaises(ValueError):
                scenario_speeds({'ego_speed': ego, 'opp_speed': opp})

    def test_only_fresh_complete_initialization_releases_manager(self):
        manager = self.node(EnvManager)
        manager.reset_stamp_ns = 100
        manager.progress.update('ego', 100)
        manager.progress.update('opp', 98)
        manager._finish_reset_transition()
        for token, elapsed in ((99, 2.0), (100, 1.49)):
            manager._imm_ready(String(data=json.dumps({'reset_stamp_ns': token,
                               'ready': True, 'observation_seconds': elapsed})))
            self.assertEqual(manager.phase, 'WARMUP')
            self.assertTrue(manager.reset_in_progress)
        manager.warmup_started_ns -= 1_500_000_000
        manager._imm_ready(String(data=json.dumps({'reset_stamp_ns': 100,
                           'ready': True, 'observation_seconds': 1.5})))
        self.assertEqual(manager.phase, 'RUNNING')
        self.assertFalse(manager.reset_in_progress)

    def test_defensive_imm_needs_continuous_observations_after_reset(self):
        node = self.node(imm.IMMFilterNode)
        node.readiness_pub = Mock()
        node._publish_prediction = Mock()
        node.publish_covariance = Mock()
        node.lifecycle.configuration = {'reset_stamp_ns': 100, 'imm_initialization_seconds': 1.5}
        from std_msgs.msg import Float64MultiArray
        for sample in range(32):
            node.get_clock = Mock(return_value=Mock(now=Mock(return_value=Time(nanoseconds=1_000_000_000 + sample*50_000_000))))
            node._cb(Float64MultiArray(data=[100.0 + sample*0.05, 0.0, 0.05]))
            ready = json.loads(node.readiness_pub.publish.call_args[0][0].data)
            self.assertEqual(ready['ready'], sample >= 30)
        node.get_clock = Mock(return_value=Mock(now=Mock(return_value=Time(nanoseconds=4_000_000_000))))
        node._cb(Float64MultiArray(data=[101.6, 0.0, 0.5]))
        self.assertFalse(json.loads(node.readiness_pub.publish.call_args[0][0].data)['ready'])

    def test_late_reset_event_removes_already_written_wrong_row(self):
        node = self.node(DataLogger)
        node.hold_until_ns = 0
        node.ego_ready = node.opp_ready = node.ego_frenet_ready = node.opp_frenet_ready = True
        node.ego_x = 123.0
        node.log_to_csv()
        reset = PoseArray(poses=[Pose(), Pose()])
        reset.header.stamp = node.get_clock().now().to_msg()
        node.ego_x = 999.0  # next spawn arrives before its reset event
        node.log_to_csv()
        node.physical_reset_cb(reset)
        with open(node.current_csv) as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        self.assertEqual(float(rows[0]['ego_x']), 123.0)

    def test_initial_velocity_is_applied_to_model_and_settle_blocks_steps(self):
        bridge = Mock()
        bridge.experiment_state = {'phase': 'SETTLING', 'reset_stamp_ns': 42,
                                   'ego_speed': 0.8, 'opp_speed': 1.2}
        bridge.applied_reset_stamp_ns = 42
        bridge.seeded_reset_stamp_ns = None
        bridge.has_opp = True
        bridge.ego_drive_published = bridge.opp_drive_published = True
        bridge.env.sim.agents = [Mock(state=np.zeros(7)), Mock(state=np.zeros(7))]
        bridge.ego_steer = bridge.opp_steer = 0.0
        bridge.env.step.return_value = ({}, 0, False, {})
        GymBridge.drive_timer_callback(bridge)
        bridge.env.step.assert_not_called()
        bridge.experiment_state['phase'] = 'WARMUP'
        GymBridge.drive_timer_callback(bridge)
        self.assertEqual(bridge.env.sim.agents[0].state[3], 0.8)
        self.assertEqual(bridge.env.sim.agents[1].state[3], 1.2)
        bridge.experiment_state['phase'] = 'FINISHED'
        bridge.env.step.reset_mock()
        GymBridge.drive_timer_callback(bridge)
        bridge.env.step.assert_not_called()
        self.assertEqual(bridge.env.sim.agents[0].state[3], 0.0)
        self.assertEqual(bridge.env.sim.agents[1].state[3], 0.0)
        self.assertEqual(bridge.obs['linear_vels_x'], [0.0, 0.0])

    def test_logger_can_join_an_already_running_scenario(self):
        node = self.node(DataLogger)
        state = {'scenario': 2, 'phase': 'RUNNING', 'reset_stamp_ns': 1,
                 'reset_settle_seconds': 2.0}
        node.experiment_cb(String(data=json.dumps(state)))
        self.assertEqual(node.current_sim, 2)
        self.assertTrue(node._settled())

    def test_mppi_rollout_uses_scenario_velocity_bounds(self):
        from dynamic_mppi_node import DynamicMPPI
        node = DynamicMPPI.__new__(DynamicMPPI)
        parameters = {'wheelbase': 0.33, 'max_steer': 0.3, 'dt': 0.1}
        node.set_parameters = Mock()
        node.get_parameter = Mock(side_effect=lambda name: Mock(value=parameters[name]))
        node.raceline = np.zeros((5, 4))
        node.u_prev = np.zeros((15, 2))
        node.configure_experiment({'ego_speed': 0.8})
        state = np.array([[0.0, 0.0, 0.0]])
        low = node.model.predict_euler(state, np.array([[0.0, 0.0]]))
        high = node.model.predict_euler(state, np.array([[10.0, 0.0]]))
        self.assertAlmostEqual(low[0, 0], 0.8 * 0.7 * 0.1)
        self.assertAlmostEqual(high[0, 0], 0.8 * 0.1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
