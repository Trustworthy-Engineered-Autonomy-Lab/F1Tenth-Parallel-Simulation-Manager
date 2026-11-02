"""Verify the merged manager, bridge and scenario logger contracts."""

import csv
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from ackermann_msgs.msg import AckermannDriveStamped

from env_manager.manager import EnvManager
from data_logger.logger import DataLogger
from f1tenth_gym_ros.gym_bridge import GymBridge

ROOT = Path(__file__).resolve().parents[1]


class CombinedWorkflowChecks(unittest.TestCase):
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
            'NUM_SIMULATIONS': '2', 'MAX_LAPS': '1', 'OVERTAKE_MARGIN': '3.0',
        })
        self.environment.start()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            if getattr(node, 'csv_file', None):
                node.csv_file.close()
            node.destroy_node()
        self.environment.stop()
        self.tmp.cleanup()

    def node(self, cls):
        node = cls()
        self.nodes.append(node)
        return node

    def test_manager_advances_after_margin_and_keeps_configured_starts(self):
        node = self.node(EnvManager)
        node.reset_in_progress = False
        node.scenario_reset_pub = Mock()
        node.ego_frenet_ready = node.opp_frenet_ready = True
        node.ego_frenet_s, node.opp_frenet_s = 100, 99
        node._check_overtake_distance()
        node.opp_frenet_s = 102.9
        node._check_overtake_distance()
        self.assertEqual(node.current_sim, 1)
        node.opp_frenet_s = 103.1
        node._check_overtake_distance()
        self.assertEqual(node.current_sim, 2)
        request = node.scenario_reset_pub.publish.call_args[0][0]
        expected = node.config['sim_start']['s_2']
        self.assertAlmostEqual(request.poses[0].position.x, expected['ego'][0])
        self.assertAlmostEqual(request.poses[1].position.x, expected['opp'][0])

    def test_manager_lap_and_final_completion(self):
        node = self.node(EnvManager)
        node.reset_in_progress = False
        node.ego_odom_ready = True
        node.ego_pose = list(node.current_lap_center)
        node._detect_lap_with_bubble()
        self.assertEqual(node.total_lap_count, 0)
        node.ego_pose[0] += 2 * node.bubble_radius
        node._detect_lap_with_bubble()
        node.ego_pose = list(node.current_lap_center)
        node._detect_lap_with_bubble()
        self.assertEqual(node.current_sim, 2)
        node._finish_reset_transition()
        node.shutdown_pub = Mock()
        node._trigger_reset('TEST_COMPLETE')
        self.assertTrue(node.shutdown_requested)
        self.assertTrue(node.shutdown_pub.publish.call_args[0][0].data)

    def test_initial_spawn_waits_for_bridge_and_cannot_count_a_false_lap(self):
        node = self.node(EnvManager)
        node.scenario_reset_pub = Mock()
        node._publish_initial_scenario()
        node.scenario_reset_pub.publish.assert_not_called()
        node.ego_odom_cb(Odometry())
        self.assertFalse(node.lap_armed)
        node.ego_frenet_ready = node.opp_frenet_ready = True
        node._publish_initial_scenario()
        node.scenario_reset_pub.publish.assert_called_once()
        self.assertFalse(node.lap_armed)
        self.assertEqual(node.current_sim, 1)

    def test_bridge_atomic_reset_and_final_shutdown_latch(self):
        manager = self.node(EnvManager)
        message = PoseArray(poses=[manager._make_pose([4, 1, 0.2]), manager._make_pose([0, 0, 0.3])])
        fake = Mock()
        fake.has_opp = True
        fake.env.reset.return_value = ({}, 0, False, {})
        GymBridge.scenario_reset_callback(fake, message)
        np.testing.assert_allclose(fake.env.reset.call_args[0][0], [[4, 1, 0.2], [0, 0, 0.3]])
        GymBridge.shutdown_callback(fake, Bool(data=True))
        self.assertEqual(fake.ego_requested_speed, 0)
        self.assertEqual(fake.opp_requested_speed, 0)
        drive = AckermannDriveStamped()
        drive.drive.speed = 2.0
        GymBridge.drive_callback(fake, drive)
        GymBridge.opp_drive_callback(fake, drive)
        self.assertEqual(fake.ego_requested_speed, 0)
        self.assertEqual(fake.opp_requested_speed, 0)

    def test_logger_rotates_files_and_preserves_defensive_telemetry(self):
        node = self.node(DataLogger)
        node.log_to_csv()
        self.assertEqual(Path(node.current_csv).read_text().count('\n'), 1)
        first_file = node.csv_file
        node.sim_num_cb(Int32(data=2))
        node.reset_cb(Bool(data=True))
        self.assertTrue(first_file.closed)
        node.hold_until_ns = 0
        node.ego_odom_cb(Odometry())
        node.opp_odom_cb(Odometry())
        node.ego_frenet_cb(Pose2D(x=10.0))
        node.opp_frenet_cb(Pose2D(x=9.0))
        node.imm_trajectory = [(1, 2)]
        node.imm_active = True
        node.mppi_state = 'DIVERT'
        node.log_to_csv()
        with open(node.current_csv) as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        self.assertNotIn(None, rows[0])
        self.assertEqual(rows[0]['mppi_state'], 'DIVERT')
        self.assertEqual(rows[0]['imm_active'], 'True')
        self.assertIn('(1, 2)', rows[0]['imm_trajectory'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
