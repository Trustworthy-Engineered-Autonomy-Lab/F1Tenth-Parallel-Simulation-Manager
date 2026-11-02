"""Check the defensive pipeline's compatibility with main's scenario reset API."""

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/lidar_processing/scripts'))
from dynamic_mppi_node import DynamicMPPI, RACELINE
from generate_candidate_trajectories import OpponentIntentPredictor
from path_converter_node import PathConverter
from sim_opp_frenet import SimOpponentFrenet

spec = importlib.util.spec_from_file_location('defensive_imm', ROOT / 'src/lidar_processing/scripts/imm.py')
defensive_imm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(defensive_imm)

from lidar_processing.scenario_lifecycle import ScenarioLifecycle
from lidar_processing.manager_config import read_config, DEFAULTS
from overtake_controller.interceptor import OvertakeInterceptorNode
from overtake_controller.imm_filterpy import IMMNode
from overtake_controller.overtake import OpponentOvertakeController
from overtake_controller.overtake_aggressive import OpponentOvertakeController as AggressiveController


def reset_message(node):
    message = PoseArray(poses=[Pose(), Pose()])
    message.header.stamp = node.get_clock().now().to_msg()
    return message


def after_hold(lifecycle):
    # Simulate elapsed settle time without sleeping in each unit test.
    lifecycle.resume_at_ns = lifecycle.node.get_clock().now().nanoseconds - 1


class ResetCompatibility(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '--params-file',
                         str(ROOT / 'src/lidar_processing/config/params_dynamic.yaml')])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()

    def node(self, cls, *args):
        node = cls(*args)
        self.nodes.append(node)
        return node

    def test_pause_stale_messages_duplicate_reset_and_shutdown(self):
        node = self.node(Node, 'lifecycle_test')
        cleared = Mock()
        lifecycle = ScenarioLifecycle(node, cleared)
        received = Mock()
        guarded = lifecycle.guard(received)
        old = Odometry()
        old.header.stamp = node.get_clock().now().to_msg()
        request = reset_message(node)
        lifecycle.reset(request)
        guarded(old)
        received.assert_not_called()
        self.assertFalse(lifecycle.ready)
        lifecycle.reset(request)
        self.assertEqual(lifecycle.generation, 1)
        after_hold(lifecycle)
        guarded(old)
        received.assert_not_called()
        fresh = Odometry()
        fresh.header.stamp = node.get_clock().now().to_msg()
        guarded(fresh)
        received.assert_called_once_with(fresh)
        lifecycle.shutdown(Bool(data=True))
        self.assertFalse(lifecycle.ready)
        lifecycle.reset(reset_message(node))
        self.assertTrue(lifecycle.stopped)

    def test_mppi_discards_divert_warm_start_and_synchronizer_queues(self):
        node = self.node(DynamicMPPI)
        node.state = 'DIVERT'
        node.maneuver = {'old': True}
        node.active_waypoints = node.raceline.copy()
        node.u_prev.fill(1.0)
        for queue in node.sync.queues:
            queue['old'] = 'queued sensor observation'
        node.drive_pub = Mock()
        node.lifecycle.reset(reset_message(node))
        self.assertEqual(node.state, RACELINE)
        self.assertIs(node.active_waypoints, node.raceline)
        self.assertIsNone(node.maneuver)
        self.assertTrue(np.all(node.u_prev == 0))
        self.assertTrue(all(not q for q in node.sync.queues))
        self.assertEqual(node.drive_pub.publish.call_args[0][0].drive.speed, 0.0)
        node.divert_cb(Float64MultiArray(data=[100.0, 100.0, 101.0, 101.0]))
        self.assertIs(node.active_waypoints, node.raceline)

    def test_both_overtakers_stop_and_reject_previous_path(self):
        for cls in (OpponentOvertakeController, AggressiveController):
            with self.subTest(controller=cls.__module__):
                node = self.node(cls)
                old = RosPath()
                old.header.stamp = node.get_clock().now().to_msg()
                node.overtake_spline = [object()] * 10
                node.prev_steer = node.pp_steer_ema = 0.3
                node.scan_msg = object()
                node.drive_pub = Mock()
                node.lifecycle.reset(reset_message(node))
                self.assertIsNone(node.overtake_spline)
                self.assertIsNone(node.scan_msg)
                self.assertEqual(node.prev_steer, 0.0)
                self.assertFalse(node.ego_data_received or node.opp_data_received)
                self.assertEqual(node.drive_pub.publish.call_args[0][0].drive.speed, 0.0)
                after_hold(node.lifecycle)
                node.spline_callback(old)
                self.assertIsNone(node.overtake_spline)
                node.control_loop()
                self.assertEqual(node.drive_pub.publish.call_count, 1)

    def test_passing_planner_forgets_locked_path_and_localization(self):
        node = self.node(OvertakeInterceptorNode)
        node.locked_spline = np.ones((10, 2))
        node.ego_data_received = node.opp_data_received = True
        node.overtake_path_pub = Mock()
        node.lifecycle.reset(reset_message(node))
        self.assertIsNone(node.locked_spline)
        self.assertFalse(node.ego_data_received or node.opp_data_received)
        self.assertEqual(len(node.overtake_path_pub.publish.call_args[0][0].poses), 0)

    def test_defensive_imm_seeds_from_new_scenario_instead_of_teleport_velocity(self):
        node = self.node(defensive_imm.IMMFilterNode)
        node._initialized = True
        node.s_imm.reinitialize(np.array([200.0, 20.0, 5.0]))
        node._last_s = 200.0
        node.lifecycle.reset(reset_message(node))
        self.assertFalse(node._initialized)
        self.assertIsNone(node._last_s)
        after_hold(node.lifecycle)
        node._cb(Float64MultiArray(data=[5.0, -0.2, 0.05]))
        self.assertTrue(node._initialized)
        for state in node.s_imm.xs:
            np.testing.assert_allclose(state, [5.0, 0.0, 0.0])

    def test_ego_prediction_imm_restarts_at_new_ego_pose(self):
        node = self.node(IMMNode)
        node.imm_model.x[:] = 100.0
        node.first_callback = False
        node.lifecycle.reset(reset_message(node))
        self.assertTrue(node.first_callback)
        self.assertTrue(node.imm_model_inactive)
        after_hold(node.lifecycle)
        node.last_odom_pub_time = node.get_clock().now() - Duration(seconds=1)
        fresh = Odometry()
        fresh.header.stamp = node.get_clock().now().to_msg()
        fresh.pose.pose.position.x, fresh.pose.pose.position.y = 30.0, 40.0
        node.odom_callback(fresh)
        self.assertFalse(node.first_callback)
        np.testing.assert_allclose(node.imm_model.x[[0, 3]], [30.0, 40.0])
        np.testing.assert_allclose(node.imm_model.x[[1, 2, 4, 5]], 0.0)

    def test_converter_drops_old_markers_after_respawn(self):
        node = self.node(PathConverter)
        marker = Marker()
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.points = [Point(x=float(i), y=0.1 * i) for i in range(3)]
        node.pub = Mock()
        node.lifecycle.reset(reset_message(node))
        after_hold(node.lifecycle)
        node.cb(MarkerArray(markers=[marker]))
        self.assertEqual(node.pub.publish.call_count, 1)  # reset invalidation only
        marker.header.stamp = node.get_clock().now().to_msg()
        node.cb(MarkerArray(markers=[marker]))
        self.assertEqual(node.pub.publish.call_count, 2)

    def test_rrt_result_started_before_reset_is_not_published(self):
        node = self.node(OpponentIntentPredictor)
        node.ego_data_received = True
        node.ego_state = (4.0, 1.0, 0.0)
        node.latest_points = [(0.0, 0.0)]
        node.latest_stamp = node.get_clock().now().to_msg()
        node.points_seq = 1
        node.publish_threat_markers = Mock(return_value=0)
        def reset_mid_plan(*_args):
            node.lifecycle.reset(reset_message(node))
            return [[]], [[]], 1.0
        node.generate_opponent_splines_rrt = Mock(side_effect=reset_mid_plan)
        node.plan_timer_cb()
        node.publish_threat_markers.assert_not_called()
        self.assertIsNone(node.latest_points)
        self.assertFalse(node.ego_data_received)

    def test_observation_bridge_restarts_elapsed_time_on_reset(self):
        node = self.node(SimOpponentFrenet)
        node.last_time = node.get_clock().now()
        node.lifecycle.reset(reset_message(node))
        self.assertIsNone(node.last_time)


class LauncherConfigChecks(unittest.TestCase):
    def test_missing_config_preserves_current_defaults(self):
        self.assertEqual(read_config('/tmp/nonexistent-manager-config.yaml'), DEFAULTS)

    def test_reads_friend_style_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'manager.yaml'
            path.write_text('num_sessions: 1\nnum_simulations: 24\nmax_laps: 1\novertake_margin: 3.0\n')
            config = read_config(path)
            self.assertEqual((config['num_sessions'], config['num_simulations'], config['max_laps']), (1, 24, 1))

    def test_rejects_invalid_config_before_container_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'manager.yaml'
            for invalid in ('num_sessions: true', 'num_simulations: 1.5',
                            'max_laps: 0', 'overtake_margin: .nan', '- not_a_mapping'):
                with self.subTest(config=invalid):
                    path.write_text(invalid)
                    with self.assertRaises(ValueError):
                        read_config(path)


if __name__ == '__main__':
    unittest.main(verbosity=2)
