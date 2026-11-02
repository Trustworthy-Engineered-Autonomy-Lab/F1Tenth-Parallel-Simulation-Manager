"""Focused regressions; run in the sourced ROS container with unittest."""

import unittest
import sys
from pathlib import Path as FilePath

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import splev

from overtake_controller.interceptor import OvertakeInterceptorNode
from frenet_frame_conv.frenet_node import FrenetNode
from f1tenth_mppi_dynamic.reference_manager import ReferencePlanner
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

sys.path.insert(0, str(FilePath(__file__).resolve().parents[1] / 'src/lidar_processing/scripts'))
from generate_candidate_trajectories import OpponentIntentPredictor


class ManeuverRegressions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_ego_prediction_does_not_replace_overtaker_pose(self):
        node = OvertakeInterceptorNode()
        try:
            odom = Odometry()
            odom.pose.pose.position.x = 2.0
            odom.pose.pose.orientation.w = 1.0
            node.opp_odom_callback(odom)
            path = Path()
            pose = PoseStamped()
            pose.pose.position.x = 8.0
            path.poses = [pose]
            node.imm_path_callback(path)
            self.assertEqual(node.opp_x, 2.0)
            odom.pose.pose.position.x = 3.0
            node.opp_odom_callback(odom)
            self.assertEqual(node.opp_x, 3.0)
        finally:
            node.destroy_node()

    def test_relative_track_position_crosses_finish_line(self):
        track = object.__new__(RacetrackUtilities)
        track.arclength = 100.0
        self.assertAlmostEqual(track.unwrap_s(99.0, 2.0), -1.0)
        self.assertAlmostEqual(track.unwrap_s(2.0, 99.0), 102.0)

    def test_default_spawn_gap_can_trigger_overtake_planning(self):
        node = OvertakeInterceptorNode()
        try:
            node.ego_data_received = node.opp_data_received = True
            node.ego_x, node.ego_y = 4.4, 0.0
            node.opp_x, node.opp_y = 0.0, 0.0
            node.continuous_spline_update()
            self.assertIsNotNone(node.locked_spline)
        finally:
            node.destroy_node()

    def test_corner_path_follows_track_and_rejects_straight_shortcut(self):
        node = OvertakeInterceptorNode()
        try:
            track = node.track
            # A feasible Spielberg bend; the maximum curvature of the smoothed
            # map includes spikes that the steering-limit check rightly rejects.
            corner_s = 30.0
            s = np.linspace(corner_s - 5.0, corner_s + 5.0, 80)
            curved = track.convert_to_cartesian_arr(s, np.zeros(len(s)))
            self.assertIsNotNone(node.validate_and_clip_path(curved))
            direction = curved[1] - curved[0]
            direction /= np.linalg.norm(direction)
            straight = curved[0] + np.linspace(0, 12, 80)[:, None] * direction
            self.assertIsNone(node.validate_and_clip_path(straight))
        finally:
            node.destroy_node()

    def test_passing_path_cannot_run_through_moving_defender(self):
        node = OvertakeInterceptorNode()
        try:
            track = node.track
            node.opp_x, node.opp_y = track.convert_to_cartesian(90.0, 0.0)
            node.ego_x, node.ego_y = track.convert_to_cartesian(94.0, 0.0)
            tangent = splev(94.0 / track.arclength, track.centerline_spline, der=1)
            node.ego_yaw = np.arctan2(tangent[1], tangent[0])
            node.ego_speed = 1.0
            s = np.linspace(90.0, 102.0, 80)
            path = track.convert_to_cartesian_arr(s, np.zeros(len(s)))
            self.assertIsNone(node.validate_and_clip_path(path))
            node.ego_x, node.ego_y = track.convert_to_cartesian(94.0, 0.7)
            self.assertIsNotNone(node.validate_and_clip_path(path))
        finally:
            node.destroy_node()

    def test_frenet_projection_matches_segment_search(self):
        node = FrenetNode()
        try:
            for point in np.random.default_rng(8).uniform(-10, 10, (20, 2)):
                best = (np.inf, 0.0, 0.0)
                for i, segment in enumerate(np.diff(node.centerline, axis=0)):
                    length = np.linalg.norm(segment)
                    if length < 1e-6:
                        continue
                    t = np.clip(np.dot(point - node.centerline[i], segment) / length**2, 0, 1)
                    diff = point - node.centerline[i] - t * segment
                    distance = np.linalg.norm(diff)
                    cross = segment[0] * diff[1] - segment[1] * diff[0]
                    if distance < best[0]:
                        best = distance, node.arc_lengths[i] + t * length, np.sign(cross) * distance
                np.testing.assert_allclose(node.cartesian_to_frenet(*point), best[1:], atol=1e-8)
        finally:
            node.destroy_node()

    def test_divert_and_return_can_cross_finish_line(self):
        theta = np.linspace(0, 2 * np.pi, 200, endpoint=False)
        xy = np.column_stack([10 * np.cos(theta), 10 * np.sin(theta)])
        raceline = ReferencePlanner.path_to_waypoints(xy, 1.5, 0.1)
        planner = ReferencePlanner(raceline)
        divert = ReferencePlanner.path_to_waypoints(np.vstack([xy[-8:], xy[:3]]), 1.5, 0.1)
        ret = ReferencePlanner.path_to_waypoints(xy[2:15], 1.5, 0.1)
        active, info = planner.splice(divert, ret)
        self.assertLess(info['divert_end_idx'], info['return_end_idx'])
        self.assertTrue(np.isfinite(active).all())
        # A car approaching from before the seam must precede the return index.
        approach_idx = np.argmin(np.linalg.norm(active[:, :2] - xy[-10], axis=1))
        self.assertLess(approach_idx, info['divert_end_idx'])

    def test_blocking_lane_is_reached_before_long_horizon_ends(self):
        planner = object.__new__(OpponentIntentPredictor)
        planner.path_resolution = 40
        planner.max_s_clearance = 5.0
        planner.ego_velocity = 1.5
        planner.lane_change_length = 2.2
        path = planner._ego_lane_path(10.0, 0.0, 0.65, 4.0)
        at_two_metres = np.interp(12.0, path[:, 1], path[:, 2])
        self.assertGreater(at_two_metres, 0.6)
        self.assertAlmostEqual(path[0, 2], 0.0)


if __name__ == '__main__':
    unittest.main()
