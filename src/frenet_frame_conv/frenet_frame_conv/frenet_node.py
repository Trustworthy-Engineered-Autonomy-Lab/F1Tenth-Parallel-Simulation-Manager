#!/usr/bin/env python3

import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from ament_index_python.packages import get_package_share_directory


"""
Frenet conversion node

Loads a 2D track centerline from generated csv file, calculates
arc length along that centerline, then converts incoming vehicle odom
from cartesian into frenet coordinates (s, d).

"""
class FrenetNode(Node):
    def __init__(self):
        super().__init__("frenet_node")
        
        # Use ROS2's package share directory instead
        pkg_share = get_package_share_directory('frenet_frame_conv')
        csv_path = os.path.join(pkg_share, 'centerline_csv', 'Spielberg_map.csv')
        
        self.centerline = self.load_centerline(csv_path)
        self.arc_lengths = self.compute_arc_lengths(self.centerline)
        self.segments = np.diff(self.centerline, axis=0)
        self.segment_lengths = np.linalg.norm(self.segments, axis=1)
        self.segment_lengths_sq = np.maximum(self.segment_lengths ** 2, 1e-12)
        self.get_logger().info(f"Total calculated track arclength: {self.arc_lengths[-1]:.2f}m")

        self.get_logger().info(
            f"Loaded {len(self.centerline)} centerline points"
        )

        self.ego_odom_sub = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.ego_odom_callback,
            10
        )

        self.opp_odom_sub = self.create_subscription(
            Odometry,
            "/opp_racecar/odom",
            self.opp_odom_callback,
            10
        )

        self.ego_frenet_pub = self.create_publisher(
            Pose2D,
            "/ego_racecar/frenet",
            10
        )

        self.opp_frenet_pub = self.create_publisher(
            Pose2D,
            "/opp_racecar/frenet",
            10
        )

        self.get_logger().info("Frenet node started")

    def load_centerline(self, path):
        points = []
        with open(path, "r") as file:
            reader = csv.reader(file)
            for row in reader:
                if row:
                    points.append([float(row[0]), float(row[1])])
        
        if len(points) > 0 and not np.allclose(points[0], points[-1]):
            points.append(points[0])

        return np.array(points)          
   
    def compute_arc_lengths(self, points):
        s = [0.0]
        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - points[i - 1])
            s.append(s[-1] + distance)
        return np.array(s)

    def cartesian_to_frenet(self, x, y):
        # Project all segments together: the bridge publishes two odometries at
        # 250 Hz, so a Python loop over thousands of segments cannot keep up.
        offsets = np.array([x, y]) - self.centerline[:-1]
        t = np.clip(np.sum(offsets * self.segments, axis=1) / self.segment_lengths_sq, 0.0, 1.0)
        diff = offsets - t[:, None] * self.segments
        distance_sq = np.sum(diff ** 2, axis=1)
        distance_sq[self.segment_lengths < 1e-6] = np.inf
        i = int(np.argmin(distance_sq))
        cross = self.segments[i, 0] * diff[i, 1] - self.segments[i, 1] * diff[i, 0]
        return (self.arc_lengths[i] + t[i] * self.segment_lengths[i],
                np.sign(cross) * np.sqrt(distance_sq[i]))

    def ego_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        s, d = self.cartesian_to_frenet(x, y)

        frenet_msg = Pose2D()
        frenet_msg.x = float(s)
        frenet_msg.y = float(d)
        frenet_msg.theta = 0.0

        self.ego_frenet_pub.publish(frenet_msg)

    def opp_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        s, d = self.cartesian_to_frenet(x, y)

        frenet_msg = Pose2D()
        frenet_msg.x = float(s)
        frenet_msg.y = float(d)
        frenet_msg.theta = 0.0

        self.opp_frenet_pub.publish(frenet_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

