#!/usr/bin/env python3

import csv
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

"""
Frenet conversion node

Loads a 2D track centerline from generated csv file, calculates
arc length along that centerline, then converts incoming vehicle odom
from cartesian into frenet coordinates (s, d).

"""
class FrenetNode(Node):
    def __init__(self):
        super().__init__("frenet_node")
        csv_path = ("./centerline_csv/spielberg_centerline.csv")

        csv_path = "example.csv"
        self.centerline = self.load_centerline(csv_path)
        self.arc_lengths = self.compute_arc_lengths(self.centerline)

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
                try:
                    x = float(row[0])
                    y = float(row[1])
                    points.append([x, y])
                except Exception:
                    continue
        return np.array(points)

    def compute_arc_lengths(self, points):
        s = [0.0]
        for i in range(1, len(points)):
            distance = np.linalg.norm(points[i] - points[i - 1])
            s.append(s[-1] + distance)
        return np.array(s)

    def cartesian_to_frenet(self, x, y):
        cart_position = np.array([x, y])
        best_dist = float("inf")
        best_s = 0.0
        best_d = 0.0

        for i in range(len(self.centerline) - 1):
            p1 = self.centerline[i]
            p2 = self.centerline[i + 1]
            segment = p2 - p1
            seg_length = np.linalg.norm(segment)

            if seg_length < 1e-6:
                continue

            # Projection of point onto segment
            t = np.dot(cart_position - p1, segment) / (seg_length ** 2)
            t = np.clip(t, 0.0, 1.0)

            projected = p1 + t * segment
            diff = cart_position - projected
            distance = np.linalg.norm(diff)

            if distance < best_dist:
                best_dist = distance

                cross = segment[0] * diff[1] - segment[1] * diff[0]
                sign = np.sign(cross)

                best_d = sign * distance
                best_s = self.arc_lengths[i] + t * seg_length

        return best_s, best_d

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


