#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from racetrack_utilities.racetrack_utilities import RacetrackUtilities


class SimOpponentFrenet(Node):
    def __init__(self):
        super().__init__("sim_opponent_frenet")

        self.track = RacetrackUtilities(
            "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv"
        )

        self.last_time = None

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/frenet_opp_state_vector",
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            "/opp_racecar/odom",
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        s, d = self.track.convert_to_frenet(x, y)

        now = self.get_clock().now()

        if self.last_time is None:
            dt = 0.05
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            dt = max(0.001, dt)

        self.last_time = now

        output = Float64MultiArray()
        output.data = [float(s), float(d), float(dt)]

        self.pub.publish(output)


def main():
    rclpy.init()
    node = SimOpponentFrenet()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
