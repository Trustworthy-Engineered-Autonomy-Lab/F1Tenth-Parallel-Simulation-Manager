#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from racetrack_utilities.racetrack_utilities import RacetrackUtilities
from lidar_processing.scenario_lifecycle import ScenarioLifecycle


class SimOpponentFrenet(Node):
    def __init__(self):
        super().__init__("sim_opponent_frenet")
        # Opponent odometry only initializes the defensive IMM during WARMUP; once the
        # scenario is RUNNING the IMM must rely on its own prediction, not ground truth.
        self.lifecycle = ScenarioLifecycle(self, self.reset_scenario, ('odom_callback',),
                                           allowed_phases=('WARMUP',))

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

    def reset_scenario(self, _message):
        self.last_time = None

    def odom_callback(self, msg):
        # The bridge repeats observations at 250 Hz; update the estimator at 20 Hz.
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_time is not None and (now - self.last_time).nanoseconds < 50_000_000:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        s, d = self.track.convert_to_frenet(x, y)

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
