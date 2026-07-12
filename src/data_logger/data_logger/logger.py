import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool, Float32MultiArray
from geometry_msgs.msg import Pose2D
import csv
import os
import math
from nav_msgs.msg import Path
from std_msgs.msg import String


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Configuration
        on_sim = True

        # Storage
        self.session_id = os.getenv("SESSION_ID", "1")
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        self.data_dir = os.path.join(self.results_dir, "data")
        os.makedirs(self.data_dir, exist_ok=True)

        # Iteration tracking
        self.current_iteration = 1
        self.current_csv = None
        self.current_csv_path = None
        self._start_new_iteration(self.current_iteration)

        # State Variables
        self.imm_active = False
        self.lap_num = 0

        self.ego_x, self.ego_y = 0.0, 0.0
        self.ego_vel = 0.0
        self.ego_s, self.ego_d = 0.0, 0.0

        self.opp_x, self.opp_y = 0.0, 0.0
        self.opp_vel = 0.0
        self.opp_s, self.opp_d = 0.0, 0.0

        self.imm_trajectory = []

        # Topics
        ego_odom = '/ego_racecar/odom' if on_sim else 'TODO'
        opp_odom = '/opp_racecar/odom' if on_sim else 'TODO'
        ego_frenet = '/ego_racecar/frenet'
        opp_frenet = '/opp_racecar/frenet'
        imm_path_topic = '/imm_path'

        # Subscriptions
        self.create_subscription(Int32, '/env_manager/iteration_num', self.iteration_num_cb, 10)
        self.create_subscription(Int32, '/env_manager/lap_num', self.lap_num_cb, 10)
        self.create_subscription(Odometry, ego_odom, self.ego_odom_cb, 10)
        self.create_subscription(Odometry, opp_odom, self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, ego_frenet, self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, opp_frenet, self.opp_frenet_cb, 10)
        self.create_subscription(Path, imm_path_topic, self.imm_cb, 10)
        self.create_subscription(String, '/imm_active', self.imm_active_cb, 10)

        # Publisher
        self.overtake_pub = self.create_publisher(Bool, '/data_logger/overtake_detected', 10)

        # Timer
        self.create_timer(0.05, self.log_to_csv)

        self.get_logger().info(f"DataLogger started | session={self.session_id}")

    def _start_new_iteration(self, iteration_num):
        """Create a new CSV file for the given iteration number"""
        self.current_csv_path = os.path.join(self.data_dir, f"iteration_{iteration_num}.csv")
        
        with open(self.current_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'lap',
                'ego_x', 'ego_y', 'ego_vel', 'ego_s', 'ego_d',
                'opp_x', 'opp_y', 'opp_vel', 'opp_s', 'opp_d',
                'rel_x', 'rel_y', 'rel_vel', 'rel_s', 'rel_d',
                'imm_trajectory', 'imm_active'
            ])
        
        self.get_logger().info(f"📝 Created new CSV: iteration_{iteration_num}.csv")

    # Subscriber callbacks
    def iteration_num_cb(self, msg):
        """Handle iteration number updates - create new CSV when iteration changes"""
        new_iteration = msg.data
        
        if new_iteration != self.current_iteration:
            self.get_logger().info(f"🔄 Iteration changed: {self.current_iteration} → {new_iteration}")
            self.current_iteration = new_iteration
            self._start_new_iteration(self.current_iteration)

    def imm_active_cb(self, msg):
        self.imm_active = (msg.data == "True")

    def lap_num_cb(self, msg):
        self.lap_num = msg.data

    def ego_odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vel = msg.twist.twist.linear.x

    def opp_odom_cb(self, msg):
        self.opp_x = msg.pose.pose.position.x
        self.opp_y = msg.pose.pose.position.y
        self.opp_vel = msg.twist.twist.linear.x

    def ego_frenet_cb(self, msg):
        self.ego_s = msg.x
        self.ego_d = msg.y

    def opp_frenet_cb(self, msg):
        self.opp_s = msg.x
        self.opp_d = msg.y

    def imm_cb(self, msg):
        waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.imm_trajectory = waypoints

    # Timer: write combined row
    def log_to_csv(self):
        if self.current_csv_path is None:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9

        rel_x = self.opp_x - self.ego_x
        rel_y = self.opp_y - self.ego_y
        rel_vel = self.opp_vel - self.ego_vel
        rel_s = self.opp_s - self.ego_s
        rel_d = self.opp_d - self.ego_d

        with open(self.current_csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, self.lap_num,
                self.ego_x, self.ego_y, self.ego_vel, self.ego_s, self.ego_d,
                self.opp_x, self.opp_y, self.opp_vel, self.opp_s, self.opp_d,
                rel_x, rel_y, rel_vel, rel_s, rel_d,
                self.imm_trajectory, self.imm_active
            ])

    # publisher
    def publish_overtake(self):
        msg = Bool()
        msg.data = True
        self.overtake_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()