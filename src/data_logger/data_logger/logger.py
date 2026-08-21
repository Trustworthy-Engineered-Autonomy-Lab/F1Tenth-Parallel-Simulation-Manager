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

        # Storage - SINGLE CSV
        self.session_id = os.getenv("SESSION_ID", "1")
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        self.data_dir = os.path.join(self.results_dir, "data")
        os.makedirs(self.data_dir, exist_ok=True)

        self.session_csv = os.path.join(self.data_dir, f"session_{self.session_id}_data.csv")
        self._init_csv()

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

        # Overtake tracking
        self.overtake_num = 0  # Starts at 0, increments on each overtake
        self.last_opp_ahead = False
        self.last_overtake_time = 0.0
        self.max_track_length = 359.85  # Default track length

        # Topics
        ego_odom = '/ego_racecar/odom' if on_sim else 'TODO'
        opp_odom = '/opp_racecar/odom' if on_sim else 'TODO'
        ego_frenet = '/ego_racecar/frenet'
        opp_frenet = '/opp_racecar/frenet'
        imm_path_topic = '/imm_path'

        # Subscriptions (NO iteration_num - single CSV only)
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

        self.get_logger().info(f"DataLogger started | session={self.session_id} | Single continuous CSV with overtake tracking")

    def _init_csv(self):
        """Initialize single session CSV file with overtake_num column"""
        with open(self.session_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'lap', 'overtake_num',
                'ego_x', 'ego_y', 'ego_vel', 'ego_s', 'ego_d',
                'opp_x', 'opp_y', 'opp_vel', 'opp_s', 'opp_d',
                'rel_x', 'rel_y', 'rel_vel', 'rel_s', 'rel_d',
                'imm_trajectory', 'imm_active'
            ])

    def _detect_and_increment_overtake(self) -> None:
        """
        Increments ``self.overtake_num`` exactly when the opponent car
        goes from behind to ahead of the ego car.
        """

        # guard against stale data
        if self.ego_s == 0.0 or self.opp_s == 0.0:
            return

        # raw relative distance (opponent – ego)
        delta = self.opp_s - self.ego_s

        # wrap‑around to keep the difference in [‑L/2 , +L/2]
        max_s = self.max_track_length
        if delta >  max_s / 2.0:
            delta -= max_s           # opponent wrapped past 0
        elif delta < -max_s / 2.0:
            delta += max_s           # ego wrapped past 0

        # who is ahead *after* we fixed wrap‑around?
        opp_ahead_now = delta > 0.0   # True if opponent is ahead

        # DEBUG – you can delete this line in production
        print(f"[DEBUG] ego_s={self.ego_s:.2f}  opp_s={self.opp_s:.2f}  "
              f"delta={delta:.2f}  opp_ahead_now={opp_ahead_now}  "
              f"last_opp_ahead={self.last_opp_ahead}")

        # crossing test: from behind (False) → ahead (True)
        if opp_ahead_now and not self.last_opp_ahead:
            self.overtake_num += 1

            # log the event – appears in *rosout*
            self.get_logger().info(
                f"🏁 OVERTAKE #{self.overtake_num} DETECTED! Opp is {delta:.1f} m ahead"
            )

            # publish a Bool so other nodes can react
            msg = Bool()
            msg.data = True
            self.overtake_pub.publish(msg)

        # update the state flag for the next callback
        self.last_opp_ahead = opp_ahead_now

    # Subscriber callbacks
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
        
        # Check for overtakes whenever opponent position updates
        self._detect_and_increment_overtake()

    def imm_cb(self, msg):
        waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.imm_trajectory = waypoints

    # Timer: write to single CSV
    def log_to_csv(self):
        # Snapshot all values atomically to prevent mid-update writes
        timestamp   = self.get_clock().now().nanoseconds / 1e9
        lap         = self.lap_num
        overtake    = self.overtake_num
        ego_x       = self.ego_x
        ego_y       = self.ego_y
        ego_vel     = self.ego_vel
        ego_s       = self.ego_s
        ego_d       = self.ego_d
        opp_x       = self.opp_x
        opp_y       = self.opp_y
        opp_vel     = self.opp_vel
        opp_s       = self.opp_s
        opp_d       = self.opp_d
        imm_traj    = list(self.imm_trajectory)
        imm_active  = self.imm_active

        rel_x   = opp_x - ego_x
        rel_y   = opp_y - ego_y
        rel_vel = opp_vel - ego_vel
        rel_s   = opp_s - ego_s
        rel_d   = opp_d - ego_d

        with open(self.session_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, lap, overtake,
                ego_x, ego_y, ego_vel, ego_s, ego_d,
                opp_x, opp_y, opp_vel, opp_s, opp_d,
                rel_x, rel_y, rel_vel, rel_s, rel_d,
                imm_traj, imm_active
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