import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Pose2D
import csv
import os
import sys

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.session_id = os.getenv("SESSION_ID", "1")
        self.num_simulations = int(os.getenv("NUM_SIMULATIONS", "3"))
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        
        os.makedirs(self.results_dir, exist_ok=True)

        # State Variables
        self.current_sim = 1
        self.current_csv = None
        self.csv_file = None
        self.csv_writer = None
        
        self.lap_num = 0
        self.overtake_num = 0

        self.ego_x, self.ego_y = 0.0, 0.0
        self.ego_vel = 0.0
        self.ego_s, self.ego_d = 0.0, 0.0

        self.opp_x, self.opp_y = 0.0, 0.0
        self.opp_vel = 0.0
        self.opp_s, self.opp_d = 0.0, 0.0

        self.last_opp_ahead = False
        self.max_track_length = 359.85

        # Subscriptions
        self.create_subscription(Int32, '/env_manager/sim_num', self.sim_num_cb, 10)
        self.create_subscription(Int32, '/env_manager/lap_num', self.lap_num_cb, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)
        self.create_subscription(Bool, '/env_manager/reset', self.reset_cb, 10)
        self.create_subscription(Bool, '/env_manager/shutdown', self.shutdown_cb, 10)

        # Publisher
        self.overtake_pub = self.create_publisher(Bool, '/data_logger/overtake_detected', 10)

        # Timer
        self.create_timer(0.05, self.log_to_csv)

        # Initialize first simulation CSV
        self._init_csv_for_sim(1)

        self.get_logger().info(
            f"DataLogger started | Session={self.session_id} | "
            f"Num Sims={self.num_simulations} | Results Dir={self.results_dir}"
        )

    def _init_csv_for_sim(self, sim_num: int):
        """Initialize CSV file for a specific simulation"""
        if self.csv_file:
            self.csv_file.close()

        self.current_sim = sim_num
        self.current_csv = os.path.join(self.results_dir, f"simulation_{sim_num}.csv")
        
        self.get_logger().info(f"📝 Initializing CSV for simulation {sim_num}: {self.current_csv}")

        self.csv_file = open(self.current_csv, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'lap', 'overtake_num',
            'ego_x', 'ego_y', 'ego_vel', 'ego_s', 'ego_d',
            'opp_x', 'opp_y', 'opp_vel', 'opp_s', 'opp_d',
            'rel_x', 'rel_y', 'rel_vel', 'rel_s', 'rel_d'
        ])
        self.csv_file.flush()

    def sim_num_cb(self, msg):
        """Handle new simulation number"""
        if msg.data != self.current_sim:
            self.get_logger().info(f"🔄 Switching to simulation {msg.data}")
            self._init_csv_for_sim(msg.data)

    def reset_cb(self, msg):
        """Handle reset event"""
        if msg.data:
            self.get_logger().info("🔄 Reset signal received")
            self.overtake_num = 0          # Reset overtake counter for new simulation
            self.last_opp_ahead = False    # Reset overtake state tracking too

    def shutdown_cb(self, msg):
        """Handle graceful shutdown"""
        if msg.data:
            self.get_logger().info("📤 Shutdown signal received - closing files and exiting")
            if self.csv_file:
                self.csv_file.close()
            sys.exit(0)

    def _detect_and_increment_overtake(self) -> None:
        """Detect overtake event"""
        if self.ego_s == 0.0 or self.opp_s == 0.0:
            return

        delta = self.opp_s - self.ego_s
        max_s = self.max_track_length
        
        if delta > max_s / 2.0:
            delta -= max_s
        elif delta < -max_s / 2.0:
            delta += max_s

        opp_ahead_now = delta > 0.0

        if opp_ahead_now and not self.last_opp_ahead:
            self.overtake_num += 1
            self.get_logger().info(f"🏁 OVERTAKE #{self.overtake_num} DETECTED! Opp {delta:.1f}m ahead")
            
            msg = Bool()
            msg.data = True
            self.overtake_pub.publish(msg)

        self.last_opp_ahead = opp_ahead_now

    # Callbacks
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
        self._detect_and_increment_overtake()

    def log_to_csv(self):
        """Write snapshot to current simulation CSV"""
        if not self.csv_writer:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        rel_x = self.opp_x - self.ego_x
        rel_y = self.opp_y - self.ego_y
        rel_vel = self.opp_vel - self.ego_vel
        rel_s = self.opp_s - self.ego_s
        rel_d = self.opp_d - self.ego_d

        self.csv_writer.writerow([
            timestamp, self.lap_num, self.overtake_num,
            self.ego_x, self.ego_y, self.ego_vel, self.ego_s, self.ego_d,
            self.opp_x, self.opp_y, self.opp_vel, self.opp_s, self.opp_d,
            rel_x, rel_y, rel_vel, rel_s, rel_d
        ])
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()