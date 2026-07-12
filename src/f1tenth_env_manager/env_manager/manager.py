import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np
import random
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory


class EnvManager(Node):
    def __init__(self):
        super().__init__('env_manager')

        self.session_id = os.getenv("SESSION_ID", "1")
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        os.makedirs(self.results_dir, exist_ok=True)

        # Publishers & Subscribers
        self.ego_reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.opp_reset_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)
        
        # New Iteration Publisher
        self.iteration_pub = self.create_publisher(Int32, '/env_manager/iteration_num', 10)

        # State Variables
        self.ego_pose = [0.0, 0.0]
        self.opp_pose = [0.0, 0.0]

        # Frenet vars
        self.ego_frenet_s, self.ego_frenet_d = 0.0, 0.0
        self.opp_frenet_s, self.opp_frenet_d = 0.0, 0.0
        self.last_opp_ahead = False
        self.centerline = self._load_centerline()
        self.arc_lengths = self._compute_arc_lengths()

        # Iteration tracking
        self.iteration_num = 1
        self.session_finished = False
        
        # Reset tracking
        self.last_reset_time = 0.0
        self.log_counter = 0

        self.get_logger().info(f"Env Manager (Iteration Tracking) | SESSION_ID={self.session_id}")

    def _load_centerline(self):
        """Load the same centerline that frenet_node uses"""
        try:
            pkg_share = get_package_share_directory('frenet_frame_conv')
            csv_path = os.path.join(pkg_share, 'centerline_csv', 'Spielberg_map.csv')
            
            points = []
            with open(csv_path, "r") as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:
                        points.append([float(row[0]), float(row[1])])
            
            # Close the loop if not already closed
            if len(points) > 0 and not np.allclose(points[0], points[-1]):
                points.append(points[0])
            
            self.get_logger().info(f"Loaded {len(points)} centerline points from {csv_path}")
            return np.array(points)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load centerline: {e}")
            return np.array([[0, 0]])  # Fallback

    def _compute_arc_lengths(self):
        s = [0.0]
        for i in range(1, len(self.centerline)):
            s.append(s[-1] + np.linalg.norm(self.centerline[i] - self.centerline[i - 1]))
        return np.array(s)

    def _frenet_to_cartesian(self, s, d):
        idx = np.searchsorted(self.arc_lengths, s)
        if idx == 0:
            idx = 1
        if idx >= len(self.centerline):
            idx = len(self.centerline) - 1
        
        s_prev = self.arc_lengths[idx - 1]
        s_next = self.arc_lengths[idx]
        t = 0 if (s_next - s_prev) < 1e-6 else (s - s_prev) / (s_next - s_prev)
        
        p_prev = self.centerline[idx - 1]
        p_next = self.centerline[idx]
        point_on_line = p_prev + t * (p_next - p_prev)
        
        tangent = p_next - p_prev
        tangent_norm = np.linalg.norm(tangent)
        yaw = 0.0
        
        if tangent_norm > 1e-6:
            tangent = tangent / tangent_norm
            normal = np.array([-tangent[1], tangent[0]])
            final_point = point_on_line + d * normal
            yaw = math.atan2(tangent[1], tangent[0])
        else:
            final_point = point_on_line
        
        return float(final_point[0]), float(final_point[1]), float(yaw)

    def _check_overtake_and_reset(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        if (now - self.last_reset_time) < 2.0:
            return

        if (self.ego_frenet_s == 0.0 and self.opp_frenet_s == 0.0):
            return 
        
        max_s = self.arc_lengths[-1] if len(self.arc_lengths) > 0 else 400.0
        if self.ego_frenet_s > max_s + 10 or self.opp_frenet_s > max_s + 10:
            return
        
        s_diff = self.opp_frenet_s - self.ego_frenet_s
        if s_diff > max_s / 2.0:
            s_diff -= max_s
        elif s_diff < -max_s / 2.0:
            s_diff += max_s
            
        opp_ahead_now = s_diff > 0
        
        # DEBUG LOG
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f"Check: s_diff={s_diff:.1f}, opp_ahead={opp_ahead_now}"
            )
        
        RESET_DISTANCE_THRESHOLD = 5.0  # adjust this value if u want reset to happen sooner/later
        
        if opp_ahead_now and s_diff >= RESET_DISTANCE_THRESHOLD:
            self.get_logger().info(
                f"🏁 OVERTAKE COMPLETE! Opp is {s_diff:.1f}m ahead, resetting..."
            )
            self._reset_to_random_positions()
            self.last_opp_ahead = False  # Reset state after resetting
        elif opp_ahead_now and not self.last_opp_ahead:
            # Opponent just passed but not far enough yet
            self.get_logger().info(f"⚡ Overtake in progress... s_diff={s_diff:.1f}m")
        
        self.last_opp_ahead = opp_ahead_now


    def _reset_to_random_positions(self):
        max_s = self.arc_lengths[-1]
        opp_s = random.uniform(0, max_s)
        separation = random.uniform(3.0, 8.0)
        ego_s = (opp_s + separation) % max_s
        ego_d = random.uniform(-0.3, 0.3)
        
        ego_x, ego_y, ego_yaw = self._frenet_to_cartesian(ego_s, ego_d)
        opp_x, opp_y, opp_yaw = self._frenet_to_cartesian(opp_s, 0.0)
        
        now_msg = self.get_clock().now().to_msg()
        
        self.get_logger().info(f"✨ RESET: Ego s={ego_s:.1f}, Opp s={opp_s:.1f}")
        
        ego_msg = PoseWithCovarianceStamped()
        ego_msg.header.stamp = now_msg
        ego_msg.header.frame_id = 'map'
        ego_msg.pose.pose.position.x = ego_x
        ego_msg.pose.pose.position.y = ego_y
        ego_msg.pose.pose.orientation.z = math.sin(ego_yaw / 2.0)
        ego_msg.pose.pose.orientation.w = math.cos(ego_yaw / 2.0)
        self.ego_reset_pub.publish(ego_msg)
        
        opp_msg = PoseStamped()
        opp_msg.header.stamp = now_msg
        opp_msg.header.frame_id = 'map'
        opp_msg.pose.position.x = opp_x
        opp_msg.pose.position.y = opp_y
        opp_msg.pose.orientation.z = math.sin(opp_yaw / 2.0)
        opp_msg.pose.orientation.w = math.cos(opp_yaw / 2.0)
        self.opp_reset_pub.publish(opp_msg)

        self.last_reset_time = self.get_clock().now().nanoseconds / 1e9
        self.last_opp_ahead = False
        
        # --- INCREMENT ITERATION HERE ---
        self.iteration_num += 1
        self.get_logger().info(f"🔄 Starting Iteration {self.iteration_num}")


    def ego_odom_cb(self, msg):
        self.ego_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        # Publish the iteration number constantly so data loggers stay synced
        iter_msg = Int32()
        iter_msg.data = self.iteration_num
        self.iteration_pub.publish(iter_msg)

    def opp_odom_cb(self, msg):
        self.opp_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def ego_frenet_cb(self, msg):
        self.ego_frenet_s = msg.x
        self.ego_frenet_d = msg.y
        self.log_counter += 1
        if self.log_counter % 50 == 0:
            self.get_logger().info(f"EGO frenet: s={msg.x:.1f}")

    def opp_frenet_cb(self, msg):
        self.opp_frenet_s = msg.x
        self.opp_frenet_d = msg.y
        if self.log_counter % 50 == 0:
            self.get_logger().info(f"OPP frenet: s={msg.x:.1f}")
        self._check_overtake_and_reset()


def main(args=None):
    rclpy.init(args=args)
    node = EnvManager()
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