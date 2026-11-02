#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from scipy.interpolate import splev
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

class OvertakeInterceptorNode(Node):
    def __init__(self):
        super().__init__('overtake_interceptor')
        self.declare_parameter('racetrack_csv', '/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv')
        self.track = RacetrackUtilities(self.get_parameter('racetrack_csv').value)
        
        # Subscriptions
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.scan_callback, 10)
        self.opp_odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_callback, 10)
        self.imm_sub = self.create_subscription(Path, '/imm_path', self.imm_path_callback, 10)
        
        # Publishers
        self.overtake_path_pub = self.create_publisher(Path, '/overtake_spline', 10)
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.marker_pub = self.create_publisher(Marker, '/overtake_marker', 10)
        
        # Ego state
        self.ego_x = self.ego_y = self.ego_yaw = self.ego_speed = 0.0
        self.ego_data_received = False
        
        # Opponent state
        self.opp_x = self.opp_y = self.opp_yaw = self.opp_speed = 0.0
        self.opp_data_received = False
        
        # IMM state
        self.imm_data_received = False
        
        # Scan data
        self.scan_ranges = np.array([])
        self.scan_angle_min = self.scan_angle_inc = 0.0
        
        # Overtake parameters
        self.car_width = 0.30
        # FIX 1: Reduced from 1.0 to 0.85 to keep spline tighter to racing line
        self.lateral_clearance = 0.60
        self.overtake_distance = 6.0
        self.path_resolution = 50
        
        # Distance window
        # Start planning while there is still room to change lanes. The default
        # spawn gap is about 4.4 m, outside the old 3.5 m trigger window.
        self.declare_parameter('trigger_distance', 6.0)
        self.max_display_distance = self.get_parameter('trigger_distance').value
        self.min_ego_distance = 0.0
        
        # State machine
        self.current_mode = "FTG"
        self.current_side = "left"
        self.locked_spline = None
        self.debug_counter = 0
        
        self.create_timer(0.05, self.continuous_spline_update)
        self.get_logger().info("🎯 PLANNER Started - SAFE WALL CLEARANCE ACTIVE!")

    def set_mode(self, new_mode):
        if new_mode != self.current_mode:
            self.get_logger().info(f"\n🚦 TRANSITION: {self.current_mode} ➡️ {new_mode}\n")
            self.current_mode = new_mode
            self.mode_pub.publish(String(data=self.current_mode))

    def ego_odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.ego_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.ego_data_received = True

    def opp_odom_callback(self, msg):
        self.opp_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.opp_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        # The overtaker's own pose must always come from its own odometry.
        self.opp_x = msg.pose.pose.position.x
        self.opp_y = msg.pose.pose.position.y
        self.opp_data_received = True

    def imm_path_callback(self, msg):
        if len(msg.poses) > 0:
            # /imm_path predicts the defending ego, not the overtaking opponent.
            self.predicted_ego_path = msg.poses
            self.imm_data_received = True

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def continuous_spline_update(self):
        self.debug_counter += 1
        if not (self.ego_data_received and self.opp_data_received):
            return

        if not self.is_obstacle_ahead():
            self.publish_empty_path()
            self.locked_spline = None
            self.set_mode("FTG")
            return

        if self.locked_spline is not None:
            closest = int(np.argmin(np.linalg.norm(
                self.locked_spline - [self.opp_x, self.opp_y], axis=1)))
            if closest >= len(self.locked_spline) - 3:
                self.locked_spline = None
                self.publish_empty_path()
                self.set_mode('FTG')
                return
            # A defender can move into a previously clear lane. Revalidate the
            # remaining path instead of freezing it through that new obstacle.
            if self.validate_and_clip_path(self.locked_spline[closest:]) is not None:
                self.publish_path(self.locked_spline)
                self.set_mode("OVERTAKE_ACTIVE")
                return
            self.locked_spline = None

        ego_pos = np.array([self.ego_x, self.ego_y])
        opp_pos = np.array([self.opp_x, self.opp_y])
        distance = np.linalg.norm(opp_pos - ego_pos)

        if distance < self.max_display_distance and distance >= self.min_ego_distance:
            self.update_overtake_side()
            new_spline = self.generate_safe_spline()
            
            if new_spline is not None:
                self.locked_spline = new_spline
                self.publish_path(new_spline)
                self.set_mode("OVERTAKE_ACTIVE")
                self.publish_continuous_marker()
            else:
                self.publish_empty_path()
                self.set_mode("FTG")
                if self.debug_counter % 20 == 0:
                    self.get_logger().warning("🚫 Walls blocking pass - trying other side!")
        else:
            self.publish_empty_path()
            self.set_mode("FTG")

    def is_obstacle_ahead(self):
        dx = self.ego_x - self.opp_x
        dy = self.ego_y - self.opp_y
        hx = math.cos(self.opp_yaw)
        hy = math.sin(self.opp_yaw)
        return (dx * hx + dy * hy) > -1.5

    def update_overtake_side(self):
        opp_forward = np.array([math.cos(self.opp_yaw), math.sin(self.opp_yaw)])
        to_ego = np.array([self.ego_x - self.opp_x, self.ego_y - self.opp_y])
        cross_product = np.cross(opp_forward, to_ego)
        
        if self.current_side == "left" and cross_product < -0.8:
            self.current_side = "right"
        elif self.current_side == "right" and cross_product > 0.8:
            self.current_side = "left"
        elif self.current_side not in ["left", "right"]:
            self.current_side = "left" if cross_product > 0 else "right"

    def generate_safe_spline(self):
        spline = self.try_generate_spline(self.current_side)
        if spline is not None: 
            return spline
        
        # If current side is blocked by wall, try the other side
        other_side = "right" if self.current_side == "left" else "left"
        spline = self.try_generate_spline(other_side)
        if spline is not None:
            self.current_side = other_side
            self.get_logger().info(f"🔄 Switched to {self.current_side.upper()} to avoid wall!")
            return spline
        return None

    def try_generate_spline(self, side):
        # Plan along the track, rather than extending a straight Bezier beyond
        # the next corner. Both cars are localized in the same Frenet frame.
        s_opp, d_opp = self.track.convert_to_frenet(self.opp_x, self.opp_y)
        s_ego, d_ego = self.track.convert_to_frenet(self.ego_x, self.ego_y)
        s_ego = self.track.unwrap_s(s_ego, s_opp)
        gap = s_ego - s_opp
        lane_d = d_ego + (self.lateral_clearance if side == 'left' else -self.lateral_clearance)
        distance = np.clip(gap + self.overtake_distance, 6.0, 12.0)
        s = np.linspace(s_opp, s_opp + distance, self.path_resolution)
        # Complete the lane change before reaching the defending car.
        u = np.clip((s - s_opp) / max(2.2, gap * 0.7), 0.0, 1.0)
        blend = u * u * (3.0 - 2.0 * u)
        d = d_opp + (lane_d - d_opp) * blend
        path = self.track.convert_to_cartesian_arr(s, d)
        return self.validate_and_clip_path(path)

    def validate_and_clip_path(self, path_points):
        path = np.asarray(path_points, dtype=float)
        if len(path) < 2 or not np.isfinite(path).all():
            return None
        # Check all four footprint corners against the known simulator track.
        # A laser return on the ego also occludes valid road beyond it; treating
        # that whole radial shadow as a wall used to reject usable passing lanes.
        tangent = np.gradient(path, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        arc_steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
        if np.any(np.abs(np.diff(heading)) / np.maximum(arc_steps, 1e-6) > 1.2):
            return None
        for front in (-1.0, 1.0):
            for left in (-1.0, 1.0):
                corners = path + front * 0.29 * tangent + left * 0.155 * normal
                sd = self.track.convert_to_frenet_arr(corners)
                lw, rw = splev(sd[:, 0] / self.track.arclength, self.track.racetrack_widths_spline)
                if np.any(sd[:, 1] > np.asarray(lw) - 0.03) or np.any(sd[:, 1] < -np.asarray(rw) + 0.03):
                    return None
        # Check the moving defender at the time the overtaker reaches each
        # point, including its current lateral motion (bounded to one second).
        own_s, _ = self.track.convert_to_frenet(self.opp_x, self.opp_y)
        ego_s, ego_d = self.track.convert_to_frenet(self.ego_x, self.ego_y)
        ego_s = self.track.unwrap_s(ego_s, own_s)
        sd = self.track.convert_to_frenet_arr(path)
        path_s = self.track.unwrap_s(sd[:, 0], own_s)
        distance = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))))
        times = distance / 2.34
        tx, ty = splev((ego_s % self.track.arclength) / self.track.arclength,
                       self.track.centerline_spline, der=1)
        lateral_speed = self.ego_speed * math.sin(self.ego_yaw - math.atan2(ty, tx))
        predicted_s = ego_s + self.ego_speed * times
        predicted_d = ego_d + lateral_speed * np.minimum(times, 1.0)
        if np.any((np.abs(path_s - predicted_s) < 0.75) & (np.abs(sd[:, 1] - predicted_d) < 0.45)):
            return None
        return path

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.1
            
            if i < len(path_points) - 1:
                yaw = math.atan2(path_points[i+1][1] - point[1], path_points[i+1][0] - point[0])
            else:
                yaw = 0.0
                
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
            
        self.overtake_path_pub.publish(path_msg)

    def publish_empty_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.overtake_path_pub.publish(path_msg)

    def publish_continuous_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "overtake_origin"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.opp_x)
        marker.pose.position.y = float(self.opp_y)
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        if self.current_side == "left":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(marker)

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
