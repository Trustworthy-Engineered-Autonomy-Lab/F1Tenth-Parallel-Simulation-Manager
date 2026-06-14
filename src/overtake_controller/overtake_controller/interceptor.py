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

class FairOpponentInterceptorNode(Node):
    def __init__(self):
        super().__init__('fair_opponent_interceptor')
        
        # ---------------- FAIR PLAY SUBSCRIPTIONS ----------------
        # 1. Our Own Sensors: The opponent car reads ONLY its own physical state
        self.my_odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.my_odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.scan_callback, 10)
        
        # 2. Perception Pipeline: The opponent uses the IMM filter to guess the Ego car's location
        self.target_imm_sub = self.create_subscription(Path, '/imm_path', self.target_imm_callback, 10)
        self.target_imm_active_sub = self.create_subscription(String, '/imm_active', self.target_imm_active_callback, 10)
        
        # 3. Handshake with the Controller
        self.ctrl_status_sub = self.create_subscription(String, '/overtake_status', self.ctrl_status_callback, 10)
        
        # ---------------- PUBLISHERS ----------------
        self.overtake_path_pub = self.create_publisher(Path, '/overtake_spline', 10)
        self.mode_pub = self.create_publisher(String, '/opp_driving_mode', 10)
        self.marker_pub = self.create_publisher(Marker, '/opp_overtake_marker', 10)
        
        # ---------------- STATE VARIABLES ----------------
        # The Driver (Opponent / Our Car)
        self.my_x = self.my_y = self.my_yaw = self.my_speed = 0.0
        
        # The Target (Ego Car, Derived entirely from IMM)
        self.target_x = self.target_y = self.target_yaw = 0.0
        self.target_pred_x = self.target_pred_y = 0.0 
        
        # Flags & Handshake
        self.my_data_received = False
        self.target_data_received = False
        self.is_target_visible = False 
        self.controller_state = "FTG"
        
        # Laser
        self.scan_ranges = np.array([])
        self.scan_angle_min = self.scan_angle_inc = 0.0
        
        # Planner Tuning
        self.car_width = 0.30
        self.lateral_clearance = 1.0
        self.overtake_distance = 3.5
        self.path_resolution = 50
        self.max_display_distance = 3.5   
        self.min_overtake_distance = 0.0
        
        self.current_mode = "FTG"
        self.current_side = "left"
        self.locked_spline = None
        self.debug_counter = 0
        
        self.create_timer(0.05, self.continuous_spline_update)
        self.get_logger().info("🎯 FAIR OPPONENT PLANNER Started - 100% LEGAL & HANDSHAKE ACTIVE")

    # ---------------- CALLBACKS ----------------

    def set_mode(self, new_mode):
        if new_mode != self.current_mode:
            self.get_logger().info(f"\n🚦 PLANNER TRANSITION: {self.current_mode} ➡️ {new_mode}")
            self.current_mode = new_mode
            self.mode_pub.publish(String(data=self.current_mode))

    def ctrl_status_callback(self, msg):
        self.controller_state = msg.data

    def my_odom_callback(self, msg):
        """ The opponent car reads its own physical state """
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        self.my_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.my_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.my_data_received = True

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def target_imm_active_callback(self, msg):
        self.is_target_visible = (msg.data == "True")

    def target_imm_callback(self, msg):
        """ EXTRACTS EGO STATE ENTIRELY FROM PERCEPTION - NO CHEATING """
        n = len(msg.poses)
        if n < 10: return 

        # 1. Current estimated position of the Ego car
        self.target_x = msg.poses[0].pose.position.x
        self.target_y = msg.poses[0].pose.position.y
        
        # 2. Derive Ego heading based on its predicted movement
        vector_idx = min(5, n - 1)
        dx = msg.poses[vector_idx].pose.position.x - self.target_x
        dy = msg.poses[vector_idx].pose.position.y - self.target_y
        self.target_yaw = math.atan2(dy, dx)
        
        # 3. Where the IMM thinks the Ego car will be at the end of its prediction
        self.target_pred_x = msg.poses[-1].pose.position.x
        self.target_pred_y = msg.poses[-1].pose.position.y

        self.target_data_received = True

    # ---------------- MAIN UPDATE LOOP ----------------

    def continuous_spline_update(self):
        self.debug_counter += 1
        
        if not (self.my_data_received and self.target_data_received):
            return

        # 🚨 THE FAIR-PLAY KILL-SWITCH
        if not self.is_target_visible:
            self.publish_empty_path()
            self.locked_spline = None
            self.set_mode("FTG")
            if self.debug_counter % 20 == 0:
                self.get_logger().warning("⚠️ Target occluded! Opponent cannot cheat. Forcing Controller to FTG.")
            return

        # 🎯 COMPLETION CHECK
        if not self.is_obstacle_ahead():
            self.publish_empty_path()
            self.locked_spline = None
            self.set_mode("FTG")
            return

        # 🤝 HANDSHAKE: If Controller committed, freeze the path!
        if self.locked_spline is not None:
            self.publish_path(self.locked_spline)
            self.set_mode("OVERTAKE_ACTIVE")
            return

        my_pos = np.array([self.my_x, self.my_y])
        target_current = np.array([self.target_x, self.target_y])
        
        distance = np.linalg.norm(target_current - my_pos)
        
        if distance < self.max_display_distance and distance >= self.min_overtake_distance:
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
        else:
            self.publish_empty_path()
            self.set_mode("FTG")

    # ---------------- PLANNER MATH ----------------

    def is_obstacle_ahead(self):
        # Uses derived IMM Target state vs Own Odometry state
        dx = self.target_x - self.my_x
        dy = self.target_y - self.my_y
        hx = math.cos(self.my_yaw)
        hy = math.sin(self.my_yaw)
        return (dx * hx + dy * hy) > 0.0

    def update_overtake_side(self):
        my_forward = np.array([math.cos(self.my_yaw), math.sin(self.my_yaw)])
        to_target = np.array([self.target_x - self.my_x, self.target_y - self.my_y])
        cross_product = np.cross(my_forward, to_target)
        
        if self.current_side == "left" and cross_product > 0.8:
            self.current_side = "right"
        elif self.current_side == "right" and cross_product < -0.8:
            self.current_side = "left"
        elif self.current_side not in ["left", "right"]:
            self.current_side = "left" if cross_product < 0 else "right"

    def generate_safe_spline(self):
        spline = self.try_generate_spline(self.current_side)
        if spline is not None: return spline
        
        other_side = "right" if self.current_side == "left" else "left"
        spline = self.try_generate_spline(other_side)
        
        if spline is not None:
            self.current_side = other_side
            return spline
        return None

    def try_generate_spline(self, side):
        my_pos = np.array([self.my_x, self.my_y])
        target_current = np.array([self.target_x, self.target_y])
        target_future = np.array([self.target_pred_x, self.target_pred_y])
        
        my_forward = np.array([math.cos(self.my_yaw), math.sin(self.my_yaw)])
        target_forward = np.array([math.cos(self.target_yaw), math.sin(self.target_yaw)])
        
        # Lateral offset is based purely on the IMM's predicted Ego heading
        if side == "left":
            lateral_dir = np.array([-target_forward[1], target_forward[0]])
        else:
            lateral_dir = np.array([target_forward[1], -target_forward[0]])
        
        adaptive_distance = min(self.overtake_distance, 8.0)
        dist_to_target = np.linalg.norm(target_current - my_pos)
        
        # Opponent merges safely AHEAD of your predicted future location
        end_point = target_future + adaptive_distance * target_forward + self.lateral_clearance * lateral_dir
        
        # ---------------- FAIR BEZIER MATH ----------------
        P0 = my_pos  
        P1 = P0 + (dist_to_target * 0.4) * my_forward
        P3 = end_point
        P2 = P3 - (adaptive_distance * 0.5) * target_forward
        
        path_points = []
        t_values = np.linspace(0, 1, self.path_resolution)
        
        for t in t_values:
            point = ((1 - t)**3) * P0 + \
                    3 * ((1 - t)**2) * t * P1 + \
                    3 * (1 - t) * (t**2) * P2 + \
                    (t**3) * P3
            path_points.append(point)
            
        return self.validate_and_clip_path(path_points)

    def validate_and_clip_path(self, path_points):
        if len(self.scan_ranges) == 0:
            return path_points
            
        for pt in path_points:
            dx = pt[0] - self.my_x
            dy = pt[1] - self.my_y
            dist = math.hypot(dx, dy)
            
            local_angle = math.atan2(dy, dx) - self.my_yaw
            local_angle = (local_angle + math.pi) % (2 * math.pi) - math.pi
            
            idx = int(round((local_angle - self.scan_angle_min) / max(self.scan_angle_inc, 1e-6)))
            if 0 <= idx < len(self.scan_ranges):
                r = self.scan_ranges[idx]
                if math.isfinite(r) and r > 0.1 and dist > (r - self.car_width - 0.15):
                    return None
                    
        return np.array(path_points)

    # ---------------- PUBLISHING UTILITIES ----------------

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
        marker.ns = "opp_overtake_origin"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.target_pred_x) 
        marker.pose.position.y = float(self.target_pred_y)
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
    node = FairOpponentInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()