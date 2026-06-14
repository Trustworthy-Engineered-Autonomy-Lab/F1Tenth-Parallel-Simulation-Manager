#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class ReactiveFollowGap(Node):
    def __init__(self, car_name, scan_topic, drive_topic):
        super().__init__(f'{car_name}_follow_gap')
        self.car_name = car_name
        self.lidarscan_topic = scan_topic
        self.drive_topic = drive_topic

        self.MAX_SPEED = 2.0
        self.MIN_SPEED = 0.45
        self.EVASIVE_SPEED = 0.25
        self.STEER_LIMIT = 0.4189
        self.GAP_THRESHOLD = 1.15
        self.GAP_WINDOW_DEG = 70.0
        self.BUBBLE_RADIUS = 18
        self.processed_lidar = []
        self.prev_steer = 0.0

        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.subscription_ = self.create_subscription(LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.get_logger().info(f"FTG Controller for {self.car_name} initialized.")

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def _clean_ranges(self, scan_msg):
        cleaned_ranges = []
        for value in scan_msg.ranges:
            if value is None or math.isnan(value) or math.isinf(value):
                cleaned_ranges.append(scan_msg.range_max)
            else:
                cleaned_ranges.append(self._clamp(value, scan_msg.range_min, scan_msg.range_max))
        return cleaned_ranges

    def _find_best_gap_index(self, ranges, scan_msg):
        if not ranges:
            return 0, 0.0

        center_index = len(ranges) // 2
        front_window = int((self.GAP_WINDOW_DEG / 180.0) * len(ranges) / 2)
        left = max(0, center_index - front_window)
        right = min(len(ranges), center_index + front_window)

        front_ranges = ranges[left:right]
        if not front_ranges:
            return center_index, 0.0

        closest_index = min(range(len(front_ranges)), key=lambda index: front_ranges[index])
        closest_distance = front_ranges[closest_index]

        bubble_radius = self.BUBBLE_RADIUS
        if closest_distance > 0.0:
            bubble_radius = int(max(self.BUBBLE_RADIUS, 25.0 / closest_distance))

        bubble_left = max(0, closest_index - bubble_radius)
        bubble_right = min(len(front_ranges), closest_index + bubble_radius + 1)

        safe_ranges = front_ranges[:]
        for index in range(bubble_left, bubble_right):
            safe_ranges[index] = 0.0

        best_start = None
        best_length = 0
        current_start = None

        for index, distance in enumerate(safe_ranges):
            if distance > self.GAP_THRESHOLD:
                if current_start is None:
                    current_start = index
            elif current_start is not None:
                current_length = index - current_start
                if current_length > best_length:
                    best_start = current_start
                    best_length = current_length
                current_start = None

        if current_start is not None:
            current_length = len(safe_ranges) - current_start
            if current_length > best_length:
                best_start = current_start
                best_length = current_length

        if best_start is None:
            best_index = len(front_ranges) // 2
        else:
            best_index = best_start + best_length // 2

        return left + best_index, front_ranges[best_index]

    def _build_drive_msg(self, scan_msg):
        ranges = self._clean_ranges(scan_msg)
        target_index, target_distance = self._find_best_gap_index(ranges, scan_msg)

        target_angle = scan_msg.angle_min + (target_index * scan_msg.angle_increment)
        steering = self._clamp(target_angle * 1.35, -self.STEER_LIMIT, self.STEER_LIMIT)
        steering = 0.65 * self.prev_steer + 0.35 * steering
        steering = self._clamp(steering, -self.STEER_LIMIT, self.STEER_LIMIT)
        self.prev_steer = steering

        if target_distance < 0.8:
            speed = self.EVASIVE_SPEED
        elif abs(steering) > 0.28:
            speed = 0.8
        elif abs(steering) > 0.15:
            speed = 1.2
        else:
            speed = self.MAX_SPEED

        if target_distance < 0.5:
            speed = min(speed, self.MIN_SPEED)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering)
        return drive_msg

    def stop_car(self):
        """Publishes a zero-speed command to safely stop the vehicle."""
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info(f"Emergency stop sent for {self.car_name}")

    def lidar_callback(self, scan_msg):
        drive_msg = self._build_drive_msg(scan_msg)
        self.publisher_.publish(drive_msg)
        self.get_logger().debug(
            f"{self.car_name}: speed={drive_msg.drive.speed:.2f}, steer={drive_msg.drive.steering_angle:.3f}"
        )

# ---------------------------------------------------------
# UPDATED MAIN: Handles Clean Exit
# ---------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    ego_node = ReactiveFollowGap("ego", "/scan", "/drive")
    opp_node = ReactiveFollowGap("opp", "/opp_scan", "/opp_drive")

    executor.add_node(ego_node)
    executor.add_node(opp_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        # 1. Log the interrupt
        print("\n[sim_ftg] Shutdown signal received. Stopping cars...")
        
        # 2. Explicitly call stop on both nodes
        ego_node.stop_car()
        opp_node.stop_car()
        
        # 3. Give ROS a split second to flush the messages to the network
        # Without this, the program might close the socket before the message leaves.
        time.sleep(0.2) 
        
    finally:
        ego_node.destroy_node()
        opp_node.destroy_node()
        rclpy.shutdown()


def ego_main(args=None):
    """Launch only the ego car FTG controller"""
    rclpy.init(args=args)
    
    ego_node = ReactiveFollowGap("ego", "/scan", "/drive")
    
    try:
        rclpy.spin(ego_node)
    except KeyboardInterrupt:
        print("\n[sim_ftg ego] Shutdown signal received. Stopping ego car...")
        ego_node.stop_car()
        time.sleep(0.2)
    finally:
        ego_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
