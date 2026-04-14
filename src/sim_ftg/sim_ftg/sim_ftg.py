#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import MultiThreadedExecutor
import time # Added for a small delay to ensure messages are sent

class ReactiveFollowGap(Node):
    def __init__(self, car_name, scan_topic, drive_topic):
        super().__init__(f'{car_name}_follow_gap')
        self.car_name = car_name
        self.lidarscan_topic = scan_topic
        self.drive_topic = drive_topic

        # ... (All your parameters and state variables remain exactly the same) ...
        self.MAX_SPEED = 2.0
        self.EVASIVE_SPEED = 0.25
        self.processed_lidar = []
        self.prev_steer = 0.0

        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.subscription_ = self.create_subscription(LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.get_logger().info(f"FTG Controller for {self.car_name} initialized.")

    def stop_car(self):
        """Publishes a zero-speed command to safely stop the vehicle."""
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info(f"Emergency stop sent for {self.car_name}")

    # ... (Keep all your existing perception and callback methods here) ...
    # def preprocess_lidar, def lidar_callback, etc.
    
    def lidar_callback(self, scan_msg):
        # (Rest of your existing callback logic...)
        pass

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

if __name__ == '__main__':
    main()