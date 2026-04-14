import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import threading
import csv
import os
import math
from datetime import datetime


class EnvManager(Node):
    def __init__(self):
        super().__init__('env_manager')

        # --- Session / Run Parameters ---
        self.session_id = os.getenv("SESSION_ID", "1")
        self.max_laps = int(os.getenv("MAX_LAPS", "3"))
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")

        os.makedirs(self.results_dir, exist_ok=True)

        # --- Output Files ---
        self.full_session_file = os.path.join(
            self.results_dir, f"full_session_data_{self.session_id}.csv"
        )
        self.summary_file = os.path.join(
            self.results_dir, f"session_summary_{self.session_id}.csv"
        )

        # --- Logging Setup ---
        with open(self.full_session_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'ego_x', 'ego_y',
                'opp_x', 'opp_y',
                'distance',
                'ego_speed', 'opp_speed',
                'ego_laps', 'opp_laps'
            ])

        # --- Publishers & Subscribers ---
        self.ego_reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.opp_reset_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)

        # --- State Variables ---
        self.ego_pose = [0.0, 0.0]
        self.opp_pose = [0.0, 0.0]
        self.ego_v, self.opp_v = 0.0, 0.0
        self.ego_last_x, self.opp_last_x = 0.0, 0.0
        self.ego_start_time, self.opp_start_time = None, None

        # --- Racing Metrics ---
        self.ego_laps = 0
        self.opp_laps = 0
        self.ego_laps_led = 0
        self.lap_winners = {}  # lap_number -> 'EGO' or 'OPP'
        self.session_finished = False
        self.winner = None
        self.start_timestamp = self.get_clock().now().nanoseconds / 1e9

        # --- Threads & Timers ---
        self.thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.thread.start()
        self.create_timer(0.05, self.log_to_csv)

        self.get_logger().info(
            f"Env Manager Initialized | SESSION_ID={self.session_id} | "
            f"MAX_LAPS={self.max_laps} | RESULTS_DIR={self.results_dir}"
        )

    def log_to_csv(self):
        if self.session_finished:
            return

        dist = math.sqrt(
            (self.ego_pose[0] - self.opp_pose[0]) ** 2 +
            (self.ego_pose[1] - self.opp_pose[1]) ** 2
        )
        timestamp = self.get_clock().now().nanoseconds / 1e9

        with open(self.full_session_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                self.ego_pose[0], self.ego_pose[1],
                self.opp_pose[0], self.opp_pose[1],
                dist,
                self.ego_v, self.opp_v,
                self.ego_laps, self.opp_laps
            ])

    def keyboard_listener(self):
        while rclpy.ok() and not self.session_finished:
            try:
                user_input = input().strip().lower()
                if user_input == 'r':
                    self.reset_cars()
                elif user_input == 'q':
                    self.get_logger().info("Manual shutdown requested.")
                    self.finish_session(reason="manual_shutdown")
            except Exception as e:
                self.get_logger().warning(f"Keyboard error: {e}")

    def reset_cars(self):
        self.get_logger().info("Resetting cars and metrics...")
        now = self.get_clock().now().nanoseconds / 1e9

        with open(self.full_session_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'ego_x', 'ego_y',
                'opp_x', 'opp_y',
                'distance',
                'ego_speed', 'opp_speed',
                'ego_laps', 'opp_laps'
            ])

        self.ego_laps, self.opp_laps, self.ego_laps_led = 0, 0, 0
        self.lap_winners = {}
        self.ego_start_time, self.opp_start_time = now, now
        self.ego_last_x, self.opp_last_x = 0.0, 0.0
        self.session_finished = False
        self.winner = None
        self.start_timestamp = now

        ego_msg = PoseWithCovarianceStamped()
        ego_msg.header.frame_id = 'map'
        ego_msg.pose.pose.position.x = 0.0
        ego_msg.pose.pose.position.y = 0.0
        ego_msg.pose.pose.orientation.w = 1.0
        self.ego_reset_pub.publish(ego_msg)

        opp_msg = PoseStamped()
        opp_msg.header.frame_id = 'map'
        opp_msg.pose.position.x = 0.7
        opp_msg.pose.position.y = 0.7
        opp_msg.pose.orientation.w = 1.0
        self.opp_reset_pub.publish(opp_msg)

    def finish_session(self, reason="max_laps_reached"):
        if self.session_finished:
            return

        self.session_finished = True
        end_timestamp = self.get_clock().now().nanoseconds / 1e9
        duration = end_timestamp - self.start_timestamp

        if self.ego_laps >= self.max_laps and self.opp_laps >= self.max_laps:
            self.winner = "TIE"
        elif self.ego_laps >= self.max_laps:
            self.winner = "EGO"
        elif self.opp_laps >= self.max_laps:
            self.winner = "OPP"
        else:
            self.winner = "UNKNOWN"

        self.get_logger().info(
            f"Session {self.session_id} finished | reason={reason} | winner={self.winner}"
        )

        with open(self.summary_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['session_id', 'max_laps', 'winner', 'ego_laps', 'opp_laps',
                             'ego_laps_led', 'duration_sec', 'reason', 'finished_at'])
            writer.writerow([
                self.session_id,
                self.max_laps,
                self.winner,
                self.ego_laps,
                self.opp_laps,
                self.ego_laps_led,
                duration,
                reason,
                datetime.utcnow().isoformat()
            ])

        # End ROS spin cleanly
        self.get_logger().info("Shutting down ROS for this session.")
        rclpy.shutdown()

    def maybe_finish_after_lap(self):
        if self.session_finished:
            return

        if self.ego_laps >= self.max_laps or self.opp_laps >= self.max_laps:
            self.finish_session(reason="max_laps_reached")

    def check_lap_status(self, car_label, curr_x, last_x, start_time):
        updated_start_time = start_time

        if last_x < 0 and curr_x >= 0:
            now = self.get_clock().now().nanoseconds / 1e9

            if start_time:
                if car_label == "EGO":
                    self.ego_laps += 1
                    current_lap_num = self.ego_laps
                else:
                    self.opp_laps += 1
                    current_lap_num = self.opp_laps

                if current_lap_num not in self.lap_winners:
                    self.lap_winners[current_lap_num] = car_label
                    if car_label == "EGO":
                        self.ego_laps_led += 1

                self.get_logger().info(f"--- {car_label} Finished Lap {current_lap_num} ---")
                self.get_logger().info(
                    f"Stats: Ego Laps: {self.ego_laps} | "
                    f"Opp Laps: {self.opp_laps} | "
                    f"Ego Laps Led: {self.ego_laps_led}"
                )
                self.get_logger().info("------------------------------------------------")

                self.maybe_finish_after_lap()

            updated_start_time = now

        return updated_start_time, curr_x

    def ego_odom_cb(self, msg):
        if self.session_finished:
            return
        self.ego_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.ego_v = msg.twist.twist.linear.x
        self.ego_start_time, self.ego_last_x = self.check_lap_status(
            "EGO", self.ego_pose[0], self.ego_last_x, self.ego_start_time
        )

    def opp_odom_cb(self, msg):
        if self.session_finished:
            return
        self.opp_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.opp_v = msg.twist.twist.linear.x
        self.opp_start_time, self.opp_last_x = self.check_lap_status(
            "OPP", self.opp_pose[0], self.opp_last_x, self.opp_start_time
        )


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