import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import threading
import csv
import os
import math
from datetime import datetime
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import numpy as np
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



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
        self.overtaking_events_file = os.path.join(
            self.results_dir, "overtaking_events.csv"
        )
        self.summary_file = os.path.join(
            self.results_dir, f"session_summary_{self.session_id}.csv"
        )

        # --- Logging Setup ---
        self._initialize_result_files()

        # --- Publishers & Subscribers ---
        self.ego_reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.opp_reset_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)
        self.lap_pub = self.create_publisher(Int32, '/env_manager/lap_num', 10)


        # --- State Variables ---
        self.ego_pose = [0.0, 0.0]
        self.opp_pose = [0.0, 0.0]
        self.ego_v, self.opp_v = 0.0, 0.0
        self.ego_last_x, self.opp_last_x = 0.0, 0.0
        self.ego_start_time, self.opp_start_time = None, None

        # --- FRENET vars --- 
        self.ego_frenet_s, self.ego_frenet_d = 0.0, 0.0
        self.opp_frenet_s, self.opp_frenet_d = 0.0, 0.0
        self.last_opp_ahead = False  # Track if opp was ahead last check
        # centerline for frenet->cartesian conversion
        self.centerline = self._load_centerline()
        self.arc_lengths = self._compute_arc_lengths()

        # --- Racing Metrics ---
        self.ego_laps = 0
        self.opp_laps = 0
        self.ego_laps_led = 0
        self.lap_winners = {}  # lap_number -> 'EGO' or 'OPP'
        self.overtaking_event_count = 0
        self.current_leader = None
        self.pending_leader = None
        self.pending_leader_count = 0
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

    def _initialize_result_files(self):
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

        with open(self.overtaking_events_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'overtaking_event_number',
                'overtaking_car',
                'lap'
            ])

    def _reset_overtake_state(self):
        self.overtaking_event_count = 0
        self.current_leader = None
        self.pending_leader = None
        self.pending_leader_count = 0

    def _progress_score(self, lap_count, x_position):
        return lap_count + (x_position / 1000.0)

    def _determine_leader(self):
        ego_score = self._progress_score(self.ego_laps, self.ego_pose[0])
        opp_score = self._progress_score(self.opp_laps, self.opp_pose[0])
        score_delta = ego_score - opp_score

        if abs(score_delta) < 0.02:
            return None

        return 'EGO' if score_delta > 0 else 'OPP'

    def _record_overtake_event(self, overtaking_car, timestamp):
        self.overtaking_event_count += 1
        lap = self.ego_laps if overtaking_car == 'EGO' else self.opp_laps

        with open(self.overtaking_events_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                self.overtaking_event_count,
                overtaking_car,
                lap,
            ])

        self.get_logger().info(
            f"Overtaking event #{self.overtaking_event_count}: {overtaking_car} ahead on lap {lap}"
        )

    def _update_overtaking_events(self, timestamp):
        leader = self._determine_leader()

        if leader is None:
            self.pending_leader = None
            self.pending_leader_count = 0
            return

        if self.current_leader is None:
            self.current_leader = leader
            self.pending_leader = None
            self.pending_leader_count = 0
            return

        if leader == self.current_leader:
            self.pending_leader = None
            self.pending_leader_count = 0
            return

        if leader != self.pending_leader:
            self.pending_leader = leader
            self.pending_leader_count = 1
            return

        self.pending_leader_count += 1
        if self.pending_leader_count >= 2:
            self._record_overtake_event(leader, timestamp)
            self.current_leader = leader
            self.pending_leader = None
            self.pending_leader_count = 0

    ## frenet
    def _load_centerline(self):
        """Load the same centerline that frenet_node uses"""
        # Adjust path if needed to match your centerline CSV location
        csv_path = "/sim_ws/src/frenet_frame_conv/centerline_csv/spielberg_centerline.csv"
        points = []
        try:
            with open(csv_path, "r") as file:
                import csv
                reader = csv.reader(file)
                for row in reader:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        points.append([x, y])
                    except:
                        continue
            self.get_logger().info(f"Loaded {len(points)} centerline points")
        except Exception as e:
            self.get_logger().error(f"Failed to load centerline: {e}")
            points = [[0, 0]]  # Fallback
        return np.array(points)

    def _compute_arc_lengths(self):
        """Compute cumulative arc length along centerline"""
        s = [0.0]
        for i in range(1, len(self.centerline)):
            distance = np.linalg.norm(self.centerline[i] - self.centerline[i - 1])
            s.append(s[-1] + distance)
        return np.array(s)

    def _frenet_to_cartesian(self, s, d):
        """Convert frenet (s, d) to cartesian (x, y)"""
        # Find the two points on centerline that bracket s
        idx = np.searchsorted(self.arc_lengths, s)
        if idx == 0:
            idx = 1
        if idx >= len(self.centerline):
            idx = len(self.centerline) - 1
        
        # Interpolate between points
        s_prev = self.arc_lengths[idx - 1]
        s_next = self.arc_lengths[idx]
        if s_next - s_prev < 1e-6:
            t = 0
        else:
            t = (s - s_prev) / (s_next - s_prev)
        
        p_prev = self.centerline[idx - 1]
        p_next = self.centerline[idx]
        
        # Point on centerline
        point_on_line = p_prev + t * (p_next - p_prev)
        
        # Perpendicular offset for d
        tangent = p_next - p_prev
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm > 1e-6:
            tangent = tangent / tangent_norm
            normal = np.array([-tangent[1], tangent[0]])  # 90 degree rotation
            final_point = point_on_line + d * normal
        else:
            final_point = point_on_line
        
        return float(final_point[0]), float(final_point[1])


    def _check_overtake_and_reset(self):
        """Detect when opponent overtakes ego and reset to new random positions"""
        # Skip check until both cars have valid frenet data (not both zero)
        if (self.ego_frenet_s == 0.0 and self.opp_frenet_s == 0.0):
            return  # Both zero = not initialized yet
        
        # Also skip if values seem invalid (way too large)
        max_s = self.arc_lengths[-1] if len(self.arc_lengths) > 0 else 400.0
        if self.ego_frenet_s > max_s + 10 or self.opp_frenet_s > max_s + 10:
            return
        
        # Opponent is ahead if their s is greater
        opp_ahead_now = self.opp_frenet_s > self.ego_frenet_s
        
        # DEBUG: Log every 2 seconds
        self.get_logger().info(
            f"Frenet check: ego_s={self.ego_frenet_s:.1f}, opp_s={self.opp_frenet_s:.1f}, "
            f"opp_ahead_now={opp_ahead_now}, last={self.last_opp_ahead}"
        , throttle_duration_sec=2.0)
        
        # Detect overtake transition (opponent just passed ego)
        if opp_ahead_now and not self.last_opp_ahead:
            self.get_logger().info(
                f"🏁 OVERTAKE DETECTED! Opp s={self.opp_frenet_s:.2f} passed Ego s={self.ego_frenet_s:.2f}"
            )
            self._reset_to_random_positions()
        
        self.last_opp_ahead = opp_ahead_now

    def _reset_to_random_positions(self):
        """Reset cars to random positions with ego ahead of opponent"""
        # Get max arc length
        max_s = self.arc_lengths[-1]
        
        # Random position for opponent (0 to max_s)
        opp_s = random.uniform(0, max_s)
        opp_d = 0.0
        
        # Ego spawns ahead (3-8 meters ahead on track)
        separation = random.uniform(3.0, 8.0)
        ego_s = (opp_s + separation) % max_s  # Wrap around if needed
        ego_d = random.uniform(-0.3, 0.3)  # Slight lane variation
        
        # Convert to cartesian
        ego_x, ego_y = self._frenet_to_cartesian(ego_s, ego_d)
        opp_x, opp_y = self._frenet_to_cartesian(opp_s, opp_d)
        
        self.get_logger().info(
            f"Resetting: Ego @ s={ego_s:.1f}, d={ego_d:.2f} | "
            f"Opp @ s={opp_s:.1f}, d={opp_d:.2f}"
        )
        
        # Publish reset commands
        ego_msg = PoseWithCovarianceStamped()
        ego_msg.header.frame_id = 'map'
        ego_msg.pose.pose.position.x = ego_x
        ego_msg.pose.pose.position.y = ego_y
        ego_msg.pose.pose.orientation.w = 1.0
        self.ego_reset_pub.publish(ego_msg)
        
        opp_msg = PoseStamped()
        opp_msg.header.frame_id = 'map'
        opp_msg.pose.position.x = opp_x
        opp_msg.pose.position.y = opp_y
        opp_msg.pose.orientation.w = 1.0
        self.opp_reset_pub.publish(opp_msg)

    def log_to_csv(self):
        if self.session_finished:
            return

        dist = math.sqrt(
            (self.ego_pose[0] - self.opp_pose[0]) ** 2 +
            (self.ego_pose[1] - self.opp_pose[1]) ** 2
        )
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self._update_overtaking_events(timestamp)

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

        self._initialize_result_files()

        self.ego_laps, self.opp_laps, self.ego_laps_led = 0, 0, 0
        self.lap_winners = {}
        self._reset_overtake_state()
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

        lap_msg = Int32()
        lap_msg.data = max(self.ego_laps, self.opp_laps)
        self.lap_pub.publish(lap_msg)
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


    def ego_frenet_cb(self, msg):
        self.ego_frenet_s = msg.x
        self.ego_frenet_d = msg.y
        self.get_logger().info(f"EGO frenet received: s={msg.x:.1f}, d={msg.y:.3f}", throttle_duration_sec=3.0)

    def opp_frenet_cb(self, msg):
        self.opp_frenet_s = msg.x
        self.opp_frenet_d = msg.y
        self.get_logger().info(f"OPP frenet received: s={msg.x:.1f}, d={msg.y:.3f}", throttle_duration_sec=3.0)
        
        # Check for overtake
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