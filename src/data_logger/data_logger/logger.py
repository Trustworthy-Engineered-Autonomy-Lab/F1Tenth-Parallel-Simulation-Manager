import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Pose2D
import csv
import os
import sys
import json
import math
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped
from lidar_processing.experiment_protocol import protocol_qos, TrackProgress

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
        self.imm_trajectory = []
        self.imm_active = False
        self.mppi_state = 'RACELINE'
        self.driving_mode = 'FTG'
        self.hold_until_ns = float('inf')  # wait for the manager's initial spawn
        self.ego_ready = self.opp_ready = False
        self.ego_frenet_ready = self.opp_frenet_ready = False
        self.reset_hold_seconds = float(os.getenv('SCENARIO_RESET_HOLD_SECONDS', '2.0'))
        self.experiment = None
        self.pending_reset_ns = 0
        self.row_offsets = []
        self.defensive_imm_trajectory = []
        self.defensive_imm_ready = False
        self.imm_observation_seconds = 0.0
        self.ego_yaw = self.opp_yaw = 0.0
        self.ego_requested_speed = self.opp_requested_speed = 0.0
        self.collisions = {'ego': False, 'opp': False}
        self.collision_seen = {'ego': False, 'opp': False}
        self.progress = TrackProgress(self.max_track_length)
        self.create_subscription(String, '/env_manager/experiment_state', self.experiment_cb, protocol_qos())
        self.create_subscription(PoseArray, '/env_manager/reset_scenario', self.physical_reset_cb, 10)
        self.create_subscription(Path, '/defensive_imm_path', self.defensive_imm_cb, 10)
        self.create_subscription(String, '/env_manager/defensive_imm_ready', self.imm_ready_cb, 10)
        for car in ('ego', 'opp'):
            self.create_subscription(Bool, '/' + car + '_racecar/collision',
                                     lambda m, c=car: self.collision_cb(c, m), 10)
            topic = '/drive' if car == 'ego' else '/opp_drive'
            self.create_subscription(AckermannDriveStamped, topic,
                                     lambda m, c=car: setattr(self, c + '_requested_speed', m.drive.speed), 10)

        # Subscriptions
        self.create_subscription(Int32, '/env_manager/sim_num', self.sim_num_cb, 10)
        self.create_subscription(Int32, '/env_manager/lap_num', self.lap_num_cb, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)
        self.create_subscription(Bool, '/env_manager/reset', self.reset_cb, 10)
        self.create_subscription(Bool, '/env_manager/shutdown', self.shutdown_cb, 10)
        self.create_subscription(Path, '/imm_path', self.imm_cb, 10)
        self.create_subscription(String, '/imm_active', self.imm_active_cb, 10)
        self.create_subscription(String, '/mppi_state', self.mppi_state_cb, 10)
        self.create_subscription(String, '/driving_mode', self.driving_mode_cb, 10)

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
        self.row_offsets = []
        self.csv_writer.writerow([
            'timestamp', 'lap', 'overtake_num',
            'ego_x', 'ego_y', 'ego_vel', 'ego_s', 'ego_d',
            'opp_x', 'opp_y', 'opp_vel', 'opp_s', 'opp_d',
            'rel_x', 'rel_y', 'rel_vel', 'rel_s', 'rel_d',
            'imm_trajectory', 'imm_active', 'mppi_state', 'driving_mode',
            'scenario', 'experiment_phase', 'ego_target_speed', 'opp_target_speed',
            'ego_yaw', 'opp_yaw', 'ego_requested_speed', 'opp_requested_speed',
            'ego_collision', 'opp_collision', 'ego_collision_seen', 'opp_collision_seen',
            'defensive_imm_trajectory', 'defensive_imm_ready', 'imm_observation_seconds',
            'rel_s_unwrapped', 'opponent_faster', 'opp_speed_limited'
        ])
        self.csv_file.flush()

    def sim_num_cb(self, msg):
        """Handle new simulation number"""
        if self.experiment is not None or self.pending_reset_ns:
            return  # the stamped experiment protocol is authoritative
        if msg.data != self.current_sim:
            self._reset_telemetry()
            self.get_logger().info(f"🔄 Switching to simulation {msg.data}")
            self._init_csv_for_sim(msg.data)

    def reset_cb(self, msg):
        """Handle reset event"""
        if self.experiment is not None or self.pending_reset_ns:
            return
        if msg.data:
            self.get_logger().info("🔄 Reset signal received")
            self.lap_num = 0
            self.overtake_num = 0          # Reset overtake counter for new simulation
            self.last_opp_ahead = False    # Reset overtake state tracking too
            self._reset_telemetry()

    def _reset_telemetry(self):
        self.hold_until_ns = self.get_clock().now().nanoseconds + int(self.reset_hold_seconds * 1e9)
        self.ego_ready = self.opp_ready = False
        self.ego_frenet_ready = self.opp_frenet_ready = False
        self.imm_trajectory = []
        self.imm_active = False
        self.mppi_state = 'RACELINE'
        self.driving_mode = 'FTG'
        self.defensive_imm_trajectory = []
        self.defensive_imm_ready = False
        self.imm_observation_seconds = 0.0
        self.collisions = {'ego': False, 'opp': False}
        self.collision_seen = {'ego': False, 'opp': False}
        self.progress = TrackProgress(self.max_track_length)

    def _settled(self):
        return (self.get_clock().now().nanoseconds >= self.hold_until_ns
                and (self.experiment is None or self.experiment['phase'] in ('WARMUP', 'RUNNING')))

    def _trim_at(self, stamp_ns):
        """Undo any snapshots written after a physical reset, even if its event arrived late."""
        if not self.csv_file or self.csv_file.closed:
            return
        for index, (timestamp, offset) in enumerate(self.row_offsets):
            if timestamp >= stamp_ns:
                self.csv_file.seek(offset)
                self.csv_file.truncate()
                self.csv_file.flush()
                self.row_offsets = self.row_offsets[:index]
                break

    def physical_reset_cb(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp <= self.pending_reset_ns:
            return
        self.pending_reset_ns = stamp
        if not self.experiment or stamp > self.experiment['reset_stamp_ns']:
            self._trim_at(stamp)
            self.hold_until_ns = float('inf')

    def experiment_cb(self, message):
        state = json.loads(message.data)
        if self.experiment and state['reset_stamp_ns'] < self.experiment['reset_stamp_ns']:
            return
        new_scenario = not self.experiment or state['reset_stamp_ns'] != self.experiment['reset_stamp_ns']
        if new_scenario and state['phase'] in ('SETTLING', 'WARMUP', 'RUNNING'):
            self._trim_at(state['reset_stamp_ns'])
            if self.current_sim != state['scenario']:
                self._init_csv_for_sim(state['scenario'])
            self.lap_num = self.overtake_num = 0
            self.last_opp_ahead = False
            self._reset_telemetry()
            self.hold_until_ns = state['reset_stamp_ns'] + int(state['reset_settle_seconds'] * 1e9)
        self.experiment = state
        if state['phase'] == 'FINISHED':
            self._trim_at(state['event_stamp_ns'])

    def defensive_imm_cb(self, message):
        if self._settled():
            self.defensive_imm_trajectory = [(p.pose.position.x, p.pose.position.y) for p in message.poses]

    def imm_ready_cb(self, message):
        state = json.loads(message.data)
        if self.experiment and state['reset_stamp_ns'] == self.experiment['reset_stamp_ns']:
            self.defensive_imm_ready = state['ready']
            self.imm_observation_seconds = state['observation_seconds']

    def collision_cb(self, car, message):
        if self._settled():
            self.collisions[car] = message.data
            self.collision_seen[car] |= message.data

    def imm_cb(self, msg):
        if self._settled():
            self.imm_trajectory = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def imm_active_cb(self, msg):
        if self._settled():
            self.imm_active = msg.data == 'True'

    def mppi_state_cb(self, msg):
        if self._settled():
            self.mppi_state = msg.data

    def driving_mode_cb(self, msg):
        if self._settled():
            self.driving_mode = msg.data

    def shutdown_cb(self, msg):
        """Handle graceful shutdown"""
        if msg.data:
            self.get_logger().info("📤 Shutdown signal received - closing files and exiting")
            if self.csv_file:
                self.csv_file.close()
            sys.exit(0)

    def _detect_and_increment_overtake(self) -> None:
        """Detect overtake event"""
        if self.experiment:
            gap = self.progress.gap
            if (self.experiment['phase'] == 'RUNNING' and gap is not None
                    and gap >= self.experiment.get('overtake_margin', 3.0)):
                self.overtake_num = 1
            return
        if not self._settled() or not (self.ego_frenet_ready and self.opp_frenet_ready):
            return

        delta = self.opp_s - self.ego_s
        max_s = self.max_track_length

        if delta > max_s / 2.0:
            delta -= max_s
        elif delta < -max_s / 2.0:
            delta += max_s

        opp_ahead_now = delta > 0.0

        # crossing test: from behind (False) → ahead (True)
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
        q = msg.pose.pose.orientation
        self.ego_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not self._settled():
            return
        self.ego_ready = True
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vel = msg.twist.twist.linear.x

    def opp_odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.opp_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not self._settled():
            return
        self.opp_ready = True
        self.opp_x = msg.pose.pose.position.x
        self.opp_y = msg.pose.pose.position.y
        self.opp_vel = msg.twist.twist.linear.x

    def ego_frenet_cb(self, msg):
        if self._settled():
            self.progress.update('ego', msg.x)
        if not self._settled():
            return
        self.ego_frenet_ready = True
        self.ego_s = msg.x
        self.ego_d = msg.y

    def opp_frenet_cb(self, msg):
        if self._settled():
            self.progress.update('opp', msg.x)
        if not self._settled():
            return
        self.opp_frenet_ready = True
        self.opp_s = msg.x
        self.opp_d = msg.y
        self._detect_and_increment_overtake()

    def log_to_csv(self):
        """Write snapshot to current simulation CSV"""
        if not self.csv_writer or not self._settled() or not all((
                self.ego_ready, self.opp_ready, self.ego_frenet_ready, self.opp_frenet_ready)):
            return

        timestamp_ns = self.get_clock().now().nanoseconds
        timestamp = timestamp_ns / 1e9

        rel_x = self.opp_x - self.ego_x
        rel_y = self.opp_y - self.ego_y
        rel_vel = self.opp_vel - self.ego_vel
        rel_s = self.opp_s - self.ego_s
        rel_d = self.opp_d - self.ego_d
        self.row_offsets.append((timestamp_ns, self.csv_file.tell()))
        state = self.experiment or {}

        self.csv_writer.writerow([
            timestamp, self.lap_num, self.overtake_num,
            self.ego_x, self.ego_y, self.ego_vel, self.ego_s, self.ego_d,
            self.opp_x, self.opp_y, self.opp_vel, self.opp_s, self.opp_d,
            rel_x, rel_y, rel_vel, rel_s, rel_d,
            self.imm_trajectory, self.imm_active, self.mppi_state, self.driving_mode,
            self.current_sim, state.get('phase', 'RUNNING'), state.get('ego_speed', ''), state.get('opp_speed', ''),
            self.ego_yaw, self.opp_yaw, self.ego_requested_speed, self.opp_requested_speed,
            self.collisions['ego'], self.collisions['opp'], self.collision_seen['ego'], self.collision_seen['opp'],
            self.defensive_imm_trajectory, self.defensive_imm_ready, self.imm_observation_seconds,
            self.progress.gap, self.opp_vel > self.ego_vel,
            bool(state and self.opp_requested_speed < state['opp_speed'] * 0.95)
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
