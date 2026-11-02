import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32, Bool
import os
import yaml
import math
from nav_msgs.msg import Odometry

class EnvManager(Node):
    def __init__(self):
        super().__init__('env_manager')

        self.session_id = os.getenv("SESSION_ID", "1")
        self.num_simulations = int(os.getenv("NUM_SIMULATIONS", "3"))
        self.max_laps_total = int(os.getenv("MAX_LAPS", "2"))
        self.overtake_margin = float(os.getenv("OVERTAKE_MARGIN", "3.0"))
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        
        os.makedirs(self.results_dir, exist_ok=True)

        # Read config
        self.config_file = os.getenv("CONFIG_FILE", "/sim_ws/src/f1tenth_gym_ros/config/manager.yaml")
        self.config = self._load_config()
        
        # Lap detection settings
        self.lap_detection_method = self.config.get('lap_detection', {}).get('method', 'bubble')
        self.bubble_radius = float(self.config.get('lap_detection', {}).get('bubble_radius', 5.0))
        self.track_length = float(self.config.get('lap_detection', {}).get('track_length', 359.85))
        
        # Subscribers
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)
        self.create_subscription(Bool, '/data_logger/overtake_detected', self.overtake_detected_cb, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)


        # Publishers
        self.lap_pub = self.create_publisher(Int32, '/env_manager/lap_num', 10)
        self.sim_num_pub = self.create_publisher(Int32, '/env_manager/sim_num', 10)
        self.reset_pub = self.create_publisher(Bool, '/env_manager/reset', 10)
        self.shutdown_pub = self.create_publisher(Bool, '/env_manager/shutdown', 10)

        # State
        self.ego_pose = [0.0, 0.0]  # x, y in global coordinates
        self.opp_pose = [0.0, 0.0]
        self.ego_frenet_s = 0.0
        self.opp_frenet_s = 0.0
        self.total_lap_count = 0
        self.sim_lap_count = 0
        self.current_sim = 1
        self.last_ego_s = 0.0
        
        # Per-simulation lap center
        self.current_lap_center = [4.0, 1.0]  # Default
        self.ego_inside_bubble = False
        
        # Reset logic
        self.laps_without_overtake = 0
        self.overtake_occurred = False
        self.max_laps_without_overtake = 2

        self._load_sim_config(1)
        self._publish_sim_num()
        
        self.get_logger().info(
            f"Env Manager | Session={self.session_id} | "
            f"NUM_SIMS={self.num_simulations} | MAX_LAPS={self.max_laps_total} | "
            f"LAP_METHOD={self.lap_detection_method} | OVERTAKE_MARGIN={self.overtake_margin}m"
        )

    def _load_config(self):
        """Load manager.yaml config"""
        try:
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f"Could not load config: {e}")
            return {}

    def _load_sim_config(self, sim_num: int):
        """Load simulation-specific config (lap center, starting positions, etc.)"""
        sim_key = f's_{sim_num}'
        
        if 'sim_start' in self.config and sim_key in self.config['sim_start']:
            sim_config = self.config['sim_start'][sim_key]
            
            # Extract lap center if provided, otherwise use ego starting position
            if 'lap_center' in sim_config:
                self.current_lap_center = sim_config['lap_center']
            else:
                # Fallback to ego starting position
                ego_start = sim_config.get('ego', [4, 1])
                self.current_lap_center = [ego_start[0], ego_start[1]]
            
            self.get_logger().info(f"Simulation {sim_num} lap center: {self.current_lap_center}")
        else:
            self.get_logger().warn(f"No config found for {sim_key}, using default lap center")

    def _publish_sim_num(self):
        """Publish current simulation number"""
        msg = Int32()
        msg.data = self.current_sim
        self.sim_num_pub.publish(msg)

    def ego_odom_cb(self, msg):
        self.ego_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def ego_frenet_cb(self, msg):
        """Called whenever ego Frenet coordinates update"""
        self.ego_frenet_s = msg.x
        if self.lap_detection_method == "bubble":
            self._detect_lap_with_bubble()
        else:
            self._detect_lap_with_frenet()

    def opp_frenet_cb(self, msg):
        self.opp_frenet_s = msg.x

    def overtake_detected_cb(self, msg):
        """Called when data_logger detects an overtake"""
        if msg.data:
            self._check_overtake_distance()

    def _detect_lap_with_bubble(self):
        """
        Detect lap crossing using a bubble around the lap center.
        A lap is counted when the ego car:
        1. Was outside the bubble
        2. Enters the bubble
        """
        # Calculate distance from ego to lap center
        dx = self.ego_pose[0] - self.current_lap_center[0]
        dy = self.ego_pose[1] - self.current_lap_center[1]
        distance_to_center = math.sqrt(dx**2 + dy**2)
        
        is_inside_bubble_now = distance_to_center <= self.bubble_radius
        
        # Crossing detection: outside → inside
        if is_inside_bubble_now and not self.ego_inside_bubble:
            self._complete_lap("BUBBLE_CROSSING")
        
        self.ego_inside_bubble = is_inside_bubble_now

    def _detect_lap_with_frenet(self):
        """Fallback: detect lap using Frenet s coordinate wrapping"""
        if self.last_ego_s > 300.0 and self.ego_frenet_s < 50.0:
            self._complete_lap("FRENET_WRAP")
        
        self.last_ego_s = self.ego_frenet_s

    def _complete_lap(self, method: str):
        """Handle lap completion"""
        self.total_lap_count += 1
        self.sim_lap_count += 1
        
        self.get_logger().info(
            f"🏁 LAP {self.total_lap_count} (sim: {self.sim_lap_count}) [{method}] | "
            f"Sim {self.current_sim}/{self.num_simulations}"
        )

        # Publish lap count
        msg = Int32()
        msg.data = self.total_lap_count
        self.lap_pub.publish(msg)

        # Check for reset conditions
        if not self.overtake_occurred:
            self.laps_without_overtake += 1
            self.get_logger().warn(
                f"⏱️  No overtake: {self.laps_without_overtake}/{self.max_laps_without_overtake}"
            )
            
            if self.laps_without_overtake >= self.max_laps_without_overtake:
                self.get_logger().warn(f"❌ FAILED OVERTAKE")
                self._trigger_reset("FAILED_OVERTAKE")
        else:
            self.laps_without_overtake = 0
            self.overtake_occurred = False

        # HARD STOP: max_laps_total reached
        if self.total_lap_count >= self.max_laps_total:
            self.get_logger().warn(
                f"⏹️  MAX_LAPS ({self.max_laps_total}) reached. "
                f"Completed {self.current_sim}/{self.num_simulations} simulations."
            )
            self._trigger_shutdown()

    def _check_overtake_distance(self):
        """Verify overtake has sufficient margin"""
        delta = self.opp_frenet_s - self.ego_frenet_s
        max_s = self.track_length
        
        if delta > max_s / 2.0:
            delta -= max_s
        elif delta < -max_s / 2.0:
            delta += max_s

        if delta >= self.overtake_margin:
            self.get_logger().info(f"✅ VALID OVERTAKE! {delta:.2f}m ahead")
            self.overtake_occurred = True
            self._trigger_reset("SUCCESSFUL_OVERTAKE")
        else:
            self.get_logger().warn(f"⚠️  Insufficient margin: {delta:.2f}m < {self.overtake_margin}m")

    def _trigger_reset(self, reason: str):
        """Reset current simulation and move to next"""
        self.get_logger().info(f"🔄 RESET: {reason}")
        
        # Move to next simulation
        self.current_sim += 1
        self.sim_lap_count = 0
        self.laps_without_overtake = 0
        self.overtake_occurred = False
        self.ego_inside_bubble = False
        
        if self.current_sim > self.num_simulations:
            self.get_logger().warn(f"All simulations ({self.num_simulations}) completed!")
            self._trigger_shutdown()
        else:
            # Load config for new simulation
            self._load_sim_config(self.current_sim)
            
            # Publish reset signal and new sim number
            self._publish_sim_num()
            msg = Bool()
            msg.data = True
            self.reset_pub.publish(msg)

    def _trigger_shutdown(self):
        """Gracefully shutdown the session"""
        self.get_logger().info("📤 Triggering session shutdown...")
        msg = Bool()
        msg.data = True
        self.shutdown_pub.publish(msg)

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