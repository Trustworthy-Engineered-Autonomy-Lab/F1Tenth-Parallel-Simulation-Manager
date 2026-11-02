import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Pose, PoseArray


class EnvManager(Node):
    def __init__(self):
        super().__init__('env_manager')

        # ---------------------------------------------------------
        # Environment configuration
        # ---------------------------------------------------------
        self.session_id = os.getenv("SESSION_ID", "1")
        self.num_simulations = int(os.getenv("NUM_SIMULATIONS", "3"))

        # MAX_LAPS is the maximum number of laps for each simulation.
        self.max_laps_per_sim = int(os.getenv("MAX_LAPS", "2"))

        self.overtake_margin = float(os.getenv("OVERTAKE_MARGIN", "3.0"))
        self.results_dir = os.getenv(
            "RESULTS_DIR",
            f"/sim_ws/results/session_{self.session_id}"
        )

        os.makedirs(self.results_dir, exist_ok=True)

        # ---------------------------------------------------------
        # Load manager.yaml
        # ---------------------------------------------------------
        self.config_file = os.getenv(
            "CONFIG_FILE",
            "/sim_ws/src/f1tenth_gym_ros/config/manager.yaml"
        )
        self.config = self._load_config()

        lap_config = self.config.get("lap_detection", {})

        self.lap_detection_method = lap_config.get("method", "bubble")
        self.bubble_radius = float(lap_config.get("bubble_radius", 5.0))
        self.track_length = float(lap_config.get("track_length", 359.85))

        # Ignore stale position messages briefly after requesting a reset.
        self.reset_settle_seconds = float(
            self.config.get("reset_settle_seconds", 2.0)
        )

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------
        self.current_sim = 1
        self.sim_lap_count = 0
        self.total_lap_count = 0

        self.ego_pose = [0.0, 0.0]

        self.ego_frenet_s = 0.0
        self.opp_frenet_s = 0.0
        self.last_ego_s = 0.0

        self.ego_odom_ready = False
        self.ego_frenet_ready = False
        self.opp_frenet_ready = False

        # A lap is counted only after the ego leaves the starting bubble
        # and subsequently re-enters it.
        self.lap_armed = False

        # The opponent must first be observed behind the ego before an
        # overtake can be accepted.
        self.overtake_armed = False

        self.reset_in_progress = False
        self.shutdown_requested = False

        self.reset_timer = None
        self.shutdown_timer = None

        # Per-simulation configuration.
        self.current_lap_center = [4.0, 1.0]
        self.current_ego_start = [4.0, 1.0, 0.0]
        self.current_opp_start = [0.0, 0.0, 0.0]

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.lap_pub = self.create_publisher(
            Int32,
            '/env_manager/lap_num',
            10
        )

        self.sim_num_pub = self.create_publisher(
            Int32,
            '/env_manager/sim_num',
            10
        )

        self.reset_pub = self.create_publisher(
            Bool,
            '/env_manager/reset',
            10
        )

        self.shutdown_pub = self.create_publisher(
            Bool,
            '/env_manager/shutdown',
            10
        )

        self.scenario_reset_pub = self.create_publisher(
            PoseArray,
            '/env_manager/reset_scenario',
            10
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.ego_odom_cb,
            10
        )

        self.create_subscription(
            Pose2D,
            '/ego_racecar/frenet',
            self.ego_frenet_cb,
            10
        )

        self.create_subscription(
            Pose2D,
            '/opp_racecar/frenet',
            self.opp_frenet_cb,
            10
        )

        # The manager performs overtake-margin detection itself.
        # The logger's overtake topic is not used to trigger resets.

        # ---------------------------------------------------------
        # Initialize simulation 1
        # ---------------------------------------------------------
        if self.num_simulations < 1:
            raise ValueError("NUM_SIMULATIONS must be at least 1")

        if self.max_laps_per_sim < 1:
            raise ValueError("MAX_LAPS must be at least 1")

        self._load_sim_config(self.current_sim)
        self._publish_sim_num()
        self._publish_lap_num()

        self.startup_reset_timer = self.create_timer(
            2.0,
            self._publish_initial_scenario
        )

        self.get_logger().info(
            f"Env Manager started | "
            f"Session={self.session_id} | "
            f"Simulations={self.num_simulations} | "
            f"Max laps per simulation={self.max_laps_per_sim} | "
            f"Lap method={self.lap_detection_method} | "
            f"Bubble radius={self.bubble_radius:.2f} m | "
            f"Overtake margin={self.overtake_margin:.2f} m"
        )

    # -------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------

    def _publish_initial_scenario(self):
        self.startup_reset_timer.cancel()
        self.destroy_timer(self.startup_reset_timer)
        self.startup_reset_timer = None

        self.reset_in_progress = True
        self._publish_scenario_reset()

        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)

        self.reset_timer = self.create_timer(
            self.reset_settle_seconds,
            self._finish_reset_transition
        )

    def _load_config(self):
        """Load manager.yaml."""
        try:
            with open(self.config_file, "r", encoding="utf-8") as config_file:
                config = yaml.safe_load(config_file)

            if config is None:
                self.get_logger().warn(
                    f"{self.config_file} is empty; using defaults."
                )
                return {}

            if not isinstance(config, dict):
                self.get_logger().warn(
                    f"{self.config_file} must contain a YAML mapping; "
                    "using defaults."
                )
                return {}

            return config

        except Exception as error:
            self.get_logger().error(
                f"Could not load {self.config_file}: {error}"
            )
            return {}

    def _load_sim_config(self, sim_num: int):
        """Load the starting positions and lap center for one simulation."""
        sim_key = f"s_{sim_num}"
        simulations = self.config.get("sim_start", {})
        sim_config = simulations.get(sim_key)

        if sim_config is None:
            raise KeyError(
                f"Simulation {sim_key} is missing from "
                f"{self.config_file}"
            )

        self.current_ego_start = list(
            sim_config.get("ego", [4.0, 1.0, 0.0])
        )
        self.current_opp_start = list(
            sim_config.get("opp", [0.0, 0.0, 0.0])
        )

        if len(self.current_ego_start) != 3:
            raise ValueError(
                f"{sim_key}.ego must be [x, y, theta]"
            )

        if len(self.current_opp_start) != 3:
            raise ValueError(
                f"{sim_key}.opp must be [x, y, theta]"
            )

        lap_center = sim_config.get("lap_center")

        if lap_center is None:
            self.current_lap_center = [
                float(self.current_ego_start[0]),
                float(self.current_ego_start[1])
            ]
        else:
            if len(lap_center) != 2:
                raise ValueError(
                    f"{sim_key}.lap_center must be [x, y]"
                )

            self.current_lap_center = [
                float(lap_center[0]),
                float(lap_center[1])
            ]

        self.get_logger().info(
            f"Loaded {sim_key} | "
            f"ego={self.current_ego_start} | "
            f"opp={self.current_opp_start} | "
            f"lap_center={self.current_lap_center}"
        )

    # -------------------------------------------------------------
    # Publishers
    # -------------------------------------------------------------

    def _make_pose(self, state):
        x, y, theta = state

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(float(theta) / 2.0)
        pose.orientation.w = math.cos(float(theta) / 2.0)

        return pose

    def _publish_scenario_reset(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.poses = [
            self._make_pose(self.current_ego_start),
            self._make_pose(self.current_opp_start)
        ]

        self.scenario_reset_pub.publish(msg)

        self.get_logger().info(
            f"Published physical reset for simulation {self.current_sim} | "
            f"ego={self.current_ego_start} | "
            f"opp={self.current_opp_start}"
        )


    def _publish_sim_num(self):
        msg = Int32()
        msg.data = self.current_sim
        self.sim_num_pub.publish(msg)

    def _publish_lap_num(self):
        # Publish the lap number for the current simulation, not the
        # session-wide total.
        msg = Int32()
        msg.data = self.sim_lap_count
        self.lap_pub.publish(msg)

    # -------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------
    def ego_odom_cb(self, msg):
        self.ego_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        self.ego_odom_ready = True

        if self.reset_in_progress or self.shutdown_requested:
            return

        if self.lap_detection_method == "bubble":
            self._detect_lap_with_bubble()

    def ego_frenet_cb(self, msg):
        self.ego_frenet_s = msg.x
        self.ego_frenet_ready = True

        if self.reset_in_progress or self.shutdown_requested:
            return

        if self.lap_detection_method != "bubble":
            self._detect_lap_with_frenet()

        self._check_overtake_distance()

    def opp_frenet_cb(self, msg):
        self.opp_frenet_s = msg.x
        self.opp_frenet_ready = True

        if self.reset_in_progress or self.shutdown_requested:
            return

        self._check_overtake_distance()

    # -------------------------------------------------------------
    # Lap detection
    # -------------------------------------------------------------
    def _detect_lap_with_bubble(self):
        """
        Count a lap after the ego:
        1. Starts inside its scenario-specific bubble.
        2. Leaves the bubble.
        3. Returns to the bubble.
        """
        if not self.ego_odom_ready:
            return

        dx = self.ego_pose[0] - self.current_lap_center[0]
        dy = self.ego_pose[1] - self.current_lap_center[1]
        distance = math.hypot(dx, dy)

        inside_bubble = distance <= self.bubble_radius

        if not inside_bubble:
            self.lap_armed = True
            return

        if inside_bubble and self.lap_armed:
            # Disarm before completing the lap so callbacks cannot count
            # the same crossing twice.
            self.lap_armed = False
            self._complete_lap("BUBBLE_CROSSING")

    def _detect_lap_with_frenet(self):
        """Fallback lap detection using Frenet wrap-around."""
        if (
            self.last_ego_s > self.track_length * 0.80
            and self.ego_frenet_s < self.track_length * 0.20
        ):
            self._complete_lap("FRENET_WRAP")

        self.last_ego_s = self.ego_frenet_s

    def _complete_lap(self, method: str):
        if self.reset_in_progress or self.shutdown_requested:
            return

        self.total_lap_count += 1
        self.sim_lap_count += 1
        self._publish_lap_num()

        self.get_logger().info(
            f"🏁 Lap {self.sim_lap_count}/"
            f"{self.max_laps_per_sim} completed "
            f"using {method} | "
            f"Simulation {self.current_sim}/"
            f"{self.num_simulations} | "
            f"Session total laps={self.total_lap_count}"
        )

        if self.sim_lap_count >= self.max_laps_per_sim:
            self._trigger_reset("MAX_LAPS_REACHED")

    # -------------------------------------------------------------
    # Overtake detection
    # -------------------------------------------------------------
    def _wrapped_frenet_delta(self):
        """Return opponent_s - ego_s with track wrap-around handled."""
        delta = self.opp_frenet_s - self.ego_frenet_s

        if delta > self.track_length / 2.0:
            delta -= self.track_length
        elif delta < -self.track_length / 2.0:
            delta += self.track_length

        return delta

    def _check_overtake_distance(self):
        """
        Trigger a reset when the opponent transitions from behind the ego
        to at least OVERTAKE_MARGIN metres ahead.
        """
        if (
            self.reset_in_progress
            or self.shutdown_requested
            or not self.ego_frenet_ready
            or not self.opp_frenet_ready
        ):
            return

        delta = self._wrapped_frenet_delta()

        # The opponent must first be observed behind the ego.
        if delta <= 0.0:
            self.overtake_armed = True
            return

        if self.overtake_armed and delta >= self.overtake_margin:
            self.overtake_armed = False

            self.get_logger().info(
                f"✅ Successful overtake | "
                f"Opponent is {delta:.2f} m ahead | "
                f"Required margin={self.overtake_margin:.2f} m"
            )

            self._trigger_reset("SUCCESSFUL_OVERTAKE")

    # -------------------------------------------------------------
    # Reset and shutdown
    # -------------------------------------------------------------
    def _trigger_reset(self, reason: str):
        """Finish the current simulation and advance to the next one."""
        if self.reset_in_progress or self.shutdown_requested:
            return

        self.reset_in_progress = True

        self.get_logger().info(
            f"🔄 Simulation {self.current_sim} finished: {reason}"
        )

        if self.current_sim >= self.num_simulations:
            self.get_logger().info(
                f"All {self.num_simulations} simulations completed."
            )
            self._trigger_shutdown()
            return

        self.current_sim += 1
        self.sim_lap_count = 0

        self.lap_armed = False
        self.overtake_armed = False

        self.ego_odom_ready = False
        self.ego_frenet_ready = False
        self.opp_frenet_ready = False

        self.last_ego_s = 0.0

        self._load_sim_config(self.current_sim)
        self._publish_scenario_reset()

        # Publish the new simulation number and zero lap count before
        # requesting the physical simulator reset.
        self._publish_sim_num()
        self._publish_lap_num()

        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)

        self.get_logger().info(
            f"Requested reset for simulation "
            f"{self.current_sim}/{self.num_simulations}"
        )

        # Allow the simulator time to apply the new starting state and
        # prevent stale messages from triggering another reset.
        self.reset_timer = self.create_timer(
            self.reset_settle_seconds,
            self._finish_reset_transition
        )

    def _finish_reset_transition(self):
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None

        if self.shutdown_requested:
            return

        self.reset_in_progress = False

        self.get_logger().info(
            f"Simulation {self.current_sim} is active."
        )

    def _trigger_shutdown(self):
        """Notify the logger and then stop this manager node."""
        if self.shutdown_requested:
            return

        self.shutdown_requested = True
        self.reset_in_progress = True

        self.get_logger().info(
            "📤 Publishing session shutdown request..."
        )

        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)

        # Keep this node alive briefly so subscribers can receive the
        # shutdown message and close their CSV files.
        self.shutdown_timer = self.create_timer(
            2.0,
            self._finish_shutdown
        )

    def _finish_shutdown(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.destroy_timer(self.shutdown_timer)
            self.shutdown_timer = None

        self.get_logger().info("Environment manager shutting down.")

        if rclpy.ok():
            rclpy.shutdown()


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