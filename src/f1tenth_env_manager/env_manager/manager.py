import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import os


class EnvManager(Node):
    def __init__(self):
        super().__init__('env_manager')

        self.session_id = os.getenv("SESSION_ID", "1")
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/results/session_{self.session_id}")
        os.makedirs(self.results_dir, exist_ok=True)

        # Subscribers only - no logging, no resetting
        self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_cb, 10)
        self.create_subscription(Pose2D, '/ego_racecar/frenet', self.ego_frenet_cb, 10)
        self.create_subscription(Pose2D, '/opp_racecar/frenet', self.opp_frenet_cb, 10)

        # State
        self.ego_pose = [0.0, 0.0]
        self.opp_pose = [0.0, 0.0]
        self.ego_frenet_s, self.ego_frenet_d = 0.0, 0.0
        self.opp_frenet_s, self.opp_frenet_d = 0.0, 0.0

        self.get_logger().info(f"Env Manager started | SESSION_ID={self.session_id}")

    def ego_odom_cb(self, msg):
        self.ego_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def opp_odom_cb(self, msg):
        self.opp_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def ego_frenet_cb(self, msg):
        self.ego_frenet_s = msg.x
        self.ego_frenet_d = msg.y

    def opp_frenet_cb(self, msg):
        self.opp_frenet_s = msg.x
        self.opp_frenet_d = msg.y


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