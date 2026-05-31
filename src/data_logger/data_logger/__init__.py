import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        #--- CONFIGURING STORAGE OF DATA LOGS ---#
        self.session_id = os.getenv("SESSION_ID", "1")
        self.results_dir = os.getenv("RESULTS_DIR", f"/sim_ws/data/session_{self.session_id}")
        self.data_dir = os.path.join(self.results_dir, "data")

        # individual folders for each data
        self.odom_dir = os.path.join(self.data_dir, "odom")
        self.accel_dir = os.path.join(self.data_dir, "accel")
        self.trajectory_dir = os.path.join(self.data_dir, "trajectory")

        # make dirs
        os.makedirs(self.odom_dir, exist_ok=True)
        os.makedirs(self.accel_dir, exist_ok=True)
        os.makedirs(self.trajectory_dir, exist_ok=True)


        #--- SUBSCRIBING TO TOPICS TO LOG ---#
        



