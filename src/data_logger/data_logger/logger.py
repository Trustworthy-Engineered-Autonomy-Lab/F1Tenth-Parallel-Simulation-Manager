import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool, Float32MultiArray, String
import csv
import os
from dataclasses import dataclass

"""
This is all the topics and values being logged by this data_logger into csv fiels that can be
found at the /sim_ftg/data/session_X/ directory where session_X is the respective session ID.

This data logger ALSO tracks when an overtake occurs and sends a message to the ENV_MANAGER
node telling it to reset the session.

    Note: Values below are logged for each the ego AND opp car unless explicilty specified

    Name - Topic - File Location

    velocity - /x_racecar/odom - ./
    frenet frame - /x_racecar/ - ./
    



"""



class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        #--- Configuration Settings ---#
        #
        on_sim = True

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


        #--- SUBSCRIBING TO TOPICS TO LOG FROM ---#
        # subscription addresses (allows us to switch easily if we use this on the irl car too)

        # session stuff
        lap_num_topic = '/env_manager/lap_num' # TODO: check
        

        # where ego is typically the blue car and opp is the orange car
        ego_odom = '/ego_racecar/odom' if on_sim else 'other_topic_for_irl_car'
        opp_odom = '/opp_racecar/odom' if on_sim else 'other_topic_for_irl_car'

        # TODO: point to correct topic (frenet frame)
        ego_frenet = '/ego_racecar/frenet'
        opp_frenet = '/opp_racecar/frenet'

        # idt there are imu topics in the sim, might need to find accel ourselves
        ego_imu = '/ego_racecar/imu'
        opp_imu = '/opp_racecar/imu'

        # TODO: point to correct topic
        imm = '/ego_racecar/imm'
        # opp_imm = '/opp_racecar/imm'


        # subscribe to the topics
        self.create_subscription(Int32, lap_num_topic, self.lap_num_cb, 10)
        self.create_subscription(Odometry, ego_odom, self.ego_odom_cb, 10)
        self.create_subscription(Odometry, opp_odom, self.opp_odom_cb, 10)
        self.create_subscription(Float32MultiArray, ego_frenet, self.ego_frenet_cb, 10)
        self.create_subscription(Float32MultiArray, opp_frenet, self.opp_frenet_cb, 10)
        self.create_subscription(Int32, imm, self.imm_cb, 10) # int as a placeholder

        #--- PUBLISHING ---#
        # publish when an overtake occurs to reset the simulator (sent to manager.py)
        self.overtake_pub = self.create_publisher(Bool, '/data_logger/overtake_detected')


    def lap_num_cb(self, msg):
        self.lap_num : int = msg

    def ego_odom_cb(self, msg):
        pass

    def opp_odom_cb(self, msg):
        pass

    def ego_frenet_cb(self, msg):
        pass

    def opp_frenet_cb(self, msg):
        pass

    def imm_cb(self, msg):
        pass

    def publish_overtake(self):
        msg = Bool()
        msg.data = True
        self.overtake_pub.publish(msg)






