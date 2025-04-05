from spatialmath  import SE3
from spatialmath.base import *
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R



class FrankaStateInterface():
    def __init__(self, node_handle):
        self.publisher_ = node_handle.create_publisher(
            JointState,
            '/franka/servo_jp', 
            10)
        
        self.subscription = node_handle.create_subscription(
            JointState,
            '/franka/measured_js',
            self.listener_callback,
            10)
        self.joint_positions = None

    def publish_joints(self, joint_command):
        msg = JointState()
        msg.position = joint_command
        self.publisher_.publish(msg)
            
    def listener_callback(self, msg):
        # Task: Read the joint states and print
        self.joint_positions = msg.position