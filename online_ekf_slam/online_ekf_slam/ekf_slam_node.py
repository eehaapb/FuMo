import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi, cos, sin, atan2, pow

class EkfSlamNode(Node):
    def __init__(self):
        super().__init__("ekf_slam_node")

        self.odom_sub = self.create_subscription(JointState, "/joint_states", self.odom_callback, 10)
        self.landmark_sub = self.create_subscription(JointState, "/landmarks", self.landmark_callback, 10)

        self.Q = np.empty((3,3), order='C')

        # Odom error covariance matrix
        self.M = np.diag([1e-3, 2e-4, 2e-4])

        # Sensor noise matrix
        self.Rk = np.diag([0.000025, 0.000025, 0.000025, 0.000025, 0.000025, 0.000025]) 

        # Save timestamps from latest messages for later comparison, initial state at t = 0.0
        self.dt_odom_prev = None
        self.dt_scan_prev = None

        # Robot states
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Robot geometry
        self.wheel_radius = 0.12
        self.axle_length = 0.44


    def odom_callback(self, msg):
        wl = msg.velocity[0]
        wr = msg.velocity[1]

        current_time_ns = Time.from_msg(msg.header.stamp).nanoseconds

        if (self.dt_odom_prev == None):
                dt = 0.0
                self.dt_odom_prev = current_time_ns
        else:
                dt = (current_time_ns - self.dt_odom_prev) / 1e9
                self.dt_odom_prev = current_time_ns
        
        # self.get_logger().info(f'dt = {dt}')


        vx = self.wheel_radius / 2 * (wr + wl)
        wz = self.wheel_radius / self.axle_length * (wr - wl)
    
        delta_tran = vx * dt
        delta_rot1 = 1/2 * wz * dt
        delta_rot2 = delta_rot1

    def landmark_callback(self, msg):
        pass


def main():
    rclpy.init()
    ekf_node = EkfSlamNode()
    rclpy.spin(ekf_node)

if __name__ == "__main__":
    main()


