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
        self.landmark_sub = self.create_subscription(JointState, "/landmarks", self.scan_callback, 10)

        # Q = L * M * L.T
        self.Qk = np.empty((3,3), order='C')

        # Odom error covariance matrix
        self.M = np.diag([1e-03, 2e-04, 2e-04])

        # Sensor noise matrix
        self.Rk = np.eye((6,6)) * 2.5e-05

        # Save timestamps from latest messages for later comparison, initial state at t = 0.0
        self.dt_odom_prev = None
        self.dt_scan_prev = None

        # Robot position history
        self.x_hist = [0.0]
        self.y_hist = [0.0]

        # Seen landmarks
        self.landmark_hist = []

        # Robot internal states:
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0

        # State vector
        self.x_vec = [self.x, self.y, self.psi].T

        # State estimation error covariance matrix
        self.P = np.eye((9,9), order='C')

        # Robot geometry
        self.wheel_radius = 0.12
        self.axle_length = 0.44


    def odom_callback(self, msg):
        wl = msg.velocity[0]
        wr = msg.velocity[1]

        current_time_ns = Time.from_msg(msg.header.stamp).nanoseconds

        if (self.dt_odom_prev == None):
                dt_sec = 0.0
                self.dt_odom_prev = current_time_ns
        else:
                dt_sec = (current_time_ns - self.dt_odom_prev) / 1e9
                self.dt_odom_prev = current_time_ns
        
        # self.get_logger().info(f'dt = {dt}')


        vx = self.wheel_radius / 2 * (wr + wl)
        wz = self.wheel_radius / self.axle_length * (wr - wl)
    
        delta_tran = vx * dt_sec
        delta_rot1 = 1/2 * wz * dt_sec
        delta_rot2 = delta_rot1

    def scan_callback(self, msg):
        for lm in msg:
            r = msg.position[lm]
            theta = msg.velocity[lm]

            x = self.x - r * np.cos(theta)
            y = self.y - r * np.sin(theta)        
            if msg.name not in self.landmark_hist:
                m = np.array([x, y]).T
                np.vstack(self.x_vec, m)

                self.landmark_hist.append(msg.name)
                self.update()


    # Internal update method for updating sizes of state dependant matrices
    def update(self):
        self.Qk = 0
        
             
             

    def plot(self):
        plt.scatter(self.x_hist, self.y_hist)
        plt.show()


def main():
    try:
        rclpy.init()
        ekf_node = EkfSlamNode()
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass

    ekf_node.plot()
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


