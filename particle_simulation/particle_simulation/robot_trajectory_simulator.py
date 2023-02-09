import rclpy
import numpy as np
import matplotlib.pyplot as plt

from  rclpy.node import Node
from math import pow, cos, sin, pi
from std_msgs.msg import Float64MultiArray


class ParticleTrajectorySim(Node):
    def __init__(self):
        super().__init__("particle_trajectory_sim")

        self.odom_sub = self.create_subscription(Float64MultiArray, "Wheels_data", self.odom_callback, 10)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("a1", 20),
                ("a2", 6),
                ("a3", 25),
                ("a4",8)
            ]
        )

        self.sample_count = 100

        self.a1 = self.get_parameter("a1").value
        self.a2 = self.get_parameter("a2").value
        self.a3 = self.get_parameter("a3").value
        self.a4 = self.get_parameter("a4").value

        self.dt = 0.1

        self.t_ = []
        self.x_prev = []
        self.y_prev = []
        self.psi_prev = []

        self.zeros = [0] * 100

        self.t_.append(self.zeros)
        self.x_prev.append(self.zeros)
        self.y_prev.append(self.zeros)
        self.psi_prev.append(self.zeros)


        self.wheel_radius = 0.1
        self.axle_length = 0.4


    def odom_callback(self, msg):
        x_curr_estimates = []
        y_curr_estimates = []
        psi_curr_estimates = []

        wl = self.rpm_to_rads(msg.data[0])
        wr = self.rpm_to_rads(msg.data[1])

        vx = self.wheel_radius / 2 * (wr + wl)
        wz = self.wheel_radius / self.axle_length * (wr - wl)

        delta_h_trans = vx * self.dt
        delta_h_rot1 = 1/2 * wz * self.dt
        delta_h_rot2 = delta_h_rot1

        eps_rot1 = np.random.normal(0, self.a1*pow(delta_h_rot1, 2) + self.a2*pow(delta_h_trans, 2), size=self.sample_count)
        eps_rot2 = np.random.normal(0, self.a1*pow(delta_h_rot2, 2) + self.a2*pow(delta_h_trans, 2), size=self.sample_count)
        eps_trans = np.random.normal(0, self.a3*pow(delta_h_trans, 2) + self.a4*(pow(delta_h_rot1, 2) + pow(delta_h_rot2, 2)), size=self.sample_count)

        for i in range(self.sample_count):
                x_curr = self.x_prev[-1][i] + (delta_h_trans + eps_trans[i]) * cos(self.psi_prev[-1][i] + delta_h_rot1 + eps_rot1[i])
                y_curr = self.y_prev[-1][i] + (delta_h_trans + eps_trans[i]) * sin(self.psi_prev[-1][i] + delta_h_rot1 + eps_rot1[i])
                psi_curr = self.psi_prev[-1][i] + delta_h_rot1 + eps_rot1[i] + delta_h_rot2 + eps_rot2[i]

                # print('x_curr: '+ str(x_curr) +', y_curr: '+ str(y_curr) +', psi_curr: '+ str(psi_curr))
                x_curr_estimates.append(x_curr)
                y_curr_estimates.append(y_curr)
                psi_curr_estimates.append(psi_curr)
        
        print(len(x_curr_estimates))
        print(len(self.x_prev))
            

        self.x_prev.append(x_curr_estimates)
        self.y_prev.append(y_curr_estimates)
        self.psi_prev.append(psi_curr_estimates)



    def rpm_to_rads(self, rpm):
        return rpm/60*2*pi

def main():
    try:
        rclpy.init()
        traj_sim = ParticleTrajectorySim()
        rclpy.spin(traj_sim)
    except KeyboardInterrupt:
        print(traj_sim.x_prev)
        res_x = np.array(traj_sim.x_prev)
        res_y = np.array(traj_sim.y_prev)

        print(np.shape(res_x))
        res_x = res_x[np.array([700,1400,2100]), :]
        res_y = res_y[np.array([700,1400,2100]), :]
        print(np.shape(res_y))


        plt.plot(res_x,res_y, 'ro')
            



        plt.show()
        


if __name__ == "__main__":
    main()

