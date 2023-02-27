import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState

class EkfSlamNode(Node):
    def __init__(self):
        super().__init__("ekf_slam_node")

        self.odom_sub = self.create_subscription(JointState, "/joint_states", self.odom_callback, 10)
        self.landmark_sub = self.create_subscription(JointState, "/landmarks", self.scan_callback, 10)

        # Odom error
        self.M = np.diag([1e-03, 2e-04, 2e-04])

        # Sensor noise matrix
        self.Rk = np.eye((6)) * 2.5e-05

        # Save timestamps from latest messages for later comparison, initial state at t = 0.0
        self.dt_odom_prev = None
        self.dt_scan_prev = None

        # Robot position history
        self.x_hist = [0.0]
        self.y_hist = [0.0]

        # Seen landmarks
        self.landmark_hist = []

        # State vectors
        self.xk = np.zeros((3,1)) # Last known state
        self.xk_hat_p = np.zeros((3,1)) # Current state prediction with correction from measurement
        self.xk_hat_m = np.zeros((3,1)) # Current state prediction without correction

        # State error matrix
        self.Pk = np.eye((3)) * 2.5e-05
        self.Pk_hat_p = np.eye((3)) * 2.5e-05
        self.Pk_hat_m = np.eye((3)) * 2.5e-05

        # Measurement vector
        self.z = np.empty((3, 2))
        self.z_hat = np.empty((3, 2))

        # Robot geometry
        self.wheel_radius = 0.12
        self.axle_length = 0.44

        # Update flag
        self.update = False


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
        
        self.predict(wl, wr, dt_sec) 

    def scan_callback(self, msg):
        self.update = True
        landmark_count = len(msg.position)

        for i in range(landmark_count):

            r = msg.position[i]
            theta = msg.velocity[i]
            m = np.array([r, theta])

            if msg.name[i] not in self.landmark_hist:

                # Polar to cartesian
                x_lm = self.xk[0][0] - r * np.cos(theta + self.xk[2][0])
                y_lm = self.xk[1][0] - r * np.sin(theta + self.xk[2][0])
                
                # Introduce first seen landmark positions to all state vectors
                self.xk = np.vstack((self.xk, [x_lm], [y_lm]))
                self.xk_hat_m = np.vstack((self.xk_hat_m, [x_lm], [y_lm]))
                self.xk_hat_p = np.vstack((self.xk_hat_p, [x_lm], [y_lm]))

                self.update_internal_matrices()

            self.z[i] = np.array([r, theta])
            self.landmark_hist.append(msg.name[i])

    def update_internal_matrices(self):
        state_size = len(self.xk_hat_m)

        M = np.zeros((state_size, state_size))
        M[:3, :3] = self.M[:3, :3]
        self.M = M

        Pk = np.eye(state_size) * 2.5e-05
        Pk_hp = np.eye(state_size) * 2.5e-05
        Pk_hm = np.eye(state_size) * 2.5e-05

        Pk[:self.Pk.shape[0], :self.Pk.shape[1]] = self.Pk[:self.Pk.shape[0], :self.Pk.shape[1]]
        Pk_hp[:self.Pk_hat_m.shape[0], :self.Pk_hat_m.shape[1]] = self.Pk_hat_m[:self.Pk_hat_m.shape[0], :self.Pk_hat_m.shape[1]]
        Pk_hm[:self.Pk_hat_p.shape[0], :self.Pk_hat_p.shape[1]] = self.Pk[:self.Pk_hat_p.shape[0], :self.Pk_hat_p.shape[1]]

        self.Pk = Pk
        self.Pk_hat_p = Pk_hp
        self.Pk_hat_m = Pk_hm

    #  Kalman filter update
    def calculate_update(self):
        # measurement_vec_size = len(self.z)
        ss = len(self.xk_hat_m)

        H = np.empty((2, 9))
        

        k = 0
        for i in range(3):
            
            lx = self.xk_hat_m[3+k][0]
            x = self.xk_hat_m[0][0]

            ly = self.xk_hat_m[4+k][0]
            y = self.xk_hat_m[1][0]

            psi = self.xk_hat_m[2][0]

            dx = lx - x
            dy = ly - y

            d2 = np.power(dx, 2) + np.power(dy, 2)
            d = np.sqrt(d2)

            # Measurement predictions based on calculated odometry
            r_hat = d
            theta_hat = np.arctan2(dx, dy) - psi

            self.z_hat[i] = np.array(([r_hat, theta_hat])).T

            # Increment k by two to get next landmark coordinates from state
            k += 2

            # Band-aid fix, no time to work out variable version
            if i == 0:
                h1_r = np.array([-dx/d, -dy/d, 0, x/d, dy/d, 0, 0, 0, 0])
                h2_t = np.array([dy/d2, -dx/d2, -1, -dy/d2, dx/d2, 0, 0, 0, 0])
                hh = np.vstack((h1_r, h2_t))            
                H = hh
            if i == 1:
                h1_r = np.array([-dx/d, -dy/d, 0, 0, 0, dx/d, dy/d, 0, 0])
                h2_t = np.array([dy/d2, -dx/d2, -1, 0, 0, -dy/d2, dx/d2, 0, 0])
                hh = np.vstack((h1_r, h2_t))            
                H = np.vstack((H, hh))
            if i == 2:
                h1_r = np.array([-dx/d, -dy/d, 0, 0, 0, 0, 0, dx/d, dy/d])
                h2_t = np.array([dy/d2, -dx/d2, -1, 0, 0, 0, 0, -dy/d2, dx/d2]) 
                hh = np.vstack((h1_r, h2_t))            
                H = np.vstack((H, hh))

          

        K = self.Pk_hat_m @ H.T @ np.linalg.inv(H @ self.Pk_hat_m @ H.T + self.Rk)
        self.Pk_hat_p = (np.eye(ss) - K @ H) @ self.Pk_hat_m
        self.xk_hat_p = self.xk_hat_m + K @ (self.z.ravel(order='C') - self.z_hat.ravel(order='C'))


    def predict(self, wl, wr, dt):
        vx = self.wheel_radius / 2 * (wr + wl)
        wz = self.wheel_radius / self.axle_length * (wr - wl)
    
        delta_tran = vx * dt
        delta_rot = 1/2 * wz * dt

        x_last_plus = self.xk[0][0]
        y_last_plus = self.xk[1][0]
        psi_last_plus = self.xk[2][0]

        self.xk_hat_m[0][0] = x_last_plus + delta_tran * np.cos(psi_last_plus + delta_rot)
        self.xk_hat_m[1][0] = y_last_plus + delta_tran * np.sin(psi_last_plus + delta_rot)
        self.xk_hat_m[2][0] = psi_last_plus + 2 * delta_rot

        state_size = self.xk_hat_m.size

        # Jacobians

        # State transition matrix
        A = np.eye(state_size)
        A[0, 2] = -delta_tran * np.sin(psi_last_plus + delta_rot)
        A[1, 2] =  delta_tran * np.cos(psi_last_plus + delta_rot)

        # State transition error , eps = {eps_trans, eps_rot1, eps_rot2}
        L = np.eye(state_size)
        L[0, 0] = np.cos(psi_last_plus + delta_rot)
        L[0, 1] = delta_tran * -np.sin(psi_last_plus + delta_rot)
        L[1, 0] = np.sin(psi_last_plus + delta_rot)
        L[1, 1] = delta_tran * np.cos(psi_last_plus + delta_rot)
        L[2, 1] = 1
        L[2, 2] = 1

        Q = L @ self.M @ L.T

        self.Pk_hat_m = A @ self.Pk @ A.T + Q

        if self.update:
            self.calculate_update()
            self.update = False
        else:
            self.xk_hat_p = self.xk_hat_m
            self.Pk_hat_p = self.Pk_hat_m

        self.xk = self.xk_hat_p
        self.Pk = self.Pk_hat_p

        self.x_hist.append(self.xk[0][0])
        self.y_hist.append(self.xk[1][0])
              
    def plot(self):
        l1x = self.xk[3][0]
        l1y = self.xk[4][0]
        l2x = self.xk[5][0]
        l2y = self.xk[6][0]
        l3x = self.xk[7][0]
        l3y = self.xk[8][0]

        lm = np.array([[l1x, l1y], [l2x, l2y], [l3x, l3y]]).T

        plt.plot(self.x_hist, self.y_hist)
        plt.scatter(lm[0], lm[1])
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


