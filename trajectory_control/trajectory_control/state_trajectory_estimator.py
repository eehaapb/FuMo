import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StateTrajectoryEstimator(Node):
    def __init__(self):
        super().__init__("state_estimator")

        self.euler_midpoint_pub = self.create_publisher(Float64MultiArray, "/euler_midpoint", 10)
        self.euler_forward_pub = self.create_publisher(Float64MultiArray, "/euler_forward", 10)

        self.odom_sub = self.create_subscription(Float64MultiArray, "Wheels_data", self.odom_callback, 10)

        # self.prev_time = Time.now()

        self.x_prev_fwd = 0.0
        self.y_prev_fwd = 0.0

        self.x_prev_midp = 0.0
        self.y_prev_midp = 0.0

        self.psi_prev = 0.0
    
    
    def odom_callback(self, msg):
        # current_time = Time.now()
        # dt = current_time - self.prev_time

        # self.prev_time = current_time

        dt = 0.1

        wl = self.rpm_to_rads(msg.data[0])
        wr = self.rpm_to_rads(msg.data[1])

        vx = 0.1/2*(wr + wl)
        wz = 0.1/0.4*(wr - wl)

        fwd_euler = Float64MultiArray()
        fwd_euler.data = [self.x_prev_fwd + vx*dt*math.cos(self.psi_prev), self.y_prev_fwd + vx*dt*math.sin(self.psi_prev),
        self.psi_prev + wz*dt]

        self.x_prev_fwd = fwd_euler.data[0]
        self.y_prev_fwd = fwd_euler.data[1]

        euler_midp = Float64MultiArray()
        euler_midp.data = [self.x_prev_midp + vx*dt*math.cos(self.psi_prev + 1/2*wz*dt), self.y_prev_midp + vx*dt*math.sin(self.psi_prev + 1/2*wz*dt),
        self.psi_prev + wz*dt]

        self.x_prev_midp = euler_midp.data[0]
        self.y_prev_midp = euler_midp.data[1]
        self.psi_prev = euler_midp.data[2]

        self.euler_forward_pub.publish(fwd_euler)
        self.euler_midpoint_pub.publish(euler_midp)


    def rpm_to_rads(self, rpm):
        return rpm/60*2*math.pi

    
def main(args=None):
    rclpy.init()
    estimator = StateTrajectoryEstimator()
    rclpy.spin(estimator)

if __name__ == "__main__":
    main()