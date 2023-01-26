from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import matplotlib.pyplot as plt
import numpy as np

t = []
t_0 = 0.0
dt = 0.1

fig,ax = plt.subplots(1,2)

euler_mid_x = []
euler_mid_y = []

euler_fwd_x = []
euler_fwd_y = []

psi = []

with Reader('../rosbags/trajectory_estimation') as reader:

    connections = [x for x in reader.connections if x.topic == '/euler_midpoint']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = deserialize_cdr(rawdata, connection.msgtype)
        euler_mid_x.append(msg.data[0])
        euler_mid_y.append(msg.data[1])
        psi.append(msg.data[2])
        t.append(t_0)
        t_0 += dt
 
    
    ax[0].plot(euler_mid_x, euler_mid_y, color='r', label='euler_midpoint', linestyle=':')
    ax[1].plot(t, euler_mid_x, label='x_mid', linestyle=':', color='r')
    ax[1].plot(t, euler_mid_y, label='y_mid', linestyle=':', color='c')
    ax[1].plot(t, psi, label='psi')

t = []
t_0 = 0.0


with Reader('../rosbags/trajectory_estimation') as reader:
    connections = [x for x in reader.connections if x.topic == '/euler_forward']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = deserialize_cdr(rawdata, connection.msgtype)
        euler_fwd_x.append(msg.data[0])
        euler_fwd_y.append(msg.data[1])
        psi.append(msg.data[2])
        t.append(t_0)
        t_0 += dt


    ax[0].plot(euler_fwd_x, euler_fwd_y, color='b', label='forward_euler', linestyle=':')
    ax[1].plot(t, euler_fwd_x, label='x_fwd', linestyle=':', color='g')
    ax[1].plot(t, euler_fwd_y, label='y_fwd', linestyle=':', color='b')



ax[0].set_title('Forward Euler vs Euler Midpoint Trajectory')
ax[0].set_ylabel('y')
ax[0].set_xlabel('x')

ax[1].set_title('Forward Euler vs Euler Midpoint State')
ax[1].set_ylabel('State')
ax[1].set_xlabel('Time (s)')

ax[0].legend()
ax[1].legend()

plt.show()

         
