import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tracetory_estimator_node = Node(
        package='trajectory_control',
        executable='state_trajectory_estimator',
        name='estimator',
    )


    return LaunchDescription([
        tracetory_estimator_node,
    ])


