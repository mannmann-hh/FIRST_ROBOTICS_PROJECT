#!/usr/bin/env python3
"""Launch file for the first_project package.

Starts:
  - odometer  : computes odometry from /bunker_status
  - tf_error  : compares GT tf (odom->base_link) against our tf (odom->base_link2)
  - rviz2     : shows the two trajectories in top view

All nodes use simulated time, so the bag must be played with --clock:
    ros2 bag play <bag_name> --clock
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('first_project')
    rviz_config = os.path.join(pkg_share, 'config', 'project.rviz')

    use_sim_time = {'use_sim_time': True}

    odometer_node = Node(
        package='first_project',
        executable='odometer',
        name='odometer',
        output='screen',
        parameters=[use_sim_time],
    )

    tf_error_node = Node(
        package='first_project',
        executable='tf_error',
        name='tf_error',
        output='screen',
        parameters=[use_sim_time],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[use_sim_time],
    )

    return LaunchDescription([
        odometer_node,
        tf_error_node,
        rviz_node,
    ])
