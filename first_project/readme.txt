=========================================================================
 First Robotics Project - README
=========================================================================

Archive content
---------------
- info.txt                     : group members (codice_persona;name;surname)
- readme.txt                   : this file
- bunker_msgs/                 : the custom message package provided by
                                 the teacher (re-included so the project
                                 builds out-of-the-box on a fresh workspace)
- first_project/               : the main package with the two nodes
- screenshot/                  : top-view rviz screenshots, one per bag,
                                 named after the bag

Build
-----
    # place both packages in the src/ folder of a ROS 2 workspace
    cd <ros2_ws>
    colcon build
    source install/setup.bash

Run
---
The bag files have been re-published by the teacher with fixed headers
and MUST be played with --clock so that use_sim_time works:

    # terminal 1
    ros2 launch first_project first_project.launch.py

    # terminal 2
    ros2 bag play <bag_name> --clock

The reset service can be called at any time:

    ros2 service call /reset std_srvs/srv/Empty

Topics
------
Subscribed:
    /bunker_status     bunker_msgs/msg/BunkerStatus  (50 Hz)

Published:
    /project_odom      nav_msgs/msg/Odometry         (50 Hz)
    /tf_error_msg      first_project/msg/tf_error_msg (10 Hz)
    /tf                geometry_msgs/msg/TransformStamped
                       odom -> base_link2  (50 Hz)

TF frames
---------
    odom -> base_link     (ground truth, provided by the bag)
    odom -> base_link2    (our computed odometry)

Algorithm - odometer node
-------------------------
/bunker_status provides the robot's linear velocity v and angular
velocity w already computed by the low-level controller from the two
track encoders (this is what the assignment calls "partial data from
encoders").

The pose is integrated with Runge-Kutta 2nd order (midpoint method),
which is more accurate than simple Euler integration whenever the
robot turns while moving:

    theta_mid = theta_k + w * dt / 2
    x_{k+1}   = x_k + v * dt * cos(theta_mid)
    y_{k+1}   = y_k + v * dt * sin(theta_mid)
    theta_{k+1} = theta_k + w * dt

The time delta dt is computed from the message header timestamps.
A guard discards updates with non-positive or suspiciously large dt
(e.g. at the very start, across a bag loop, or after a reset).

The reset service (std_srvs/srv/Empty) sets x, y, theta back to zero.

Algorithm - tf_error node
-------------------------
A 10 Hz timer queries the tf tree for
    odom -> base_link    (GT)
    odom -> base_link2   (our)
and publishes a first_project/msg/tf_error_msg with:
    - tf_error           : Euclidean distance between the two origins
    - time_from_start    : seconds elapsed since the first GT sample
    - travelled_distance : integrated GT path length in metres

RViz
----
The launch file opens rviz2 with config/project.rviz preloaded.
Default view is top-down orthographic, fixed frame = odom.
Two Odometry displays (Keep = 100000) draw the GT path (green) and
our path (red); the TF display shows the three frames with their
names.

=========================================================================
