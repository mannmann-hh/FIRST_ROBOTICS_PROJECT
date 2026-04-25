cat > ~/colcon_ws/src/first_project/launch/first_project.launch.py << 'LAUNCH_EOF'
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('first_project')
    rviz_config = os.path.join(pkg_share, 'config', 'first_project.rviz')

    use_sim_time = {'use_sim_time': True}
    odometry_params = {
        'rpm_to_mps':  0.001172,
        'track_width': 0.555,
    }

    return LaunchDescription([
        Node(
            package='first_project',
            executable='odometer',
            name='odometer',
            output='screen',
            parameters=[use_sim_time, odometry_params],
        ),
        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            output='screen',
            parameters=[use_sim_time],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[use_sim_time],
        ),
    ])
LAUNCH_EOF
echo "launch file updated!"