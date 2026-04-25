from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('first_project')
    rviz_config = os.path.join(pkg_share, 'rviz', 'first_project.rviz')

    odometer_node = Node(
        package='first_project',
        executable='odometer',
        name='odometer',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'wheel_radius': 0.085},
            {'track_width': 0.785},
            {'rpm_scale': 1.0}
        ]
    )

    tf_error_node = Node(
        package='first_project',
        executable='tf_error',
        name='tf_error',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        odometer_node,
        tf_error_node,
        rviz_node
    ])