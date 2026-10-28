import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('delivery_robot_gazebo'),
        'launch',
        'delivery_robot.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        Node(
            package='line_follower',
            executable='line_detector',
            name='line_detector',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen'
        ),

        Node(
            package='aruco_detection',
            executable='aruco_viewer',
            name='aruco_viewer',
            output='screen'
        ),
    ])
