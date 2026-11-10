
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='usb_camera',
        executable='camera_publisher',
        name='usb_camera',
        output='screen'
    )

    line_follower_node = Node(
        package='line_follower',
        executable='line_detector',
        name='line_detector',
        output='screen'
    )

    aruco_detector_node = Node(
        package='aruco_detection',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen'
    )

    aruco_monitor_node = Node(
        package='aruco_detection',
        executable='aruco_target_monitor',
        name='aruco_target_monitor',
        output='screen'
    )

    aruco_viewer_node = Node(
        package='aruco_detection',
        executable='aruco_viewer',
        name='aruco_viewer',
        output='screen'
    )

    esp32_node = Node(
        package='robot_communication',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        line_follower_node,
        aruco_detector_node,
        aruco_monitor_node,
        aruco_viewer_node,
        esp32_node
    ])
