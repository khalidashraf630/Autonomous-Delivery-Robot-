import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory(
        'delivery_robot_description'
    )
    gazebo_dir = get_package_share_directory('delivery_robot_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    models_dir = os.path.join(description_dir, 'models')
    model_file = os.path.join(
        models_dir, 'turtlebot3_burger_cam', 'model.sdf'
    )
    world_file = os.path.join(
        gazebo_dir, 'worlds', 'empty_world.world'
    )
    bridge_config = os.path.join(
        gazebo_dir, 'params', 'turtlebot3_burger_cam_bridge.yaml'
    )

    model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', models_dir
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'delivery_robot',
            '-file', model_file,
            '-x', '2', '-y', '0', '-z', '0.1',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        model_path,
        gazebo,
        spawn_robot,
        TimerAction(period=7.0, actions=[bridge, camera_bridge]),
    ])
