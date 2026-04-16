import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('orion_description'),
        'urdf', 'orion.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('orion_description'),
        'rviz', 'urdf_config.rviz'
    )

    gazebo_config_path = os.path.join(
        get_package_share_directory('orion_bringup'),
        'config', 'gazebo_bridge.yaml'
    )

    world_path = os.path.join(
        get_package_share_directory('orion_bringup'),
        'worlds', 'CAF_arena.sdf'
    )

    robot_description = Command(['xacro ', urdf_path])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': world_path + ' -r'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        spawn_robot,
        robot_state_publisher,
        rviz,
    ])