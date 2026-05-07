from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuração dos caminhos
    urdf_path = PathJoinSubstitution([
        FindPackageShare('slam_odometry'),
        'urdf',
        'esteira.xacro'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('slam_odometry'),
        'rviz',
        'esteira_odometry.rviz'
    ])
    
    filter_config_path = PathJoinSubstitution([
        FindPackageShare('slam_odometry'),
        'laser_filter',
        'filter_180.yaml'
    ])

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    # Joint State Publisher GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # # Laser Filter Node
    # laser_filter_node = Node(
    #     package='laser_filters',
    #     executable='scan_to_scan_filter_chain',
    #     name='laser_filter',
    #     parameters=[filter_config_path],
    #     remappings=[
    #         ('scan', '/scan'),
    #         ('scan_filtered', '/scan_filtered')
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
        # laser_filter_node
    ])