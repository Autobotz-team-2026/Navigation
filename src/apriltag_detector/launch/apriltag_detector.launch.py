from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_detector',
            executable='apriltag_pose_node',
            name='apriltag_pose_node',
            output='screen',
            parameters=[{
                'tag_size': 0.038,
                'tag_family': 'DICT_APRILTAG_36h11',
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'target_frame': 'base_gripper_link',
                'detection_scale': 2.0,
                'try_inverted': True,
                'log_interval_sec': 1.0,
            }],
        ),
    ])