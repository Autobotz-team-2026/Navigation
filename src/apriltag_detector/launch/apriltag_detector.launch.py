"""
Launch para o apriltag_pose_node.

Ajuste os parâmetros conforme o tamanho real da sua tag impressa
e os tópicos da sua câmera.
"""

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
                'tag_size': 0.05,                                 
                'tag_family': 'DICT_APRILTAG_36h11',
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'camera_optical_frame': 'camera_optical_frame',   
            }],
        ),
    ])
