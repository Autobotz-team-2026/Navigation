import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Caminhos para os pacotes e arquivos
    pkg_orion_bringup = get_package_share_directory('orion_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Substitutos para os caminhos (equivalente ao <let> do XML)
    map_path = os.path.join(pkg_orion_bringup, 'maps', 'CAF_arena.yaml')
    nav2_params_path = os.path.join(pkg_orion_bringup, 'config', 'nav2_params.yaml')

    # 2. Inclusão do launch oficial do Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    # 3. Retorno da descrição do Launch
    return LaunchDescription([
        nav2_bringup_launch
    ])