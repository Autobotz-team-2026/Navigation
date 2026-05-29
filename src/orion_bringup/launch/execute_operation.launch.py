import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Captura o caminho do pacote (assumindo orion_bringup conforme a estrutura do seu projeto)
    pkg_dir = get_package_share_directory('orion_bringup')
    
    # 1. Launch do Gazebo: Incluído normalmente (roda no terminal principal onde você deu o comando)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'orion_gazebo.launch.py')
        )
    )
    
    # 2. Launch do Navigation: Executado como um processo externo para abrir em uma janela separada
    navigation_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'orion_bringup', 'orion_navigation.launch.py'],
        prefix=['xterm -hold -e'],
        output='screen'
    )
    
    # Retorna os dois para serem iniciados juntos
    return LaunchDescription([
        gazebo_launch,
        navigation_launch
    ])