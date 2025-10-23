from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_x500 = get_package_share_directory('x500_nav2')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(pkg_x500, 'maps', 'mapa.yaml'),
                'params_file': os.path.join(pkg_x500, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'True'
            }.items()
        )
    ])
