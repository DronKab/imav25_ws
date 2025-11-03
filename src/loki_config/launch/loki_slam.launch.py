import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------------------------
    # 1. Rutas y Archivos
    # ----------------------------------------------------
    
    pkg_name = 'loki_config'
    pkg_path = get_package_share_directory(pkg_name)
    
    # Usaremos un archivo YAML para la configuración de SLAM Toolbox
    # Esto asegura que los límites del sensor (15.0 m) se carguen correctamente
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_params.yaml')

    # ----------------------------------------------------
    # 2. Argumentos de Lanzamiento (Permiten anular valores por defecto)
    # ----------------------------------------------------
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Nombre del tópico LaserScan de entrada.'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Establecer en True para entornos de simulación.'
    )
    
    # ----------------------------------------------------
    # 3. Nodos
    # ----------------------------------------------------

    # NODO: SLAM Toolbox (Odometría y Mapeo)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        # Usamos el modo síncrono para mayor precisión en la cartografía
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        
        # Carga de parámetros: primero el archivo YAML, luego las anulaciónes
        parameters=[
            slam_params_file,
            {'scan_topic': LaunchConfiguration("scan_topic")},
            {'use_sim_time': LaunchConfiguration("use_sim_time")}
        ]
    )

    # ----------------------------------------------------
    # 4. Devolver LaunchDescription
    # ----------------------------------------------------
    
    return LaunchDescription([
        scan_topic_arg,
        use_sim_time_arg,
        slam_toolbox_node,
    ])  