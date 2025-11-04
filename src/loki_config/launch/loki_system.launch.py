import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =================================================================
    # 1. PARÁMETROS GENERALES DEL ROBOT (URDF)
    # =================================================================
    
    # Obtener ruta al paquete y al URDF
    pkg_path = get_package_share_directory('loki_config')
    urdf_file = os.path.join(pkg_path, 'urdf', 'loki.urdf')

    # Argumento para el URDF
    urdf_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file,
        description='Ruta al archivo URDF del robot'
    )
    
    # =================================================================
    # 2. PARÁMETROS ESPECÍFICOS DEL LIDAR LD06
    # =================================================================
    
    lidar_serial_port_arg = DeclareLaunchArgument(
        name='serial_port', 
        default_value='/dev/ttyUSB0',
        description='LD06 Serial Port'
    )
    lidar_topic_name_arg = DeclareLaunchArgument(
        name='topic_name', 
        default_value='scan',
        description='LD06 Topic Name'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        name='lidar_frame', 
        default_value='lidar_link',
        description='Lidar Frame ID'
    )
    lidar_range_threshold_arg = DeclareLaunchArgument(
        name='range_threshold', 
        default_value='0.005',
        description='Range Threshold'
    )

    # =================================================================
    # 3. DEFINICIÓN DE NODOS
    # =================================================================
    
    # NODO: robot_state_publisher (Publica el modelo URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[LaunchConfiguration('model')]
    )

    # NODO: ldlidar (Controlador del LiDAR LD06)
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'serial_port': LaunchConfiguration("serial_port")},
            {'topic_name': LaunchConfiguration("topic_name")},
            {'lidar_frame': LaunchConfiguration("lidar_frame")},
            {'range_threshold': LaunchConfiguration("range_threshold")}
        ]
    )
    
    # 🆕 NODO: rf2o_laser_odometry (Odometría a partir del LiDAR)
    rf2o_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            # El tópico de entrada debe coincidir con la salida del ldlidar_node
            'laser_scan_topic': LaunchConfiguration("topic_name"), # Usamos el argumento 'scan'
            'odom_topic': '/odom',
            'publish_tf': True,
            # Asegúrate de que 'base_link' y 'odom' coincidan con tu configuración
            'base_frame_id': 'base_link', 
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0}],
    )


    # =================================================================
    # 4. DEVOLVER DESCRIPCIÓN DEL LAUNCH
    # =================================================================
    
    return LaunchDescription([
        # 4.1. Argumentos del URDF
        urdf_arg,
        
        # 4.2. Argumentos del LiDAR
        lidar_serial_port_arg,
        lidar_topic_name_arg,
        lidar_frame_arg,
        lidar_range_threshold_arg,

        # 4.3. Nodos
        robot_state_publisher_node,
        ldlidar_node,
        rf2o_odometry_node,  # 🚀 Nuevo nodo de Odometría
        
        # Puedes incluir joint_state_publisher_gui_node si lo necesitas para depuración
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # )
    ])