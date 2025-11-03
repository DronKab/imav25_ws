import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =================================================================
    # 1. PARÁMETROS GENERALES DEL ROBOT (URDF)
    # =================================================================
    pkg_path = get_package_share_directory('loki_config')
    urdf_file = os.path.join(pkg_path, 'urdf', 'loki.urdf')

    urdf_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_file,
        description='Ruta al archivo URDF del robot'
    )
    
    # =================================================================
    # 2. PARÁMETROS DEL LIDAR LD06
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
    # 3. NODOS DEL SISTEMA ROBÓTICO
    # =================================================================
    
    # Publica el modelo URDF del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[LaunchConfiguration('model')]
    )

    # Nodo del LiDAR LD06
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

    # Nodo de odometría basada en láser (RF2O)
    rf2o_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': LaunchConfiguration("topic_name"),
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link', 
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }]
    )

    # =================================================================
    # 4. NODOS DE COMUNICACIÓN Y VISIÓN
    # =================================================================
    
    # Micro XRCE Agent para conexión con PX4
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
        output="log"
    )

    # Nodo de control PX4
    px4_driver_node = Node(
        package="imav25",
        executable="px4_driver",
        output="screen"
    )

    # Nodo de cámara (v4l2_camera)
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        remappings=[
            ("__ns", "/pi_camera")
        ],
        parameters=[{
            "video_device": "/dev/video2",
            "image_size": [320, 240]
        }]
    )


    # =================================================================
    # 5. DEVOLVER DESCRIPCIÓN COMPLETA
    # =================================================================
    return LaunchDescription([
        # Argumentos
        urdf_arg,
        lidar_serial_port_arg,
        lidar_topic_name_arg,
        lidar_frame_arg,
        lidar_range_threshold_arg,

        # Nodos del sistema base
        robot_state_publisher_node,
        ldlidar_node,
        rf2o_odometry_node,

        # Nodos PX4 + visión
        microxrce_agent,
        px4_driver_node,
        v4l2_camera_node,
    ])
