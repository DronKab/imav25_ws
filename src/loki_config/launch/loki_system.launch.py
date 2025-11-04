import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # =================================================================
    # 1. RUTAS Y CONFIGURACIONES BASE
    # =================================================================
    pkg_path = get_package_share_directory('loki_config')
    urdf_file = os.path.join(pkg_path, 'urdf', 'loki.urdf')
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file = os.path.join(depthai_prefix, "config", "driver.yaml")

    parameters=[os.path.join(
    get_package_share_directory('depthai_ros_driver'),
    'config', 'driver.yaml')]

    # =================================================================
    # 2. NODOS DEL ROBOT Y SENSORES
    # =================================================================

    # Publica el modelo URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[urdf_file]
    )

    # Nodo del LiDAR LD06
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'topic_name': 'scan'},
            {'lidar_frame': 'lidar_link'},
            {'range_threshold': 0.005}
        ]
    )

    # Nodo de odometría RF2O
    rf2o_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': 'scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 10.0
        }]
    )

    # =================================================================
    # 3. COMUNICACIÓN PX4
    # =================================================================
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
        output="log"
    )

    px4_driver_node = Node(
        package="imav25",
        executable="px4_driver",
        output="screen"
    )

    # =================================================================
    # 4. CÁMARAS
    # =================================================================

   

    # ---- Cámara USB / PiCamera ----
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        namespace="pi_camera",
        output="screen",
        parameters=[{
            "video_device": "/dev/video2",
            "image_size": [320, 240]
        }]
    )

    # =================================================================
    # 5. DEVOLVER DESCRIPCIÓN COMPLETA
    # =================================================================
    return [
        robot_state_publisher_node,
        ldlidar_node,
        rf2o_odometry_node,
        microxrce_agent,
        px4_driver_node,
        v4l2_camera_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
