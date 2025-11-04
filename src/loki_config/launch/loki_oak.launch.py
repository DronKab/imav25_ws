import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('loki_config')

    # ---- 1. Robot URDF ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[os.path.join(pkg_path, 'urdf', 'loki.urdf')],
        parameters=[{'use_sim_time': False}]
    )

    # ---- 2. LiDAR ----
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

    # ---- 3. Odometría RF2O ----
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

    # ---- 4. PX4 ----
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
        output="screen"
    )

    px4_driver_node = Node(
        package="imav25",
        executable="px4_driver",
        name="px4_driver",
        output="screen"
    )

    # ---- 5. Cámaras OAK-D Lite ----
    oak1_node = Node(
        package="depthai_ros_driver",
        executable="camera_node",
        name="oak1",
        output="screen",
        parameters=[os.path.join(pkg_path, "config", "oak1_rgb.yaml")]
    )

    oak2_node = Node(
        package="depthai_ros_driver",
        executable="camera_node",
        name="oak2",
        output="screen",
        parameters=[os.path.join(pkg_path, "config", "oak2_rgb.yaml")]
    )

    # ---- 6. Cámara USB / Pi ----
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        namespace="pi_camera",
        output="screen",
        parameters=[os.path.join(pkg_path, "config", "usb_camera.yaml")]
    )

    # ---- 7. Retornar todos los nodos ----
    return LaunchDescription([
        robot_state_publisher_node,
        ldlidar_node,
        rf2o_odometry_node,
        microxrce_agent,
        px4_driver_node,
        oak1_node,
        oak2_node,
        v4l2_camera_node
    ])
