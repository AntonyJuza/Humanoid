"""
launch/flamebot_full.launch.py

Complete flamebot launch file that starts all components in the following order:
  1. camera_stream.py          — camera streaming
  2. humanoid_motor_control    — motor control (cytron_controller, encoder_node, diff_drive_controller)
  3. ros2_mpu6050              — IMU sensor (mpu6050_sensor)
  4. sensor_pkg                — environmental sensors (dht22_node, mq2_node, flame_node)
  5. rover_navigation          — navigation (ultrasonic_sensor_node, navigation_node)
"""

import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # ============================================================================
    # 1. CAMERA STREAM (camera_stream.py)
    # ============================================================================
    # Get the path to camera_stream.py
    camera_stream_py = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'camera_stream.py'
    )
    
    camera_stream_node = Node(
        package='humanoid_vlm_bridge',  # or use a generic launcher
        executable='python3',
        arguments=[camera_stream_py],
        name='camera_stream',
        output='screen',
        emulate_tty=True,
    )

    # ============================================================================
    # 2. HUMANOID MOTOR CONTROL
    # ============================================================================
    motor_control_config = PathJoinSubstitution([
        FindPackageShare('humanoid_motor_control'),
        'config',
        'motor_control.yaml'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Cytron motor controller node
    cytron_node = Node(
        package='humanoid_motor_control',
        executable='cytron_controller_node',
        name='cytron_controller',
        output='screen',
        emulate_tty=True,
        parameters=[motor_control_config, {'use_sim_time': use_sim_time}]
    )

    # Encoder reader node
    encoder_node = Node(
        package='humanoid_motor_control',
        executable='encoder_node',
        name='encoder_reader',
        output='screen',
        emulate_tty=True,
        parameters=[motor_control_config, {'use_sim_time': use_sim_time}]
    )

    # Differential drive controller node
    diff_drive_node = Node(
        package='humanoid_motor_control',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        emulate_tty=True,
        parameters=[motor_control_config, {'use_sim_time': use_sim_time}]
    )

    # ============================================================================
    # 3. ROS2 MPU6050 (IMU Sensor)
    # ============================================================================
    mpu6050_config = os.path.join(
        get_package_share_directory('ros2_mpu6050'),
        'config',
        'params.yaml'
    )

    mpu6050_node = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        name='mpu6050_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[mpu6050_config]
    )

    # ============================================================================
    # 4. SENSOR_PKG (Environmental Sensors: DHT22, MQ2, Flame)
    # ============================================================================
    # DHT22 Temperature & Humidity Sensor
    dht22_node = Node(
        package='sensor_pkg',
        executable='dht22_node',
        name='dht22_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gpio_pin': 27,
            'publish_rate': 0.5
        }]
    )

    # MQ2 Gas Sensor
    mq2_node = Node(
        package='sensor_pkg',
        executable='mq2_node',
        name='mq2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'spi_channel': 0,
            'digital_gpio': 17,
            'publish_rate': 2.0,
            'gas_threshold': 300.0
        }]
    )

    # Flame Sensor
    flame_node = Node(
        package='sensor_pkg',
        executable='flame_node',
        name='flame_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'digital_gpio': 18,
            'spi_channel': 1,
            'publish_rate': 5.0
        }]
    )

    # ============================================================================
    # 5. ROVER NAVIGATION
    # ============================================================================
    nav_config = os.path.join(
        get_package_share_directory('rover_navigation'),
        'config',
        'navigation.yaml'
    )

    ultrasonic_node = Node(
        package='rover_navigation',
        executable='ultrasonic_sensor_node',
        name='ultrasonic_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[nav_config]
    )

    navigation_node = Node(
        package='rover_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        emulate_tty=True,
        parameters=[nav_config]
    )

    # ============================================================================
    # BUILD LAUNCH DESCRIPTION IN ORDER
    # ============================================================================
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        )
    )

    # 1. Camera stream
    ld.add_action(camera_stream_node)

    # 2. Motor control
    ld.add_action(cytron_node)
    ld.add_action(encoder_node)
    ld.add_action(diff_drive_node)

    # 3. MPU6050 IMU
    ld.add_action(mpu6050_node)

    # 4. Environmental sensors
    ld.add_action(dht22_node)
    ld.add_action(mq2_node)
    ld.add_action(flame_node)

    # 5. Navigation
    ld.add_action(ultrasonic_node)
    ld.add_action(navigation_node)

    return ld
