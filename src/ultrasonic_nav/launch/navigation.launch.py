import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    nav_config = os.path.join(
        get_package_share_directory('ultrasonic_nav'),
        'config',
        'navigation.yaml'
    )

    # Start pigpiod daemon if not already running
    start_pigpiod = ExecuteProcess(
        cmd=['bash', '-c', 'sudo pigpiod || true'],
        output='screen'
    )

    sensor_node = Node(
        package='ultrasonic_nav',
        executable='ultrasonic_sensor_node',
        name='ultrasonic_sensor_node',
        parameters=[nav_config],
        output='screen'
    )

    navigation_node = Node(
        package='ultrasonic_nav',
        executable='navigation_node',
        name='navigation_node',
        parameters=[nav_config],
        output='screen'
    )

    return LaunchDescription([
        start_pigpiod,
        sensor_node,
        navigation_node
    ])
