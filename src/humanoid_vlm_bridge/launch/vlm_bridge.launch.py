from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_vlm_bridge'),
        'config',
        'vlm_config.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # VLM node (runs on gaming PC)
    vlm_node = Node(
        package='humanoid_vlm_bridge',
        executable='vlm_node',
        name='vlm_processor',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    # Intent executor node
    intent_executor = Node(
        package='humanoid_vlm_bridge',
        executable='intent_executor',
        name='intent_executor',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        vlm_node,
        intent_executor,
    ])