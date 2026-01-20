from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('robot2_controller')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'camera_config.yaml')
    
    # Declare launch arguments
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use configuration file for camera settings'
    )
    
    # Camera node
    camera_node = Node(
        package='robot2_controller',
        executable='camera_node',
        name='robot2_camera',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        use_config_arg,
        camera_node
    ])