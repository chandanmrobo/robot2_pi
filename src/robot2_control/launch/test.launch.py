from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot2_control',
            executable='test_pub',
            name='robot2_test_publisher',
            output='screen'
        )
    ])