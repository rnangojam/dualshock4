from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ds4_driver',
            executable='ds4_test_node.py',
            name='ds4_test_node',
            output='screen',
        )
    ])