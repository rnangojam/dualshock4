from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prototype_move',
            executable='teleop_keyboard',
            name='teleop_keyboard_node',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='prototype_motor',
            executable='nurimotor',
            name='nurimotor_node',
            output='screen',
        )
    ])