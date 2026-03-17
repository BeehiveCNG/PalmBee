from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pb_control',
            executable='control_manager',
            name='control_manager',
            output='screen'
        )
    ])
