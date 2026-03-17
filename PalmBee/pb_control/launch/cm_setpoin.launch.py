from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pb_control',
            executable='cm_setpoint',
            name='cm_setpoint',
            output='screen'
        )
    ])
