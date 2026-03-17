from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # map -> slam (rotate -90° around Z = yaw = -π/2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_slam_tf',
            arguments=[
                '0', '0', '0',          # translation
                # '-1.5708', '0.0', '0.0',  # yaw, pitch, roll # Realsense
                '0.0', '0.0', '0.0',  # yaw, pitch, roll # ZED
                'slam', 'odom'           # parent → child
            ],
            output='screen'
        ),

        # slam -> world (rotate π around X = roll = π)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_slam_to_world_tf',
            arguments=[
                '0', '0', '0',          # translation
                '0.0', '0.0', '3.14159',  # yaw, pitch, roll
                'world', 'odom'         # parent → child
            ],
            output='screen'
        ),
    ])