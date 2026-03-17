#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare launch argument for image topic source
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        # default_value='/zed/zed_node/left/image_rect_color',
        default_value='/zed/zed_node/left_gray/image_rect_gray',
        description='Image topic for AprilTag detection'
    )

    camera_info_arg = DeclareLaunchArgument(
        'camera_info_topic',
        # default_value='/zed/zed_node/left/camera_info',
        default_value='/zed/zed_node/left_gray/camera_info',
        description='Camera info topic'
    )

    # Define the AprilTag detection node
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag',
        namespace='apriltag',
        remappings=[
            ('/apriltag/image_rect', LaunchConfiguration('image_topic')),
            ('/apriltag/camera_info', LaunchConfiguration('camera_info_topic'))
        ],
        parameters=[{
            'family': '36h11',
            'size': 0.1434, # 0.0916,  # Set AprilTag size in meters
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Define the container to hold the AprilTag node
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_arg,
        container
    ])
