#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile

parameters = []
remappings = []

def launch_setup(context: LaunchContext, *args, **kwargs):
    parameters=[{'frame_id': 'zed_camera_link',
                 'subscribe_stereo': True,
                 'approx_sync': False,
                 'wait_imu_to_init': True}]

    # remappings=[
    #     ('imu', '/zed/zed_node/imu/data'),
    #     ('left/image_rect', '/zed/zed_node/left/image_rect_color'),
    #     ('right/image_rect', '/zed/zed_node/right/image_rect_color'),
    #     ('left/camera_info', '/zed/zed_node/left/camera_info'),
    #     ('right/camera_info', '/zed/zed_node/right/camera_info')
    # ]
    remappings=[
        ('imu', '/zed/zed_node/imu/data'),
        ('left/image_rect', '/zed/zed_node/left_gray/image_rect_gray'),
        ('right/image_rect', '/zed/zed_node/right_gray/image_rect_gray'),
        ('left/camera_info', '/zed/zed_node/left_gray/camera_info'),
        ('right/camera_info', '/zed/zed_node/right_gray/camera_info')
    ]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})
    
    return [
        # Launch ZED camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('pb_perception'), 'launch'),
                '/zed_stereo_launch.py']),
                launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                  'publish_tf': LaunchConfiguration('use_zed_odometry'),
                                  'publish_map_tf': 'false'}.items(),
        ),

        # Launch SLAM tf publisher world->slam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('pb_perception'), 'launch'),
                '/zed_tf_publisher_launch.py']),
        ),

        # # Sensing and perception manager
        # Node(
        #     package='pb_perception', executable='sensing_manager.py', output='screen',
        # ),

        # Stereo Odometry Node
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # Include Foxglove Bridge launch file
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('foxglove_bridge'),
        #             'launch',
        #             'foxglove_bridge_launch.xml'
        #         )
        #     ),
        # ),
    ]

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description='Use ZED\'s computed odometry instead of using RTAB-Map\'s stereo odometry.'),
        
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description="The model of the ZED camera. Options: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),
        
        OpaqueFunction(function=launch_setup)
    ])