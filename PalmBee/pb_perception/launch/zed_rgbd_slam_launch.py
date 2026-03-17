#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition


parameters = []
remappings = []


def launch_setup(context: LaunchContext, *args, **kwargs):

    # =====================================================
    # TEMP YAML OVERRIDE (ZED FULL CONFIG ENABLE)
    # =====================================================
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:

        zed_override_file.write(
            "---\n"
            "/**:\n"
            "  ros__parameters:\n"
            "\n"
            "    general:\n"
            "      grab_resolution: 'VGA'\n"
            "\n"
            "    depth:\n"
            "      depth_mode: 'PERFORMANCE'\n"
            "      min_depth: 0.3\n"
            "      max_depth: 10.0\n"
            "\n"
            "    pos_tracking:\n"
            "      pos_tracking_enabled: true\n"
            "      publish_tf: false\n"
            "\n"
            "    object_detection:\n"
            "      od_enabled: true\n"
            "      model: 'CUSTOM_BOX_OBJECTS'\n"
            "      tracking: true\n"
        )

        zed_override_file.flush()

    # =====================================================
    # RTABMAP PARAMETERS
    # =====================================================
    parameters = [{
        'frame_id': 'zed_camera_link',
        'subscribe_rgbd': True,
        'approx_sync': False,
        'wait_imu_to_init': True
    }]

    remappings = [('imu', '/zed/zed_node/imu/data')]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})

    return [

        # =====================================================
        # ZED CAMERA DRIVER
        # =====================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py'
                )
            ]),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'ros_params_override_path': zed_override_file.name,
                'publish_tf': LaunchConfiguration('use_zed_odometry'),
                'publish_map_tf': 'false'
            }.items(),
        ),

        # =====================================================
        # RGBD SYNC
        # =====================================================
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=parameters,
            remappings=[
                ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                ('depth/image', '/zed/zed_node/depth/depth_registered')
            ]),

        # =====================================================
        # VISUAL ODOMETRY
        # =====================================================
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            condition=UnlessCondition(
                LaunchConfiguration('use_zed_odometry')),
            parameters=parameters,
            remappings=remappings),

        # =====================================================
        # SLAM
        # =====================================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
    ]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_zed_odometry',
            default_value='false',
            description='Use ZED odometry instead of RTABMAP odometry.'
        ),

        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model'
        ),

        OpaqueFunction(function=launch_setup)
    ])

