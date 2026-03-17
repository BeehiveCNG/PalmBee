# PalmBee Project Testing Guide

This guide provides detailed steps to execute various tests for the PalmBee project using SLAM and object detection with a ZED 2i camera and Jetson Orin Nano on **20/03/2025 in Mamuang**.


## Preparation

1. **Install AprilTag ROS2 driver (ROS2 Humble)**
    ```bash
    sudo apt update
    sudo apt install ros-humble-apriltag*
    ```

2. **Download and display AprilTag on a screen (or print on a piece of paper)**
    Download AprilTag markers from the directory `PalmBee/pb_perception/AprilTags/AprilTags.pdf`. Display the marker on a different monitor/screen or print it on papers.
    > **Warning:** Measure the size of the marker in meter. this measurement data will be useful to determine the coordinate of the marker.

3. **Install PalmBee in a palmbee_ws and compile it**
    ```bash
    cd ~/palmbee_ws/src
    git clone https://github.com/ep51lon/PalmBee.git
    cd ~/palmbee_ws
    colcon build
    source install/setup.bash
    ```

4. **Add the source command to your .bashrc file**
    ```bash
    echo "source ~/palmbee_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5. Open launchfile `palmbee/pb_perception/launch/zed_apriltag_launch.py` and modify the size of the tag in the file based on the measurement (for example the size of the tag is 0.916m):
    ```python
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
            'size': 0.0916,  # Modify this part!!
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    ```

6. **Install rqt_image_view package**
    ```bash
    sudo apt install ros-humble-rqt-image-view
    ```


## Test 1: RTABMAP's Stereo SLAM with ZED 2i

1. **Run the roslaunch inside pb_perception to run the ZED 2i with RTABMAP's stereo SLAM**
    ```bash
    ros2 launch pb_perception zed_stereo_slam_launch.py
    ```

2. **Check if the rtabmap viz displays a map and trajectory**
    - Open `rtabmapviz` and verify the map and trajectory are displayed.

3. **Move the camera around**
    - Observe the changes in the map and trajectory in `rtabmapviz`.


## Test 2: RTABMAP's RGBD SLAM with ZED 2i

1. **Run the roslaunch inside pb_perception to run the ZED 2i with RTABMAP's RGBD SLAM**
    ```bash
    ros2 launch pb_perception zed_rgbd_slam_launch.py
    ```

2. **Check if the rtabmap viz displays a map and trajectory**
    - Open `rtabmapviz` and verify the map and trajectory are displayed.

3. **Move the camera around**
    - Observe the changes in the map and trajectory in `rtabmapviz`.


## Test 3: Object Detection using AprilTag with ZED Camera

1. **Run the launchfile of AprilTag detection in pb_perception to detect AprilTag using ZED 2i camera**
    ```bash
    ros2 launch pb_perception zed_apriltag_launch.py
    ```

2. **Move around the camera and point the camera to the printed/displayed AprilTag marker**
    - Verify that the AprilTag is detected and its position is displayed.


## Test 4: Object Detection and Localization using Stereo SLAM

1. **Run the roslaunch inside pb_perception to run the ZED 2i with RTABMAP's stereo SLAM**
    ```bash
    ros2 launch pb_perception zed_stereo_slam_launch.py
    ```

2. **Run the launchfile of AprilTag detection in pb_perception to detect AprilTag using ZED 2i left camera**
    ```bash
    ros2 launch pb_perception zed_apriltag_launch.py
    ```

3. **Run the get_markers.py ROS2 node to get the position of detected markers in the map frame and annotate the position on an image**
    ```bash
    ros2 run pb_perception get_markers.py
    ```

4. **Open rqt_image_view to display the image from ROS2 topic `palmbee/apriltag/annotated_image`**
    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```
    Select `palmbee/apriltag/annotated_image` to display the image.

5. **Move the camera around a tag and check if the coordinates of the tags are static**
    - Verify that the coordinates of the detected tags remain static relative to the map frame.

6. **Display the tf using rviz2**
    - Open `rviz2` in a terminal:
        ```bash
        rviz2
        ```
    - Add a `TF` display type in `rviz2`.
    - Ensure the `Fixed Frame` is set to the appropriate frame (e.g., `map`).
    - Verify that the transforms between the camera, AprilTag, and other frames are displayed correctly.


## Test 5: Object Detection and Localization using RGBD SLAM

1. **Run the roslaunch inside pb_perception to run the ZED 2i with RTABMAP's RGBD SLAM**
    ```bash
    ros2 launch pb_perception zed_rgbd_slam_launch.py
    ```

2. **Run the launchfile of AprilTag detection in pb_perception to detect AprilTag using ZED 2i left camera**
    ```bash
    ros2 launch pb_perception zed_apriltag_launch.py
    ```

3. **Run the get_markers.py ROS2 node to get the position of detected markers in the map frame and annotate the position on an image**
    ```bash
    ros2 run pb_perception get_markers.py
    ```

4. **Open rqt_image_view to display the image from ROS2 topic `palmbee/apriltag/annotated_image`**
    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```
    - Select `palmbee/apriltag/annotated_image` to display the image.

5. **Move the camera around a tag and check if the coordinates of the tags are static**
    - Verify that the coordinates of the detected tags remain static relative to the map frame.

6. **Display the tf using rviz2**
    - Open `rviz2` in a terminal:
        ```bash
        rviz2
        ```
    - Add a `TF` display type in `rviz2`.
    - Ensure the `Fixed Frame` is set to the appropriate frame (e.g., `map`).
    - Verify that the transforms between the camera, AprilTag, and other frames are displayed correctly.