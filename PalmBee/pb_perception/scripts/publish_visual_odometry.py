#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from px4_msgs.msg import VehicleOdometry
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np

class VisualOdometryPublisher(Node):
    def __init__(self):
        super().__init__('visual_odometry_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )

        self.published = False

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        try:
            if self.published == False:
                self.get_logger().info("Publishing TF...")
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'world', 'zed_camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )

            msg = VehicleOdometry()

            # Timestamp (convert from ROS Time)
            now = self.get_clock().now().to_msg()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000  # px4 expects time in microseconds

            # Position
            msg.position = [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            ]

            # Orientation
            msg.q = [
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z
            ]

            # Set velocity and covariance to unknown
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD  # assuming world = FRD
            # msg.q_offset = [1.0, 0.0, 0.0, 0.0]
            msg.position_variance = [1e-3, 1e-3, 1e-3]
            msg.orientation_variance = [1e-3, 1e-3, 1e-3]
            msg.velocity_variance = [1e2, 1e2, 1e2]  # very high uncertainty

            self.publisher.publish(msg)
            # self.get_logger().info('Published visual odometry')
            self.published = True

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()