#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.local_position = None
        self.visual_odometry = None

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos
        )
        self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.visual_odometry_callback,
            qos
        )

    def local_position_callback(self, msg):
        self.local_position = msg
        self.display_data()

    def visual_odometry_callback(self, msg):
        self.visual_odometry = msg
        self.display_data()

    def display_data(self):
        if self.local_position is not None and self.visual_odometry is not None:
            # VehicleLocalPosition: x, y, z
            # VehicleOdometry: position[0], position[1], position[2]
            self.get_logger().info(
                f"Local: {self.local_position.x:.3f}, {self.local_position.y:.3f}, {self.local_position.z:.3f}, "
                f"VIO: {self.visual_odometry.position[0]:.3f}, {self.visual_odometry.position[1]:.3f}, {self.visual_odometry.position[2]:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PositionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()