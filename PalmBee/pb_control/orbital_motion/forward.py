#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PointStamped

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Inisialisasi publisher
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            10
        )
        
        # Ganti subscription ke PointStamped
        self.create_subscription(
            PointStamped,
            '/palmbee/tree/target_position',
            self.publish_trajectory_setpoint,
            10
        )

    def publish_trajectory_setpoint(self, msg):
        """Langsung memproses data posisi dan menerbitkan trajectory setpoint."""
        x_safe = 0.0  
        y_safe = 2.0  

        # Hitung posisi drone berdasarkan target
        x = msg.point.x - x_safe  
        y = msg.point.y - y_safe  
        z = msg.point.z

        # Buat pesan trajectory setpoint
        msg_trajectory = TrajectorySetpoint()
        msg_trajectory.position = [float(x), float(y), float(z)]
        msg_trajectory.yaw = float('nan')
        msg_trajectory.timestamp = self.get_clock().now().nanoseconds // 1000  

        # Publikasikan trajectory setpoint
        self.trajectory_setpoint_publisher.publish(msg_trajectory)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()