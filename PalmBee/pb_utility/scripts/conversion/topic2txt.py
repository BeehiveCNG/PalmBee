#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Topic2Text(Node):

    def __init__(self):
        super().__init__('topic2text')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.file = open('rtabmap_slam2.txt', 'a')

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.file.write(f"{timestamp} {position.x} {position.y} {position.z} {orientation.x} {orientation.y} {orientation.z} {orientation.w}\n")

    def __del__(self):
        self.file.close()

def main(args=None):
    rclpy.init(args=args)
    node = Topic2Text()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()