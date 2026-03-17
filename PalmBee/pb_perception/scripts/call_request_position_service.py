#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Use AddTwoInts service to handle integer tag_id

class CallRequestPositionServiceNode(Node):
    def __init__(self):
        super().__init__('call_request_position_service_node')
        self.client = self.create_client(AddTwoInts, 'palmbee/tree/request_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service palmbee/tree/request_position...')
        self.get_logger().info('Service palmbee/tree/request_position is available.')
        self.call_service(5)  # Example tag_id, replace with dynamic input if needed

    def call_service(self, tag_id):
        request = AddTwoInts.Request()
        request.a = tag_id  # Use the first integer (a) to send the tag_id
        request.b = 0  # Placeholder value for the second integer
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded with response sum: {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CallRequestPositionServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
