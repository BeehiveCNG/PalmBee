#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Initialize YOLOv8 model
        self.get_logger().info("Loading YOLOv8 model...")
        # self.model = YOLO('yolov8n.pt')  # Use a lightweight YOLOv8 model
        model_dir = '/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/model'
        # self.model = YOLO(f'{model_dir}/best_bunga_buah.pt')  # Bunga-buah
        self.model = YOLO(f'{model_dir}/best_pohon.pt')  # pohon
        # self.model = YOLO(f'{model_dir}/yolov8n.pt')  # Use a lightweight YOLOv8 model
        self.model.to('cuda')  # Use GPU for inference if available

        # ROS2 subscription to image topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/right/image_rect_color',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Object Detection Node has started.")

    def image_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            results = self.model(cv_image)

            # Draw bounding boxes on the image
            annotated_image = results[0].plot()

            # Display the image (for debugging purposes)
            cv2.imshow("YOLOv8 Object Detection", annotated_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()