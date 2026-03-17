#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point  # Import Point message
from tf2_ros import TransformException
import math
from apriltag_msgs.msg import AprilTagDetectionArray  # Import the AprilTagDetectionArray message
import pandas as pd  # Import pandas
from sensor_msgs.msg import Image  # Import Image message
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge and CvBridgeError
import cv2  # Import OpenCV
import numpy as np  # Import NumPy
from example_interfaces.srv import AddTwoInts  # Import AddTwoInts service

def calculate_distance(transform):
    translation = transform.transform.translation
    distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
    return distance

def print_transform(node, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    node.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
    node.get_logger().info(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

class CalculateDistanceNode(Node):
    def __init__(self):
        super().__init__('calculate_distance_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.detections = []
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'apriltag/detections',
            self.detection_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            # 'camera/infra1/image_rect_raw',
            'zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )
        self.image_publisher = self.create_publisher(Image, 'palmbee/apriltag/annotated_image', 10)
        self.tree_position_publisher = self.create_publisher(Point, 'palmbee/current_tree_position', 10)
        self.bridge = CvBridge()
        self.current_image = None
        self.df = pd.DataFrame(columns=['timestamp', 'tag_id', 'pos_x', 'pos_y', 'pos_z', 'distance'])
        self.last_append_time = self.get_clock().now().to_msg().sec
        self.tag_detection_times = {}
        self.last_detected_tag_id = None
        self.last_detected_position = None
        self.user_defined_tag_id = 0  # User-defined tag_id to keep publishing its position
        self.srv = self.create_service(AddTwoInts, 'update_tag_id', self.update_tag_id_callback)  # Create service

    def update_tag_id_callback(self, request, response):
        self.user_defined_tag_id = request.a  # Use the first integer as the tag_id
        response.sum = self.user_defined_tag_id  # Return the updated tag_id as the sum
        self.get_logger().info(f"Updated user_defined_tag_id to {self.user_defined_tag_id}")
        return response

    def detection_callback(self, msg):
        self.detections = [detection for detection in msg.detections]

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg().sec
        if self.current_image is not None:
            annotated_image = self.current_image.copy()
            for detection in self.detections:
                tag_id = detection.id
                try:
                    transform = self.tf_buffer.lookup_transform('map', f'tag36h11:{tag_id}', rclpy.time.Time())
                    # transform = self.tf_buffer.lookup_transform('camera_infra1_optical_frame', f'tag36h11:{tag_id}', rclpy.time.Time())
                    print_transform(self, transform)
                    distance = calculate_distance(transform)
                    self.get_logger().info(f"Marker {tag_id} -> Distance: {distance}")

                    # Draw box and label on the image
                    corners = [(int(point.x), int(point.y)) for point in detection.corners]
                    cv2.polylines(annotated_image, [np.array(corners, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
                    translation = transform.transform.translation
                    label = (f"ID: {tag_id},"
                             f"Pos: ({translation.x:.2f}, {translation.y:.2f}, {translation.z:.2f}),"
                             f"Dist: {distance:.2f}m")
                    cv2.putText(annotated_image, label, (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # Save to DataFrame
                    timestamp = self.get_clock().now().to_msg().sec
                    new_row = pd.DataFrame([{
                        'timestamp': timestamp,
                        'tag_id': tag_id,
                        'pos_x': translation.x,
                        'pos_y': translation.y,
                        'pos_z': translation.z,
                        'distance': distance
                    }])
                    self.df = pd.concat([self.df, new_row], ignore_index=True)
                    
                    # Track detection time
                    if tag_id not in self.tag_detection_times:
                        self.tag_detection_times[tag_id] = current_time
                    elif current_time - self.tag_detection_times[tag_id] >= 3:
                        # Publish position if detected for 3 seconds
                        point = Point(x=translation.x, y=translation.y, z=translation.z)
                        self.tree_position_publisher.publish(point)
                        self.get_logger().info(f"Published position of tag {tag_id} to palmbee/current_tree_position")
                        self.last_detected_tag_id = tag_id
                        self.last_detected_position = point
                        del self.tag_detection_times[tag_id]
                except TransformException as ex:
                    self.get_logger().warn(f"Transform for marker {tag_id} not available")
                    if tag_id in self.tag_detection_times:
                        del self.tag_detection_times[tag_id]

            # Publish the annotated image
            try:
                annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                self.image_publisher.publish(annotated_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to convert annotated image: {e}")
        else:
            self.get_logger().info(f"Waiting for tf...")

        if self.user_defined_tag_id and self.user_defined_tag_id not in [detection.id for detection in self.detections]:
            # Publish the last detected position if the user-defined tag is no longer detected
            self.tree_position_publisher.publish(self.last_detected_position)
            self.get_logger().info(f"Re-published last detected position of user-defined tag {self.user_defined_tag_id} to palmbee/current_tree_position")

        if current_time - self.last_append_time >= 5:
            self.save_to_csv()
            self.last_append_time = current_time

    def save_to_csv(self):
        self.df.to_csv('detected_markers.csv', index=False)

def main(args=None):
    rclpy.init(args=args)
    node = CalculateDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()