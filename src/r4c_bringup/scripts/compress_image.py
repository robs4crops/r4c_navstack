#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('image_republisher')
        self.subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(CompressedImage, 'output_image', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform image compression using OpenCV
            _, compressed_data = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            
            # Create a CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = compressed_data.tobytes()

            compressed_image_msg.header = msg.header
            
            # Publish the compressed image
            self.publisher.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error("Error processing image: {}".format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    image_republisher = ImageRepublisher()
    rclpy.spin(image_republisher)
    image_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()