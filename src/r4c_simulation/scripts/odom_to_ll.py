#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped
from robot_localization.srv import ToLL
from sensor_msgs.msg import NavSatFix

class OdomToLLConverter(Node):
    def __init__(self):
        super().__init__('odom_to_ll_converter')

        # Subscribe to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/r4c_tractor/odom_gzb',
            self.odom_callback,
            10
        )

        # Create publisher for NavSatFix message
        self.publisher = self.create_publisher(
            NavSatFix,
            '/r4c_tractor/gnss/fix',
            10
        )

        # Wait for the ToLL service to be available
        self.ll_service_client = self.create_client(ToLL, 'toLL')
        while not self.ll_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ToLL service not available, waiting...')

    def odom_callback(self, msg):
        # Call the ToLL service to convert odometry to latitude and longitude
        request = ToLL.Request()
        request.map_point = msg.pose.pose.position
        future = self.ll_service_client.call_async(request)
        future.add_done_callback(self.ll_callback)

    def ll_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                lat = response.ll_point.latitude
                lon = response.ll_point.longitude

                navsat_fix_msg = NavSatFix()
                navsat_fix_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_fix_msg.header.frame_id = "base_footprint"
                navsat_fix_msg.status.status = 2
                navsat_fix_msg.latitude = lat
                navsat_fix_msg.longitude = lon
                self.publisher.publish(navsat_fix_msg)
            else:
                self.get_logger().error('Failed to call ToLL service')
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToLLConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()