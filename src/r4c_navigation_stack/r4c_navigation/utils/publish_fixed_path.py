#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from eut_crop_row_estimator.srv import GetPath


class PathPublisher(Node):

    def __init__(self, path):
        super().__init__('publish_fixed_path')

        self.fixed_path_publisher = self.create_publisher(Path, 'fixed_path', 10)
        self.srv = self.create_service(GetPath, 'fixed_path/get_path', self.getPathCallback)

        # Define frames
        self.frame_id = "base_footprint"
    
        self.last_path = Path()
        self.time_tolerance = 5.0

        # Create timer
        rate = 20.0 # Hz
        timer_period = 1/rate # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        path = Path()
        path.header = header

        # Point 1
        ps = PoseStamped()
        ps.header = header
        ps.pose.orientation.w = 1.0
        ps.pose.position.x = 0.0
        ps.pose.position.y = 0.0
        path.poses.append(ps)

        # Point 2
        ps = PoseStamped()
        ps.header = header
        ps.pose.orientation.w = 1.0
        ps.pose.position.x = 0.0
        ps.pose.position.x = 10.0
        ps.pose.position.y = 0.0
        path.poses.append(ps)

        self.last_path = path

        # Publish path
        self.fixed_path_publisher.publish(path)
    
    def getPathCallback(self, request, response):
        now_stamp = self.get_clock().now().to_msg()
        now_stamp_sec = now_stamp.sec + now_stamp.nanosec * 1e-9
        path_stamp = self.last_path.header.stamp
        path_stamp_sec = path_stamp.sec + path_stamp.nanosec * 1e-9
        elapsed_time_sec = now_stamp_sec - path_stamp_sec
        
        
        if elapsed_time_sec < self.time_tolerance:
            response.path = self.last_path
        else:
            empty_path = Path()
            response.path = empty_path

        return response


def main(args=None):
    rclpy.init(args=args)

    # Create path you desire to follow
    path = Path()
    
    path_publisher = PathPublisher(path)

    rclpy.spin(path_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()