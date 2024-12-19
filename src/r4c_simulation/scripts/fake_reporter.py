#!/usr/bin/env python3

import json
import sys

import rclpy
from rclpy.node import Node
from r4c_interfaces.msg import TractorStatus


class FakeReporter(Node):

    def __init__(self):
        super().__init__('fake_reporter')

        self.publisher = self.create_publisher(TractorStatus, '/lspsim/tractor_status', 10)

        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info("Fake Reporter Initialized")

    def publish_message(self):
        msg = TractorStatus()
        msg.state = 1
        msg.localization_status.state = 1
        msg.communication_status.state = 1
        msg.sensors_status.state = 1
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeReporter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
