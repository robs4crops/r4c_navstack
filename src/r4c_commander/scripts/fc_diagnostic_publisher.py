#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist


class FCDiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('fc_diagnostic_publisher')

        self.declare_parameter('diagnostic_frequency', 1.0)

        self.declare_parameter('navsat_fix_topic_name',"fix")
        self.declare_parameter('velocity_topic_name',"vel")

        self.declare_parameter('diagnostic_trigger_topic_name',"trigger_flag")
        self.declare_parameter('diagnostic_emergency_stop_topic_name',"emergency_stop_flag")
        self.declare_parameter('diagnostic_speed_topic_name',"speed")
        self.declare_parameter('diagnostic_navsatfix_topic_name',"Nav")

        self.diagnostic_frequency = self.get_parameter('diagnostic_frequency').value
        self.navsat_fix_topic_name = self.get_parameter('navsat_fix_topic_name').value
        self.velocity_topic_name = self.get_parameter('velocity_topic_name').value
        self.diagnostic_trigger_topic_name = self.get_parameter('diagnostic_trigger_topic_name').value
        self.diagnostic_emergency_stop_topic_name = self.get_parameter('diagnostic_emergency_stop_topic_name').value
        self.diagnostic_speed_topic_name = self.get_parameter('diagnostic_speed_topic_name').value
        self.diagnostic_navsatfix_topic_name = self.get_parameter('diagnostic_navsatfix_topic_name').value


        self.trigger_flag = Bool()
        self.trigger_flag.data = True

        self.speed = Float32()

        self.emergency_stop_flag = Bool()
        self.emergency_stop_flag.data = False

        self.nav = NavSatFix()

        # Subscribe to the original NavSatFix topic
        self.navsat_fix_subscription = self.create_subscription(
            NavSatFix,
            self.navsat_fix_topic_name,
            self.navsat_fix_callback,
            10
        )

        # Subscribe to the original speed topic
        self.speed_subscription = self.create_subscription(
            Twist,
            self.velocity_topic_name,
            self.speed_callback,
            10
        )

        self.trigger_flag_publisher = self.create_publisher(
            Bool,
            self.diagnostic_trigger_topic_name,
            10
        )

        self.speed_publisher = self.create_publisher(
            Float32,
            self.diagnostic_speed_topic_name,
            10
        )

        self.emergency_stop_flag_publisher = self.create_publisher(
            Bool,
            self.diagnostic_emergency_stop_topic_name,
            10
        )

        self.nav_publisher = self.create_publisher(
            NavSatFix,
            self.diagnostic_navsatfix_topic_name,
            10
        )

        self.timer = self.create_timer(self.diagnostic_frequency, self.timer_callback)
        

    def navsat_fix_callback(self, msg):
        self.nav = msg

    def speed_callback(self, msg):
        self.speed.data = float(msg.linear.x)

    def timer_callback(self):
        self.trigger_flag_publisher.publish(self.trigger_flag)
        self.speed_publisher.publish(self.speed)
        self.emergency_stop_flag_publisher.publish(self.emergency_stop_flag)
        self.nav_publisher.publish(self.nav)


def main(args=None):
    rclpy.init(args=args)
    node = FCDiagnosticPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()