import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class ElapsedTimePublisher(Node):
    def __init__(self):
        super().__init__('elapsed_time_publisher')
        self.publisher_ = self.create_publisher(Float64, 'elapsed_time', 10)
        self.subscription_ = self.create_subscription(Twist, '/r4c_tractor/nav_vel', self.twist_callback, 10)
        self.timer_ = self.create_timer(1.0, self.publish_elapsed_time)
        self.elapsed_time = 0.0
        self.last_time = time.time()
        self.sub_timeout = 3.0

    def publish_elapsed_time(self):
        msg = Float64()
        msg.data = self.elapsed_time
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing elapsed time: %.2f seconds' % self.elapsed_time)
    
    def twist_callback(self, msg):
        # Check if the twist message has nonzero linear velocity
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        # Check for subscriber timeout
        if elapsed_time > self.sub_timeout:
            elapsed_time = 0.0
        linear_velocity = msg.linear.x
        if abs(linear_velocity) > 0.5:
            self.elapsed_time += elapsed_time
            self.last_time = current_time
        else:
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    elapsed_time_publisher = ElapsedTimePublisher()
    rclpy.spin(elapsed_time_publisher)
    elapsed_time_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()