# Điều khiển led (On/Off)

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool, String


class LedPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.publisher_ = self.create_publisher(Bool, 'led_toggle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = False

    def timer_callback(self):
        self.state = not self.state
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending Led State: {msg.data}')
        
        
def main(args=None):
    rclpy.init(args=args)
    led_publisher = LedPublisher()
    rclpy.spin(led_publisher)
    led_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()