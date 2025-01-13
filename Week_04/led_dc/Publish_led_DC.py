# Điều khiển led, động cơ DC qua chân GPIO của jetson dùng ROS, tạo node Publisher để tạo file truyền dữ liệu điều khiển động cơ DC theo yêu cầu:
# Động cơ quay thuận 3s
# Động cơ quay nghịch 3s
# Động cơ dừng
# Sau đó lặp lại

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time



class LedPublisher(Node):

    def __init__(self):
        super().__init__('led_publisher')
        self.publisher_ = self.create_publisher(Bool, 'led_toggle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = False

    def timer_callback(self):
        self.state = not self.state
        msg = Bool()
        msg.data = "On" if self.state else "Off"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending Led State: {msg.data}')
        
class DCPublisher(Node):
    
    def __init__(self):
        super().__init__('dc_publisher')
        self.publisher_ = self.create_publisher(String, 'dc_control', 10)
        self.state = 0 # 0: Forward, 1: Reverse, 2: Stop
        self.timer = self.create_timer(3.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        if self.state == 0:
            msg.data = "Forward"
            self.state = 1
        elif self.state == 1:
            msg.data = "Reverse"
            self.state = 2
        else:
            msg.data = "Stop"
            self.state = 0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending DC State: {msg.data}')
        
        
def main(args=None):
    rclpy.init(args=args)

    led_publisher = LedPublisher()
    dc_publisher = DCPublisher()

    try:
        rclpy.spin(led_publisher)
        rclpy.spin(dc_publisher)
    except KeyboardInterrupt:
        pass

    led_publisher.destroy_node()
    dc_publisher.destroy_node()
    
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()