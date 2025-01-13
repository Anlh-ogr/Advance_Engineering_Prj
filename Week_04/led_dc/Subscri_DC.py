# Điều khiển động cơ DC qua chân GPIO của jetson dùng ROS, tạo node Subscriber để tạo file truyền dữ liệu điều khiển động cơ DC theo yêu cầu:
# Động cơ quay thuận 3s (Nhấn F động cơ quay thuận)
# Động cơ quay nghịch 3s (Nhấn R động cơ quay nghịch 3s)
# Động cơ dừng (Nhấn S động cơ dừng)
# Sau đó lặp lại

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as GPIO

# Khai báo chân GPIO
GPIO.setmode(GPIO.BOARD)

pin_1 = 13
pin_2 = 15

GPIO.setup(pin_1, GPIO.OUT)
GPIO.setup(pin_2, GPIO.OUT)

class DC_Subscriber(Node):
    def __init__(self):
        super().__init__('dc_publisher')
        self.publisher_ = self.create_publisher(String, 'dc_toggle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('DC Publisher has been started')
    
    def timer_callback(self):
        command = input("Press 'F' for forward, 'R' for reverse, 'S' for stop: ").string().upper()

        msg = String()
        msg.data = command
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending DC State: {msg.data}')
        
        
def main(args=None):
    rclpy.init(args=args)
    
    dc_subscriber = DC_Subscriber()
    
    rclpy.spin(dc_subscriber)
    
    dc_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()