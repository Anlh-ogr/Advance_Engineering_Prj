# Điều khiển động cơ DC qua chân GPIO của jetson dùng ROS, tạo node Publisher để tạo file truyền dữ liệu điều khiển động cơ DC theo yêu cầu:
# Động cơ quay thuận 3s (Nhấn F động cơ quay thuận)
# Động cơ quay nghịch 3s (Nhấn R động cơ quay nghịch 3s)
# Động cơ dừng (Nhấn S động cơ dừng)
# Sau đó lặp lại

import rclpy
from std_msgs.msg import String
import time
from rclpy.node import Node
import Jetson.GPIO as GPIO

# Khai báo chân GPIO
GPIO.setmode(GPIO.BOARD)

pin_1 = 13
pin_2 = 15

GPIO.setup(pin_1, GPIO.OUT)
GPIO.setup(pin_2, GPIO.OUT)


class DC_Publisher (Node):
    def __init__(self):
        super().__init__('dc_publisher')
        self.publisher_ = self.create_publisher(String, 'dc_toggle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.state = 'S'
        self.get_logger().info('DC Publisher has been started')
        
        
    def timer_callback(self):
        command = input("Press 'F' for forward, 'R' for reverse, 'S' for stop: ").string().upper()
        
        msg = String()
        
        if command == "F":
            self.forward()
            msg.data = 'Thuan'
            
        elif command == "R":
            self.reverse()
            msg.data = 'Nghich'
            
        elif command == "S":
            self.stop()
            msg.data = 'Stop'
            
        else:
            msg.data = 'Invalid Command'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending DC State: {msg.data}')
        
    def forward(self):
        GPIO.output(pin_1, GPIO.HIGH)
        GPIO.output(pin_2, GPIO.LOW)
        time.sleep(3)
        self.stop()
    
    def reverse(self):
        GPIO.output(pin_1, GPIO.LOW)
        GPIO.output(pin_2, GPIO.HIGH)
        time.sleep(3)
        self.stop()
        
    def stop(self):
        GPIO.output(pin_1, GPIO.LOW)
        GPIO.output(pin_2, GPIO.LOW)
        
def main(args=None):
    rclpy.init(args=args)
    dc_publisher = DC_Publisher()
    rclpy.spin(dc_publisher)
    dc_publisher.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()
    
if __name__ == '__main__':
    main()
        
    
                
        