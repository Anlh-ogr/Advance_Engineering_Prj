# Điều khiển led, động cơ DC qua chân GPIO của jetson dùng ROS, tạo node Subscriber để tạo file truyền dữ liệu điều khiển động cơ DC theo yêu cầu:
# Động cơ quay thuận 3s
# Động cơ quay nghịch 3s
# Động cơ dừng
# Sau đó lặp lại

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import Jetson.GPIO as GPIO

class LedSubscriber(Node):
    def __init__(self):
        super().__init__('led_subscriber')
        self.led_pin = 18 # GPIO18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
        
        self.subscription = self.create_subscription(Bool, 'led_toggle', self.toggle_led, 10)
        self.subscription
        
        
    def toggle_led(self, msg):
        if msg.data:
            GPIO.output(self.led_pin, GPIO.HIGH)
            self.get_logger().info('LED ON')
        else:
            GPIO.output(self.led_pin, GPIO.LOW)
            self.get_logger().info('LED OFF')
            
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()
        
        
class DCSubscriber(Node):
    def __init__(self):
        super().__init__('dc_subscriber')
        self.dc_forward_pin = 15
        self.dc_reverse_pin = 13
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dc_forward_pin, GPIO.OUT)
        GPIO.setup(self.dc_reverse_pin, GPIO.OUT)
        
        self.subscription = self.create_subscription(String, 'dc_control', self.control_dc, 10)
        self.subscription
        
    def control_dc(self, msg):
        if msg.data == "Forward":
            GPIO.output(self.dc_forward_pin, GPIO.HIGH)
            GPIO.output(self.dc_reverse_pin, GPIO.LOW)
            self.get_logger().info('DC Forward')
        
        elif msg.data == "Reverse":
            GPIO.output(self.dc_forward_pin, GPIO.LOW)
            GPIO.output(self.dc_reverse_pin, GPIO.HIGH)
            self.get_logger().info('DC Reverse')
        
        else:
            GPIO.output(self.dc_forward_pin, GPIO.LOW)
            GPIO.output(self.dc_reverse_pin, GPIO.LOW)
            self.get_logger().info('DC Stop')
            
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    led_subscriber = LedSubscriber()
    dc_subscriber = DCSubscriber()

    try:
        rclpy.spin(led_subscriber)
        rclpy.spin(dc_subscriber)
        
    except KeyboardInterrupt:
        pass
    
    led_subscriber.destroy_node()
    dc_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()