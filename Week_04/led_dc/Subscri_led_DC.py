import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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

def main(args=None):
    rclpy.init(args=args)
    
    led_subscriber = LedSubscriber()
    
    rclpy.spin(led_subscriber)
    
    led_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()