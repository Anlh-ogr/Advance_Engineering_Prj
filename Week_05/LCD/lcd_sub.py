import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
from RPi_GPIO_i2c_LCD import lcd # type: ignore
import time
import json # use to convert dictionary to string

class LCDSub(Node, lcd):
    def __init__(self, i2c_addr):
        super().__init__('lcd_subscriber')
        self.lcd = lcd(i2c_addr)
        self.subscription = self.create_subscription(String, 'lcd_display', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        try:
            if msg.data.strips().startswith("{"):
                self.get_logger().info(f"Received: {msg.data}")
                command = json.loads(msg.data)
                if command.get('m1') == 'stop' and command.get('m2') == 'stop':
                    self.get_logger().info('Received stop command')
                    self.lcd.clear()
                    return
                
            self.get_logger().info(f"Received: {msg.data}")
            lines = msg.data.split('\n')
            for idx in range(min(4, len(lines))):
                if lines[idx]:
                    self.lcd.display_string(lines[idx], idx + 1)
                    time.sleep(0.05)
        
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")            
    
    def destroy_node(self):
        self.lcd.clear()
        self.get_logger().info('LCD cleared')
        super().destroy_node()
    
    
def main(args=None):
    rclpy.init(args=args)
    lcd_subscriber = LCDSub(0x27)
    try:
        rclpy.spin(lcd_subscriber)
    except KeyboardInterrupt:
        lcd_subscriber.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        lcd_subscriber.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()