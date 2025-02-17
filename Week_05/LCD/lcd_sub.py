import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
from RPi_GPIO_i2c_LCD import lcd # type: ignore
import time
import json # use to convert dictionary to string

class LCDSub(Node, lcd):
    def __init__(self, i2c_addr):
        super().__init__('lcd_sub')
        self.lcd = lcd(i2c_addr)
        self.subscription = self.create_subscription(String, 'lcd_display', self.listener, 10)
        
    def listener(self, msg):
        received_data = msg.data
        lines = received_data.split("\n")
        for i in range(min(4, len(lines))):
            if lines[i]:
                self.lcd.lcd_string(lines[i], i+1)
                time.sleep(1e-3)
            
    def destroy_node(self):
        self.lcd.clear()
        super().destroy_node()
    
    
def main(args=None):
    rclpy.init(args=args)
    lcd_sub = LCDSub(0x27) # 0x27 is the default i2c address of the LCD    
    
    try:
        rclpy.spin(lcd_sub)
    
    except KeyboardInterrupt:
        lcd_sub.get_logger().info('Keyboard interrupt, shutting down...')
    
    finally:
        lcd_sub.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()