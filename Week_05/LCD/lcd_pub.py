"""Tạo node Publisher và node Subscriber để tạo file truyền dữ liệu hiển thị lên LCD
theo yêu cầu:
 - Dòng 1 : Tên nhóm
 - Dòng 2 : Họ tên thành viên 1
 - Dòng 3 : Họ tên thành viên 2
 - Dòng 4 : Họ tên thành viên 3"""

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
import json # use to convert dictionary to string

class LCDPublisher(Node):
    def __init__(self):
        super().__init__('lcd_publish')
        self.publisher_ = self.create_publisher(String, 'lcd_display', 10)
        self.message_data = ("MaiQuocHuy\nNguyen.L.HoangAn\nLeNgocBaoKha")
        self.timer = self.create_timer(5.0, self.publisher_message)

    def publisher_message(self):
        msg = String()
        msg.data = self.message_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        
    def stop_and_cleanup(self):
        stop_msg = String()
        stop_msg.data = json.dumps({'m1' : 'stop', 'm2' : 'stop'})
        self.publisher_.publish(stop_msg)
        self.get_logger().info(f"Sending stop message: {stop_msg.data}")
        self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    lcd_publisher = LCDPublisher()
    
    try:
        rclpy.spin(lcd_publisher)
    except KeyboardInterrupt:
        lcd_publisher.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        lcd_publisher.stop_and_cleanup()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()