import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
import json

# Cấu hình Serial
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

class LedSubscriber(Node):
    def __init__(self):
        super().__init__('led_subscriber')
        self.subscription = self.create_subscription(
            String,
            'iphone12',
            self.toggle_led,
            10
        )

        # Cấu hình ban đầu
        self.max = 2500
        self.min = 500
        self.servo1 = 1500
        self.servo2 = 1500
        self.servo3 = 1500
        self.servo4 = 1500
        self.servo5 = 1500
        
        self.servo_end = 2500
        self.servo_min = 600
        self.servo_max = 2000

    def toggle_led(self, msg):
        try:
            # Giải mã JSON từ tin nhắn
            data = json.loads(msg.data)
            self.get_logger().info(f"Received data: {data}")

            # Xử lý các nút Button - servo1
            btn_north = data.get("BTN_NORTH", 0)
            btn_south = data.get("BTN_SOUTH", 0)
            
            # Xử lý các nút Button - servo2
            btn_east = data.get("BTN_EAST", 0)
            btn_west = data.get("BTN_WEST", 0)
            
            # Xử lý các nút Button - servo3
            btn_tl = data.get("BTN_TL", 0)
            btn_tr = data.get("BTN_TR", 0)
            
            # Xử lý các nút Button - servo4
            btn_tl2 = data.get("ABS_Z", 0)
            btn_tr2 = data.get("ABS_RZ", 0)
            # Xử lý các nút Button - servo5
            btn_tl1 = data.get("ABS_HAT0Y", 0)
            btn_tr1 = data.get("ABS_HAT0Y", 0)
            
            # Xử lý các nút Button - servo6
            btn_hatL = data.get("ABS_HAT0X", 0)
            btn_hatR = data.get("ABS_HAT0X", 0)


            if btn_north == 1:
                self.servo1 = min(self.max, self.servo1 + 100)
                data_send = f"#1P{int(self.servo1)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_south == 1:
                self.servo1 = max(self.min, self.servo1 - 100)
                data_send = f"#1P{int(self.servo1)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
                
            if btn_east == 1:
                self.servo2 = min(self.max, self.servo2 + 100)
                data_send = f"#2P{int(self.servo2)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_west == 1:
                self.servo2 = max(self.min, self.servo2 - 100)
                data_send = f"#2P{int(self.servo2)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
                
            if btn_tl == 1:
                self.servo3 = min(self.max, self.servo3 + 100)
                data_send = f"#3P{int(self.servo3)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_tr == 1:
                self.servo3 = max(self.min, self.servo3 - 100)
                data_send = f"#3P{int(self.servo3)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)

            if btn_tl2 == 255:
                self.servo4 = min(self.max, self.servo4 + 100)
                data_send = f"#4P{int(self.servo4)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_tr2 == 255:
                self.servo4 = max(self.min, self.servo4 - 100)
                data_send = f"#4P{int(self.servo4)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
                
            if btn_tl1 == -1:
                self.servo5 = min(self.max, self.servo5 + 100)
                data_send = f"#5P{int(self.servo5)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_tr1 == 1:
                self.servo5 = max(self.min, self.servo5 - 100)
                data_send = f"#5P{int(self.servo5)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            
            if btn_hatL == -1:
                self.servo5 = min(self.max, self.servo5 + 100)
                data_send = f"#6P{int(self.servo5)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)
            elif btn_hatR == 1:
                self.servo5 = max(self.min, self.servo5 - 100)
                data_send = f"#6P{int(self.servo5)}PT100D100\r\n"
                ser.write(data_send.encode())
                self.get_logger().info('Sent: "%s"' % data_send)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
            self.get_logger().error(f"Received message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    led_subscriber = LedSubscriber()
    rclpy.spin(led_subscriber)
    led_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()