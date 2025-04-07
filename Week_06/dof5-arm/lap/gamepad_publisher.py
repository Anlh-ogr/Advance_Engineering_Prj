import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame as pg
import csv
from datetime import datetime

class GamePadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_arm_control', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Khởi tạo GamePad
        pg.init()
        pg.joystick.init()
        self.joystick = pg.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"GamePad initialized: {self.joystick.get_name()}")

        # Lưu dữ liệu vào file CSV
        self.csv_file = open('robot_arm_data.csv', 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'base0', 'base1', 'base2', 'base3', 'base5', 'button'])

    def timer_callback(self):
        pg.event.pump()

        # Đọc dữ liệu từ GamePad (joystick - analog L) | axis_0 - x, axis_1 - y
        axis_0 = self.joystick.get_axis(0)  # base0-servo0: left-right 360
        axis_1 = self.joystick.get_axis(1)  # base1-servo1: up-down (shoulder)

        # Đọc dữ liệu từ GamePad (joystick - analog R) | axis_2 - x, axis_3 - y
        axis_2 = self.joystick.get_axis(2)  # base2-servo2: up-down (elbow)
        axis_3 = self.joystick.get_axis(3)  # base3-servo3: up-down (wrist)

        # Đọc dữ liệu từ GamePad (button) | button_0 - A
        button_0 = self.joystick.get_button(0)  # base5: open-close gripper

        # Thêm deadzone để tránh rung (10%)
        if abs(axis_0) < 0.1:
            axis_0 = 0
        if abs(axis_1) < 0.1:
            axis_1 = 0
        if abs(axis_2) < 0.1:
            axis_2 = 0
        if abs(axis_3) < 0.1:
            axis_3 = 0

        # Chuyển đổi giá trị axis (-1.0 -> 1.0) sang góc servo (500 - 2500 microseconds)
        base0 = int(1500 + (axis_0 * 1000))  # base0-servo0: left-right 360
        base1 = int(1500 + (axis_1 * 1000))  # base1-servo1: up-down (shoulder)
        base2 = int(1500 + (axis_2 * 1000))  # base2-servo2: up-down (elbow)
        base3 = int(1500 + (axis_3 * 1000))  # base3-servo3: up-down (wrist)
        base5 = 1500 if button_0 == 0 else 2000  # base5: 1500 (open) - 2000 (close)

        # Tạo message điều khiển (bỏ servo 4)
        control_msg = f"#0P{base0}#1P{base1}#2P{base2}#3P{base3}#5P{base5}\r\n"
        msg = String()
        msg.data = control_msg

        # Publish message lên topic /robot_arm_control
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {control_msg}")

        # Lưu dữ liệu vào file CSV
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.csv_writer.writerow([timestamp, base0, base1, base2, base3, base5, button_0])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        pg.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GamePadPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()