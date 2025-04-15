import rclpy                        # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore
from inputs import get_gamepad      # type: ignore


class GamePadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_arm_control', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

        # khoi tao servo, giu servo4 mac dinh de du phong
        self.servo0 = self.servo1 = self.servo2 = self.servo3 = self.servo4 = self.servo5 = 1500
        # nut gamepad
        self.btn_x, self.btn_y, self.btn_b, self.btn_a, self.dpad_x, self.dpad_y = 0, 0, 0, 0, 0, 0
        self.get_logger().info("Gamepad publisher initialized")

    def timer_callback(self):
        try:
            events = get_gamepad()
        except Exception as e:
            self.get_logger().error(f"Gamepad not connected: {e}. Shutting down...")
            rclpy.shutdown()
            return
        
        for event in events:
            if event.ev_type in ["Key", "Absolute"]:
                if event.code == "ABS_HAT0X":
                    self.dpad_x = event.state
                if event.code == "ABS_HAT0Y":
                    self.dpad_y = event.state
                if event.code == "BTN_WEST":
                    self.btn_x = event.state
                if event.code == "BTN_NORTH":
                    self.btn_y = event.state
                if event.code == "BTN_EAST":
                    self.btn_b = event.state
                if event.code == "BTN_SOUTH":
                    self.btn_a = event.state
        
        step = 60
        hold_multiplier = 1.0

        # xu ly hold dpad_x
        if self.dpad_x != getattr(self, 'last_dpad_x', 0):
            self.hold_time_x = 0.0
        self.last_dpad_x = self.dpad_x
        if self.dpad_x != 0:
            if not hasattr(self, 'hold_time_x'):
                self.hold_time_x = 0.0
            self.hold_time_x += 1
            if self.hold_time_x > 50:
                hold_multiplier = 2.0
        else:
            self.hold_time_x = 0.0

        # xu ly hold dpad_y
        if self.dpad_y != getattr(self, 'last_dpad_y', 0):
            self.hold_time_y = 0.0
        self.last_dpad_y = self.dpad_y
        if self.dpad_y != 0:
            if not hasattr(self, 'hold_time_y'):
                self.hold_time_y = 0.0
            self.hold_time_y += 1
            if self.hold_time_y > 50:
                hold_multiplier = 2.0
        else:
            self.hold_time_y = 0.0

        # dieu khien servo0 (x + dpad_x)
        if self.btn_x == 1 and self.dpad_x != 0:
            self.servo0 = max(500, min(2500, self.servo0 - int(self.dpad_x * step * hold_multiplier)))

        # dieu khien servo1 (x + dpad_y)
        if self.btn_x == 1 and self.dpad_y != 0:
            self.servo1 = max(500, min(2500, self.servo1 - int(self.dpad_y * step * hold_multiplier)))

        # dieu khien servo2 (y + dpad_y)
        if self.btn_y == 1 and self.dpad_y != 0:
            self.servo2 = max(500, min(2500, self.servo2 - int(self.dpad_y * step * hold_multiplier)))

        # dieu khien servo3 (b + dpad_y)
        if self.btn_b == 1 and self.dpad_y != 0:
            self.servo3 = max(500, min(2500, self.servo3 - int(self.dpad_y * step * hold_multiplier)))

        # dieu khien servo5 (grip voi nut a)
        if self.btn_a == 1:
            self.servo5 = 1000  # gia tri grip (co the dieu chinh)
        else:
            self.servo5 = 2000  # gia tri tha grip (co the dieu chinh)

        # tao va publish lenh, bao gom servo4 mac dinh
        new_msg = f"#0P{self.servo0}#1P{self.servo1}#2P{self.servo2}#3P{self.servo3}#4P{self.servo4}#5P{self.servo5}\r\n"
        if not hasattr(self, 'last_msg') or self.last_msg != new_msg:
            msg = String()
            msg.data = new_msg
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.last_msg = new_msg

    def destroy_node(self):
        self.get_logger().info("Shutting down gamepad publisher...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GamePadPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()