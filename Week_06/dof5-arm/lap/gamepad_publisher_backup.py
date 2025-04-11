import rclpy                    # type: ignore
from rclpy.node import Node     # type: ignore
from std_msgs.msg import String # type: ignore
from inputs import get_gamepad  # type: ignore

class GamePadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_arm_control', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)    # 100Hz

        # 500 - 2500 microseconds (1ms - 2.5ms) | 1500: center (neutral position)
        self.base0, self.base1, self.base2, self.base3, self.base4, self.base5 = 1500, 1500, 1500, 1500, 1500
        # btn_north(x), btn_west(y), btn_east(b), btn_south(a), abs_hat0x(dpad_x), abs_hat0y(dpad_y)
        self.btn_x, self.btn_y, self.btn_b, self.btn_a, self.dpad_x, self.dpad_y = 0, 0, 0, 0, 0, 0

        self.get_logger().info("Gamepad publisher initialized")

    def timer_callback(self):
        try:
            events = get_gamepad()
        except Exception as e:
            self.get_logger().warning(f"Error reading gamepad: {e}")
            return
        
        for event in events:
            if event.ev_type in ["Key", "Absolute"]:
                if event.code == "ABS_HAT0X":
                    self.dpad_x = event.state           
                
                # read button states
                if event.code == "BTN_NORTH":
                    self.btn_x = event.state
                if event.code == "BTN_WEST":
                    self.btn_y = event.state
                if event.code == "BTN_EAST":
                    self.btn_b = event.state
                if event.code == "BTN_SOUTH":
                    self.btn_a = event.state
                    self.base5 = 1500 if self.btn_a == 0 else 2000
                if event.code == "ABS_HAT0Y":
                    self.dpad_y = event.state

        # control base1, base2, base3 with buttons (-1) up, (1) down
        step = 60
        if self.dpad_x == -1:
            self.base0 = max(500, self.base0 - step)
        elif self.dpad_x == 1:
            self.base0 = min(2500, self.base0 + step)
        
        if self.btn_x == 1 and self.dpad_y != 0:
            self.base1 = max(500, min(2500, self.base1 - (self.dpad_y * step)))
        if self.btn_y == 1 and self.dpad_y != 0:
            self.base2 = max(500, min(2500, self.base2 - (self.dpad_y * step)))
        if self.btn_b == 1 and self.dpad_y != 0:
            self.base3 = max(500, min(2500, self.base3 + (self.dpad_y * step)))
            # kiem tra huong canh tay neu khong dung huong ()
            # - (self.dpad_y * step) -> + (self.dpad_y * step)
            
        
        # create a message and publish it
        control_msg = f"#0P{self.base0}#1P{self.base1}#2P{self.base2}#3P{self.base3}#5P{self.base5}\r\n"            
        msg = String()
        msg.data = control_msg
        
        # publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
            
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