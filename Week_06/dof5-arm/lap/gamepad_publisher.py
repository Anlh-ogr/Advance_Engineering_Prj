import rclpy                        # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore
from inputs import get_gamepad      # type: ignore
import json


class GamePadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_arm_control', 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz

        # khoi tao servo
        self.servo0 , self.servo1 , self.servo2 , self.servo3 , self.servo4 , self.servo5 = 1500, 1500, 1500, 1500, 1500, 1500
        # nut gamepad
        self.btn_x, self.btn_b, self.tr, self.dpad_y, self.tl, self.tl2, self.tr2 = 0, 0, 0, 0, 0, 0, 0   
        self.get_logger().info("Gamepad publisher initialized")

    def timer_callback(self):
        try:
            events = get_gamepad()
        except Exception as e:
            self.get_logger().error(f"Gamepad not connected: {e}. Shutting down...")
            rclpy.shutdown()
            return
        
        for event in events:
            if event.ev_type in ["Key", "Absolute"]: # Filter for Button, Hat & Axis
                # Gửi thông tin nút nhấn dưới dạng JSON để debug
                if event.code in ["BTN_NORTH", "BTN_SOUTH", "BTN_EAST", "BTN_WEST",
                                  "BTN_SELECT", "BTN_START",
                                  "BTN_TL", "BTN_TR", "BTN_TL2", "BTN_TR2",
                                  "BTN_THUMBL", "BTN_THUMBR",
                                  "ABS_HAT0Y", "ABS_HAT0X",
                                  "ABS_Y", "ABS_X", "ABS_Z", "ABS_RZ"]:
                    msg = String()
                    data = {event.code: event.state} # Create dictionary
                    msg.data = json.dumps(data) # Create JSON data
                    self.publisher_.publish(msg) # Publish
                    self.get_logger().info('Send: "%s"' % msg.data)
                
                # Cập nhật trạng thái nút (theo yêu cầu)
                if event.code == "BTN_TL":
                    self.tl = event.state
                if event.code == "ABS_HAT0Y":
                    self.dpad_y = event.state
                if event.code == "BTN_WEST":
                    self.btn_x = event.state
                if event.code == "BTN_EAST":
                    self.btn_b = event.state
                if event.code == "BTN_TR":
                    self.tr = event.state
                if event.code == "BTN_TL2":
                    self.tl2 = event.state
                if event.code == "BTN_TR2":
                    self.tr2 = event.state
        
        step = 60

        # dieu khien servo0 (x + dpad_y) - Base rotation
        if self.btn_x == 1 and self.dpad_y != 0:
            self.servo0 = max(500, min(2500, self.servo0 - int(self.dpad_y * step)))

        # dieu khien servo1 (b + dpad_y) - Shoulder
        if self.btn_b == 1 and self.dpad_y != 0:
            self.servo1 = max(500, min(2500, self.servo1 - int(self.dpad_y * step)))

        # dieu khien servo2 (tr + dpad_y) - Elbow
        if self.tr == 1 and self.dpad_y != 0:
            self.servo2 = max(500, min(2500, self.servo2 - int(self.dpad_y * step)))
            
        # dieu khien servo3 (tl2 + dpad_y) - Wrist rotation
        if self.tl2 == 1 and self.dpad_y != 0:
            self.servo3 = max(500, min(2500, self.servo3 - int(self.dpad_y * step)))
            
        # dieu khien servo4 (tr2 + dpad_y) - Wrist tilt
        if self.tr2 == 1 and self.dpad_y != 0:
            self.servo4 = max(500, min(2500, self.servo4 - int(self.dpad_y * step)))

        # dieu khien servo5 (gripper voi tl)
        if self.tl == 1:
            self.servo5 = 1000  # grip
        else:
            self.servo5 = 2000  # tha grip

        # tao va publish lenh
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