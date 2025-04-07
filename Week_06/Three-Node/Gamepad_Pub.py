import json
import rclpy                    # type: ignore
from rclpy.node import Node     # type: ignore
from std_msgs.msg import String # type: ignore
from inputs import get_gamepad  # type: ignore

class GamepadDataPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_data_publisher')  # Node name
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'gamepad_data', 10)
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        events = get_gamepad()
        for event in events:
            if event.ev_type in ["Key", "Absolute"]:
                if event.code in ["BTN_NORTH", "BTN_SOUTH", "BTN_EAST", "BTN_WEST",
                                  "BTN_TL", "BTN_TR", "BTN_TL2", "BTN_TR2",
                                  "BTN_SELECT", "BTN_START",
                                  "BTN_THUMBL", "BTN_THUMBR",
                                  "ABS_HAT0Y", "ABS_HAT0X",
                                  "ABS_Y", "ABS_X", "ABS_Z", "ABS_RZ"]:
                    msg = String()
                    data = {event.code: event.state} # Create a dictionary
                    msg.data = json.dumps(data)      # Convert to JSON
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: "{msg.data}"')
                    
def main(args=None):
    rclpy.init(args=args)
    gamepad_data_publisher = GamepadDataPublisher()
    rclpy.spin(gamepad_data_publisher)
    gamepad_data_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()