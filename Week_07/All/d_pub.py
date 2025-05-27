import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from inputs import get_gamepad
import json

class GamepadDataPublisher(Node):
	# Fucntion inits
	def __init__(self):
		super().__init__('gamepad_data_pub') # Name of Node
		# Create a Publisher
		self.publisher_ = self.create_publisher(String, 'iphone12', 10)
		# Create a Publisher node (Type: String, Topic: "gamepad_data_topic", Queue: 10)
		# Create a Timer
		timer_period = 0.001 # Value time of Timer - seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		# Create a Timer for excute "timer_callback"
		# Function for Timer
	def timer_callback(self):
		events = get_gamepad()
		for event in events:
			if event.ev_type in ["Key", "Absolute"]: # Filter for Button, Hat & Axis
				if event.code in ["BTN_NORTH", "BTN_SOUTH", "BTN_EAST", "BTN_WEST",
						"BTN_TL", "BTN_TR", "BTN_TL2", "BTN_TR2",
						"BTN_SELECT", "BTN_START",
						"BTN_THUMBL", "BTN_THUMBR",
						"ABS_HAT0Y", "ABS_HAT0X",
						"ABS_Y", "ABS_X", "ABS_Z", "ABS_RZ"]:
					msg = String()
					data = {event.code: event.state} # Create dictionary
					msg.data = json.dumps(data) # Create JSON data
					self.publisher_.publish(msg) # Publish
					self.get_logger().info('Send: "%s"' % msg.data) # Print screen
def main(args=None):
	rclpy.init(args=args)
	gamepad_data_pub = GamepadDataPublisher() # Node is created
	rclpy.spin(gamepad_data_pub)
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	gamepad_data_pub.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
	main()