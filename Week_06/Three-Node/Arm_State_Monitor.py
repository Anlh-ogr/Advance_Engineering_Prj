import rclpy                                # type: ignore
from rclpy.node import Node                 # type: ignore
from std_msgs.msg import Float64MultiArray  # type: ignore

class ArmStateMonitor(Node):
    def __init__(self):
        super().__init__('arm_state_monitor')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'servo_angles',
            self.servo_callback,
            10
        )
        
    def servo_callback(self, msg):
        angles = msg.data
        state = (f"Arm State: Base: {angles[0]:.2f}, Shoulder: {angles[1]:.2f}, "
                 f"Elbow: {angles[2]:.2f}, Wrist: {angles[3]:.2f}, Gripper: {angles[4]:.2f}")
        self.get_logger().info(state)

def main(args=None):
    rclpy.init(args=args)
    arm_state_monitor = ArmStateMonitor()
    rclpy.spin(arm_state_monitor)
    arm_state_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()