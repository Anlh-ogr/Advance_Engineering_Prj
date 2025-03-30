# xu ly thong tin tu gamepad dieu khien robot

import rclpy                        # type: ignore
import serial                       # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore

class RobotArmSubscriber(Node):
    def __init__(self):
        super().__init__('robot_arm_subscriber')
        self.subscription = self.create_subscription(String, '/robot_arm_control', self.control_callback, 10)

        # ket noi serial voi canh tay robot
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # change baudrate : 9600
        self.get_logger().info('Serial connection initialized')
        
    def control_callback(self, msg):
        control_data = msg.data
        self.get_logger().info(f'Received: {control_data}')
        
        # gui du lieu den canh tay robot
        self.ser.write(control_data.encode('utf-8'))
        
    def destroy_node(self):
        self.ser.close()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotArmSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()