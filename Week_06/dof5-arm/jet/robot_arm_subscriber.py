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
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # change baudrate : 9600
            self.get_logger().info("Serial connection initialized successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise RuntimeError("Serial connection failed")
        
    def control_callback(self, msg):
        control_data = msg.data
        self.get_logger().info(f'Received: {control_data}')
        
        # gui du lieu den canh tay robot
        try:
            self.ser.write(control_data.encode('utf-8'))
            self.get_logger().info("Command sent to robot arm")
        except serial.SerialException as e:
            self.get_logger().warning(f"Failed to send command: {e}")
    
    def destroy_node(self):
        self.get_logger().info("Closing serial connection")
        self.ser.close()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotArmSubscriber()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()