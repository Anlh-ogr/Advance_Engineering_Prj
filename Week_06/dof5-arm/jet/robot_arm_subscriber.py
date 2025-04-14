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
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                self.get_logger().info(f"Serial connected on {port}")
                break
            except serial.SerialException:
                continue
        else:
            self.get_logger().error("No serial ports available")
            raise RuntimeError("Serial connection failed")
        
    def control_callback(self, msg):
        control_data = msg.data
        self.get_logger().info(f'Received: {control_data}')
        
        # gioi han tan so gui serial
        if not hasattr(self, 'last_send') or (self.get_clock().now().nanoseconds - self.last_send) > 10000000:  # 10ms
            try:
                self.ser.write(control_data.encode('utf-8'))
                self.get_logger().info("Command sent to robot arm")
                self.last_send = self.get_clock().now().nanoseconds
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