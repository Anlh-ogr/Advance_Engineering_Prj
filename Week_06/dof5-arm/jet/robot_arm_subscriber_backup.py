# xu ly thong tin tu gamepad va color_info de dieu khien robot

import rclpy                        # type: ignore
import serial                       # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore

class RobotArmSubscriber(Node):
    def __init__(self):
        super().__init__('robot_arm_subscriber')
        # subscription cho topic /robot_arm_control (gamepad)
        self.control_subscription = self.create_subscription(
            String, '/robot_arm_control', self.control_callback, 10
        )
        # subscription cho topic /color_info (camera)
        self.color_subscription = self.create_subscription(
            String, '/color_info', self.color_callback, 10
        )

        # ket noi serial voi canh tay robot
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial connection initialized successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise RuntimeError("Serial connection failed")
        
        # khoi tao vi tri mac dinh cho canh tay
        self.base0 = 1500  # vi tri xoay
        self.base1 = 1500  # goc 1
        self.base2 = 1500  # goc 2
        self.base3 = 1500  # goc 3
        self.base5 = 1500  # gam kep
        
    def control_callback(self, msg):
        # xu ly lenh tu gamepad
        control_data = msg.data
        self.get_logger().info(f'Received from gamepad: {control_data}')
        
        # gui du lieu den canh tay robot
        try:
            self.ser.write(control_data.encode('utf-8'))
            self.get_logger().info("Command sent to robot arm from gamepad")
        except serial.SerialException as e:
            self.get_logger().warning(f"Failed to send command: {e}")
    
    def color_callback(self, msg):
        # xu ly thong tin mau sac va vi tri
        self.get_logger().info(f'Received color info: {msg.data}')
        
        # phan tich du lieu: "color:yellow,x:100,y:200"
        try:
            data = dict(item.split(":") for item in msg.data.split(","))
            color = data["color"]
            x = int(data["x"])
            y = int(data["y"])
        except (KeyError, ValueError) as e:
            self.get_logger().warning(f"Invalid message format: {e}")
            return
        
        # chuyen vi tri (x, y) thanh lenh dieu khien
        # vi du: dieu chinh base0 dua tren x, base1 dua tren y
        step = 10
        if x < 200:  # vat the ben trai
            self.base0 = max(500, self.base0 - step)
        elif x > 440:  # vat the ben phai
            self.base0 = min(2500, self.base0 + step)
            
        if y < 150:  # vat the o tren
            self.base1 = max(500, self.base1 - step)
        elif y > 330:  # vat the o duoi
            self.base1 = min(2500, self.base1 + step)
            
        # neu la mau pink, kich hoat gam kep
        if color == "pink":
            self.base5 = 2000  # kep
        else:
            self.base5 = 1500  # tha
            
        # tao lenh dieu khien
        control_data = f"#0P{self.base0}#1P{self.base1}#2P{self.base2}#3P{self.base3}#5P{self.base5}\r\n"
        
        # gui lenh den canh tay robot
        try:
            self.ser.write(control_data.encode('utf-8'))
            self.get_logger().info(f"Command sent to robot arm from color: {control_data}")
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