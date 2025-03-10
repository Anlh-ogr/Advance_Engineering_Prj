import json
import rclpy                    # type: ignore
import serial                   # type: ignore
from std_msgs.msg import String # type: ignore
from rclpy.node import Node     # type: ignore

ser = serial.Serial(port='/dev/ttyTHS1',
                    # port='/dev/ttyUSB0',
                    # port='/dev/ttyACM0',
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1)

class MinimalSubscribe(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
    
    def listener_callback(self, msg):
        data = json.loads(msg.data) # string -> dict
        print(data)
        Motor1 = int(data['Motor1'])
        Motor2 = int(data['Motor2'])
        data_send = "#12P%d#14P%dPT100D100\r\n" % (Motor1, Motor2)
        ser.write(data_send.encode())
        self.get_logger().info('I heard: "%s"' % msg.data)
        
def main(args = None):
    rclpy.init (args = args)
    minimal_subscriber = MinimalSubscribe()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()