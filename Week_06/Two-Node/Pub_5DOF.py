import json
import rclpy                    # type: ignore
from rclpy.node import Node     # type: ignore
from std_msgs.msg import String # type: ignore

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.temp1 = 1200   # thong so khoi tao
        self.temp2 = 1000   # thong so khoi tao
        
    def timer_callback(self):
        msg = String()
        data = {"Motor1": self.temp1, "Motor2": self.temp2}
        msg.data = json.dumps(data)     # dict -> string
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.temp1 += 100
        self.temp2 += 100
        if (self.temp1 > 1500):
            self.temp1 = 1200
        if (self.temp2 > 2000):
            self.temp2 = 1000
            
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()