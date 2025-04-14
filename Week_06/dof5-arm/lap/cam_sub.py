# subscriber nhan du lieu tu topic /color_info tren laptop

import rclpy                        # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # subscription cho topic /color_info
        self.subscription = self.create_subscription(
            String, '/color_info', self.color_callback, 10
        )
        self.get_logger().info("Camera subscriber initialized successfully")
        
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
        
        # vi du: in thong tin hoac xu ly them
        self.get_logger().info(f"Detected object - Color: {color}, Position: ({x}, {y})")
        # them logic neu can (vi du: luu du lieu, dieu khien khac, ...)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()