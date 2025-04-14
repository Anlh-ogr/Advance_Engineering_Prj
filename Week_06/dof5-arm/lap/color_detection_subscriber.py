# subscriber nhan du lieu tu topic /color_info va /camera/image_raw tren laptop
# hien thi video voi bounding box va nhan mau sac

import rclpy                        # type: ignore
import cv2                          # type: ignore
import numpy as np                  # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore
from sensor_msgs.msg import Image   # type: ignore
from cv_bridge import CvBridge      # type: ignore


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription_info = self.create_subscription(String, '/color_info', self.color_callback, 10)
        self.subscription_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cv_bridge = CvBridge()
        self.current_color = None  # luu mau sac hien tai
        self.get_logger().info("Camera subscriber initialized successfully")
        
    def color_callback(self, msg):
        # xu ly thong tin mau sac
        try:
            data = msg.data
            if data.startswith("color:"):
                self.current_color = data.split(":")[1]  # lay yellow, green, pink
                self.get_logger().info(f"Received color: {self.current_color}")
            else:
                self.get_logger().warning(f"Invalid color format: {data}")
        except (KeyError, ValueError) as e:
            self.get_logger().warning(f"Invalid message format: {e}")

    def image_callback(self, msg):
        # nhan va hien thi hinh anh
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # neu co mau sac duoc phat hien, xu ly bounding box
        if self.current_color:
            # chuyen frame sang HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            colors = {'yellow': ([20, 100, 100], [30, 255, 255]),
                      'green': ([40, 100, 100], [80, 255, 255]),
                      'pink': ([140, 100, 100], [170, 255, 255])}
            lower, upper = colors.get(self.current_color, ([0, 0, 0], [0, 0, 0]))
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 500 < area < 1500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"color:{self.current_color}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # hien thi frame
        cv2.imshow('Color Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()