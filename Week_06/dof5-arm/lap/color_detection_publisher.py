# nhan dien mau sac tu camera va xuat ra vi tri
# gui thong tin qua topic /color_info

import rclpy                        # type: ignore
import cv2                          # type: ignore
import numpy as np                  # type: ignore
from rclpy.node import Node         # type: ignore
from std_msgs.msg import String     # type: ignore


class ColorDetectionPublisher(Node):
    def __init__(self):
        super().__init__('color_detection_publisher')
        self.publisher_ = self.create_publisher(String, '/color_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.get_logger().info("Camera initialized")
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame")
            return
        
        # chuyen sang HSV nhan dien mau sac
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # nguong mau sac
        colors = {'yellow': ([20, 100, 100], [30, 255, 255]),
                  'green': ([40, 100, 100], [80, 255, 255]),
                  'pink': ([140, 100, 100], [170, 255, 255])}
        for color, (lower, upper) in colors.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # loc dien tich khoi 3cmx3cmx3cm
                if 500 < area < 1500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    msg = String()
                    msg.data = f"Color: {color}, Area: {area}, Position: ({x}, {y})"
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Detected: {msg.data}")
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
        # hien thi hinh anh
        cv2.imshow('Color Detection', frame)
        cv2.waitKey(1)
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
        
def main(args=None):
    rclpy.init()
    node = ColorDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
