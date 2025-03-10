import json
import rclpy                                        # type: ignore
from rclpy.node import Node                         # type: ignore
from std_msgs.msg import String, Float64MultiArray  # type: ignore

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.servo_publisher = self.create_publisher(Float64MultiArray, 'servo_angles', 10)
        self.subscription = self.create_subscription(String, 'gamepad_data', self.gamepad_callback, 10)
        # 5 DoF: base, shoulder, elbow, wrist, gripper
        self.servo_angles = [90.0, 90.0, 90.0, 90.0, 90.0]
        self.step = 1.0  # Step size for servo movement
        
        # Timer to publish servo angles periodically
        self.timer = self.create_timer(0.1, self.publish_servo_angles)
    
    def gamepad_callback(self, msg):
        data = json.loads(msg.data)
        for code, state in data.items():
            if state != 0:
                if code == "ABS_X":      # Base rotation
                    self.servo_angles[0] += self.step if state > 0 else -self.step
                elif code == "ABS_Y":    # Shoulder
                    self.servo_angles[1] += self.step if state < 0 else -self.step
                elif code == "ABS_RZ":   # Elbow
                    self.servo_angles[2] += self.step if state > 0 else -self.step
                elif code == "ABS_Z":    # Wrist
                    self.servo_angles[3] += self.step if state > 0 else -self.step
                elif code == "BTN_TR":   # Gripper close
                    self.servo_angles[4] += self.step
                elif code == "BTN_TL":   # Gripper open
                    self.servo_angles[4] -= self.step
                
                # Limit servo angles to 0-180 degrees
                self.servo_angles = [max(0.0, min(180.0, angle)) for angle in self.servo_angles]
    
    def publish_servo_angles(self):
        msg = Float64MultiArray()
        msg.data = self.servo_angles
        self.servo_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()