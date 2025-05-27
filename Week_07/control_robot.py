import cv2
import numpy as np
import serial
import time

class RobotVisionController:
    def __init__(self, com_port="COM4", baudrate=115200, camera_id=1):
        # Khởi tạo camera
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            print("Không mở được camera!")
            exit()
        
        # Khởi tạo Serial
        try:
            self.serial = serial.Serial(com_port, baudrate, timeout=1)
            time.sleep(2)
            print(f"Connected to {com_port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Lỗi kết nối Serial: {e}")
            exit()

        # Trạng thái PWM (kênh 1–5, tương ứng servo 0, 1, 2, 3, 5)
        self.servo_position = {
            1: 1500,  # Servo 0 (kênh 1)
            2: 1500,  # Servo 1 (kênh 2)
            3: 1500,  # Servo 2 (kênh 3)
            4: 1500,  # Servo 3 (kênh 4)
            5: 2500   # Servo 5 (kênh 5, kẹp, giả định đóng)
        }

        # Thông số từ gamepad.py
        self.pwm_step = {
            1: 20,   # Servo 0
            2: 20,   # Servo 1
            3: 50,   # Servo 2
            4: 50,   # Servo 3
            5: 200   # Servo 5 (kẹp)
        }
        self.pwm_min = 500
        self.pwm_max = 2500
        self.move_time = 50
        self.delay = 50
        self.line_ending = "\r\n"

        # Thông số lưới
        self.grid_size = 3  # 3cm mỗi ô
        self.grid_rows = 10  # 30cm / 3cm
        self.grid_cols = 10
        self.work_area = (300, 300)  # mm (30x30 cm)

        # Thông số màu (HSV)
        self.colors = {
            "green": ([40, 50, 50], [80, 255, 255]),   # Xanh lá
            "pink": ([140, 50, 50], [170, 255, 255]),  # Hồng
            "yellow": ([20, 50, 50], [40, 255, 255])   # Vàng
        }

        # Hiệu chỉnh PWM (giả lập, cần điều chỉnh thực tế)
        self.pwm_range = {
            1: (1000, 2000),  # Servo 0 (x)
            2: (1000, 2000),  # Servo 1 (y)
            3: (1000, 2000),  # Servo 2 (z hoặc góc)
            4: (1000, 2000),  # Servo 3 (góc)
            5: (2300, 2500)   # Servo 5 (kẹp: mở/đóng)
        }

    def send_servo_cmd(self, commands):
        command_str = ""
        for servo_id, pwm in commands.items():
            pwm = max(self.pwm_min, min(self.pwm_max, pwm))
            self.servo_position[servo_id] = pwm
            command_str += f"#{servo_id}P{pwm}"
        command_str += f"T{self.move_time}D{self.delay}{self.line_ending}"
        try:
            self.serial.write(command_str.encode())
            print(f"Sent: {command_str.strip()}")
        except serial.SerialException as e:
            print(f"Lỗi gửi lệnh: {e}")

    def capture_background(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Không chụp được ảnh nền!")
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def detect_object(self, background):
        ret, frame = self.cap.read()
        if not ret:
            print("Không chụp được ảnh hiện tại!")
            return None, None

        # Chuyển sang grayscale và trừ nền
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(background, gray)
        _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        # Tach biên (Canny)
        edges = cv2.Canny(thresh, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Tìm vật thể
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Lọc nhiễu
                x, y, w, h = cv2.boundingRect(contour)
                if abs(w - h) < 50 and 50 < w < 150:  # Kích thước vật ~3x3 cm
                    # Phát hiện màu
                    roi = frame[y:y+h, x:x+w]
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    for color, (lower, upper) in self.colors.items():
                        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                        if cv2.countNonZero(mask) > 0.5 * w * h:
                            # Tính ô lưới
                            center_x = x + w // 2
                            center_y = y + h // 2
                            grid_x = int(center_x * self.grid_cols / self.cap.get(3))  # Chiều rộng frame
                            grid_y = int(center_y * self.grid_rows / self.cap.get(4))  # Chiều cao frame
                            return (grid_x, grid_y), color
        return None, None

    def map_to_pwm(self, grid_x, grid_y):
        # Giả lập IK: Ánh xạ (x, y) sang PWM (cần hiệu chỉnh thực tế)
        x_mm = grid_x * self.grid_size
        y_mm = grid_y * self.grid_size

        # Ánh xạ tuyến tính (30cm -> PWM range)
        pwm1 = int(np.interp(x_mm, [0, self.work_area[0]], self.pwm_range[1]))  # Servo 0 (x)
        pwm2 = int(np.interp(y_mm, [0, self.work_area[1]], self.pwm_range[2]))  # Servo 1 (y)
        pwm3 = 1500  # Servo 2 (giữ cố định, cần điều chỉnh)
        pwm4 = 1500  # Servo 3 (giữ cố định, cần điều chỉnh)
        return {1: pwm1, 2: pwm2, 3: pwm3, 4: pwm4}

    def pick_and_place(self, grid_pos, color):
        if not grid_pos:
            print("Không tìm thấy vật thể!")
            return

        grid_x, grid_y = grid_pos
        print(f"Vật {color} tại ô ({grid_x}, {grid_y})")

        # Di chuyển đến vị trí vật
        commands = self.map_to_pwm(grid_x, grid_y)
        self.send_servo_cmd(commands)
        time.sleep(0.1)  # Chờ di chuyển (~50ms + 50ms)

        # Mở kẹp (servo 5, PWM 2300)
        self.send_servo_cmd({5: 2300})
        time.sleep(0.1)

        # Đóng kẹp (servo 5, PWM 2500)
        self.send_servo_cmd({5: 2500})
        time.sleep(0.1)

        # Di chuyển đến vị trí thả (ô 0,0)
        commands = self.map_to_pwm(0, 0)
        self.send_servo_cmd(commands)
        time.sleep(0.1)

        # Mở kẹp
        self.send_servo_cmd({5: 2300})
        time.sleep(0.1)

    def run(self):
        print("Nhấn 'b' để chụp ảnh nền, 'q' để thoát")
        background = None
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Lỗi đọc frame từ camera!")
                continue

            # Hiển thị lưới
            h, w = frame.shape[:2]
            for i in range(self.grid_rows):
                y = int(i * h / self.grid_rows)
                cv2.line(frame, (0, y), (w, y), (255, 0, 0), 1)
            for j in range(self.grid_cols):
                x = int(j * w / self.grid_cols)
                cv2.line(frame, (x, 0), (x, h), (255, 0, 0), 1)

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('b'):
                background = self.capture_background()
                print("Đã chụp ảnh nền")
            elif key == ord('q'):
                break
            elif background is not None:
                grid_pos, color = self.detect_object(background)
                if grid_pos:
                    self.pick_and_place(grid_pos, color)
                    background = self.capture_background()  # Cập nhật nền

        self.cap.release()
        self.serial.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    controller = RobotVisionController(com_port="COM4", baudrate=115200, camera_id=1)
    controller.run()