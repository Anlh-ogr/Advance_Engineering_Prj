import cv2
import time

def check_camera(camera_id=1):
    # Khởi tạo camera
    cap = cv2.VideoCapture(camera_id)
    
    # Kiểm tra camera có mở được không
    if not cap.isOpened():
        print(f"Không thể mở camera ID {camera_id}. Thử ID khác (0, 1, 2, ...)")
        return
    
    # Lấy thông tin camera
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Camera ID {camera_id} opened successfully!")
    print(f"Độ phân giải: {width}x{height}")
    print(f"FPS: {fps if fps > 0 else 'Không xác định'}")
    
    print("Nhấn 's' để chụp ảnh, 'q' để thoát")
    
    snapshot_count = 0
    while True:
        # Đọc frame từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc frame từ camera!")
            break
        
        # Vẽ hình chữ nhật 30x30 cm (giả sử 1 cm = 37.8 pixel, tùy thuộc vào camera)
        cm_to_pixel = 37.8  # Tỉ lệ chuyển đổi từ cm sang pixel
        rect_width = int(30 * cm_to_pixel)  # Chiều rộng hình chữ nhật (30 cm)
        rect_height = int(30 * cm_to_pixel)  # Chiều cao hình chữ nhật (30 cm)
        top_left = (50, 50)  # Góc trên bên trái của hình chữ nhật
        bottom_right = (top_left[0] + rect_width, top_left[1] + rect_height)
        color = (0, 255, 0)  # Màu xanh lá cây
        thickness = 2  # Độ dày của đường viền
        cv2.rectangle(frame, top_left, bottom_right, color, thickness)
        
        # Hiển thị frame
        cv2.imshow("Camera Feed", frame)
        
        # Xử lý phím
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Thoát chương trình")
            break
        elif key == ord('s'):
            # Chụp và lưu ảnh
            snapshot_filename = f"snapshot_{snapshot_count}.jpg"
            cv2.imwrite(snapshot_filename, frame)
            print(f"Đã lưu ảnh: {snapshot_filename}")
            snapshot_count += 1
    
    # Giải phóng camera và đóng cửa sổ
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Thử camera ID 0 (mặc định), có thể đổi thành 1, 2 nếu cần
    check_camera(camera_id=1)