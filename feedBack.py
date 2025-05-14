import socket
import time
import json

# Server thông tin
TCP_IP = "127.0.0.1"
TCP_PORT = 5000

# Hàm để đọc dữ liệu từ file JSON
def load_feedback_from_json(filename):
    try:
        with open(filename, 'r') as file:
            data = json.load(file)
        return data  # Dữ liệu là một list các điểm (x, y, yaw)
    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        return []
    except json.JSONDecodeError:
        print(f"Error: Failed to decode JSON from file {filename}.")
        return []

# Hàm để gửi feedback
def send_feedback():
    try:
        # Tạo kết nối đến server
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        print(f"Đã kết nối đến server tại {TCP_IP}:{TCP_PORT}")

        # Đọc dữ liệu feedback từ file JSON
        feedback_data = load_feedback_from_json("path_data.json")

        if not feedback_data:
            print("Không có dữ liệu feedback trong file.")
            return

        # Gửi feedback định kỳ từ file JSON
        for point in feedback_data:
            x, y, yaw = point
            # Tạo thông điệp để gửi: "x,y,yaw"
            message = f"{x:.2f},{y:.2f},{yaw:.2f}\n"
            # Gửi thông điệp qua TCP
            sock.sendall(message.encode("utf-8"))
            print(f"Đã gửi feedback: {message.strip()}")

            # Chờ 1 giây trước khi gửi feedback tiếp theo
            time.sleep(0.1)

    except Exception as e:
        print(f"Lỗi khi kết nối hoặc gửi dữ liệu: {e}")
    finally:
        sock.close()
        print("Đã đóng kết nối.")

# Chạy chương trình gửi feedback
if __name__ == "__main__":
    send_feedback()
