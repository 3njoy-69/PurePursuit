import json
import math
import socket

# === Simulation parameters ===
dt = 0.1  # time step [s]
L_lookahead = 18  # look-ahead distance [m]

# Vehicle parameters (m)
LENGTH = 43
WIDTH = 21
BACKTOWHEEL = 9
WHEEL_LEN = 6
WHEEL_WIDTH = 2.2
TREAD = 16
WB = 25

# Define the vehicle class
class Vehicle:
    """Rear-axle bicycle kinematics."""

    def __init__(self, x, y, yaw, vel=0, max_steering_angle_deg=30):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        # Giới hạn góc lái là 30 độ (tức là từ -30 đến 30 độ)
        self.max_steering_angle = math.radians(max_steering_angle_deg)  # convert to radians

class Trajectory:
    """Hold path points and provide look-ahead target."""
    def __init__(self, xs, ys):
        self.xs = xs
        self.ys = ys
        self.last_idx = 0

    def getTargetPoint(self, pos):
        idx = self.last_idx
        # iterate forward through path
        while idx + 1 < len(self.xs) and getDistance(pos, [self.xs[idx], self.ys[idx]]) < L_lookahead:
            idx += 1
        self.last_idx = idx
        return [self.xs[idx], self.ys[idx]]

class PI:
    """Simple PI controller."""
    def __init__(self, kp=1.0, ki=0.1):
        self.kp = kp
        self.ki = ki
        self.I = 0.0

    def control(self, error):
        self.I += error * dt
        return self.kp * error + self.ki * self.I


def loadTrajectory(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    # list of [x,y,yaw]
    return data

def send_control_command(sock, v, delta_rad):
    """
    Gửi lệnh điều khiển tới server qua TCP.
    Định dạng:  "v,delta_deg\n"
        v          : vận tốc (cm/s)
        delta_deg  : góc lái, chuyển sang độ trước khi gửi
    """
    try:
        delta_deg = math.degrees(delta_rad)  # rad → °
        message = f"{v / 100:.3f},{delta_deg:.3f}\n"  # 3 chữ số lẻ cho gọn
        sock.sendall(message.encode("utf-8"))
    except Exception as e:
        print(f"Lỗi khi gửi dữ liệu: {e}")

# Function to receive real-time feedback (position data) from another program
def receive_feedback(sock):
    try:
        # Receive data from the connected client (feedback from sensors, like GPS, IMU, etc.)
        data = sock.recv(1024).decode('utf-8')  # Receive feedback data as string
        if data:
            feedback = data.split(",")  # Assuming the format is: "x,y,yaw"
            return float(feedback[0]), float(feedback[1]), float(feedback[2])  # return x, y, yaw
        return None, None, None
    except Exception as e:
        print(f"Lỗi khi nhận dữ liệu: {e}")
        return None, None, None

def getDistance(p1, p2):
    """Euclidean distance between two points."""
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def check_socket_connection(sock):
    """Check if socket is still connected."""
    try:
        sock.send(b'')  # Try sending an empty byte to see if it's still open
        return True
    except (socket.error, OSError):
        return False

def main():
    # Địa chỉ IP và cổng TCP server
    TCP_IP = "127.0.0.1"  # Thay bằng địa chỉ thực tế
    TCP_PORT = 5000

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((TCP_IP, TCP_PORT))
        sock.listen(1)
        print(f"Đang lắng nghe kết nối từ chương trình khác tại {TCP_IP}:{TCP_PORT}...")
        client_socket, client_address = sock.accept()
        print(f"Đã kết nối với {client_address}")
    except Exception as e:
        print(f"Lỗi kết nối TCP: {e}")
        return

    # load path
    path = loadTrajectory('path_data.json')
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    yaws = [p[2] for p in path]

    # initialize vehicle at first point, reverse speed
    target_vel = -10.0  # m/s (negative => reverse)
    ego = Vehicle(xs[0], ys[0], yaws[0], vel=target_vel)

    # controllers
    pid_acc   = PI(kp=1.0, ki=0.1)
    pid_steer = PI(kp=2.0, ki=0.1)

    traj = Trajectory(xs, ys)
    goal = [xs[-1], ys[-1]]

    while getDistance([ego.x, ego.y], goal) > 1:
        # look-ahead target
        tp = traj.getTargetPoint([ego.x, ego.y])

        # longitudinal control
        acc = pid_acc.control(target_vel - ego.vel)

        # desired heading
        yaw_des = math.atan2(tp[1]-ego.y, tp[0]-ego.x)
        if ego.vel < 0:
            # flip for reverse
            yaw_des = (yaw_des + math.pi) % (2*math.pi)

        # heading error normalized
        err = (yaw_des - ego.yaw + math.pi) % (2*math.pi) - math.pi
        # steering control
        ctrl = pid_steer.control(err)
        # invert steering for reverse
        delta = -ctrl if ego.vel < 0 else ctrl

        # update vehicle
        # Receive feedback (position data) from another program (e.g., GPS, IMU)
        if check_socket_connection(client_socket):
            ego.x, ego.y, ego.yaw = receive_feedback(client_socket)
        else:
            print("Socket connection lost. Terminating.")
            break

        # Gửi lệnh (v, delta) qua TCP
        send_control_command(client_socket, ego.vel, delta)

        # In ra target point, velocity, và steering angle
        print(f"Target Point: {tp}")
        print(f"Current Velocity: {ego.vel} m/s")
        print(f"Steering Angle: {math.degrees(delta):.2f} degrees")

    print('Done reverse follow')
    client_socket.close()
    sock.close()

if __name__ == '__main__':
    main()
