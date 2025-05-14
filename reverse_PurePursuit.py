import json
import math
import numpy as np
import socket

L = 18  # Look ahead distance in meters
dt = 0.1  # Discrete time step (seconds)

# Vehicle parameters (meters)
LENGTH = 43
WIDTH = 21
BACKTOWHEEL = 9
WHEEL_LEN = 6
WHEEL_WIDTH = 2.2
TREAD = 16
WB = 25

# Define the vehicle class
class Vehicle:
    def __init__(self, x, y, yaw, vel=-10, max_steering_angle_deg=30):  # Negative velocity for reverse
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        self.max_steering_angle = math.radians(max_steering_angle_deg)  # Convert to radians

# Trajectory class for handling path points
class Trajectory:
    def __init__(self, traj_x, traj_y):
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx)

# Helper function for calculating distance
def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

# PI Controller for controlling vehicle speed and steering angle
class PI:
    def __init__(self, kp=1.0, ki=0.1):
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        self.Pterm = self.kp * error
        self.Iterm += error * dt
        output = self.Pterm + self.ki * self.Iterm
        return output

# Load trajectory from file
def load_trajectory_from_file(filename):
    try:
        with open(filename, 'r') as file:
            data = json.load(file)
        return [(point[0], point[1], point[2]) for point in data]
    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        return []
    except json.JSONDecodeError:
        print(f"Error: Failed to decode JSON from file {filename}.")
        return []

# Function to send control commands via TCP
def send_control_command(sock, v, delta_rad):
    try:
        delta_deg = math.degrees(delta_rad)  # Convert rad to degrees
        message = f"{v / 100:.3f},{delta_deg:.3f}\n"  # Format the message
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

# Main function to control the vehicle
def main():
    TCP_IP = "127.0.0.1"
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

    # Load trajectory from file
    json_file = "path_data.json"
    traj_data = load_trajectory_from_file(json_file)

    if not traj_data:
        print("Quỹ đạo không được tải về, dừng chương trình.")
        return

    traj_x = [point[0] for point in traj_data]
    traj_y = [point[1] for point in traj_data]
    traj_yaw = [point[2] for point in traj_data]

    start_x, start_y, start_yaw = traj_x[0], traj_y[0], traj_yaw[0]
    target_vel = -10  # Constant negative velocity for reverse (cm/s)
    traj = Trajectory(traj_x, traj_y)
    goal = traj.getPoint(len(traj_x) - 1)

    ego = Vehicle(start_x, start_y, start_yaw)
    PI_acc = PI()
    PI_yaw = PI()

    try:
        while getDistance([ego.x, ego.y], goal) > 1:  # Loop until the vehicle is within 1 meter of the goal
            # Calculate target point for look ahead
            target_point = traj.getTargetPoint([ego.x, ego.y])

            # Receive feedback (position data) from another program (e.g., GPS, IMU)
            ego.x, ego.y, ego.yaw = receive_feedback(client_socket)

            if ego.x is None:  # If no valid feedback is received, continue with the loop
                print("Không nhận được dữ liệu feedback, tiếp tục.")
                continue

            # Calculate error for velocity and yaw
            vel_err = target_vel - ego.vel
            acc = PI_acc.control(vel_err)

            # Calculate yaw error as the difference between target yaw and current yaw
            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw

            # Flip for reverse (reverse vehicle has the opposite yaw control)
            yaw_err = (yaw_err + math.pi) % (2 * math.pi)

            # Normalize yaw error to be within [-pi, pi]
            yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi  # Normalize between -pi and pi

            # If yaw error is very small (meaning we are going straight), set delta to 0
            if abs(yaw_err) < 0.05:  # Tolerance threshold for yaw error
                delta = 0
            else:
                delta = PI_yaw.control(yaw_err) * -1

            # Ensure delta is within the allowed range [-30, 30] degrees
            delta = max(-math.radians(30), min(math.radians(30), delta))

            # Check if the vehicle has reached the goal (within a 1-meter threshold)
            if getDistance([ego.x, ego.y], goal) <= 1:
                print("Xe đã đến điểm cuối. Dừng lại.")
                ego.vel = 0  # Set velocity to 0 to stop the vehicle
                delta = 0  # Ensure the steering angle is also set to 0

            # Send control command (velocity and steering angle) over TCP
            send_control_command(client_socket, ego.vel, delta)

            # Print current target point, velocity and steering angle
            print(f"Target Point: {target_point}")
            print(f"Current Velocity: {ego.vel} cm/s")
            print(f"Steering Angle: {math.degrees(delta):.2f} degrees")

    finally:
        client_socket.close()
        sock.close()
        print("Đã đóng kết nối TCP.")

if __name__ == "__main__":
    main()
