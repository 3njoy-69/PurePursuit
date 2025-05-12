import json
import math
import matplotlib.pyplot as plt
import numpy as np
import socket

L = 38  # look ahead distance
dt = 0.1  # discrete time

# Vehicle parameters (m)
LENGTH = 43
WIDTH = 21
BACKTOWHEEL = 9
WHEEL_LEN = 6
WHEEL_WIDTH = 2.2
TREAD = 16
WB = 25


def plotVehicle(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):
    """
    The function is to plot the vehicle
    it is copied from https://github.com/AtsushiSakai/PythonRobotics/blob/187b6aa35f3cbdeca587c0abdb177adddefc5c2a/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L109
    """
    outline = np.array(
        [
            [
                -BACKTOWHEEL,
                (LENGTH - BACKTOWHEEL),
                (LENGTH - BACKTOWHEEL),
                -BACKTOWHEEL,
                -BACKTOWHEEL,
            ],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    fr_wheel = np.array(
        [
            [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [
                -WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
            ],
        ]
    )

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array(
        [[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]]
    )

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(
        np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor
    )
    plt.plot(
        np.array(fr_wheel[0, :]).flatten(),
        np.array(fr_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(rr_wheel[0, :]).flatten(),
        np.array(rr_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(fl_wheel[0, :]).flatten(),
        np.array(fl_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(rl_wheel[0, :]).flatten(),
        np.array(rl_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(x, y, "*")


def getDistance(p1, p2):
    """
    Calculate distance
    :param p1: list, point1
    :param p2: list, point2
    :return: float, distance
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


class Vehicle:
    def __init__(self, x, y, yaw, vel=0, max_steering_angle_deg=30):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        :param max_steering_angle_deg: float, max steering angle in degrees
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel
        # Giới hạn góc lái là 30 độ (tức là từ -30 đến 30 độ)
        self.max_steering_angle = math.radians(max_steering_angle_deg)  # convert to radians

    def update(self, acc, delta):
        """
        Vehicle motion model, here we are using simple bicycle model
        :param acc: float, acceleration
        :param delta: float, heading control (steering angle)
        """
        # Giới hạn góc lái trong phạm vi từ -30 độ đến 30 độ (tương đương với -math.radians(30) đến math.radians(30))
        delta = max(-self.max_steering_angle, min(self.max_steering_angle, delta))

        self.x += self.vel * math.cos(self.yaw) * dt
        self.y += self.vel * math.sin(self.yaw) * dt
        self.yaw += self.vel * math.tan(delta) / WB * dt
        self.vel += acc * dt


class Trajectory:
    def __init__(self, traj_x, traj_y):
        """
        Define a trajectory class
        :param traj_x: list, list of x position
        :param traj_y: list, list of y position
        """
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        """
        Get the next look ahead point
        :param pos: list, vehicle position
        :return: list, target point
        """
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx)


class PI:
    def __init__(self, kp=1.0, ki=0.1):
        """
        Define a PID controller class
        :param kp: float, kp coeff
        :param ki: float, ki coeff
        :param kd: float, kd coeff
        """
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        """
        PID main function, given an input, this function will output a control unit
        :param error: float, error term
        :return: float, output control
        """
        self.Pterm = self.kp * error
        self.Iterm += error * dt

        self.last_error = error
        output = self.Pterm + self.ki * self.Iterm
        return output


def load_trajectory_from_file(filename):
    """
    Đọc dữ liệu quỹ đạo từ file JSON và trả về danh sách các điểm [x, y, yaw].
    """
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


def main():
    # Địa chỉ IP và cổng TCP server
    TCP_IP = "127.0.0.1"  # Thay bằng địa chỉ thực tế
    TCP_PORT = 5000

    # Tạo socket TCP
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        print("Đã kết nối TCP tới server.")
    except Exception as e:
        print(f"Lỗi kết nối TCP: {e}")
        return

    # Đọc quỹ đạo từ file JSON
    json_file = "path_data.json"
    traj_data = load_trajectory_from_file(json_file)

    if not traj_data:
        print("Quỹ đạo không được tải về, dừng chương trình.")
        return

    traj_x = [point[0] for point in traj_data]
    traj_y = [point[1] for point in traj_data]
    traj_yaw = [point[2] for point in traj_data]

    start_x, start_y, start_yaw = traj_x[0], traj_y[0], traj_yaw[0]
    target_vel = 10
    traj = Trajectory(traj_x, traj_y)
    goal = traj.getPoint(len(traj_x) - 1)

    ego = Vehicle(start_x, start_y, start_yaw)
    PI_acc = PI()
    PI_yaw = PI()

    traj_ego_x, traj_ego_y = [], []

    plt.figure(figsize=(12, 8))

    try:
        while getDistance([ego.x, ego.y], goal) > 1:
            target_point = traj.getTargetPoint([ego.x, ego.y])
            vel_err = target_vel - ego.vel
            acc = PI_acc.control(vel_err)
            yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
            delta = PI_yaw.control(yaw_err)

            ego.update(acc, delta)

            # Gửi lệnh (v, delta) qua TCP
            send_control_command(sock, ego.vel, delta)

            traj_ego_x.append(ego.x)
            traj_ego_y.append(ego.y)

            plt.cla()
            plt.plot(traj_x, traj_y, "-r", linewidth=5, label="course")
            plt.plot(traj_ego_x, traj_ego_y, "-b", linewidth=2, label="trajectory")
            plt.plot(target_point[0], target_point[1], "og", ms=5, label="target point")
            plotVehicle(ego.x, ego.y, ego.yaw, delta)
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.legend()
            plt.grid(True)
            plt.pause(0.1)
    finally:
        sock.close()
        print("Đã đóng kết nối TCP.")


if __name__ == "__main__":
    main()
