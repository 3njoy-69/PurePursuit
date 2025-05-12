import json
import math
import matplotlib.pyplot as plt
import numpy as np
import socket

# === Simulation parameters ===
dt = 0.1          # time step [s]
L_lookahead = 18 # look-ahead distance [m]

# Vehicle parameters (m)
LENGTH = 43
WIDTH = 21
BACKTOWHEEL = 9
WHEEL_LEN = 6
WHEEL_WIDTH = 2.2
TREAD = 16
WB = 25


def plotVehicle(x, y, yaw, steer, color='-k'):
    """
    Plot bicycle footprint: chassis and wheels.
    Rear-axle reference at (x,y).
    """
    # chassis outline local coords
    pts = np.array([
        [-BACKTOWHEEL, LENGTH-BACKTOWHEEL, LENGTH-BACKTOWHEEL, -BACKTOWHEEL, -BACKTOWHEEL],
        [ WIDTH/2,        WIDTH/2,        -WIDTH/2,      -WIDTH/2,      WIDTH/2]
    ])
    # rotation for yaw
    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    chassis = R_yaw.dot(pts)
    chassis[0,:] += x; chassis[1,:] += y
    plt.plot(chassis[0], chassis[1], color)

    # wheel template around axle
    w = np.array([
        [ WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN,  WHEEL_LEN,  WHEEL_LEN],
        [ WHEEL_WIDTH/2,  WHEEL_WIDTH/2, -WHEEL_WIDTH/2, -WHEEL_WIDTH/2, WHEEL_WIDTH/2]
    ])
    # wheel positions: front-left, front-right, rear-left, rear-right
    pos = [
        ( WB,  TREAD/2),  # front-left
        ( WB, -TREAD/2),  # front-right
        (  0,  TREAD/2),  # rear-left
        (  0, -TREAD/2)   # rear-right
    ]
    for i,(dx,dy) in enumerate(pos):
        wheel = w.copy()
        if i < 2:
            # apply steering rotation for front wheels
            R_steer = np.array([[math.cos(steer), -math.sin(steer)], [math.sin(steer), math.cos(steer)]])
            wheel = R_steer.dot(wheel)
        # translate to axle
        wheel[0,:] += dx; wheel[1,:] += dy
        # apply vehicle yaw + translate global
        wheel = R_yaw.dot(wheel)
        wheel[0,:] += x; wheel[1,:] += y
        plt.plot(wheel[0], wheel[1], color)


def getDistance(p1, p2):
    """Euclidean distance between two points."""
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

class Vehicle:
    """Rear-axle bicycle kinematics."""

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
        # Giới hạn góc lái trong phạm vi từ -30 độ đến 30 độ (tương đương với -math.radians(30) đến math.radians(30))
        delta = max(-self.max_steering_angle, min(self.max_steering_angle, delta))
        # velocity update
        self.vel += acc * dt
        # position update
        self.x   += self.vel * math.cos(self.yaw) * dt
        self.y   += self.vel * math.sin(self.yaw) * dt
        # heading update
        self.yaw += self.vel / WB * math.tan(delta) * dt

class Trajectory:
    """Hold path points and provide look-ahead target."""
    def __init__(self, xs, ys):
        self.xs = xs; self.ys = ys; self.last_idx = 0

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
        self.kp = kp; self.ki = ki; self.I = 0.0

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
    history_x, history_y = [], []

    plt.figure(figsize=(8,6))
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
        ego.update(acc, delta)

        # Gửi lệnh (v, delta) qua TCP
        send_control_command(sock, ego.vel, delta)

        history_x.append(ego.x)
        history_y.append(ego.y)

        # plot
        plt.cla()
        plt.plot(xs, ys, '-r', label='path')
        plt.plot(history_x, history_y, '-b', label='trajectory')
        plt.plot(tp[0], tp[1], 'og', label='target')
        plotVehicle(ego.x, ego.y, ego.yaw, delta)
        plt.axis('equal'); plt.grid(); plt.legend()
        plt.pause(0.05)

    print('Done reverse follow')
    plt.show()

if __name__ == '__main__':
    main()
