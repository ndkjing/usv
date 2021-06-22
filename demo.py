import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')


class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if (F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        # 观测量协方差矩阵
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(I - np.dot(K, self.H), self.P)


def example():
    dt = 1.0 / 60
    F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    H = np.array([1, 0, 0]).reshape(1, 3)
    Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
    R = np.array([1]).reshape(1, 1)

    itr = 100

    def f(x):
        return np.dot(F, x) + np.random.normal(0, 5, 3)

    real_state = []
    x = np.array([0, 0, 0])

    for i in range(itr):
        real_state.append(x[0])
        x = f(x)

    measurements = [x - 1 + np.random.normal(0, 1) for x in real_state]

    kf = KalmanFilter(F=F, H=H, Q=Q, R=R)
    predictions = []
    for z in measurements:
        predictions.append(kf.predict()[0])
        kf.update(z)
    print(predictions, measurements)
    plt.plot(range(len(measurements)), measurements, label='Measurements')
    plt.plot(range(len(predictions)), np.array(predictions), label='Kalman Filter Prediction')
    plt.plot(range(len(real_state)), real_state, label='Real statement')
    # plt.legend()
    plt.show()


import json

if __name__ == '__main__':
    import enum
    class CurrentPlatform(enum.Enum):
        windwos = 1
        linux = 2
        pi = 3
        others = 4
    a = [(CurrentPlatform.windwos,CurrentPlatform.linux),(CurrentPlatform.windwos,CurrentPlatform.windwos)]
    b = (CurrentPlatform.windwos,CurrentPlatform.windwos)
    print(a.index(b))
    # 格式化成2016-03-20 11:45:39形式
    # example()
    # data = {1: 2}
    # with open('test.json', 'w') as f:
    #     json.dump(data, f)
    # with open('test.json', 'r') as f:
    #     data1 = json.load(f)
    #     print(type(data1))
    class ShipStatus(enum.Enum):
        """
        船状态
        idle  等待状态
        remote_control 遥控器控制
        computer_control 页面手动控制
        computer_auto 自动控制
        backhome_low_energy 低电量返航状态
        backhome_network 低电量返航状态
        at_home 在家
        tasking  执行检测/抽水任务中
        """
        idle = 1
        remote_control = 2
        computer_control = 3
        computer_auto = 4
        tasking = 5
        avoidance = 6
        backhome_low_energy = 7
        backhome_network = 8
        at_home = 9
    status_change_list = {
        (ShipStatus.idle, ShipStatus.computer_control):1,
        (ShipStatus.idle, ShipStatus.remote_control):1,
        (ShipStatus.computer_control, ShipStatus.tasking):1,

    }
    print(len(status_change_list))
