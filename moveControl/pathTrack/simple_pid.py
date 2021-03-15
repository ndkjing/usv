import threading
import math
import time
import json
from numpy import e
import os
from utils import lng_lat_calculate
from utils import log
import config
logger = log.LogHandler('pi_log')


class SimplePid:
    def __init__(self):
        self.errorSum = 0
        self.currentError = 0
        self.previousError = 0
        # 左右侧超声波距离，没有返回None  -1 表示距离过近
        self.left_distance=None
        self.right_distance=None

    def distance_p(self, distance, theta_error):
        # motor_forward = int(config.motor_forward * (180-abs(theta_error))/(180*1.4))
        motor_forward = int(config.motor_forward)
        pwm = int((distance * (motor_forward/config.full_speed_meter)))
        if pwm >= config.motor_forward:
            pwm = config.motor_forward
        pwm = int(pwm * (180-abs(theta_error))/180)
        return pwm

    def update_steer_pid(self, theta_error):
        errorSum = self.errorSum + theta_error
        control = config.kp * theta_error + config.ki * errorSum + \
            config.kd * (theta_error - self.previousError)
        self.previousError = theta_error
        # 控制量归一化
        pwm = int((control / 180.0) * config.motor_steer)
        if pwm >= config.motor_steer:
            pwm = config.motor_steer
        return pwm

    def update_steer_pid_1(self,steer_distance):
        error_i = steer_distance
        errorSum = self.errorSum + error_i
        control = config.kp * error_i + config.ki * errorSum + \
            config.kd * (error_i - self.previousError)
        self.previousError = error_i
        return control

    def pid_pwm(self, distance, theta_error):
        forward_pwm = self.distance_p(distance, theta_error)
        steer_pwm = self.update_steer_pid(theta_error)
        # 当前进分量过小时等比例减小转弯分量
        # steer_pwm = int(steer_pwm*forward_pwm/config.motor_forward)
        if (forward_pwm + steer_pwm) == 0:
            return 1500, 1500
        # scale_pwm = (config.max_pwm-config.stop_pwm)/(forward_pwm+abs(steer_pwm))
        scale_pwm = 1
        left_pwm = 1500 + int(forward_pwm*scale_pwm) - int(steer_pwm*scale_pwm)
        right_pwm = 1500 + int(forward_pwm*scale_pwm) + int(steer_pwm*scale_pwm)
        # print('theta_error forward_pwm,steer_pwm,left_pwm,right_pwm',theta_error, forward_pwm,steer_pwm,left_pwm,right_pwm)
        return left_pwm, right_pwm

    @staticmethod
    def make_to_pwm(a):
        d = ((0.5 * a + 1.5) * 1000)
        return d

    def pid_pwm_1(self, distance, theta_error):
        steer_control = self.update_steer_pid_1(theta_error)
        steer_uniform = 2.0 / (1.0 + e**(-0.1 * steer_control * 20)) - 1.0
        forward_pwm = SimplePid.make_to_pwm(2.0 / (1.0 + e**(-0.1 * distance * 3)) - 1.0)
        left_steer_pwm = SimplePid.make_to_pwm(steer_uniform)
        right_steer_pwm = SimplePid.make_to_pwm(-steer_uniform)
        steer_ratio = 0.9 * abs(left_steer_pwm - 1500) / (abs(right_steer_pwm - 1500) + abs(forward_pwm - 1500))
        left_pwm = (left_steer_pwm - 1500) * steer_ratio + (forward_pwm - 1500) * (1 - steer_ratio) + 1500
        right_pwm = (right_steer_pwm - 1500) * steer_ratio + (forward_pwm - 1500) * (1 - steer_ratio) + 1500
        print('steer_uniform,forward_pwm,left_steer_pwm,left_pwm,right_pwm',steer_uniform,forward_pwm,left_steer_pwm,left_pwm,right_pwm)
        return left_pwm, right_pwm

    def yaw_control(self, yaw):
        """
        设置旋转指定角度
        :param yaw:
        :return:
        """
        steer_pwm = self.update_steer_pid(yaw)
        left_pwm = 1500 - steer_pwm
        right_pwm = 1500 - steer_pwm
        return left_pwm, right_pwm

    def point_control(self,distance,theta_error):
        """
        到达指定目标点
        :param distance:距离目标点距离
        :param theta_error: 船头与目标角度偏差角度
        :return:左右点击PWM值
        """
        if abs(theta_error) > 180:
            if theta_error > 0:
                theta_error = theta_error - 360
            else:
                theta_error = 360 + theta_error
        left_pwm, right_pwm = self.pid_pwm(distance, theta_error)
        return left_pwm, right_pwm
