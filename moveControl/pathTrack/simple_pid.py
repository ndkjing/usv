import threading
import math
import time
import json
import copy
import numpy as np
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
        self.left_distance = None
        self.right_distance = None
        # 调节p数组
        self.adjust_p_size = 10
        self.adjust_p_list = []

    def distance_p(self, distance, theta_error):
        # motor_forward = int(config.motor_forward * (180-abs(theta_error))/(180*1.4))
        motor_forward = int(config.motor_forward)
        pwm = int((distance * (motor_forward / config.full_speed_meter)))
        if pwm >= config.motor_forward:
            pwm = config.motor_forward
        pwm = int(pwm * (180 - abs(theta_error)) / 180)
        return pwm

    def update_steer_pid(self, theta_error):
        errorSum = self.errorSum + theta_error
        control = config.kp * theta_error + config.ki * errorSum + \
                  config.kd * (theta_error - self.previousError)
        self.previousError = theta_error
        max_error_sum = 1000
        if errorSum>max_error_sum:
            errorSum = max_error_sum
        elif errorSum<-max_error_sum:
            errorSum = -max_error_sum
        self.errorSum = errorSum
        return control

    def update_steer_pid_1(self, theta_error):
        self.adjust_p_list.append(theta_error)
        # 统计累计误差
        if len(self.adjust_p_list) == self.adjust_p_size:
            # 通过误差角度队列修正p
            del self.adjust_p_list[0]
            self.adjust_p_list.append(theta_error)
            del self.adjust_p_list[self.adjust_p_list.index(max(self.adjust_p_list))]
            del self.adjust_p_list[self.adjust_p_list.index(min(self.adjust_p_list))]
            error_sum = sum(self.adjust_p_list)
        else:
            self.adjust_p_list.append(theta_error)
            error_sum = 0
        control = config.kp * theta_error + config.ki * error_sum + \
                  config.kd * (theta_error - self.previousError)
        self.previousError = theta_error
        return control

    def update_p(self):
        # 删除最大值和最小值
        del self.adjust_p_list[self.adjust_p_list.index(max(self.adjust_p_list))]
        del self.adjust_p_list[self.adjust_p_list.index(min(self.adjust_p_list))]
        adjust_p_list = copy.deepcopy(self.adjust_p_list)
        adjust_p_array = np.array(adjust_p_list)
        # 计算均值
        mean_var = np.nanmean(adjust_p_array)
        # 标准差
        std_var = np.nanstd(adjust_p_array)
        # 变异系数
        cv_var = np.nanmean(adjust_p_array) / np.nanstd(adjust_p_array)
        # 计算第一个值的z-分数
        z_var = (adjust_p_array[0] - np.nanmean(adjust_p_array)) / np.nanstd(adjust_p_array)
        # 如果都在0 附近则不用调节
        if abs(mean_var) < 30 and abs(std_var) < 30:
            pass
        # 如果出现大于和小于0的很大的数则减小p
        elif abs(mean_var) < 30 and abs(std_var) > 30:
            config.kp -= 0.02
        # 如果数值一直都在一侧且减小的很慢则增大p
        elif abs(mean_var) > 30 and abs(std_var) < 30:
            config.kp += 0.02

    def pid_pwm(self, distance, theta_error):
        # 更新最近的误差角度队列
        if len(self.adjust_p_list) < self.adjust_p_size - 1:
            self.adjust_p_list.append(theta_error)
        elif len(self.adjust_p_list) == self.adjust_p_size - 1:
            # 通过误差角度队列修正p
            self.update_p()
            del self.adjust_p_list[0]
            self.adjust_p_list.append(theta_error)

        forward_pwm = self.distance_p(distance, theta_error)
        steer_pwm = self.update_steer_pid(theta_error)
        # 当前进分量过小时等比例减小转弯分量
        # steer_pwm = int(steer_pwm*forward_pwm/config.motor_forward)
        if (forward_pwm + steer_pwm) == 0:
            return 1500, 1500
        # scale_pwm = (config.max_pwm-config.stop_pwm)/(forward_pwm+abs(steer_pwm))
        scale_pwm = 1
        left_pwm = 1500 + int(forward_pwm * scale_pwm) - int(steer_pwm * scale_pwm)
        right_pwm = 1500 + int(forward_pwm * scale_pwm) + int(steer_pwm * scale_pwm)
        print('theta_error',theta_error,'config.kp',config.kp,'config.kd',config.kd)
        # print('theta_error forward_pwm,steer_pwm,left_pwm,right_pwm',theta_error, forward_pwm,steer_pwm,left_pwm,right_pwm)
        return left_pwm, right_pwm

    @staticmethod
    def make_to_pwm(a):
        d = (0.5*a + 1.5) * 1000
        return d

    def pid_pwm_1(self, distance, theta_error):
        steer_control = self.update_steer_pid(theta_error)
        steer_uniform = 2.0 / (1.0 + e ** (-2 * steer_control))-1
        forward_pwm = SimplePid.make_to_pwm(1.0 / (1.0 + e ** (-0.3 * distance)))
        left_steer_pwm = SimplePid.make_to_pwm(steer_uniform)
        right_steer_pwm = SimplePid.make_to_pwm(-steer_uniform)
        steer_ratio = 0.9 * abs(left_steer_pwm - 1500) / (abs(right_steer_pwm - 1500) + abs(forward_pwm - 1500))
        left_pwm = (left_steer_pwm - 1500) * steer_ratio + (forward_pwm - 1500) * (1 - steer_ratio) + 1500
        right_pwm = (right_steer_pwm - 1500) * steer_ratio + (forward_pwm - 1500) * (1 - steer_ratio) + 1500
        print('steer_uniform,forward_pwm,left_steer_pwm,left_pwm,right_pwm', steer_uniform, forward_pwm, left_steer_pwm,
              left_pwm, right_pwm)
        return left_pwm, right_pwm

    def pid_pwm_2(self, distance, theta_error):
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (1.0 / (1.0 + e ** (-0.02 * steer_control)) - 0.5) * 1000
        forward_pwm = (1.0 / (1.0 + e ** (-0.2 * distance)) - 0.5) * 1000
        # 缩放到指定最大值范围内
        max_control = config.max_pwm-config.stop_pwm
        if forward_pwm+abs(steer_pwm) > max_control:
            forward_pwm = max_control*(forward_pwm)/(forward_pwm+abs(steer_pwm))
            steer_pwm = max_control*steer_pwm/(forward_pwm+abs(steer_pwm))
        left_pwm = config.stop_pwm + int(forward_pwm) - int(steer_pwm)
        right_pwm = config.stop_pwm + int(forward_pwm) + int(steer_pwm)
        print('steer_control,forward_pwm,left_pwm,right_pwm', steer_control, forward_pwm, left_pwm,
              right_pwm)
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

    def point_control(self, distance, theta_error):
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
