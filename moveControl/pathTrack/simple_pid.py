import copy
import time

import numpy as np
from numpy import e
from utils import log
from collections import deque
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
        self.adjust_p_size = 6
        self.adjust_p_list = deque(maxlen=self.adjust_p_size)

    def update_steer_pid_1(self, theta_error):
        # 统计累计误差
        self.adjust_p_list.append(theta_error)
        error_sum = sum(self.adjust_p_list)
        max_error_sum = 1000
        if error_sum >= max_error_sum:
            error_sum = max_error_sum
        elif error_sum <= -max_error_sum:
            error_sum = -max_error_sum
        control = config.kp * theta_error + config.ki * error_sum + \
                  config.kd * (theta_error - self.previousError)
        # print(time.time(), 'theta_error', theta_error, 'error_sum', error_sum, 'delta_error',
        #       theta_error - self.previousError)
        self.previousError = theta_error
        return control

    def update_p(self):
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

    # def pid_pwm(self, distance, theta_error):
    #     # 更新最近的误差角度队列
    #     if len(self.adjust_p_list) < self.adjust_p_size - 1:
    #         self.adjust_p_list.append(theta_error)
    #     elif len(self.adjust_p_list) == self.adjust_p_size - 1:
    #         # 通过误差角度队列修正p
    #         self.update_p()
    #         del self.adjust_p_list[0]
    #         self.adjust_p_list.append(theta_error)
    #     forward_pwm = self.distance_p(distance, theta_error)
    #     steer_pwm = self.update_steer_pid(theta_error)
    #     # 当前进分量过小时等比例减小转弯分量
    #     # steer_pwm = int(steer_pwm*forward_pwm/config.motor_forward)
    #     if (forward_pwm + steer_pwm) == 0:
    #         return 1500, 1500
    #     # scale_pwm = (config.max_pwm-config.stop_pwm)/(forward_pwm+abs(steer_pwm))
    #     scale_pwm = 1
    #     left_pwm = 1500 + int(forward_pwm * scale_pwm) - int(steer_pwm * scale_pwm)
    #     right_pwm = 1500 + int(forward_pwm * scale_pwm) + int(steer_pwm * scale_pwm)
    #     return left_pwm, right_pwm

    def pid_pwm_2(self, distance, theta_error):
        """
        直接用距离和角度计算，距离控制速度  角度控制转向
        :param distance:
        :param theta_error:
        :return:
        """
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (0.6 / (1.0 + e ** (-0.015 * steer_control)) - 0.3) * 1000
        forward_pwm = (1.0 / (1.0 + e ** (-0.2 * distance)) - 0.5) * 1000
        # 缩放到指定最大值范围内
        max_control = config.max_pwm - config.stop_pwm
        if forward_pwm + abs(steer_pwm) > max_control:
            temp_forward_pwm = forward_pwm
            forward_pwm = max_control * (temp_forward_pwm) / (temp_forward_pwm + abs(steer_pwm))
            steer_pwm = max_control * (steer_pwm / (temp_forward_pwm + abs(steer_pwm)))
        left_pwm = config.stop_pwm + int(forward_pwm) - int(steer_pwm)
        right_pwm = config.stop_pwm + int(forward_pwm) + int(steer_pwm)
        return left_pwm, right_pwm

    def pid_pwm_3(self, distance, theta_error):
        """
        使用距离和角度计算横向偏差，横向偏差与速度和角度一起计算需要的转向调节值输入到pid中，
        使用速度期望控制前进速度，起步 阶段增大  中间阶段恒定  避障和快要到达时减小
        :param distance:
        :param theta_error:
        :param v_exp 期望速度使用【-500，500】pwm值调节范围表示
        :return:
        """
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        forward_pwm = int(distance)
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (0.6 / (1.0 + e ** (-0.01 * steer_control)) - 0.3) * 1000
        # 缩放到指定最大值范围内
        max_control = config.max_pwm - config.stop_pwm
        if forward_pwm + abs(steer_pwm) > max_control:
            temp_forward_pwm = forward_pwm
            forward_pwm = max_control * (temp_forward_pwm) / (temp_forward_pwm + abs(steer_pwm))
            steer_pwm = max_control * (steer_pwm / (temp_forward_pwm + abs(steer_pwm)))
        left_pwm = config.stop_pwm + int(forward_pwm) - int(steer_pwm)
        right_pwm = config.stop_pwm + int(forward_pwm) + int(steer_pwm)
        return left_pwm, right_pwm

    def pid_pwm_4(self, distance, theta_error):
        """
        距离余弦值和角度计算，距离控制速度  角度控制转向
        :param distance:
        :param theta_error:
        :return:
        """
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (0.6 / (1.0 + e ** (-0.015 * steer_control)) - 0.3) * 1000
        forward_pwm = (1.0 / (1.0 + e ** (-0.3 * distance)) - 0.5) * 1000
        # 缩放到指定最大值范围内
        max_control = config.max_pwm - config.stop_pwm
        if forward_pwm + abs(steer_pwm) > max_control:
            temp_forward_pwm = forward_pwm
            forward_pwm = max_control * (temp_forward_pwm) / (temp_forward_pwm + abs(steer_pwm))
            steer_pwm = max_control * (steer_pwm / (temp_forward_pwm + abs(steer_pwm)))
        left_pwm = config.stop_pwm + int(forward_pwm) - int(steer_pwm)
        right_pwm = config.stop_pwm + int(forward_pwm) + int(steer_pwm)
        return left_pwm, right_pwm

    def pid_turn_pwm(self, angular_velocity_error):
        steer_control = self.update_steer_pid_1(angular_velocity_error)
        steer_pwm = (1.0 / (1.0 + e ** (-0.02 * steer_control)) - 0.5) * 1000
        left_pwm = config.stop_pwm - int(steer_pwm)
        right_pwm = config.stop_pwm + int(steer_pwm)
        return left_pwm, right_pwm

    def pid_angle_pwm(self, angle_error):
        steer_control = self.update_steer_pid_1(angle_error)
        steer_pwm = (1.0 / (1.0 + e ** (-0.02 * steer_control)) - 0.5) * 1000
        left_pwm = config.stop_pwm - int(steer_pwm)
        right_pwm = config.stop_pwm + int(steer_pwm)
        return left_pwm, right_pwm
