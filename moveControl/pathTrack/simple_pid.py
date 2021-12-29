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

    def update_steer_pid_1(self, theta_error, b_dock=False):
        # 统计累计误差
        self.adjust_p_list.append(theta_error)
        error_sum = sum(self.adjust_p_list)
        max_error_sum = 1000
        if error_sum >= max_error_sum:
            error_sum = max_error_sum
        elif error_sum <= -max_error_sum:
            error_sum = -max_error_sum
        if b_dock:
            control = config.dock_kp * theta_error + config.dock_ki * error_sum + \
                      config.dock_kd * (theta_error - self.previousError)
            print('error_sum,config.dock_kp,config.dock_ki,config.dock_kd', error_sum, config.dock_kp, config.dock_ki,
                  config.dock_kd)
        else:
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
        根据距离和偏差角度计算左右点击输出
        @param distance:
        @param theta_error:
        @return:
        """
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (1.0 / (1.0 + e ** (-0.02 * steer_control)) - 0.5) * 1000
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

    def pid_pwm_forawrd(self, distance, theta_error):
        """
        前进时pid控制
        @param distance:
        @param theta_error:
        @return:
        """
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error)
        steer_pwm = (1.0 / (1.0 + e ** (-0.05 * steer_control)) - 0.5) * 1000
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

    def pid_back_dock(self, distance, theta_error, debug=False):
        # (1 / (1 + e ^ -0.2x) - 0.5) * 1000
        steer_control = self.update_steer_pid_1(theta_error, b_dock=True)
        steer_coefficient = -config.dock_steer_coefficient
        forward_coefficient = -config.dock_forward_coefficient
        steer_pwm = (0.4 / (1 + e ** (steer_coefficient * steer_control)) - 0.2) * 1000
        forward_pwm = (0.5 / (1.0 + e ** (forward_coefficient * distance)) - 0.25) * 1000
        # if forward_pwm < 50:
        #     forward_pwm += 50 - forward_pwm
        # 缩放到指定最大值范围内
        # max_control = config.max_pwm - config.stop_pwm
        max_control = 250
        if forward_pwm + abs(steer_pwm) > max_control:
            temp_forward_pwm = forward_pwm
            forward_pwm = max_control * (temp_forward_pwm) / (temp_forward_pwm + abs(steer_pwm))
            steer_pwm = max_control * (steer_pwm / (temp_forward_pwm + abs(steer_pwm)))
        if debug:
            print(time.time(),
                  "dock_steer_coefficient:%f,dock_forward_coefficient:%f,dock_kp:%f,dock_ki:%f,dock_kd:%f" % (
                      config.dock_steer_coefficient, config.dock_forward_coefficient, config.dock_kp, config.dock_ki,
                      config.dock_kd))
            print(time.time(), "theta_error:%f,steer_control:%f,steer_pwm:%f,forward_pwm:%f" % (
                theta_error, steer_control, steer_pwm, forward_pwm))
        left_pwm = config.stop_pwm -(int(forward_pwm) + int(steer_pwm))
        right_pwm = config.stop_pwm - (int(forward_pwm) - int(steer_pwm))
        return left_pwm, right_pwm