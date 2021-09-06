import os
import sys
import threading
import pigpio
import time
import copy
import math

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from utils import log
from utils import data_valid
from storage import save_data
from drivers import pi_softuart, com_data
import config
from moveControl.pathTrack import simple_pid

logger = log.LogHandler('pi_log')


class PiMain:
    def __init__(self, logger_=None):
        if logger_ is not None:
            self.logger_obj = logger_
        else:
            self.logger_obj = logger
        self.water_data_dict = {}
        # 遥控器控制外设标志位
        self.remote_draw_status = 0  # 遥控器控制抽水状态 0 未抽水 1抽水
        self.remote_drain_status = 0  # 遥控器控制排水状态 0 未排水 1排水
        self.remote_draw_steer = config.max_deep_steer_pwm  #
        self.remote_side_light_status = 2  # 遥控器控制舷灯状态 0 关闭 1 打开  2不管
        self.remote_head_light_status = 2  # 遥控器控制大灯状态 0 关闭 1 打开   2 不管
        # 树莓派pwm波控制对象
        self.left_pwm = config.stop_pwm
        self.right_pwm = config.stop_pwm
        self.target_left_pwm = config.stop_pwm
        self.target_right_pwm = config.stop_pwm
        self.pice = 20000
        self.diff = int(20000 / self.pice)
        self.hz = 50
        self.pi = pigpio.pi()
        # gpio脚的编号顺序依照Broadcom number顺序，请自行参照gpio引脚图里面的“BCM编码”，
        self.pi.set_PWM_frequency(config.left_pwm_pin, self.hz)  # 设定左侧电机引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_frequency(config.right_pwm_pin, self.hz)  # 设定右侧电机引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_range(config.left_pwm_pin, self.pice)
        self.pi.set_PWM_range(config.right_pwm_pin, self.pice)

        self.tick_0 = [None, None, None]
        self.tick_1 = [None, None, None]
        # self.temp_read = [[150 for col in range(21)] for row in range(7)]
        # self.count = [1, 0, 0, 0, 0, 0, 0]
        # self.save = [1, 161, 150, 157, 152, 142, 101]
        self.channel_row_input_pwm = 0
        self.channel_col_input_pwm = 0
        # 当前是否是遥控器控制  2.4g遥控器和lora遥控器都用这个标志位
        self.b_start_remote = 0
        if config.current_platform == config.CurrentPlatform.pi:
            self.gps_obj = self.get_gps_obj()
            self.compass_obj = self.get_compass_obj()
            self.weite_compass_obj = self.get_weite_compass_obj()
            if config.b_pin_stc:
                self.stc_obj = self.get_stc_obj()
            if config.b_laser:
                self.laser_obj = self.get_laser_obj()
            if config.b_sonar:
                self.sonar_obj = self.get_sonar_obj()
            if config.b_millimeter_wave:
                self.millimeter_wave_obj = self.get_millimeter_wave_obj()
            if config.b_lora_remote_control:
                self.remote_control_obj = self.get_remote_control_obj()
            if config.b_use_remote_control:
                self.cb1 = self.pi.callback(config.channel_1_pin, pigpio.EITHER_EDGE, self.mycallback)
                self.cb2 = self.pi.callback(config.channel_3_pin, pigpio.EITHER_EDGE, self.mycallback)
                self.cb3 = self.pi.callback(config.channel_remote_pin, pigpio.EITHER_EDGE, self.mycallback)
        # GPS
        self.lng_lat = None
        # GPS 误差
        self.lng_lat_error = None
        # 罗盘角度   还没收到罗盘数据则为NOne  收到后正常为罗盘值 ** 当罗盘失效后为 -1
        self.theta = None
        self.last_theta = None
        # 罗盘提示消息
        self.compass_notice_info = ''
        # 距离矩阵
        self.distance_dict = {}
        self.field_of_view = config.field_of_view  # 视场角
        self.view_cell = config.view_cell  # 量化角度单元格
        self.cell_size = int(self.field_of_view / self.view_cell)
        self.obstacle_list = [0] * self.cell_size  # 自动避障列表
        self.control_obstacle_list = [0] * self.cell_size  #
        # 设置为GPIO输出模式 输出高低电平
        """
        self.pi.set_mode(config.side_left_gpio_pin, pigpio.OUTPUT)
        self.pi.set_mode(config.side_right_gpio_pin, pigpio.OUTPUT)
        self.pi.set_mode(config.headlight_gpio_pin, pigpio.OUTPUT)
        self.pi.set_mode(config.audio_light_alarm_gpio_pin, pigpio.OUTPUT)
        self.pi.set_mode(config.draw_left_gpio_pin, pigpio.OUTPUT)
        self.pi.set_mode(config.draw_right_gpio_pin, pigpio.OUTPUT)
        """
        # 抽水泵舵机
        self.draw_steer_pwm = config.max_deep_steer_pwm
        # 云台舵机角度
        self.pan_angle_pwm = 1500
        self.tilt_angle_pwm = 1500
        # 记录继电器输出电平 1 高电平 0 低电平
        self.left_motor_output = 0
        self.right_motor_output = 0
        self.headlight_output = 0
        self.alarm_light_output = 0
        self.left_sidelight_output = 0
        self.right_sidelight_output = 0
        # 遥控器控制数据
        self.remote_control_data = []
        # 求角速度     逆时针方向角速度为正值
        self.theta_list = []  # (时间，角度)
        self.angular_velocity = None
        self.last_angular_velocity = None
        self.pid_obj = simple_pid.SimplePid()
        # 将舵机归位
        # self.set_draw_deep(deep_pwm=self.draw_steer_pwm)
        # 记录罗盘数据用于分析 每一千次存储一次
        self.compass_data_list = []
        # 记录上一次收到有效lora遥控器数据时间
        self.lora_control_receive_time = None
        # 串口数据收发对象
        if config.b_com_stc:
            self.com_data_obj = self.get_com_obj(port=config.stc_port,
                                                 baud=config.stc_baud,
                                                 timeout=config.stc2pi_timeout
                                                 )
        self.dump_energy = None
        self.last_dump_energy = None  # 用于判断记录日志用

    # 获取串口对象
    @staticmethod
    def get_com_obj(port, baud, logger_=None, timeout=0.4):
        return com_data.ComData(
            port,
            baud,
            timeout=timeout,
            logger=logger_)

    def get_millimeter_wave_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.millimeter_wave_rx, tx_pin=config.millimeter_wave_tx,
                                      baud=config.millimeter_wave_baud, time_out=0.02)

    def get_compass_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_compass_rx, tx_pin=config.pin_compass_tx,
                                      baud=config.pin_compass_baud, time_out=0.1)

    def get_weite_compass_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=21, tx_pin=20,
                                      baud=9600)

    def get_remote_control_obj(self):
        """
        遥控器lora串口
        :return:
        """
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.lora_rx, tx_pin=config.lora_tx,
                                      baud=config.lora_baud, time_out=0.16)

    def get_gps_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_gps_rx, tx_pin=config.pin_gps_tx,
                                      baud=config.pin_gps_baud)

    def get_laser_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=19, tx_pin=13, baud=115200, time_out=0.01)

    def get_sonar_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.sonar_rx, tx_pin=config.sonar_tx,
                                      baud=config.sonar_baud, time_out=0.01)

    def get_stc_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.stc_rx, tx_pin=config.stc_tx,
                                      baud=config.stc_baud, time_out=1)

    # 罗盘角度滤波
    def compass_filter(self, theta_):
        current_time = time.time()
        if theta_:
            if len(self.theta_list) >= 20:
                self.theta_list.pop(0)
            self.theta_list.append((current_time, theta_))
            if len(self.theta_list) >= 4:
                self.angular_velocity = round((self.theta_list[-1][1] - self.theta_list[-4][1]) / (
                        self.theta_list[-1][0] - self.theta_list[-4][0]), 1)
                self.last_angular_velocity = self.angular_velocity
            if not self.last_theta:
                self.last_theta = theta_
                return_theta = theta_
            else:
                if abs(theta_ - self.last_theta) > 180:
                    return_theta = self.last_theta
                else:
                    self.last_theta = theta_
                    return_theta = theta_
                self.last_theta = theta_
        else:
            self.angular_velocity = self.last_angular_velocity
            return_theta = self.last_theta
        return return_theta

    # 检查遥控器输入
    def check_remote_pwm(self, b_lora_remote_control=True):
        """
        遥控器输入
        :return:
        """
        if b_lora_remote_control:
            if self.remote_control_data:
                remote_forward_pwm = int((99 - self.remote_control_data[2]) * 10 + 1000)
                remote_steer_pwm = int(self.remote_control_data[3] * 10 + 1000)
            else:
                remote_forward_pwm = 1500
                remote_steer_pwm = 1500
        else:
            remote_forward_pwm = copy.deepcopy(int(self.channel_col_input_pwm))
            remote_steer_pwm = copy.deepcopy(int(self.channel_row_input_pwm))
        # 防止抖动
        if 1600 > remote_forward_pwm > 1400:
            remote_forward_pwm = config.stop_pwm
        # 防止过大值
        elif remote_forward_pwm >= 2000:
            remote_forward_pwm = 2000
        # 防止初始读取到0电机会转动， 设置为1500
        elif remote_forward_pwm < 900:
            remote_forward_pwm = config.stop_pwm
        # 防止过小值
        elif 1000 >= remote_forward_pwm >= 900:
            remote_forward_pwm = 1000
        # 防止抖动
        if 1600 > remote_steer_pwm > 1400:
            remote_steer_pwm = config.stop_pwm
        # 防止过大值
        elif remote_steer_pwm >= 2000:
            remote_steer_pwm = 2000
        # 防止初始读取到0电机会转动， 设置为1500
        elif remote_steer_pwm < 900:
            remote_steer_pwm = config.stop_pwm
        # 防止过小值
        elif 1000 >= remote_steer_pwm >= 900:
            remote_steer_pwm = 1000
        if remote_forward_pwm == config.stop_pwm and remote_steer_pwm == config.stop_pwm:
            remote_left_pwm = config.stop_pwm
            remote_right_pwm = config.stop_pwm
        else:
            remote_left_pwm = 1500 + (remote_forward_pwm - 1500) + (remote_steer_pwm - 1500)
            remote_right_pwm = 1500 + (remote_forward_pwm - 1500) - (remote_steer_pwm - 1500)
        return remote_left_pwm, remote_right_pwm

    # 获取pwm波输入
    def mycallback(self, gpio, level, tick):
        if level == 0:
            if int(gpio) == int(config.channel_1_pin):
                self.tick_0[0] = tick
                if self.tick_1[0] is not None:
                    diff = pigpio.tickDiff(self.tick_1[0], tick)
                    self.channel_row_input_pwm = int(diff)
                    # self.temp_read[0][self.count[0]] = diff
                    # self.save[0] = int(self.temp_read[0][self.count[0]])
            if int(gpio) == int(config.channel_3_pin):
                self.tick_0[1] = tick
                if self.tick_1[1] is not None:
                    diff = pigpio.tickDiff(self.tick_1[1], tick)
                    self.channel_col_input_pwm = int(diff)
                    # self.temp_read[1][self.count[1]] = diff
                    # self.save[1] = int(self.temp_read[1][self.count[1]])
            if int(gpio) == config.channel_remote_pin:
                self.tick_0[2] = tick
                if self.tick_1[2] is not None:
                    diff = pigpio.tickDiff(self.tick_1[2], tick)
                    if diff > 1800:
                        self.b_start_remote = 1
                    elif diff < 1200:
                        self.b_start_remote = 0
        else:
            if gpio == int(config.channel_1_pin):
                self.tick_1[0] = tick
            if gpio == int(config.channel_3_pin):
                self.tick_1[1] = tick
            if gpio == config.channel_remote_pin:
                self.tick_1[2] = tick

    def forward(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = config.stop_pwm + int(config.speed_grade) * 100
        if right_pwm is None:
            right_pwm = config.stop_pwm + int(config.speed_grade) * 100
        self.set_pwm(left_pwm, right_pwm)

    def backword(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = config.stop_pwm - int(config.speed_grade) * 100
        if right_pwm is None:
            right_pwm = config.stop_pwm - int(config.speed_grade) * 100
        self.set_pwm(left_pwm, right_pwm)

    def left(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1300
        if right_pwm is None:
            right_pwm = 1700
        self.set_pwm(left_pwm, right_pwm)

    def right(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1700
        if right_pwm is None:
            right_pwm = 1300
        self.set_pwm(left_pwm, right_pwm)

    # 设置云台舵机角度
    def set_ptz_camera(self, pan_angle_pwm_=1500, tilt_angle_pwm_=1500):
        """
        设置云台舵机角度
        :param pan_angle_pwm_: 500-2500 设置水平角度
        :param tilt_angle_pwm_: 500-2500  设置垂直角度
        :return:
        """
        pin_pan = 2
        pin_tilt = 3
        self.pi.set_servo_pulsewidth(pin_pan, pan_angle_pwm_)
        self.pi.set_servo_pulsewidth(pin_tilt, tilt_angle_pwm_)
        self.pan_angle_pwm = pan_angle_pwm_
        self.tilt_angle_pwm = tilt_angle_pwm_

    # 设置抽水泵舵深度
    def set_draw_deep(self, deep_pwm, b_slow=True):
        """
        设置抽水泵深度
        :param b_slow:
        :param deep_pwm:电机目标深度
        :return:
        """
        # 如果没有可调节深度舵机跳过调节
        if not config.b_control_deep:
            return
        if b_slow:
            delta_change = 10
            while self.draw_steer_pwm != deep_pwm:
                add_or_sub = 1 if deep_pwm - self.draw_steer_pwm > 0 else -1
                self.draw_steer_pwm = self.draw_steer_pwm + delta_change * add_or_sub
                self.pi.set_servo_pulsewidth(config.draw_steer, self.draw_steer_pwm)
                time.sleep(0.06)
                # if self.draw_steer_pwm < 1500:
                #     time.sleep(0.07)
                # else:
                #     time.sleep(0.01)
        else:
            self.pi.set_servo_pulsewidth(config.draw_steer, deep_pwm)
            self.draw_steer_pwm = deep_pwm

    # 固定速度转向
    def turn_angular_velocity(self, is_left=1, debug=False):
        """
        固定速度转向 config.angular_velocity
        :param is_left 是否是左转  1 是左转   0 右转
        :param debug 是否输出调试信息
        :return:
        """
        while True:
            if self.angular_velocity:
                if is_left:
                    angular_velocity_error = self.angular_velocity - config.angular_velocity
                else:
                    angular_velocity_error = self.angular_velocity - (-1 * config.angular_velocity)
                left_pwm, right_pwm = self.pid_obj.pid_turn_pwm(angular_velocity_error)
            else:
                if is_left:
                    left_pwm, right_pwm = 1700, 1300
                else:
                    left_pwm, right_pwm = 1300, 1700
            if debug:
                print('self.angular_velocity', self.angular_velocity)
            self.set_pwm(left_pwm, right_pwm)
            time.sleep(0.2)

    def turn_angle(self, angle):
        """
        旋转到指定角度
        :param angle: 0 --360 逆时针为正
        :return:
        """
        angle_error = self.theta - angle
        left_pwm, right_pwm = self.pid_obj.pid_turn_pwm(angle_error)
        self.set_pwm(left_pwm, right_pwm)

    def stop(self):
        self.set_pwm(config.stop_pwm, config.stop_pwm)

    # 初始化电机
    def init_motor(self):
        self.set_pwm(config.stop_pwm, config.stop_pwm)
        time.sleep(2)
        self.set_pwm(config.stop_pwm + 200, config.stop_pwm + 200)
        time.sleep(3)
        self.set_pwm(config.stop_pwm - 200, config.stop_pwm - 200)
        time.sleep(2)
        self.set_pwm(config.stop_pwm, config.stop_pwm)
        time.sleep(config.motor_init_time)

    def set_pwm(self, set_left_pwm, set_right_pwm):
        """
        设置pwm波数值
        :param set_left_pwm:
        :param set_right_pwm:
        :return:
        """
        # 判断是否大于阈值
        if set_left_pwm >= config.max_pwm:
            set_left_pwm = config.max_pwm
        if set_left_pwm <= config.min_pwm:
            set_left_pwm = config.min_pwm
        if set_right_pwm >= config.max_pwm:
            set_right_pwm = config.max_pwm
        if set_right_pwm <= config.min_pwm:
            set_right_pwm = config.min_pwm

        # 如果有反桨叶反转电机pwm值
        if config.left_motor_cw == 1:
            set_left_pwm = config.stop_pwm - (set_left_pwm - config.stop_pwm)
        if config.right_motor_cw == 1:
            set_right_pwm = config.stop_pwm - (set_right_pwm - config.stop_pwm)
        self.target_left_pwm = int(set_left_pwm / (20000 / self.pice) / (50 / self.hz))
        self.target_right_pwm = int(set_right_pwm / (20000 / self.pice) / (50 / self.hz))

    # 循环修改pwm波值
    def loop_change_pwm(self):
        """
        一直修改输出pwm波到目标pwm波
        :return:
        """
        sleep_time = 0.01
        change_pwm_ceil = 5
        while True:
            if abs(self.left_pwm - self.target_left_pwm) != 0 or abs(self.right_pwm != self.target_right_pwm) != 0:
                if abs(self.target_left_pwm - self.left_pwm) == 0:
                    pass
                else:
                    self.left_pwm = self.left_pwm + (self.target_left_pwm - self.left_pwm) // abs(
                        self.target_left_pwm - self.left_pwm) * change_pwm_ceil
                if abs(self.target_right_pwm - self.right_pwm) == 0:
                    pass
                else:
                    self.right_pwm = self.right_pwm + (self.target_right_pwm - self.right_pwm) // abs(
                        self.target_right_pwm - self.right_pwm) * change_pwm_ceil
                self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
                self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
                if self.b_start_remote:
                    time.sleep(sleep_time / 4)
                else:
                    time.sleep(sleep_time)
            else:
                if self.b_start_remote:
                    time.sleep(sleep_time / 4)
                else:
                    time.sleep(sleep_time)

    def set_steer_engine(self, angle):
        self.pi.set_PWM_dutycycle(26, angle)

    # 记录罗盘数据
    def save_compass_data(self, theta_):
        int_time_x = int(time.time() * 10)
        self.compass_data_list.append((int_time_x, theta_))
        save_compass_data_path = os.path.join(config.save_compass_data_dir, 'compass_data.json')
        if len(self.compass_data_list) >= 1000:
            if os.path.exists(save_compass_data_path):
                temp_data_dict = save_data.get_data(save_compass_data_path)
                if temp_data_dict:
                    temp_data_list = temp_data_dict.get('compass_data')
                    if temp_data_list:
                        temp_data_list.extend(self.compass_data_list)
                    else:
                        temp_data_list = []
                        temp_data_list.extend(self.compass_data_list)
                    temp_data_dict.update({'compass_data': temp_data_list})
                    print('save_data')
                else:
                    temp_data_dict = {'compass_data': self.compass_data_list}
            else:
                temp_data_dict = {'compass_data': self.compass_data_list}
                print('save_data')
            if len(temp_data_dict.get('compass_data')) > 36000:
                temp_data_list = temp_data_dict.get('compass_data')
                del temp_data_list[:18000]
                temp_data_dict.update({'compass_data': temp_data_list})
                save_data.set_data(temp_data_dict, save_compass_data_path)
            self.compass_data_list.clear()

    # 获取舵机＋激光雷达数据距离字典
    def get_distance_dict(self):
        if config.b_laser:
            self.laser_obj = self.get_laser_obj()
        b_add = 1
        # 角度限制
        steer_max_angle = config.steer_max_angle
        min_i = 100 - int(steer_max_angle * 1000 / 900)
        max_i = 100 + int(steer_max_angle * 1000 / 900)
        i = min_i
        start_time = time.time()
        while True:
            # if b_add == 1:
            angle_pwm = 500 + i * 10
            if config.b_laser:
                self.set_steer_engine(angle_pwm)
            # else:
            #     angle_pwm = 2500 - i * 10
            #     self.set_steer_engine(angle_pwm)
            # 更新位置矩阵 连续五次检查不到将该位置置位0表示该位置没有距离信息
            laser_distance = 0
            for j in range(5):
                if config.b_laser:
                    laser_distance = self.laser_obj.read_laser()
                    print('laser_distance', laser_distance)
                if laser_distance:
                    break
                else:
                    laser_distance = 0
                    time.sleep(0.05)
            # 角度左正右负
            angle = (i - 100) * 0.9
            print(i, angle)
            self.distance_dict.update({angle: laser_distance})
            # 将21个角度归结无五个方向 0 1 2 3 4 ，右 右前 前 左前 左
            if -steer_max_angle <= angle < -steer_max_angle * 3 / 5:
                obstacle_index = 0
            elif -steer_max_angle * 3 / 5 <= angle < -steer_max_angle * 1 / 5:
                obstacle_index = 1
            elif -steer_max_angle * 1 / 5 <= angle < steer_max_angle * 1 / 5:
                obstacle_index = 2
            elif steer_max_angle * 1 / 5 <= angle < steer_max_angle * 3 / 5:
                obstacle_index = 3
            else:
                obstacle_index = 4
            if laser_distance == 0 or laser_distance > config.min_steer_distance:
                b_obstacle = 0
            else:
                b_obstacle = 1
            self.obstacle_list[obstacle_index] = b_obstacle
            if time.time() - start_time >= 5:
                print('self.distance_dict', self.distance_dict)
                print('self.obstacle_list', self.obstacle_list)
                start_time = time.time()
            i += 1 * b_add
            if i >= max_i or i <= min_i:
                b_add = -1 if b_add == 1 else 1
            time.sleep(0.02)

    # 获取毫米波雷达数据距离字典
    def get_distance_dict_millimeter(self, debug=False):
        # 角度限制
        count = 0
        max_count = 40
        average_angle_dict = {}
        average_distance_dict = {}
        while True:
            data_dict = self.millimeter_wave_obj.read_millimeter_wave()
            if data_dict:
                for obj_id in data_dict:
                    distance_row = data_dict[obj_id][0]
                    angle_row = data_dict[obj_id][1]
                    # 将该对象次的平均值作为角度值
                    if obj_id in average_angle_dict:
                        if len(average_angle_dict.get(obj_id)) >= max_count:
                            average_angle_dict.get(obj_id).pop(0)
                        average_angle_dict.get(obj_id).append(angle_row)
                    else:
                        average_angle_dict.update({obj_id: [angle_row]})
                    # 将该对象平均值作为距离值
                    if obj_id in average_distance_dict:
                        if len(average_distance_dict.get(obj_id)) >= max_count:
                            average_distance_dict.get(obj_id).pop(0)
                        average_distance_dict.get(obj_id).append(distance_row)
                    else:
                        average_distance_dict.update({obj_id: [distance_row]})
                    angle_average = int(sum(average_angle_dict.get(obj_id)) / len(average_angle_dict.get(obj_id)))
                    distance_average = sum(average_distance_dict.get(obj_id)) / len(average_distance_dict.get(obj_id))
                    # 丢弃大于视场角范围的数据
                    if abs(angle_average) >= (self.field_of_view / 2):
                        continue
                    angle_key = int(angle_average - angle_average % 2)
                    self.distance_dict.update({obj_id: [distance_average, angle_key]})
            # 没有检测到目标处理方式
            else:
                for obj_id in self.distance_dict.copy():
                    # print('average_distance_dict,average_angle_dict',average_distance_dict,average_angle_dict)
                    if obj_id in average_distance_dict and obj_id in average_angle_dict:
                        if len(average_distance_dict.get(obj_id)) >= max_count:
                            average_distance_dict.get(obj_id).pop(0)
                        average_distance_dict.get(obj_id).append(0)
                        distance_average = sum(average_distance_dict.get(obj_id)) / len(
                            average_distance_dict.get(obj_id))
                        # 连续多次不出现该id的目标时平均距离会=0小于0.5时删除该目标
                        if distance_average < 0.5:
                            self.distance_dict.pop(obj_id)
                            average_angle_dict.pop(obj_id)
                            average_distance_dict.pop(obj_id)
                            continue
                        angle_key = self.distance_dict.get(obj_id)[1]
                        self.distance_dict.update({obj_id: [distance_average, angle_key]})
                    elif obj_id in average_distance_dict and obj_id not in average_angle_dict:
                        self.distance_dict.pop(obj_id)
                        average_distance_dict.pop(obj_id)
                        continue
                    elif obj_id in average_angle_dict and obj_id not in average_distance_dict:
                        self.distance_dict.pop(obj_id)
                        average_angle_dict.pop(obj_id)
                        continue
            for obj_id in self.distance_dict:
                distance_average = self.distance_dict.get(obj_id)[0]
                angle_average = self.distance_dict.get(obj_id)[1]
                obstacle_index = angle_average // self.view_cell + 9
                if distance_average > config.min_steer_distance:
                    b_obstacle = 0
                else:
                    b_obstacle = 1
                self.obstacle_list[obstacle_index] = b_obstacle
                if distance_average > config.control_obstacle_distance:
                    b_control_obstacle = 0
                else:
                    b_control_obstacle = 1
                self.control_obstacle_list[obstacle_index] = b_control_obstacle
            if count == max_count - 1:
                self.obstacle_list = [0] * int(self.field_of_view / self.view_cell)
            if debug:
                print('data_dict', data_dict)
                print('self.distance_dict', self.distance_dict)
                print('self.obstacle_list', self.obstacle_list)
            count += 1
            count %= max_count
            time.sleep(0.001)

    @staticmethod
    def dump_energy_cal(adc):
        """
        输入ADC采集电压返回剩余电量
        电量与电压对应关系  各个阶段之内用线性函数计算  下表为实验数据
        电量     电压      6S电池     ADC采集数值
        100%----4.20V     25.2      3900
        90%-----4.06V     24.36     3755
        80%-----3.98V     23.88     3662
        70%-----3.92V     23.52     3561
        60%-----3.87V     23.22     3536
        50%-----3.82V     22.92     3535
        40%-----3.79V▲    22.74     3520
        30%-----3.77V     22.62     3432
        20%-----3.74V     22.44     3461
        0%-----3.7V       22.2      3318
        """
        adc_list = [3900, 3755, 3662, 3561, 3536, 3520, 3432, 3318]
        cap_list = [100, 90, 80, 70, 60, 40, 20, 1]
        # for index, adc_item in enumerate(adc_list):
        #     if index == 0:
        #         if adc > adc_item:
        #             return_cap = cap_list[index]
        #             break
        #     elif index == (len(adc_list) - 1):
        #         if adc < adc_item:
        #             return_cap = cap_list[index]
        #             break
        #     else:
        #         if adc_list[index + 1] < adc < adc_list[index]:
        #             return_cap = cap_list[index + 1] + (adc - adc_list[index + 1]) / (
        #                         adc_list[index] - adc_list[index + 1])
        if adc_list[0] <= adc:
            return_cap = cap_list[0]
        elif adc_list[1] <= adc < adc_list[0]:
            return_cap = cap_list[1] + (adc - adc_list[1]) * (cap_list[0] - cap_list[1]) / (
                    adc_list[0] - adc_list[1])
        elif adc_list[2] <= adc < adc_list[1]:
            return_cap = cap_list[2] + (adc - adc_list[2]) * (cap_list[1] - cap_list[2]) / (
                    adc_list[1] - adc_list[2])
        elif adc_list[3] <= adc < adc_list[2]:
            return_cap = cap_list[3] + (adc - adc_list[3]) * (cap_list[2] - cap_list[3]) / (
                    adc_list[2] - adc_list[3])
        elif adc_list[4] <= adc < adc_list[3]:
            return_cap = cap_list[4] + (adc - adc_list[4]) * (cap_list[3] - cap_list[4]) / (
                    adc_list[3] - adc_list[4])
        elif adc_list[5] <= adc < adc_list[4]:
            return_cap = cap_list[5] + (adc - adc_list[5]) * (cap_list[4] - cap_list[5]) / (
                    adc_list[4] - adc_list[5])
        elif adc_list[6] <= adc < adc_list[5]:
            return_cap = cap_list[6] + (adc - adc_list[6]) * (cap_list[5] - cap_list[6]) / (
                    adc_list[5] - adc_list[6])
        elif adc_list[7] <= adc < adc_list[6]:
            return_cap = cap_list[7] + (adc - adc_list[7]) * (cap_list[6] - cap_list[7]) / (
                    adc_list[6] - adc_list[7])
        else:
            return_cap = 1  # 小于最小电量置为1
        return int(return_cap)

    # 读取单片机数据 软串口
    def get_stc_data(self):
        """
        读取单片机数据
        :return:
        """
        while True:
            stc_data_read_dict = self.stc_obj.read_stc_data()
            if stc_data_read_dict.get('dump_energy') and len(stc_data_read_dict.get('dump_energy')) == 1:
                # 处理剩余电量数据
                self.dump_energy = PiMain.dump_energy_cal(stc_data_read_dict.get('dump_energy')[0])
                if self.last_dump_energy is None or abs(self.dump_energy - self.last_dump_energy) > 5:
                    self.logger_obj.info({'stc_data dump energy', self.dump_energy})
                    self.last_dump_energy = self.dump_energy
            elif stc_data_read_dict.get('water') and len(stc_data_read_dict.get('water')) == 5:
                stc_data_read = stc_data_read_dict.get('water')
                ec_data = stc_data_read[4]
                ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
                self.water_data_dict.update({'EC': ec_data})
                do_data = stc_data_read[1]
                do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
                self.water_data_dict.update({'DO': do_data})
                td_data = stc_data_read[2]
                td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
                self.water_data_dict.update({'TD': td_data})
                ph_data = stc_data_read[3]
                ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
                self.water_data_dict.update({'pH': ph_data})
                wt_data = stc_data_read[0]
                wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
                self.water_data_dict.update({'wt': wt_data})
            time.sleep(0.5)

    def get_sonar_data(self):
        """
        获取声呐检测深度数据
        :return:
        """
        while True:
            deep = self.sonar_obj.read_sonar()
            if deep:
                self.logger_obj.info(str(deep))
            time.sleep(1)

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        last_read_time = time.time()
        while True:
            # 水质数据 b''BBD:0,R:158,Z:36,P:0,T:16.1,V:92.08,x1:0,x2:2,x3:1,x4:0\r\n'
            # 经纬度数据 b'AA2020.2354804,N,11425.41234568896,E\r\n'
            try:
                time.sleep(0.5)
                row_com_data_read = self.com_data_obj.readline()
                # 如果数据不存在或者数据过短都跳过
                if row_com_data_read:
                    if len(str(row_com_data_read)) < 7:
                        continue
                else:
                    continue
                com_data_read = str(row_com_data_read)[2:-5]
                if time.time() - last_read_time > 3:
                    last_read_time = time.time()
                # 解析串口发送过来的数据
                if com_data_read is None:
                    continue
                # 读取数据过短跳过
                if len(com_data_read) < 5:
                    continue
                if com_data_read.startswith('AA'):
                    com_data_list = com_data_read.split(',')
                    lng, lat = round(float(com_data_list[2][:3]) +
                                     float(com_data_list[2][3:]) /
                                     60, 6), round(float(com_data_list[0][2:4]) +
                                                   float(com_data_list[0][4:]) /
                                                   60, 6)
                    self.second_lng_lat = [lng, lat]
                    if time.time() - last_read_time > 10:
                        logger.info({'second_lng_lat': self.second_lng_lat})
                elif com_data_read.startswith('BB'):
                    com_data_list = com_data_read.split(',')
                    ec_data = float(com_data_list[1].split(':')[1]) / math.pow(10, int(com_data_list[7][3:]))
                    ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
                    self.water_data_dict.update({'EC': ec_data})
                    do_data = float(com_data_list[0][2:].split(':')[1]) / math.pow(10, int(com_data_list[6][3:]))
                    do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
                    self.water_data_dict.update({'DO': do_data})
                    td_data = float(com_data_list[2].split(':')[1]) / math.pow(10, int(com_data_list[8][3:]))
                    td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
                    self.water_data_dict.update({'TD': td_data})
                    ph_data = float(com_data_list[3].split(':')[1]) / math.pow(10, int(com_data_list[9][3:]))
                    ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
                    self.water_data_dict.update({'pH': ph_data})
                    wt_data = float(com_data_list[4].split(':')[1])
                    wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
                    self.water_data_dict.update({'wt': wt_data})
                    self.dump_energy = float(com_data_list[5].split(':')[1])
                    if time.time() - last_read_time > 5:
                        last_read_time = time.time()
                        logger.info({'str com_data_read': com_data_read})
                        print({'water_data_dict': self.water_data_dict})
                        logger.info({'water_data_dict': self.water_data_dict})
            except Exception as e1:
                logger.error({'串口数据解析错误': e1})

    # 读取GY-26罗盘数据
    def get_compass_data(self, debug=False):
        if config.current_platform == config.CurrentPlatform.pi:
            # 记录上一次发送数据
            last_send_data = None
            while True:
                # 检查罗盘是否需要校准 # 开始校准
                if int(config.calibration_compass) == 1:
                    if last_send_data != 'C0':
                        info_data = self.compass_obj.read_compass(send_data='C0')
                        time.sleep(0.05)
                        self.compass_notice_info = info_data
                        last_send_data = 'C0'
                # 结束校准
                elif int(config.calibration_compass) == 2:
                    if last_send_data != 'C1':
                        self.compass_notice_info = ''
                        info_data = self.compass_obj.read_compass(send_data='C1')
                        time.sleep(0.05)
                        self.compass_notice_info = info_data
                        last_send_data = 'C1'
                        # 发送完结束校准命令后将配置改为 0
                        config.calibration_compass = 0
                        config.write_setting(b_height=True)
                else:
                    theta_ = self.compass_obj.read_compass(send_data='31')
                    try:
                        self.save_compass_data(theta_=theta_)
                    except Exception as s_e:
                        self.logger_obj.error({'save_compass_data': s_e})
                    theta_ = self.compass_filter(theta_)
                    if theta_:
                        self.theta = theta_
                if debug:
                    print('time', time.time(), self.theta, self.angular_velocity)

    # 读取维特罗盘数据
    def get_weite_compass_data(self):
        if config.current_platform == config.CurrentPlatform.pi:
            # 记录上一次发送数据
            last_send_data = None
            while True:
                # 检查罗盘是否需要校准 # 开始校准
                if int(config.calibration_compass) == 1:
                    if last_send_data != 'C0':
                        info_data = self.weite_compass_obj.read_weite_compass(send_data='C0')
                        time.sleep(0.05)
                        self.compass_notice_info = info_data
                        last_send_data = 'C0'
                # 结束校准
                elif int(config.calibration_compass) == 2:
                    if last_send_data != 'C1':
                        self.compass_notice_info = ''
                        info_data = self.compass_obj.read_compass(send_data='C1')
                        time.sleep(0.05)
                        self.compass_notice_info = info_data
                        last_send_data = 'C1'
                        # 发送完结束校准命令后将配置改为 0
                        config.calibration_compass = 0
                        config.write_setting(b_height=True)
                else:
                    theta_ = self.weite_compass_obj.read_weite_compass(send_data='31')
                    theta_ = self.compass_filter(theta_)
                    if theta_:
                        self.theta = theta_

    # 读取gps数据
    def get_gps_data(self):
        if config.current_platform == config.CurrentPlatform.pi:
            while True:
                gps_data_read = self.gps_obj.read_gps()
                if gps_data_read:
                    self.lng_lat = gps_data_read[0:2]
                    self.lng_lat_error = gps_data_read[2]

    # 读取lora遥控器数据
    def get_remote_control_data(self, debug=False):
        """
        读取lora遥控器数据
        :param debug:打印数据
        :return:
        """
        while True:
            return_remote_data = self.remote_control_obj.read_remote_control(debug=debug)
            if return_remote_data and len(return_remote_data) >= 13:
                if debug:
                    print("######################", time.time(), 'return_remote_data', return_remote_data)
                self.remote_control_data = return_remote_data
                self.lora_control_receive_time = time.time()
                # 判断遥控器使能
                if int(self.remote_control_data[12]) == 1:
                    self.b_start_remote = 1
                else:
                    self.b_start_remote = 0
                if self.b_start_remote:
                    # 判断开始抽水  结束抽水
                    if int(self.remote_control_data[5]) == 10:
                        self.remote_draw_status = 1
                    elif int(self.remote_control_data[5]) == 1:
                        self.remote_draw_status = 0
                    elif int(self.remote_control_data[5]) == 0:
                        self.remote_draw_status = 0
                    # 判断开始排水  结束排水
                    if int(self.remote_control_data[7]) == 10:
                        self.remote_drain_status = 1
                    elif int(self.remote_control_data[7]) == 1:
                        self.remote_drain_status = 0
                    elif int(self.remote_control_data[7]) == 0:
                        self.remote_drain_status = 0
                    # 判断收起舵机  展开舵机
                    if int(self.remote_control_data[10]) == 1:
                        self.remote_draw_steer = config.min_deep_steer_pwm
                    else:
                        self.remote_draw_steer = config.max_deep_steer_pwm
                    # 判断打开舷灯  关闭舷灯
                    if int(self.remote_control_data[6]) == 10:
                        self.remote_side_light_status = 1
                    elif int(self.remote_control_data[6]) == 1:
                        self.remote_side_light_status = 0
                    else:
                        self.remote_side_light_status = 2
                    # 判断打开大灯  关闭大灯
                    if int(self.remote_control_data[8]) == 10:
                        self.remote_head_light_status = 1
                    elif int(self.remote_control_data[8]) == 1:
                        self.remote_head_light_status = 0
                    else:
                        self.remote_head_light_status = 2
            else:
                if self.lora_control_receive_time and time.time() - self.lora_control_receive_time > 20:
                    self.b_start_remote = 0
            time.sleep(0.01)


if __name__ == '__main__':
    pi_main_obj = PiMain()
    from drivers import com_data

    if os.path.exists(config.stc_port):
        com_data_obj = com_data.ComData(
            config.stc_port,
            config.stc_baud,
            timeout=config.stc2pi_timeout,
            logger=logger)
    loop_change_pwm_thread = threading.Thread(target=pi_main_obj.loop_change_pwm)
    loop_change_pwm_thread.start()

    while True:
        try:
            # 按键后需要按回车才能生效
            print('w,a,s,d 为前后左右，q为停止\n'
                  'r 初始化电机\n'
                  'R lora遥控器数据读取，R 读一次  R1一直读\n'
                  't 控制抽水舵机和抽水\n'
                  'f  读取单片机数据\n'
                  'g  获取gps数据\n'
                  'h  单次获取罗盘数据  h1 持续读取罗盘数据求角速度\n'
                  'H  读取维特罗盘数据  校准 s  开始  e 结束  a 设置自动回传  i 初始化 其他为读取\n'
                  'z 退出\n'
                  'x  2.4g遥控器输入\n'
                  'c  声呐数据\n'
                  'b  毫米波数据\n'
                  'n  距离字典\n'
                  'A0 A1  关闭和开启水泵\n'
                  'B0 B1  关闭和开启舷灯\n'
                  'C0 C1  关闭和开启前面大灯\n'
                  'D0 D1  关闭和开启声光报警器\n'
                  'E0 E1 E2 E3 E4  状态灯\n'
                  )
            key_input = input('please input:')
            # 前 后 左 右 停止  右侧电机是反桨叶 左侧电机是正桨叶
            gear = None
            if key_input.startswith('w') or key_input.startswith(
                    'a') or key_input.startswith('s') or key_input.startswith('d'):
                try:
                    print(config.left_pwm_pin, )
                    if len(key_input) > 1:
                        gear = int(key_input[1])
                except Exception as e:
                    print({'error': e})
                    gear = None
            if key_input.startswith('w'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    print(1600 + 100 * gear, 1600 + 100 * gear)
                    pi_main_obj.forward(
                        1600 + 100 * gear, 1600 + 100 * gear)
                else:
                    pi_main_obj.forward()
            elif key_input.startswith('a'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.left(
                        1400 - 100 * gear, 1600 + 100 * gear)
                else:
                    pi_main_obj.left()
            elif key_input.startswith('s'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    print(1400 - 100 * gear, 1400 - 100 * gear)
                    pi_main_obj.backword(
                        1400 - 100 * gear, 1400 - 100 * gear)
                else:
                    pi_main_obj.backword()
            elif key_input.startswith('d'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.right(
                        1600 + 100 * gear, 1400 - 100 * gear)
                else:
                    pi_main_obj.right()
            elif key_input == 'q':
                pi_main_obj.stop()
            elif key_input.startswith('r'):
                pi_main_obj.init_motor()
            elif key_input.startswith('R'):
                if len(key_input) > 1 and key_input[1] == '1':
                    pi_main_obj.get_remote_control_data(debug=True)
                    pi_main_obj.check_remote_pwm()
                else:
                    pi_main_obj.remote_control_obj.read_remote_control(debug=True)
            elif key_input.startswith('t'):
                if len(key_input) > 1:
                    try:
                        pwm_deep = int(key_input[1:])
                        print('pwm_deep', pwm_deep)
                        pi_main_obj.set_draw_deep(deep_pwm=pwm_deep)
                    except Exception as e:
                        print('pwm_deep', e)
                else:
                    pi_main_obj.set_draw_deep(deep_pwm=config.min_deep_steer_pwm)  # 旋转舵机
                    time.sleep(3)
                    pi_main_obj.stc_obj.pin_stc_write('A1Z', debug=True)
                    time.sleep(5)
                    pi_main_obj.stc_obj.pin_stc_write('A0Z', debug=True)
                    pi_main_obj.set_draw_deep(deep_pwm=config.max_deep_steer_pwm)
                    time.sleep(3)
            # 获取读取单片机数据
            elif key_input.startswith('f'):
                stc_data = pi_main_obj.stc_obj.read_stc_data(debug=True)
                print('stc_data', stc_data)
            elif key_input.startswith('g'):
                gps_data = pi_main_obj.gps_obj.read_gps(debug=True)
                print('gps_data', gps_data)
            elif key_input.startswith('h'):
                if len(key_input) == 1:
                    key_input = input('input:  C0  开始  C1 结束 其他为读取 >')
                    if key_input == 'C0':
                        theta_c = pi_main_obj.compass_obj.read_compass(send_data='C0', debug=True)
                    elif key_input == 'C1':
                        theta_c = pi_main_obj.compass_obj.read_compass(send_data='C1', debug=True)
                    else:
                        theta_c = pi_main_obj.compass_obj.read_compass(debug=True)
                    print('theta_c', theta_c)
                elif len(key_input) > 1 and int(key_input[1]) == 1:
                    pi_main_obj.get_compass_data(debug=True)
            elif key_input.startswith('H'):
                key_input = input('input:  校准 s  开始  e 结束  a 设置自动回传  i 初始化 其他为读取 >')
                if key_input == 's':
                    theta = pi_main_obj.weite_compass_obj.read_weite_compass(send_data='41542B43414C493D310D0A',
                                                                             debug=True)
                elif key_input == 'e':
                    theta = pi_main_obj.weite_compass_obj.read_weite_compass(send_data='41542B43414C493D300D0A',
                                                                             debug=True)
                elif key_input == 'a':
                    theta = pi_main_obj.weite_compass_obj.read_weite_compass(send_data='41542B50524154453D3130300D0A',
                                                                             debug=True)
                elif key_input == 'i':
                    theta = pi_main_obj.weite_compass_obj.read_weite_compass(send_data='41542B494E49540D0A', debug=True)
                else:
                    theta = pi_main_obj.weite_compass_obj.read_weite_compass(send_data='41542B50524154453D300D0A',
                                                                             debug=True)
                print('theta', theta)
            elif key_input.startswith('x'):
                while True:
                    try:
                        time.sleep(0.1)
                        print('b_start_remote', pi_main_obj.b_start_remote)
                        print('channel_row_input_pwm', pi_main_obj.channel_row_input_pwm)
                        print('channel_col_input_pwm', pi_main_obj.channel_col_input_pwm)
                        if pi_main_obj.b_start_remote:
                            pi_main_obj.set_pwm(set_left_pwm=pi_main_obj.channel_row_input_pwm,
                                                set_right_pwm=pi_main_obj.channel_col_input_pwm)
                    except KeyboardInterrupt:
                        break
            # 读取毫米波雷达
            elif key_input.startswith('b'):
                millimeter_wave_data = pi_main_obj.millimeter_wave_obj.read_millimeter_wave(debug=True)
                print('millimeter_wave', millimeter_wave_data)
            elif key_input.startswith('n'):
                pi_main_obj.get_distance_dict_millimeter(debug=True)
            elif key_input.startswith('m'):
                pi_main_obj.init_motor()
            elif key_input[0] in ['A', 'B', 'C', 'D', 'E']:
                print('len(key_input)', len(key_input))
                if len(key_input) == 2 and key_input[1] in ['0', '1', '2', '3', '4']:
                    send_data = key_input + 'Z'
                    print('send_data', send_data)
                    if config.b_com_stc:
                        pi_main_obj.com_data_obj.send_data(send_data)
                        row_com_data_read = pi_main_obj.com_data_obj.readline()
                        print('row_com_data_read', row_com_data_read)
                    elif config.b_pin_stc:
                        pi_main_obj.stc_obj.send_stc_data(send_data)

            # TODO
            # 角度控制
            # 到达目标点控制
            # 简单走矩形区域
            # 退出
            elif key_input.startswith('Z'):
                pi_main_obj.turn_angular_velocity(debug=True)
        except KeyboardInterrupt:
            break
        # except Exception as e:
        #     print({'error': e})
        #     continue
