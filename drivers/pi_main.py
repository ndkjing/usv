import os
import sys
import threading
import pigpio
import time
import copy

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from utils import log
from drivers import pi_softuart
import config
from moveControl.pathTrack import simple_pid

logger = log.LogHandler('pi_log')


class PiMain:
    def __init__(self):
        import config
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

        self.save = [1, 161, 150, 157, 152, 142, 101]
        self.set = 1
        self.tick_0 = [None, None, None, None, None, None, None]
        self.tick_1 = [None, None, None, None, None, None, None]
        self.temp_read = [[150 for col in range(21)] for row in range(7)]
        self.count = [1, 0, 0, 0, 0, 0, 0]

        self.channel_row_input_pwm = 0
        self.channel_col_input_pwm = 0
        # 当前是否是遥控器控制  2.4g遥控器和lora遥控器都用这个标志位
        self.b_start_remote = 0
        if config.b_use_remote_control:
            self.cb1 = self.pi.callback(config.channel_1_pin, pigpio.EITHER_EDGE, self.mycallback)
            self.cb2 = self.pi.callback(config.channel_3_pin, pigpio.EITHER_EDGE, self.mycallback)
            self.cb3 = self.pi.callback(config.channel_remote_pin, pigpio.EITHER_EDGE, self.mycallback)

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
        self.draw_steer_pwm = 500
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
        # 遥控器控制开始抽水
        self.start_draw = 0
        # 遥控器控制数据
        self.remote_control_data = []

        # 求角速度     逆时针方向角速度为正值
        self.theta_list = []  # (时间，角度)
        self.angular_velocity = None
        self.last_angular_velocity = None
        self.pid_obj = simple_pid.SimplePid()
        # 将舵机归位
        self.set_draw_deep(deep_pwm=500)

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
                                      baud=config.lora_baud, time_out=0.2)

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
                                      baud=config.stc_baud, time_out=0.1)

    def get_stc_data(self):
        """
        读取单片机数据
        :return:
        """
        data = self.stc_obj.read_stc_data()
        return data

    def get_sonar_data(self):
        """
        获取声呐检测深度数据
        :return:
        """
        deep = self.sonar_obj.read_sonar()
        return deep

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
                    theta_ = self.compass_filter(theta_)
                    if theta_:
                        self.theta = theta_
                if debug:
                    print('time', time.time(), self.theta, self.angular_velocity)

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

    def get_gps_data(self):
        if config.current_platform == config.CurrentPlatform.pi:
            while True:
                gps_data_read = self.gps_obj.read_gps()
                if gps_data_read:
                    self.lng_lat = gps_data_read[0:2]
                    self.lng_lat_error = gps_data_read[2]

    def check_lora_remote_pwm(self):
        """
        遥控器输入
        :return:
        """
        remote_forward_pwm = int((99 - self.remote_control_data[2]) * 10 + 1000)
        remote_steer_pwm = int(self.remote_control_data[3] * 10 + 1000)
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

    def check_remote_pwm(self):
        """
        遥控器输入
        :return:
        """
        remote_forward_pwm = copy.deepcopy(int(self.channel_col_input_pwm))
        remote_steer_pwm = copy.deepcopy(int(self.channel_row_input_pwm))
        # print('remote', remote_forward_pwm, remote_steer_pwm)
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

    def mycallback(self, gpio, level, tick):
        if level == 0:
            if int(gpio) == int(config.channel_1_pin):
                self.tick_0[0] = tick
                if self.tick_1[0] is not None:
                    diff = pigpio.tickDiff(self.tick_1[0], tick)
                    self.channel_row_input_pwm = int(diff)
                    # print('self.channel_row_input_pwm ', self.channel_row_input_pwm ,diff)
                    self.temp_read[0][self.count[0]] = diff
                    self.save[0] = int(self.temp_read[0][self.count[0]])
            if int(gpio) == int(config.channel_3_pin):
                self.tick_0[1] = tick
                if self.tick_1[1] is not None:
                    diff = pigpio.tickDiff(self.tick_1[1], tick)
                    self.channel_col_input_pwm = int(diff)
                    self.temp_read[1][self.count[1]] = diff
                    self.save[1] = int(self.temp_read[1][self.count[1]])
            if int(gpio) == 11:
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
            if gpio == 11:
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

    def turn_angular_velocity(self, is_left=1,debug=False):
        """
        :param is_left 是否是左转  1 是左转   0 右转
        固定速度转向 config.angular_velocity
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
                print('self.angular_velocity',self.angular_velocity)
            self.set_pwm(left_pwm, right_pwm)

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

    # 死循环函数放在线程中执行
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
                # print('self.left_pwm,self.target_left_pwm', self.left_pwm, self.target_left_pwm)
                self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
                self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
                time.sleep(sleep_time)
            else:
                time.sleep(sleep_time)
        # 不支持输出获取pwm状态，以后再调试
        # print('left_pwm:',self.left_pwm,self.pi.get_PWM_dutycycle(config.left_pwm_pin),'right_pwm:',self.right_pwm,self.pi.get_PWM_dutycycle(config.right_pwm_pin))

    def set_steer_engine(self, angle):
        self.pi.set_PWM_dutycycle(26, angle)

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

    def set_gpio(self,
                 control_left_motor=0,
                 control_right_motor=0,
                 control_headlight=0,
                 control_alarm_light=0,
                 control_left_sidelight=0,
                 control_right_sidelight=0
                 ):
        """
        :param control_left_motor: 1 控制该位置引脚输出高电平 0控制该位置引脚输出低电平下同
        :param control_right_motor:
        :param control_headlight:
        :param control_alarm_light:
        :param control_left_sidelight:
        :param control_right_sidelight:
        :return:
        """
        if control_left_motor:
            self.pi.write(config.draw_left_gpio_pin, pigpio.LOW)
            self.left_motor_output = 0
        else:
            self.pi.write(config.draw_left_gpio_pin, pigpio.HIGH)
            self.left_motor_output = 1
        if control_right_motor:
            self.pi.write(config.draw_right_gpio_pin, pigpio.LOW)
            self.right_motor_output = 0
        else:
            self.pi.write(config.draw_right_gpio_pin, pigpio.HIGH)
            self.right_motor_output = 1
        if control_headlight:
            self.pi.write(config.headlight_gpio_pin, pigpio.LOW)
            self.headlight_output = 0
        else:
            self.pi.write(config.headlight_gpio_pin, pigpio.HIGH)
            self.headlight_output = 1
        if control_alarm_light:
            self.pi.write(config.audio_light_alarm_gpio_pin, pigpio.LOW)
            self.alarm_light_output = 0
        else:
            self.pi.write(config.audio_light_alarm_gpio_pin, pigpio.HIGH)
            self.alarm_light_output = 1
        if control_left_sidelight:
            self.pi.write(config.side_left_gpio_pin, pigpio.LOW)
            self.left_sidelight_output = 0
        else:
            self.pi.write(config.side_left_gpio_pin, pigpio.HIGH)
            self.left_sidelight_output = 1
        if control_right_sidelight:
            self.pi.write(config.side_right_gpio_pin, pigpio.LOW)
            self.right_sidelight_output = 0
        else:
            self.pi.write(config.side_right_gpio_pin, pigpio.HIGH)
            self.right_sidelight_output = 1

    def set_ptz_camera(self, pan_angle_pwm_=1500, tilt_angle_pwm_=1500):
        """
        设置云台舵机角度
        :param pan_angle_pwm_: 500-2500 设置水平角度
        :param tilt_angle_pwm_: 500-2500  设置垂直角度
        :return:
        """
        self.pi.set_servo_pulsewidth(config.pin_pan, pan_angle_pwm_)
        self.pi.set_servo_pulsewidth(config.pin_tilt, tilt_angle_pwm_)
        self.pan_angle_pwm = pan_angle_pwm_
        self.tilt_angle_pwm = tilt_angle_pwm_

    def set_draw_deep(self, deep_pwm=500):
        """
        设置抽水泵深度
        :param deep_pwm:
        :return:
        """
        self.pi.set_servo_pulsewidth(config.pin_tilt, deep_pwm)
        self.draw_steer_pwm = deep_pwm

    def get_remote_control_data(self, debug=False):
        while True:
            self.remote_control_data = self.remote_control_obj.read_remote_control(debug=debug)
            if self.remote_control_data and len(self.remote_control_data) == 14:
                if self.remote_control_data[-2] == 1:
                    self.b_start_remote = 1
            time.sleep(0.01)

    """
        # 在线程中读取 gps
    def get_gps_data(self):
        last_read_time = None
        while True:
            try:
                data = self.gps_obj.readline()
                str_data = data.decode('ascii')
                gps_dict = resolve_gps_data.resolve_gps(str_data)
                lng = gps_dict.get('lng')
                lat = gps_dict.get('lat')
                lng_lat_error = gps_dict.get('lng_lat_error')
                if lng and lat:
                    if lng >= 1 and lat >= 1:
                        # 替换上一次的值
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                        self.lng_lat = [lng, lat]
                        self.lng_lat_error = lng_lat_error
                        if not last_read_time:
                            last_read_time = time.time()
                        if self.lng_lat_error and self.lng_lat_error < 2.5:
                            if self.last_lng_lat:
                                # 计算当前行驶里程
                                speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
                                                                                          self.last_lng_lat[1],
                                                                                          self.lng_lat[0],
                                                                                          self.lng_lat[1])
                                self.run_distance += speed_distance
                                # 计算速度
                                self.speed = round(speed_distance / (time.time() - last_read_time), 1)
                                last_read_time = time.time()
            except Exception as e:
                self.logger.error({'error': e})
                time.sleep(0.5)

    # 在线程中读取罗盘
    def get_compass_data(self):
        # 累计50次出错则记为None
        count = 50
        # 记录上一次发送数据
        last_send_data = None
        while True:
            try:
                # 检查罗盘是否需要校准 # 开始校准
                if int(config.calibration_compass) == 1:
                    if last_send_data != 'C0':
                        self.compass_obj.send_data('C0', b_hex=True)
                        time.sleep(0.05)
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        last_send_data = 'C0'
                # 结束校准
                elif int(config.calibration_compass) == 2:
                    if last_send_data != 'C1':
                        self.compass_notice_info = ''
                        self.compass_obj.send_data('C1', b_hex=True)
                        time.sleep(0.05)
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0) > 3:
                            self.compass_notice_info += str_data0
                        last_send_data = 'C1'
                        # 发送完结束校准命令后将配置改为 0
                        config.calibration_compass = 0
                        config.write_setting(b_height=True)
                # 隐藏修改 设置为10 改为启用gps计算角度  设置为9 改为启用二号罗盘
                elif int(config.calibration_compass) in [9, 10]:
                    self.theta = None
                else:
                    self.compass_obj.send_data('31', b_hex=True)
                    time.sleep(0.05)
                    data0 = self.compass_obj.readline()
                    # 角度
                    str_data0 = data0.decode('ascii')[:-3]
                    if len(str_data0) < 1:
                        continue
                    float_data0 = float(str_data0)
                    self.theta = 360 - float_data0
                    time.sleep(config.compass_timeout / 4)
                    count = 50
                    last_send_data = None
                    # self.compass_notice_info = ''
            except Exception as e:
                if count > 0:
                    count = count - 1
                else:
                    self.logger.error({'error': e})
                    count = 50
                time.sleep(1)
    """


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
                  't 控制抽水舵机和抽水\n'
                  'y,u,i,o,p 摄像头舵机 y回中 u右  i左  o上   p下\n'
                  'f  读取单片机数据\n'
                  'g  获取gps数据\n'
                  'h  单次获取罗盘数据  h1 持续读取罗盘数据求角速度\n'
                  'H  读取维特罗盘数据  校准 s  开始  e 结束  a 设置自动回传  i 初始化 其他为读取\n'
                  'j  声光报警器  j1 开 j0关\n '
                  'k  左舷灯 k1 开 k0关\n '
                  'l  右舷灯  l1 开 l0关\n '
                  'v  大灯控制  v1 开 v0关\n '
                  'z 退出\n'
                  'x  接受遥控器输入\n'
                  'c  声呐数据\n'
                  'b  毫米波数据\n'
                  'n  距离字典\n'
                  'A0 A1  关闭和开启水泵\n'
                  'B0 B1  关闭和开启舷灯\n'
                  'C0 C1  关闭和开启前面大灯\n'
                  'D0 D1  关闭和开启声光报警器\n'
                  'E0 E1 E2 E3 E4  状态灯\n'
                  'R 遥控器\n'
                  )
            key_input = input('please input:')
            # 前 后 左 右 停止  右侧电机是反桨叶 左侧电机是正桨叶
            gear = None
            if key_input.startswith('w') or key_input.startswith(
                    'a') or key_input.startswith('s') or key_input.startswith('d'):
                try:
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
            elif key_input.startswith('t'):
                pi_main_obj.set_draw_deep(deep_pwm=2500)  # 旋转舵机
                time.sleep(3)
                pi_main_obj.stc_obj.pin_stc_write('A1Z', debug=True)
                time.sleep(5)
                pi_main_obj.stc_obj.pin_stc_write('A0Z', debug=True)
                pi_main_obj.set_draw_deep(deep_pwm=500)
                time.sleep(3)
            # 设置云台相机角度
            elif key_input.startswith('y'):
                pi_main_obj.set_ptz_camera(1500, 1500)
            elif key_input.startswith('u'):
                pan_angle_pwm = pi_main_obj.pan_angle_pwm + 100
                if pan_angle_pwm > 2500:
                    pan_angle_pwm = 2500
                pi_main_obj.set_ptz_camera(pan_angle_pwm, pi_main_obj.tilt_angle_pwm)
            elif key_input.startswith('i'):
                pan_angle_pwm = pi_main_obj.pan_angle_pwm - 100
                if pan_angle_pwm < 500:
                    pan_angle_pwm = 500
                pi_main_obj.set_ptz_camera(pan_angle_pwm, pi_main_obj.tilt_angle_pwm)
            elif key_input.startswith('o'):
                tilt_angle_pwm = pi_main_obj.tilt_angle_pwm + 100
                if tilt_angle_pwm > 2500:
                    tilt_angle_pwm = 2500
                pi_main_obj.set_ptz_camera(pi_main_obj.pan_angle_pwm, tilt_angle_pwm)
            elif key_input.startswith('p'):
                tilt_angle_pwm = pi_main_obj.tilt_angle_pwm - 100
                if tilt_angle_pwm < 500:
                    tilt_angle_pwm = 500
                pi_main_obj.set_ptz_camera(pi_main_obj.pan_angle_pwm, tilt_angle_pwm)
            # 获取读取单片机数据
            elif key_input.startswith('f'):
                laser_distance = pi_main_obj.stc_obj.pin_stc_read(debug=True)
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
            elif key_input.startswith('g'):
                gps_data = pi_main_obj.gps_obj.read_gps(debug=True)
                print('gps_data', gps_data)
            # 控制声光报警器
            elif key_input.startswith('j'):
                if key_input.endswith('1'):
                    pi_main_obj.set_gpio(control_alarm_light=1)
                else:
                    pi_main_obj.set_gpio(control_alarm_light=0)
            # 控制左舷灯
            elif key_input.startswith('k'):
                if key_input.endswith('1'):
                    pi_main_obj.set_gpio(control_left_sidelight=1)
                else:
                    pi_main_obj.set_gpio(control_left_sidelight=0)
            # 控制右舷灯
            elif key_input.startswith('l'):
                if key_input.endswith('1'):
                    pi_main_obj.set_gpio(control_right_sidelight=1)
                else:
                    pi_main_obj.set_gpio(control_right_sidelight=0)
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
                        com_data_obj.send_data(send_data)
                        row_com_data_read = com_data_obj.readline()
                        print('row_com_data_read', row_com_data_read)
                    elif config.b_pin_stc:
                        pi_main_obj.stc_obj.write_data(send_data, debug=True)
            elif key_input.startswith('R'):
                if len(key_input) > 1 and key_input[1] == '1':
                    pi_main_obj.get_remote_control_data(debug=True)
                else:
                    pi_main_obj.remote_control_obj.read_remote_control(debug=True)
            # TODO
            # 返航
            # 角度控制
            # 到达目标点控制
            # 简单走矩形区域
            # 退出
            elif key_input.startswith('Z'):
                pi_main_obj.turn_angular_velocity(debug=True)
            elif key_input.startswith('z'):
                break
        except KeyboardInterrupt:
            break
        # except Exception as e:
        #     print({'error': e})
        #     continue
