import os
import sys
import threading

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from utils import log
from drivers import pi_softuart
import config
import pigpio
import time

logger = log.LogHandler('pi_log')


class PiMain:
    def __init__(self):
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
        self.pi.set_PWM_frequency(config.left_pwm_pin, self.hz)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_frequency(config.right_pwm_pin, self.hz)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_range(config.left_pwm_pin, self.pice)
        self.pi.set_PWM_range(config.right_pwm_pin, self.pice)
        self.init_motor()
        # 设置舵机输出
        self.pi.set_PWM_frequency(config.steer_engine_pin, self.hz)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_range(config.steer_engine_pin, self.pice)

        self.save = [1, 161, 150, 157, 152, 142, 101]
        self.set = 1
        self.tick_0 = [None, None, None, None, None, None, None]
        self.tick_1 = [None, None, None, None, None, None, None]
        self.temp_read = [[150 for col in range(21)] for row in range(7)]
        self.count = [1, 0, 0, 0, 0, 0, 0]

        self.channel_row_input_pwm = 0
        self.channel_col_input_pwm = 0
        # 当前是否是遥控器控制
        self.b_start_remote = 0
        self.cb1 = self.pi.callback(config.channel_1_pin, pigpio.EITHER_EDGE, self.mycallback)
        self.cb2 = self.pi.callback(config.channel_3_pin, pigpio.EITHER_EDGE, self.mycallback)
        self.cb3 = self.pi.callback(config.start_remote_pin, pigpio.EITHER_EDGE, self.mycallback)
        if config.b_use_ultrasonic and config.current_platform == config.CurrentPlatform.pi:
            self.left_ultrasonic_obj = self.get_left_ultrasonic_obj()
            self.right_ultrasonic_obj = self.get_right_ultrasonic_obj()
        if config.current_platform == config.CurrentPlatform.pi:
            self.gps_obj = self.get_gps_obj()
            self.compass_obj = self.get_compass_obj()
        if config.b_laser and config.current_platform == config.CurrentPlatform.pi:
            self.laser_obj = self.get_laser_obj()
        # 左右侧超声波距离，没有返回None  -1 表示距离过近
        self.left_distance = None
        self.right_distance = None
        # 上次左侧距离 上次右侧距离
        self.last_left_distance = None
        self.last_right_distance = None
        # GPS
        self.lng_lat = None
        # GPS 误差
        self.lng_lat_error = None
        # 罗盘角度
        self.theta = None
        self.last_theta = None
        # 罗盘提示消息
        self.compass_notice_info = ''
        # 距离矩阵
        self.distance_dict = {}
        self.obstacle_list = [0, 0, 0, 0, 0]
        # 设置为GPIO输出模式 输出高低电平
        self.pi.set_mode(config.gpio_output_1, pigpio.OUTPUT)
        # 云台舵机角度
        self.pan_angle_pwm = 1500
        self.tilt_angle = 1500
        # 记录继电器输出电平 1 高电平 0 低电平
        self.left_motor_output = 0
        self.right_motor_output = 0
        self.alarm_light_output = 0
        self.left_sidelight_output = 0
        self.right_sidelight_output = 0

    def get_left_ultrasonic_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.left_rx, tx_pin=config.left_tx,
                                      baud=config.ultrasonic_baud)

    def get_right_ultrasonic_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.right_rx, tx_pin=config.right_tx,
                                      baud=config.ultrasonic_baud)

    def get_compass_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_compass_rx, tx_pin=config.pin_compass_tx,
                                      baud=config.pin_compass_baud)

    def get_gps_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_gps_rx, tx_pin=config.pin_gps_tx,
                                      baud=config.pin_gps_baud)

    def get_laser_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.laser_rx, tx_pin=config.laser_tx,
                                      baud=config.laser_baud, time_out=0.01)

    # 对距离进行滤波处理
    def distance_filter(self, distance, left=1):
        if left:
            if distance:
                if not self.last_left_distance:
                    self.last_left_distance = distance
                    return distance
                else:
                    if abs(distance - self.last_left_distance) > 1:
                        return self.last_left_distance
                    else:
                        self.last_left_distance = distance
                        return distance
            else:
                return self.last_left_distance
        else:
            if distance:
                if not self.last_right_distance:
                    self.last_right_distance = distance
                    return distance
                else:
                    if abs(distance - self.last_right_distance) > 1:
                        return self.last_right_distance
                    else:
                        self.last_right_distance = distance
                        return distance
            else:
                return self.last_right_distance

    # 罗盘角度滤波
    def compass_filter(self, theta):
        if theta:
            if not self.last_theta:
                self.last_theta = theta
                return theta
            else:
                # if abs(theta-self.last_theta)>180:
                #     return self.last_theta
                # else:
                #     self.last_theta = theta
                #     return theta
                self.last_theta = theta
                return theta
        else:
            return self.last_theta

    # 在线程中读取超声波
    def get_left_distance(self):
        if config.b_use_ultrasonic and config.current_platform == config.CurrentPlatform.pi:
            while True:
                l_distance = self.left_ultrasonic_obj.read_ultrasonic()
                # 执行滤波
                l_distance = self.distance_filter(l_distance)
                if l_distance:
                    self.left_distance = l_distance

    def get_right_distance(self):
        if config.b_use_ultrasonic and config.current_platform == config.CurrentPlatform.pi:
            while True:
                r_distance = self.right_ultrasonic_obj.read_ultrasonic()
                r_distance = self.distance_filter(r_distance, left=0)
                if r_distance:
                    self.right_distance = r_distance

    def get_compass_data(self):
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

    def get_gps_data(self):
        if config.current_platform == config.CurrentPlatform.pi:
            while True:
                gps_data = self.gps_obj.read_gps()
                if gps_data:
                    self.lng_lat = gps_data[0:2]
                    self.lng_lat_error = gps_data[2]

    def remote_control(self):
        """
        遥控器输入
        :return:
        """
        while True:
            try:
                remote_forward_pwm = int(self.channel_col_input_pwm)
                remote_steer_pwm = int(self.channel_row_input_pwm)
                # print('remote', remote_forward_pwm, remote_steer_pwm)
                # 防止抖动
                if remote_forward_pwm < 1550 and remote_forward_pwm > 1450:
                    remote_forward_pwm = config.stop_pwm
                # 防止过大值
                elif remote_forward_pwm >= 1900:
                    remote_forward_pwm = 1900
                # 防止初始读取到0电机会转动， 设置为1500
                elif remote_forward_pwm < 1000:
                    remote_forward_pwm = config.stop_pwm
                # 防止过小值
                elif remote_forward_pwm <= 1100 and remote_forward_pwm >= 1000:
                    remote_forward_pwm = 1100
                # 防止抖动
                if remote_steer_pwm < 1550 and remote_steer_pwm > 1450:
                    remote_steer_pwm = config.stop_pwm
                # 防止过大值
                elif remote_steer_pwm >= 1900:
                    remote_steer_pwm = 1900
                # 防止初始读取到0电机会转动， 设置为1500
                elif remote_steer_pwm < 1000:
                    remote_steer_pwm = config.stop_pwm
                # 防止过小值
                elif remote_steer_pwm <= 1100 and remote_steer_pwm >= 1000:
                    remote_steer_pwm = 1100
                # print('remote_forward_pwm,remote_steer_pwm',remote_forward_pwm,remote_steer_pwm)
                remote_left_pwm = 1500 + (remote_forward_pwm - 1500) + (remote_steer_pwm - 1500)
                remote_right_pwm = 1500 + (remote_forward_pwm - 1500) - (remote_steer_pwm - 1500)
                # self.set_pwm(remote_left_pwm, remote_right_pwm)
                time.sleep(0.1)
            except Exception as e:
                logger.error({'error': e})

    def mycallback(self, gpio, level, tick):
        if level == 0:
            if int(gpio) == int(config.channel_1_pin):
                self.tick_0[0] = tick
                if self.tick_1[0] is not None:
                    diff = pigpio.tickDiff(self.tick_1[0], tick)
                    self.channel_row_input_pwm = int(diff)
                    self.temp_read[0][self.count[0]] = diff
                    self.save[0] = int(self.temp_read[0][self.count[0]])
            if int(gpio) == int(config.channel_3_pin):
                self.tick_0[1] = tick
                if self.tick_1[1] is not None:
                    diff = pigpio.tickDiff(self.tick_1[1], tick)
                    self.channel_col_input_pwm = int(diff)
                    self.temp_read[1][self.count[1]] = diff
                    self.save[1] = int(self.temp_read[1][self.count[1]])
            if int(gpio) == int(config.start_remote_pin):
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
            if gpio == int(config.start_remote_pin):
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

    def stop(self):
        self.set_pwm(config.stop_pwm, config.stop_pwm)

    def init_motor(self):
        self.set_pwm(config.stop_pwm, config.stop_pwm)
        time.sleep(config.motor_init_time)

    def set_pwm(self, set_left_pwm, set_right_pwm, pwm_timeout=None):
        """
        设置pwm波数值
        :param pwm_timeout:
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
        sleep_time = 0.02
        delta_time = 0.0001 / 500.0
        # start_pwm_time = time.time()
        while True:
            if abs(self.left_pwm - self.target_left_pwm) != 0 or abs(self.right_pwm != self.target_right_pwm) != 0:
                if abs(self.target_left_pwm - self.left_pwm) == 0:
                    pass
                else:
                    self.left_pwm = self.left_pwm + (self.target_left_pwm - self.left_pwm) // abs(
                        self.target_left_pwm - self.left_pwm) * 5
                if abs(self.target_right_pwm - self.right_pwm) == 0:
                    pass
                else:
                    self.right_pwm = self.right_pwm + (self.target_right_pwm - self.right_pwm) // abs(
                        self.target_right_pwm - self.right_pwm) * 5
                print('self.left_pwm,self.target_left_pwm', self.left_pwm, self.target_left_pwm)
                print('config.right_pwm_pin', config.left_pwm_pin, config.right_pwm_pin)
                self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
                self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
                time.sleep(sleep_time)
                # sleep_time = sleep_time + delta_time
            else:
                time.sleep(0.01)
                # if pwm_timeout and time.time()-start_pwm_time > pwm_timeout:
                #     break
            # if time.time()-start_pwm_time<config.pid_interval:
            #     time.sleep(config.pid_interval-(time.time()-start_pwm_time))
        # self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
        # self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
        # 不支持输出获取pwm状态，以后再调试
        # print('left_pwm:',self.left_pwm,self.pi.get_PWM_dutycycle(config.left_pwm_pin),'right_pwm:',self.right_pwm,self.pi.get_PWM_dutycycle(config.right_pwm_pin))
        # print(time.time(), 'left_pwm:', self.left_pwm, 'right_pwm:', self.right_pwm)

    def set_steer_engine(self, angle):
        self.pi.set_PWM_dutycycle(config.steer_engine_pin, angle)

    def get_distance_dict(self):
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
            self.set_steer_engine(angle_pwm)
            # else:
            #     angle_pwm = 2500 - i * 10
            #     self.set_steer_engine(angle_pwm)
            # 更新位置矩阵 连续五次检查不到将该位置置位0表示该位置没有距离信息
            laser_distance = 0
            for j in range(5):
                laser_distance = self.laser_obj.read_laser()
                if laser_distance:
                    break
                else:
                    laser_distance = 0
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
            time.sleep(0.01)

    def set_gpio(self, control_left_motor=False,
                  control_right_motor=False,
                  control_alarm_light=False,
                  control_left_sidelight=False,
                  control_right_sidelight=False
                  ):
        if control_left_motor:
            if self.left_motor_output:
                self.pi.write(config.gpio_output_1, pigpio.LOW)
                self.left_motor_output = 0
            else:
                self.pi.write(config.gpio_output_1, pigpio.HIGH)
                self.left_motor_output = 1
        if control_right_motor:
            if self.right_motor_output:
                self.pi.write(config.gpio_output_2, pigpio.LOW)
                self.right_motor_output = 0
            else:
                self.pi.write(config.gpio_output_2, pigpio.HIGH)
                self.right_motor_output = 1
        if control_alarm_light:
            if self.alarm_light_output:
                self.pi.write(config.gpio_output_3, pigpio.LOW)
                self.alarm_light_output = 0
            else:
                self.pi.write(config.gpio_output_3, pigpio.HIGH)
                self.alarm_light_output = 1
        if control_left_sidelight:
            if self.left_sidelight_output:
                self.pi.write(config.gpio_output_4, pigpio.LOW)
                self.left_sidelight_output = 0
            else:
                self.pi.write(config.gpio_output_4, pigpio.HIGH)
                self.left_sidelight_output = 1
        if control_right_sidelight:
            if self.right_sidelight_output:
                self.pi.write(config.gpio_output_5, pigpio.LOW)
                self.right_sidelight_output = 0
            else:
                self.pi.write(config.gpio_output_5, pigpio.HIGH)
                self.right_sidelight_output = 1

    def set_ptz_camera(self, pan_angle_pwm=1500, tilt_angle_pwm=1500):
        """
        设置云台舵机角度
        :param pan_angle_pwm: 500-2500 设置水平角度
        :param tilt_angle_pwm: 500-2500  设置垂直角度
        :return:
        """
        self.pi.set_servo_pulsewidth(config.pin_pan, pan_angle_pwm)
        self.pi.set_servo_pulsewidth(config.pin_tilt, tilt_angle_pwm)
        self.pan_angle_pwm = pan_angle_pwm
        self.tilt_angle_pwm = tilt_angle_pwm
        # i = 0
        # b_add = 1
        # min_i = 0
        # max_i = 10
        # while True:
        #     angle_pwm = 500 + i * 200
        #     self.pi.set_servo_pulsewidth(config.pin_pan,angle_pwm)
        #     self.pi.set_servo_pulsewidth(config.pin_tilt,angle_pwm)
        #     i += 1 * b_add
        #     if i >= max_i or i <= min_i:
        #         b_add = -1 if b_add == 1 else 1
        #     time.sleep(0.3)


if __name__ == '__main__':
    pi_main_obj = PiMain()
    loop_change_pwm_thread = threading.Thread(target=pi_main_obj.loop_change_pwm)
    loop_change_pwm_thread.start()
    while True:
        try:
            # 已经使用w,a,s,d,q,  r,t  ,y,u,i,o,p,  f
            # w,a,s,d 为前后左右，q为停止 按键后需要按回车才能生效
            print('w,a,s,d 为前后左右，q为停止\n'
                  'r,t 左右抽水泵\n'
                  'y,u,i,o,p 摄像头舵机 y回中 u右  i左  o上   p下\n'
                  'f  测距\n'
                  'g  获取gps数据\n'
                  'h  获取罗盘数据\n'
                  'j  声光报警器\n'
                  'k  左舷灯\n'
                  'l  右舷灯\n'
                  'z 退出\n'
                  'x  接受遥控器输入\n'
                  'c  声呐数据\n'
                  '')
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
                pi_main_obj.set_gpio(control_left_motor=True)
            elif key_input.startswith('t'):
                pi_main_obj.set_gpio(control_right_motor=True)
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
            # 获取激光雷达测距数据
            elif key_input.startswith('f'):
                pi_main_obj.get_distance_dict()
            elif key_input.startswith('g'):
                key_input = input('input:  C0  开始  C1 结束 其他为读取 >')
                if key_input == 'C0':
                    theta = pi_main_obj.compass_obj.read_compass(send_data='C0')
                elif key_input == 'C1':
                    theta = pi_main_obj.compass_obj.read_compass(send_data='C1')
                else:
                    theta = pi_main_obj.compass_obj.read_compass()
                print('theta', theta)
            elif key_input.startswith('h'):
                gps_data = pi_main_obj.gps_obj.read_gps()
                print('gps_data', gps_data)
            # 控制声光报警器
            elif key_input.startswith('j'):
                pi_main_obj.set_gpio(control_alarm_light=True)
            # 控制左舷灯
            elif key_input.startswith('k'):
                pi_main_obj.set_gpio(control_left_sidelight=True)
            # 控制右舷灯
            elif key_input.startswith('l'):
                pi_main_obj.set_gpio(control_right_sidelight=True)
            elif key_input.startswith('x'):
                while True:
                    try:
                        pi_main_obj.set_pwm(set_left_pwm=pi_main_obj.channel_row_input_pwm, set_right_pwm=pi_main_obj.channel_col_input_pwm)
                    except KeyboardInterrupt:
                        break
            # TODO
            # 返航
            # 角度控制
            # 到达目标点控制
            # 简单走矩形区域
            # m 退出
            elif key_input.startswith('z'):
                break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print({'error': e})
            continue
