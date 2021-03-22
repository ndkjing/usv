import copy
import threading
import math
import time
import json
import os
import sys
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)

from utils import lng_lat_calculate
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
            self.gps_obj = self.get_gps_obj()
            self.compass_obj = self.get_compass_obj()
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

    def get_left_ultrasonic_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.left_rx, tx_pin=config.left_tx, baud=config.ultrasonic_baud)

    def get_right_ultrasonic_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.right_rx, tx_pin=config.right_tx,
                                      baud=config.ultrasonic_baud)

    def get_compass_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_compass_rx, tx_pin=config.pin_compass_tx, baud=config.pin_compass_baud)

    def get_gps_obj(self):
        return pi_softuart.PiSoftuart(pi=self.pi, rx_pin=config.pin_gps_rx, tx_pin=config.pin_gps_tx, baud=config.pin_gps_baud)

    # 对距离进行滤波处理
    def distance_filter(self, distance, left=1):
        if left:
            if distance:
                if not self.last_left_distance:
                    self.last_left_distance = distance
                    return distance
                else:
                    if abs(distance-self.last_left_distance) > 1:
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
                    if abs(distance-self.last_right_distance) > 1:
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
        if config.b_use_ultrasonic and config.current_platform == config.CurrentPlatform.pi:
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
        if config.b_use_ultrasonic and config.current_platform == config.CurrentPlatform.pi:
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
                if remote_forward_pwm<1550 and remote_forward_pwm >1450:
                    remote_forward_pwm=config.stop_pwm
                # 防止过大值
                elif remote_forward_pwm>=1900:
                    remote_forward_pwm=1900
                # 防止初始读取到0电机会转动， 设置为1500
                elif remote_forward_pwm<1000 :
                    remote_forward_pwm=config.stop_pwm
                # 防止过小值
                elif remote_forward_pwm<=1100 and remote_forward_pwm>=1000:
                    remote_forward_pwm=1100
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
                remote_left_pwm = 1500 + (remote_forward_pwm-1500) + (remote_steer_pwm-1500)
                remote_right_pwm = 1500 + (remote_forward_pwm-1500) - (remote_steer_pwm-1500)
                # self.set_pwm(remote_left_pwm, remote_right_pwm)
                time.sleep(0.1)
            except Exception as e :
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
                    if diff>1800:
                        self.b_start_remote = 1
                    elif diff<1200:
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
            left_pwm = config.stop_pwm+int(config.speed_grade)*100
        if right_pwm is None:
            right_pwm = config.stop_pwm+int(config.speed_grade)*100
        self.set_pwm(left_pwm, right_pwm)

    def backword(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = config.stop_pwm-int(config.speed_grade)*100
        if right_pwm is None:
            right_pwm = config.stop_pwm-int(config.speed_grade)*100
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
        sleep_time = 0.001
        delta_time = 0.0001 / 500.0
        # start_pwm_time = time.time()
        while True:
            if abs(self.left_pwm - self.target_left_pwm) != 0 or abs(self.right_pwm != self.target_right_pwm) != 0:
                if abs(self.target_left_pwm - self.left_pwm) == 0:
                    pass
                else:
                    self.left_pwm = self.left_pwm + (self.target_left_pwm - self.left_pwm) // abs(self.target_left_pwm - self.left_pwm) * 1
                if abs(self.target_right_pwm - self.right_pwm) == 0:
                    pass
                else:
                    self.right_pwm = self.right_pwm + (self.target_right_pwm - self.right_pwm) // abs(self.target_right_pwm - self.right_pwm) * 1
                self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
                self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
                time.sleep(sleep_time)
                sleep_time = sleep_time + delta_time
            else:
                time.sleep(0.02)
                # if pwm_timeout and time.time()-start_pwm_time > pwm_timeout:
                #     break
            # if time.time()-start_pwm_time<config.pid_interval:
            #     time.sleep(config.pid_interval-(time.time()-start_pwm_time))
        # self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
        # self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
        # 不支持输出获取pwm状态，以后再调试
        # print('left_pwm:',self.left_pwm,self.pi.get_PWM_dutycycle(config.left_pwm_pin),'right_pwm:',self.right_pwm,self.pi.get_PWM_dutycycle(config.right_pwm_pin))
        # print(time.time(), 'left_pwm:', self.left_pwm, 'right_pwm:', self.right_pwm)


if __name__ == '__main__':
    pi_main_obj = PiMain()
    while True:
        try:
            # w,a,s,d 为前后左右，q为后退 按键后需要按回车才能生效
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

            # 获取当前GPS位置和船头朝向
            elif key_input == 'l':
                pi_main_obj.get_current_location()

            # 设置当前GPS为返航点
            elif key_input.startswith('h'):
                pi_main_obj.set_home_location()

            # 返航
            elif key_input.startswith('b'):
                if not os.path.exists(config.home_location_path):
                    print('不存在home点')
                    continue
                with open(config.home_location_path,'r') as f:
                    home_lng_lat_json = json.load(f)
                home_lng_lat = home_lng_lat_json['home_lng_lat']
                print('home_lng_lat',home_lng_lat)
                pi_main_obj.home_lng_lat = home_lng_lat
                print('目标地点', pi_main_obj.home_lng_lat)
                home_distance = lng_lat_calculate.distanceFromCoordinate(
                    pi_main_obj.lng_lat[0],
                    pi_main_obj.lng_lat[1],
                    pi_main_obj.home_lng_lat[0],
                    pi_main_obj.home_lng_lat[1])
                while home_distance > config.arrive_distance:
                    home_distance = lng_lat_calculate.distanceFromCoordinate(
                        pi_main_obj.lng_lat[0],
                        pi_main_obj.lng_lat[1],
                        pi_main_obj.home_lng_lat[0],
                        pi_main_obj.home_lng_lat[1])
                    if int(time.time())%2==0:
                        print('距离目标点距离: ', home_distance)
                    left_pwm, right_pwm = pi_main_obj.point_control(
                        pi_main_obj.home_lng_lat)
                    print('left_pwm, right_pwm',left_pwm, right_pwm)
                    pi_main_obj.pi_obj.set_pwm(left_pwm, right_pwm)
                    time.sleep(config.pid_interval)
                pi_main_obj.pi_obj.stop()

            # 角度控制
            elif key_input.startswith('r'):
                try:
                    theta = int(key_input[1:])
                    print('theta:', theta)
                    if theta>0:
                        target_theta = (pi_main_obj.theta+theta)%360
                    else:
                        target_theta = (pi_main_obj.theta + (360+theta)) % 360
                    error_theta = abs(target_theta-pi_main_obj.theta)
                    while error_theta>10:
                        left_pwm, right_pwm =pi_main_obj.yaw_control(theta)
                        if int(time.time()) % 2 == 0:
                            print('left_pwm, right_pwm', left_pwm, right_pwm)
                        pi_main_obj.forward(left_pwm, right_pwm)
                        time.sleep(config.pid_interval)
                except Exception as e:
                    print({'error': e})

            # 到达目标点控制
            elif key_input.startswith('t'):
                point_x = None
                point_y = None
                try:
                    str_x, str_y = key_input[1:].split(',')
                    point_x = int(str_x)
                    point_y = int(str_y)
                    print(point_x, point_y)
                    theta = math.atan2(point_x, point_y)
                except Exception as e:
                    print({'error': e})
                    continue
                if theta >= 0:
                    theta = (360 + theta - 90) % 360
                else:
                    theta = (360 - abs(theta) + 90) % 360
                # 距离转换为经纬度
                target_lng_lat = lng_lat_calculate.one_point_diatance_to_end(
                    pi_main_obj.lng_lat[0],
                    pi_main_obj.lng_lat[1],
                    math.sqrt(
                        math.pow(
                            point_x,
                            point_y)),
                    theta)
                distance = lng_lat_calculate.distanceFromCoordinate(
                    pi_main_obj.lng_lat[0],
                    pi_main_obj.lng_lat[1],
                    target_lng_lat[0],
                    target_lng_lat[1])
                while distance > config.arrive_distance:
                    distance = lng_lat_calculate.distanceFromCoordinate(
                        pi_main_obj.lng_lat[0],
                        pi_main_obj.lng_lat[1],
                        target_lng_lat[0],
                        target_lng_lat[1])
                    if int(time.time()) % 2 == 0:
                        print('距离目标点距离: ', distance)
                    left_pwm, right_pwm = pi_main_obj.point_control(
                        target_lng_lat)
                    print('left_pwm, right_pwm', left_pwm, right_pwm)
                    pi_main_obj.forward(left_pwm, right_pwm)
                    time.sleep(config.pid_interval)

            # 简单走矩形区域
            elif key_input.startswith('p'):
                # 半边长
                half_w = int(key_input[1:])
                point_list_status = [0,0,0,0]
                point_list = []
                p1 = lng_lat_calculate.one_point_diatance_to_end(pi_main_obj.lng_lat[0],pi_main_obj.lng_lat[1],45,half_w*2)
                p2 = lng_lat_calculate.one_point_diatance_to_end(pi_main_obj.lng_lat[0],pi_main_obj.lng_lat[1],135,half_w*2)
                p3 = lng_lat_calculate.one_point_diatance_to_end(pi_main_obj.lng_lat[0],pi_main_obj.lng_lat[1],225,half_w*2)
                p4 = lng_lat_calculate.one_point_diatance_to_end(pi_main_obj.lng_lat[0],pi_main_obj.lng_lat[1],315,half_w*2)
                point_list.append(p1)
                point_list.append(p2)
                point_list.append(p3)
                point_list.append(p4)
                while point_list_status.count(0)>0:
                    index = point_list_status.index(0)
                    distance = lng_lat_calculate.distanceFromCoordinate(
                        pi_main_obj.lng_lat[0],
                        pi_main_obj.lng_lat[1],
                        point_list[index][0],
                        point_list[index][1])
                    while distance > config.arrive_distance:
                        distance = lng_lat_calculate.distanceFromCoordinate(
                            pi_main_obj.lng_lat[0],
                            pi_main_obj.lng_lat[1],
                            point_list[index][0],
                            point_list[index][1])
                        left_pwm, right_pwm = pi_main_obj.point_control(point_list[index])
                        if int(time.time()) % 2 == 0:
                            print('距离目标点距离: ', distance)
                            print('left_pwm, right_pwm', left_pwm, right_pwm)
                        pi_main_obj.forward(left_pwm, right_pwm)
                        time.sleep(config.pid_interval)
                    point_list_status[index]=1
                pi_main_obj.stop()
            # m 退出
            elif key_input.startswith('m'):
                break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print({'error': e})
            continue

