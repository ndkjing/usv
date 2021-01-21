import copy
import threading
import math
import time
import json
import os
import sys

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)

sys.path.append(
    os.path.join(
        root_dir,
        'dataGetSend'))
sys.path.append(
    os.path.join(
        root_dir,
        'utils'))
sys.path.append(
    os.path.join(
        root_dir,
        'piControl'))
from piControl import simplePid
from piControl import pi_control

from utils import lng_lat_calculate
from utils import log
from dataGetSend import com_data
import config
logger = log.LogHandler('pi_log')


class PiMain:
    def __init__(self):
        self.gps_obj = com_data.SerialData(
            config.gps_port,
            config.gps_baud,
            timeout=1 / config.com2pi_interval,
            logger=logger)

        self.compass_obj = com_data.SerialData(
            config.compass_port,
            config.compass_baud,
            timeout=0.2,
            logger=logger)
        # 经纬度 和 船头角度 北为0 逆时针为正
        self.lng_lat = None
        self.lng_lat_error = None

        self.theta = None
        # 树莓派pwm波控制对象
        if config.current_platform == 'l':
            self.pi_obj = pi_control.PiControl()
        # 返航点
        self.home_lng_lat = None

        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd

        self.errorSum = 0
        self.currentError = 0
        self.previousError = 0

    def distance_p(self, distance):
        pwm = int((distance * 30))
        if pwm >= config.motor_forward:
            pwm = config.motor_forward
        return pwm

    def update_steer_pid(self, theta_error):
        errorSum = self.errorSum + theta_error
        control = self.kp * theta_error + self.ki * errorSum + \
            self.kd * (theta_error - self.previousError)
        self.previousError = theta_error
        # 控制量归一化
        pwm = int((control / 180.0) * config.motor_steer)
        return pwm

    def pid_pwm(self, distance, theta_error):
        forward_pwm = self.distance_p(distance)
        steer_pwm = self.update_steer_pid(theta_error)
        left_pwm = 1500 + forward_pwm - steer_pwm
        right_pwm = 1500 + forward_pwm + steer_pwm
        return left_pwm, right_pwm

    def get_gps_data(self):
        while True:
            # serial_obj.send_data('31',b_hex=True)
            try:
                data = self.gps_obj.readline()
                str_data = data.decode('ascii')
                if str_data.startswith('$GNGGA'):
                    data_list = str_data.split(',')
                    # print(data_list)
                    lng, lat = round(float(data_list[4][:3]) +
                                     float(data_list[4][3:]) /
                                     60, 6), round(float(data_list[2][:2]) +
                                                   float(data_list[2][2:]) /
                                                   60, 6)
                    self.lng_lat = [lng, lat]
                    self.lng_lat_error = float(data_list[8])
            except Exception as e:
                logger.error({'error': e})

    def get_compass_data(self):
        while True:
            try:
                self.compass_obj.send_data('31', b_hex=True)
                data = self.compass_obj.readline()
                # 角度
                str_data = data.decode('ascii')[:-3]
                if len(str_data) < 2:
                    continue
                float_data = float(str_data)
                self.theta = 360 - float_data
                time.sleep(config.pid_interval)
            except Exception as e:
                logger.error({'error': e})

    def get_current_location(self):
        print(
            '经纬度:',
            self.lng_lat,
            'gps偏差: ',
            self.lng_lat_error,
            ', 船头方向:',
            self.theta)

    def set_home_location(self):
        if self.lng_lat is None:
            logger.error('当前无GPS信号，无法设置返航点')
        # gps为靠近0
        elif abs(self.lng_lat[0])<10:
            logger.error('当前无GPS信号弱，无法设置返航点')
        else:
            self.home_lng_lat = copy.deepcopy(self.lng_lat)
            print({'保存路径':config.home_location_path})
            with open(config.home_location_path,'w') as f:
                json.dump({'home_lng_lat':self.home_lng_lat},f)

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

    def point_control(self, lng_lat):
        """
        到达指定目标点
        :param lng_lat:
        :return:
        """
        distance = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0], self.lng_lat[1], lng_lat[0], lng_lat[1])
        theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
                                                      self.lng_lat[1],
                                                      lng_lat[0],
                                                      lng_lat[1])
        # 求船头与目标角度偏差角度
        theta_error =  theta-self.theta
        if abs(theta_error) > 180:
            if theta_error > 0:
                theta_error = theta_error - 360
            else:
                theta_error = 360 + theta_error
        if int(time.time()) % 2 == 0:
            print('距离: ',distance,'目标角度: ',theta,' 船头角度: ',self.theta,'偏差角度: ',theta_error)
        left_pwm, right_pwm = self.pid_pwm(distance, theta_error)
        return left_pwm, right_pwm


if __name__ == '__main__':
    pi_main_obj = PiMain()
    compass_thread = threading.Thread(target=pi_main_obj.get_compass_data)
    gps_thread = threading.Thread(target=pi_main_obj.get_gps_data)

    compass_thread.setDaemon(True)
    gps_thread.setDaemon(True)

    compass_thread.start()
    gps_thread.start()

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
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.pi_obj.forward(
                        1600 + 100 * gear, 1600 + 100 * gear)
                else:
                    pi_main_obj.pi_obj.forward()
            elif key_input.startswith('a'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.pi_obj.left(
                        1400 - 100 * gear, 1600 + 100 * gear)
                else:
                    pi_main_obj.pi_obj.left()
            elif key_input.startswith('s'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.pi_obj.backword(
                        1400 - 100 * gear, 1400 - 100 * gear)
                else:
                    pi_main_obj.pi_obj.backword()
            elif key_input.startswith('d'):
                if gear is not None:
                    if gear >= 4:
                        gear = 4
                    pi_main_obj.pi_obj.right(
                        1600 + 100 * gear, 1400 - 100 * gear)
                else:
                    pi_main_obj.pi_obj.right()
            elif key_input == 'q':
                pi_main_obj.pi_obj.stop()

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
                    pi_main_obj.pi_obj.forward(left_pwm, right_pwm)
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
                        pi_main_obj.pi_obj.forward(left_pwm, right_pwm)
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
                    pi_main_obj.pi_obj.forward(left_pwm, right_pwm)
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
                        pi_main_obj.pi_obj.forward(left_pwm, right_pwm)
                        time.sleep(config.pid_interval)
                    point_list_status[index]=1
                pi_main_obj.pi_obj.stop()
            # m 退出
            elif key_input.startswith('m'):
                break
        except KeyboardInterrupt:
            continue
        except Exception as e:
            print({'error': e})
            continue
