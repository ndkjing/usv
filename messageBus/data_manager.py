"""
管理数据收发
"""
import time
import json
import copy
import os
import math
import enum
import numpy as np
from collections import deque
from messageBus import data_define
from externalConnect import server_data
from drivers import com_data
from utils.log import LogHandler
from externalConnect import baidu_map
from drivers import audios_manager, drone_kit_control, pi_main
from utils import lng_lat_calculate
from utils import check_network
from storage import save_data
import config
from moveControl.pathTrack import simple_pid, pure_pursuit


class ShipStatus(enum.Enum):
    """
    船当前状态
    """
    manual = 1
    auto = 2
    backhome = 3


class DataManager:
    def __init__(self):
        self.data_define_obj = data_define.DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log', level=20)
        self.data_save_logger = LogHandler('data_save_log', level=20)
        self.com_data_read_logger = LogHandler('com_data_read_logger', level=20)
        self.com_data_send_logger = LogHandler('com_data_send_logger', level=20)
        self.server_log = LogHandler('server_data')
        self.map_log = LogHandler('map_log')
        self.gps_log = LogHandler('gps_log')
        self.compass_log = LogHandler('compass_log')
        self.compass_log1 = LogHandler('compass_log1')
        # 路径跟踪方法
        if config.path_track_type == 2:
            self.path_track_obj = pure_pursuit.PurePursuit()
        else:
            self.path_track_obj = simple_pid.SimplePid()
        # 使用真实GPS 还是 初始化高德GPS
        self.use_true_gps = 0
        if config.current_platform == config.CurrentPlatform.pi:
            # 串口数据收发对象
            if os.path.exists(config.stc_port):
                self.com_data_obj = self.get_com_obj(port=config.stc_port, baud=config.stc_baud,
                                                     timeout=config.stc2pi_timeout,
                                                     logger=self.com_data_read_logger)
            if os.path.exists(config.gps_port):
                self.use_true_gps = 1
                self.gps_obj = self.get_com_obj(config.gps_port, config.gps_baud, self.gps_log)
            if os.path.exists(config.compass_port):
                self.compass_obj = self.get_com_obj(config.compass_port, config.compass_baud, self.compass_log)
            if os.path.exists(config.compass_port1):
                self.compass_obj1 = self.get_com_obj(config.compass_port1, config.compass_baud1, self.compass_log1)
            self.pi_main_obj = pi_main.PiMain()
            if config.b_use_pix:
                self.drone_obj = drone_kit_control.DroneKitControl(config.pix_port)
                self.drone_obj.download_mission(True)
                self.drone_obj.arm()
        # mqtt服务器数据收发对象
        self.server_data_obj = server_data.ServerData(self.server_log, topics=self.data_define_obj.topics)
        # 规划路径
        self.plan_path = []
        # 规划路径状态
        self.plan_path_status = None
        # 规划路径点状态
        self.plan_path_points_status = []
        # 船当前正确前往哪个目的地
        self.current_ststus_index = -1
        self.baidu_map_obj = None
        # 船最终控制移动方向
        self.ship_move_direction = str(360)
        # 船当前朝向
        self.ship_current_direction = -1
        # 左右侧超声波距离
        self.l_distance = None
        self.r_distance = None
        # 记录当前目标点经纬度（避免重复发送）
        self.path_target_lng_lats = None
        # 记录当前发送给单片机目标点经纬度（避免重复发送）
        self.send_target_gaode_lng_lat = None
        # 路径跟踪是否被终止
        self.b_stop_path_track = False
        # 返回状态消息中提示消息
        self.notice_info = ''
        # 记录里程与时间数据
        self.totle_distance = 0
        self.plan_start_time = None
        # 开机时间
        self.start_time = time.time()
        # 提示消息
        # 距离下一个目标点距离
        self.distance_p = 0
        # 路径结束计算得分
        self.distance_move_score = 0
        # 目标点信息
        self.path_info = [0, 0]
        # 手动控制时信息
        self.control_info = ""
        # 开关控制信息
        self.switch_info = 0
        # 当前是否抽水
        self.current_b_draw = 0
        # 抽水开始时间
        self.draw_start_time = None
        # 抽水结束发送数据
        self.b_draw_over_send_data = False
        # 单片机串口读取到的数据
        self.second_lng_lat = None
        self.water_data_dict = {}
        # 剩余电量
        self.dump_energy = None
        # 求剩余电量均值
        self.dump_energy_deque = deque(maxlen=20)
        # 电量告警
        self.low_dump_energy_warnning = 0
        # 返航点
        self.home_lng_lat = None
        # 船速度
        self.speed = None
        # 船行驶里程
        self.run_distance = 0
        # 经纬度 和 船头角度 北为0 逆时针为正
        self.lng_lat = None
        self.lng_lat_error = None
        # 罗盘一角度
        self.theta = None
        # 罗盘一角度
        self.theta1 = None
        # 罗盘一角度
        self.current_theta = None
        # 偏差角度
        self.theta_error = 0
        # 罗盘提示消息
        self.compass_notice_info = ''
        self.compass_notice_info1 = ''
        # 记录pwm调节时间和数值用于在家调试
        self.last_left_pwm = 1500
        self.last_right_pwm = 1500
        self.last_change_pwm_time = time.time()
        # 记录上一次经纬度
        self.last_lng_lat = None
        # 记录船状态
        self.ship_status = ShipStatus.manual
        # 记录平滑路径
        self.smooth_path_lng_lat=None

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        last_read_time = time.time()
        while True:
            # 水质数据 b''BBD:0,R:158,Z:36,P:0,T:16.1,V:92.08,x1:0,x2:2,x3:1,x4:0\r\n'
            # 经纬度数据 b'AA2020.2354804,N,11425.41234568896,E\r\n'
            try:
                row_com_data_read = self.com_data_obj.readline()
                com_data_read = str(row_com_data_read)[2:-5]
                # if time.time()-last_read_time>3:
                #     last_read_time = time.time()
                #     self.com_data_read_logger.info({'str com_data_read': com_data_read})
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
                        self.com_data_read_logger.info({'second_lng_lat': self.second_lng_lat})
                elif com_data_read.startswith('BB'):
                    com_data_list = com_data_read.split(',')
                    self.water_data_dict.update(
                        {'EC': float(com_data_list[1].split(':')[1]) / math.pow(10, int(com_data_list[7][3:]))})
                    self.water_data_dict.update(
                        {'DO': float(com_data_list[0][2:].split(':')[1]) / math.pow(10, int(com_data_list[6][3:]))})
                    self.water_data_dict.update(
                        {'TD': float(com_data_list[2].split(':')[1]) / math.pow(10, int(com_data_list[8][3:]))})
                    self.water_data_dict.update(
                        {'pH': float(com_data_list[3].split(':')[1]) / math.pow(10, int(com_data_list[9][3:]))})
                    self.water_data_dict.update({'wt': float(com_data_list[4].split(':')[1])})
                    self.dump_energy = float(com_data_list[5].split(':')[1])
                    if time.time() - last_read_time > 5:
                        last_read_time = time.time()
                        self.com_data_read_logger.info({'str com_data_read': com_data_read})
                        self.com_data_read_logger.info({'water_data_dict': self.water_data_dict})
            except Exception as e:
                self.com_data_read_logger.error({'串口数据解析错误': e})

    def get_com_obj(self, port, baud, logger, timeout=0.4):
        return com_data.ComData(
            port,
            baud,
            timeout=timeout,
            logger=logger)

    # 在线程中读取 gps
    def get_gps_data(self):
        last_read_time = time.time()
        last_read_lng_lat = None
        while True:
            try:
                data = self.gps_obj.readline()
                str_data = data.decode('ascii')
                if str_data.startswith('$GNGGA'):
                    data_list = str_data.split(',')
                    if len(data_list) < 8:
                        continue
                    lng, lat = round(float(data_list[4][:3]) +
                                     float(data_list[4][3:]) /
                                     60, 6), round(float(data_list[2][:2]) +
                                                   float(data_list[2][2:]) /
                                                   60, 6)
                    if lng < 1 or lat < 1:
                        pass
                    else:
                        # 替换上一次的值
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                        self.lng_lat = [lng, lat]
                    self.lng_lat_error = float(data_list[8])
                    if self.lng_lat_error < 3:
                        if last_read_lng_lat is None:
                            last_read_lng_lat = copy.deepcopy(self.lng_lat)
                            last_read_time = time.time()
                        else:
                            # 计算当前行驶里程
                            speed_distance = lng_lat_calculate.distanceFromCoordinate(last_read_lng_lat[0],
                                                                                      last_read_lng_lat[1],
                                                                                      self.lng_lat[0],
                                                                                      self.lng_lat[1])
                            self.run_distance += speed_distance
                            # 计算速度
                            self.speed = round(speed_distance / (time.time() - last_read_time), 1)
                            last_read_lng_lat = copy.deepcopy(self.lng_lat)
                            last_read_time = time.time()
            except Exception as e:
                last_read_lng_lat = None
                last_read_time = time.time()
                self.logger.error({'error': e})
                time.sleep(1)

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
                    if last_send_data !='C0':
                        self.compass_obj.send_data('C0', b_hex=True)
                        time.sleep(0.05)
                        data0 = self.compass_obj.readline()
                        str_data0 = data0.decode('ascii')
                        if len(str_data0)>3:
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
                elif int(config.calibration_compass) in [9,10]:
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
                    time.sleep(config.compass_timeout / 3)
                    count = 50
                    last_send_data = None
                    # self.compass_notice_info = ''
            except Exception as e:
                if count > 0:
                    count = count - 1
                else:
                    pass
                    # self.theta = None
                self.logger.error({'error': e})
                time.sleep(1)

    def set_home_location(self):
        if self.lng_lat is None:
            self.logger.error('当前无GPS信号，无法设置返航点')
        else:
            if abs(self.lng_lat[0]) < 10:
                # gps为靠近0
                self.logger.error('当前无GPS信号弱，无法设置返航点')
            else:
                self.home_lng_lat = copy.deepcopy(self.lng_lat)
            # print({'保存路径':config.home_location_path})
            # with open(config.home_location_path,'w') as f:
            #     json.dump({'home_lng_lat':self.home_lng_lat},f)

    # 获取备份罗盘数据
    def get_compass1_data(self):
        if os.path.exists(config.compass_port1):
            # 累计50次出错则记为None
            count = 50
            # 记录上一次发送数据
            last_send_data = None
            while True:
                try:
                    # 检查罗盘是否需要校准 # 开始校准
                    if int(config.calibration_compass) == 1:
                        if last_send_data != 'C0':
                            self.compass_obj1.send_data('C0', b_hex=True)
                            time.sleep(0.05)
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            last_send_data = 'C0'
                    # 结束校准
                    elif int(config.calibration_compass) == 2:
                        if last_send_data != 'C1':
                            self.compass_obj1.send_data('C1', b_hex=True)
                            time.sleep(0.05)
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            data0 = self.compass_obj1.readline()
                            str_data0 = data0.decode('ascii')
                            if len(str_data0) > 3:
                                self.compass_notice_info1 += str_data0
                            last_send_data = 'C1'
                    # 隐藏修改 设置为10 改为启用gps计算角度  设置为9 改为启用二号罗盘
                    elif int(config.calibration_compass) in [10]:
                        self.theta1 = None
                    else:
                        self.compass_obj1.send_data('31', b_hex=True)
                        time.sleep(0.05)
                        data0 = self.compass_obj1.readline()
                        # 角度
                        str_data0 = data0.decode('ascii')[:-3]
                        if len(str_data0) < 1:
                            continue
                        float_data0 = float(str_data0)
                        self.theta1 = 360 - float_data0
                        time.sleep(config.compass_timeout / 3)
                        count = 50
                        last_send_data = None
                        # self.compass_notice_info1 = ''
                except Exception as e:
                    if count > 0:
                        count = count - 1
                    else:
                        self.theta1 = None
                    self.logger.error({'error': e})
                    time.sleep(1)

    def draw(self):
        # 判断是否抽水
        if self.server_data_obj.mqtt_send_get_obj.b_draw:
            if self.draw_start_time is None or self.current_b_draw == 0:
                if self.draw_start_time is not None:
                    # 超时中断抽水
                    if time.time() - self.draw_start_time > config.draw_time:
                        self.b_draw_over_send_data = True
                        self.com_data_obj.send_data('A0Z')
                        self.current_b_draw = 0
                        self.draw_start_time = None
                else:
                    self.com_data_obj.send_data('A1Z')
                    self.current_b_draw = 1
                    self.draw_start_time = time.time()
            elif self.draw_start_time is not None and self.current_b_draw == 1:
                # 超时中断抽水
                print(time.time() - self.draw_start_time)
                if time.time() - self.draw_start_time > config.draw_time:
                    self.b_draw_over_send_data = True
                    self.com_data_obj.send_data('A0Z')
                    self.current_b_draw = 0
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                    self.draw_start_time = None
        else:
            if self.current_b_draw is None or self.current_b_draw == 1:
                self.com_data_obj.send_data('A0Z')
                self.current_b_draw = 0
                self.draw_start_time = None

    def clear_status(self):
        """
        按了暂停或者不满足开始状态时候清除状态
        :return:
        """
        if not config.home_debug:
            self.pi_main_obj.stop()
        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
        # 清空目标点和状态
        self.server_data_obj.mqtt_send_get_obj.sampling_points = []
        self.server_data_obj.mqtt_send_get_obj.path_planning_points = []
        self.server_data_obj.mqtt_send_get_obj.sampling_points_status = []
        # 清空里程和时间
        self.totle_distance = 0
        self.plan_start_time = None
        # 清空提醒
        self.path_info = [0, 0]
        self.distance_p = 0
        # 清空路径
        self.smooth_path_lng_lat=None

    def check_remote_pwm(self):
        """
        检查遥控器PWM波值
        :return:
        """
        remote_forward_pwm = copy.deepcopy(int(self.pi_main_obj.channel_col_input_pwm))
        remote_steer_pwm = copy.deepcopy(int(self.pi_main_obj.channel_row_input_pwm))
        # print('remote', remote_forward_pwm, remote_steer_pwm)
        # 防止抖动
        if 1600 > remote_forward_pwm > 1400:
            remote_forward_pwm = config.stop_pwm
        # 防止过大值
        elif remote_forward_pwm >= 1900:
            remote_forward_pwm = 1900
        # 防止初始读取到0电机会转动， 设置为1500
        elif remote_forward_pwm < 1000:
            remote_forward_pwm = config.stop_pwm
        # 防止过小值
        elif 1100 >= remote_forward_pwm >= 1000:
            remote_forward_pwm = 1100
        # 防止抖动
        if 1600 > remote_steer_pwm > 1400:
            remote_steer_pwm = config.stop_pwm
        # 防止过大值
        elif remote_steer_pwm >= 1900:
            remote_steer_pwm = 1900
        # 防止初始读取到0电机会转动， 设置为1500
        elif remote_steer_pwm < 1000:
            remote_steer_pwm = config.stop_pwm
        # 防止过小值
        elif 1100 >= remote_steer_pwm >= 1000:
            remote_steer_pwm = 1100
        if remote_forward_pwm == config.stop_pwm and remote_steer_pwm == config.stop_pwm:
            remote_left_pwm = config.stop_pwm
            remote_right_pwm = config.stop_pwm
        else:
            remote_left_pwm = 1500 + (remote_forward_pwm - 1500) + (remote_steer_pwm - 1500)
            remote_right_pwm = 1500 + (remote_forward_pwm - 1500) - (remote_steer_pwm - 1500)
        return remote_left_pwm, remote_right_pwm

    def check_backhome(self):
        if self.server_data_obj.mqtt_send_get_obj.b_network_backhome:
            # 记录是因为按了长时间没操作判断为返航
            self.ship_status = ShipStatus.backhome
        if self.low_dump_energy_warnning:
            # 记录是因为按了低电量判断为返航
            self.ship_status = ShipStatus.backhome

    def back_home(self):
        """
        紧急情况下返航
        :return:
        """
        # 有返航点下情况下返回返航点，没有则停止
        if self.home_lng_lat is None:
            if not config.home_debug:
                self.pi_main_obj.stop()
        else:
            # 计算目标真实经纬度,将目标经纬度转换为真实经纬度
            home_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.home_lng_lat)
            self.path_target_lng_lats = home_gaode_lng_lat
            self.points_arrive_control(home_gaode_lng_lat, True)
            self.pi_main_obj.stop()

    def smooth_path(self):
        smooth_path_lng_lat = []
        current_index = 1
        for index, target_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points):
            if index == 0:
                theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
                                                self.lng_lat[1],
                                                target_lng_lat[0],
                                                target_lng_lat[1])
                distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                target_lng_lat[0],
                                                                target_lng_lat[1])
                for i in range(1,int((distance//config.forward_see_distance)+1)):
                    cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                theta,
                                                                config.forward_see_distance*i)
                    smooth_path_lng_lat.append(cal_lng_lat)
            else:
                theta = lng_lat_calculate.angleFromCoordinate(self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][0],
                                                self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][1],
                                                target_lng_lat[0],
                                                target_lng_lat[1])
                distance = lng_lat_calculate.distanceFromCoordinate(self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][0],
                                                                    self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][1],
                                                                    target_lng_lat[0],
                                                                    target_lng_lat[1])
                # if distance<config.forward_see_distance:
                #     current_index-=1
                # else:
                #     current_index = index
                for i in range(1, int((distance//config.forward_see_distance) + 1)):
                    cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][0],
                                                                              self.server_data_obj.mqtt_send_get_obj.path_planning_points[index-1][1],
                                                                              theta,
                                                                              config.forward_see_distance * i)
                    smooth_path_lng_lat.append(cal_lng_lat)
                if distance < config.forward_see_distance:
                    smooth_path_lng_lat.append(target_lng_lat)

        return smooth_path_lng_lat

    def calc_target_lng_lat(self):
        """
        根据当前点和路径计算下一个经纬度点
        :return:
        """
        # 离散按指定间距求取轨迹点数量
        if not self.smooth_path_lng_lat:
            self.smooth_path_lng_lat = self.smooth_path()
        # 搜索最临近的路点
        distance_list = []
        for target_lng_lat in self.smooth_path_lng_lat:
            distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                target_lng_lat[0],
                                                                target_lng_lat[1])
            distance_list.append(distance)
        index = distance_list.index(min(distance_list))
        lng_lat = self.smooth_path_lng_lat[index]
        index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                        self.lng_lat[1],
                                                                        lng_lat[0],
                                                                        lng_lat[1])
        while config.forward_see_distance > index_point_distance and (index + 1) < len(self.smooth_path_lng_lat):
            index += 1
            lng_lat = self.smooth_path_lng_lat[index]
            index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                            self.lng_lat[1],
                                                                            lng_lat[0],
                                                                            lng_lat[1])
        # print('index,len(self.smooth_path_lng_lat) index_point_distance',index,len(self.smooth_path_lng_lat),index_point_distance)
        return self.smooth_path_lng_lat[index]

    def build_obstacle_map(self):
        """
        根据超声波距离构建障碍物地图
        :return: 障碍物位置举证
        """
        map_size = int(10/0.5)
        obstacle_map = np.zeros((map_size,map_size))
        # 判断前方距离是否有障碍物，根据障碍物改变目标点
        if config.b_use_ultrasonic:
            if self.l_distance <= 0.25:
                self.l_distance=None
            if self.r_distance <= 0.25:
                self.r_distance=None
            if self.l_distance:
                y_l = int(map_size/2)-1
                x_l = int(map_size/2) - int(self.l_distance/0.5)
                obstacle_map[0:x_l][y_l] = 1
            if self.r_distance:
                y_l = int(map_size/2)+1
                x_l = int(map_size/2) - int(self.r_distance/0.5)
                obstacle_map[0:x_l][y_l] = 1
        return obstacle_map

    def get_avoid_obstacle_point(self,path_planning_point_gps):
        """
        根据障碍物地图获取下一个运动点
        :return:
        """
        # obstacle_map = self.build_obstacle_map()
        next_point_lng_lat = path_planning_point_gps
        if config.b_use_ultrasonic:
            if self.l_distance and self.r_distance:
                if self.l_distance <= 1.5 or self.r_distance <= 1.5:
                    distance = 1.5
                    theta = 180
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     theta,
                                                                                     distance)
                elif self.l_distance<3 and self.r_distance>3:
                    distance = 3
                    theta = -90
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     theta,
                                                                                     distance)
                elif self.l_distance>3 and self.r_distance<3:
                    distance = 3
                    theta = 90
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     theta,
                                                                                     distance)
        return next_point_lng_lat

    def points_arrive_control(self, target_lng_lat, sample_lng_lat, b_force_arrive=False):
        """
        :param sample_lng_lat:
        :param target_lng_lat: 目标点高德经纬度
        :param b_force_arrive: 是否约束一定要到达
        :return:
        """
        # 获取下一个路径点坐标，计算目标真实经纬度,将目标经纬度转换为真实经纬度
        if config.home_debug:
            path_planning_point_gps = target_lng_lat
            sample_lng_lat_gps = sample_lng_lat
        else:
            pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
            path_planning_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
                                                                         pi_current_gaode_lng_lat,
                                                                         target_lng_lat)
            sample_lng_lat_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
                                                                         pi_current_gaode_lng_lat,
                                                                         sample_lng_lat)
        distance = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0],
            self.lng_lat[1],
            sample_lng_lat_gps[0],
            sample_lng_lat_gps[1])
        self.distance_p = distance
        if b_force_arrive and distance < config.arrive_distance:
            return True
        while distance > config.arrive_distance:
            # 更新提醒的距离信息
            distance_sample = lng_lat_calculate.distanceFromCoordinate(
                self.lng_lat[0],
                self.lng_lat[1],
                sample_lng_lat_gps[0],
                sample_lng_lat_gps[1])
            self.distance_p = distance_sample
            # 避障判断下一个点
            path_planning_point_gps = self.get_avoid_obstacle_point(path_planning_point_gps)
            # 计算到下一个点距离
            all_distance = lng_lat_calculate.distanceFromCoordinate(
                self.lng_lat[0], self.lng_lat[1], path_planning_point_gps[0],
                path_planning_point_gps[1])
            # 当前点到目标点角度
            point_theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                path_planning_point_gps[0],
                                                                path_planning_point_gps[1])

            theta_error = point_theta - self.current_theta
            if abs(theta_error) > 180:
                if theta_error > 0:
                    theta_error = theta_error - 360
                else:
                    theta_error = 360 + theta_error
            self.theta_error = theta_error
            if config.path_track_type == 2:
                left_pwm, right_pwm = self.path_track_obj.pure_pwm(distance=all_distance,
                                                                   theta_error=theta_error)
            else:
                # 计算前向距离与水平距离
                if 0 <= theta_error < 90:
                    forward_distance = all_distance * math.cos(math.radians(theta_error))
                    steer_distance = -all_distance * math.sin(math.radians(theta_error))
                elif 90 <= theta_error < 180:
                    forward_distance = -all_distance * math.sin(math.radians(theta_error - 90))
                    steer_distance = -all_distance * math.cos(math.radians(theta_error - 90))
                elif -90 <= theta_error < 0:
                    forward_distance = all_distance * math.sin(math.radians(theta_error + 90))
                    steer_distance = all_distance * math.cos(math.radians(theta_error))
                else:
                    forward_distance = -all_distance * math.cos(math.radians(theta_error + 180))
                    steer_distance = all_distance * math.sin(math.radians(theta_error + 180))
                b_use_theta_error = True
                if b_use_theta_error:
                    left_pwm, right_pwm = self.path_track_obj.pid_pwm(distance=all_distance,
                                                                      theta_error=theta_error)
                else:
                    left_pwm, right_pwm = self.path_track_obj.pid_pwm_1(
                        forward_distance=forward_distance,
                        steer_distance=steer_distance)
                # 在家调试模式下预测目标经纬度
                if config.home_debug:
                    time.sleep(0.5)
                    # 计算当前行驶里程
                    if self.last_lng_lat:
                        speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
                                                                                  self.last_lng_lat[1],
                                                                                  self.lng_lat[0],
                                                                                  self.lng_lat[1])
                        self.run_distance += speed_distance
                    left_delta_pwm = int(self.last_left_pwm + left_pwm)/2 - config.stop_pwm
                    right_delta_pwm = int(self.last_right_pwm + right_pwm)/2 - config.stop_pwm
                    steer_power = left_delta_pwm - right_delta_pwm
                    forward_power = left_delta_pwm + right_delta_pwm
                    delta_distance = forward_power * 0.01
                    delta_theta = steer_power * 0.05
                    # if time.time() % 2 < 1:
                        # print('left_delta_pwm, right_delta_pwm left_pwm,right_pwm', left_delta_pwm, right_delta_pwm, left_pwm, right_pwm)
                        # print('delta_distance,delta_theta, forward_power steer_power',
                        #       delta_distance, delta_theta, forward_power,
                        #       steer_power)
                    self.last_lng_lat = copy.deepcopy(self.lng_lat)
                    if self.current_theta is not None:
                        self.current_theta = (self.current_theta - delta_theta/2) % 360
                    self.lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                               self.lng_lat[1],
                                                                               self.current_theta,
                                                                               delta_distance)
                self.last_left_pwm = left_pwm
                self.last_right_pwm = right_pwm
            if not config.home_debug:
                self.pi_main_obj.set_pwm(left_pwm, right_pwm)
            # 清空规划点
            if int(self.server_data_obj.mqtt_send_get_obj.control_move_direction) == -1:
                # 记录是因为按了暂停按钮而终止
                self.b_stop_path_track = True
                return False
            if not config.home_debug and self.pi_main_obj.b_start_remote:
                # 记录是因为按了遥控而终止
                self.b_stop_path_track = True
                break
            if self.server_data_obj.mqtt_send_get_obj.b_network_backhome:
                # 记录是因为按了长时间没操作判断为返航
                self.b_stop_path_track = True
                self.ship_status = ShipStatus.backhome
                break
            if self.low_dump_energy_warnning:
                # 记录是因为按了低电量判断为返航
                self.b_stop_path_track = True
                self.ship_status = ShipStatus.backhome
                break
            # 如果目标点改变并且不是强制到达 b_force_arrive
            if not b_force_arrive:
                break
            else:
                if distance_sample < config.arrive_distance:
                    return True

    # 发送函数会阻塞 必须使用线程
    def send_com_data(self):
        # 0 自动  1手动
        manul_or_auto = 1
        # 记录上次手动发送
        last_control = None
        # 记录发送的经纬度
        b_log_points = 1
        while True:
            time.sleep(config.pi2com_timeout)
            if config.b_use_ultrasonic:
                self.logger.info(
                    {'left_distance': self.pi_main_obj.left_distance, 'right_distance': self.pi_main_obj.left_distance})
            # 判断当前是手动控制还是自动控制
            d = int(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
            if d in [-1, 0, 90, 180, 270]:
                manul_or_auto = 1
                b_log_points = 1
            # 检查是否需要返航
            self.check_backhome()
            if self.ship_status == ShipStatus.backhome:
                self.clear_status()
                self.back_home()
            # 使用路径规划
            if len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                manul_or_auto = 0
                # 此时清除d
                self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
            # 调试模式下无法使用遥控器
            if not config.home_debug and config.current_platform ==config.CurrentPlatform.pi:
                remote_left_pwm, remote_right_pwm = self.check_remote_pwm()
            else:
                remote_left_pwm, remote_right_pwm = config.stop_pwm, config.stop_pwm
            # 遥控器模式
            if not config.home_debug and config.current_platform == config.CurrentPlatform.pi and self.pi_main_obj.b_start_remote:
                self.pi_main_obj.set_pwm(left_pwm=remote_left_pwm, right_pwm=remote_right_pwm)
            # 手动模式
            elif manul_or_auto == 1:
                if d == 0:
                    temp_com_data = 1
                    pwm_data = {'1': 1900, '3': 1900}
                elif d == 90:
                    temp_com_data = 3
                    pwm_data = {'1': 1900, '3': 1100}
                elif d == 180:
                    temp_com_data = 2
                    pwm_data = {'1': 1100, '3': 1100}
                elif d == 270:
                    temp_com_data = 4
                    pwm_data = {'1': 1100, '3': 1900}
                elif d == -1:
                    temp_com_data = 5
                    pwm_data = {'1': 1500, '3': 1500}
                else:
                    temp_com_data = None
                    pwm_data = None
                # 使用飞控
                if config.b_use_pix and pwm_data is not None:
                    if last_control is None or last_control != pwm_data:
                        last_control = pwm_data
                        self.drone_obj.channel_control(pwm_data)
                        self.com_data_send_logger.info({'com pwm data': pwm_data})
                # 使用树莓派
                elif config.b_use_pi:
                    if d == 0:
                        self.control_info = '向前'
                        self.pi_main_obj.forward()
                    elif d == 90:
                        self.control_info = '向左'
                        self.pi_main_obj.left()
                    elif d == 180:
                        self.control_info = '向后'
                        self.pi_main_obj.backword()
                    elif d == 270:
                        self.control_info = '向右'
                        self.pi_main_obj.right()
                    elif d == -1:
                        self.control_info = '停止'
                        if not config.home_debug:
                            self.pi_main_obj.stop()
                    # 改变状态不再重复发送指令
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                # 手动模式下判断是否抽水
                self.draw()
            # 自动模式计算角度
            elif manul_or_auto == 0:
                self.control_info = ''
                if self.lng_lat is None and self.second_lng_lat is None:
                    self.logger.error('无当前GPS，不能自主巡航')
                    time.sleep(0.5)
                    self.clear_status()
                    continue
                # 第一次进入路径规划时候的点设置为返航点
                if self.home_lng_lat is None:
                    self.set_home_location()
                if self.plan_start_time is None:
                    self.plan_start_time = time.time()
                # 设置自动路径搜索为False
                self.b_stop_path_track = False
                if b_log_points:
                    self.logger.info({'点击地点': self.server_data_obj.mqtt_send_get_obj.path_planning_points})
                # 计算船当前高德经纬度
                if self.lng_lat is None and self.second_lng_lat is not None:
                    if config.home_debug:
                        pi_current_gaode_lng_lat = self.lng_lat
                    else:
                        pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.second_lng_lat)
                else:
                    if config.home_debug:
                        pi_current_gaode_lng_lat = self.lng_lat
                    else:
                        if self.lng_lat:
                            pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
                        else:
                            pi_current_gaode_lng_lat=None
                # 计算船起始运行距离
                start_distance = self.run_distance
                # 判断是否是寻点模式点了寻点但是还没点开始
                if self.server_data_obj.mqtt_send_get_obj.row_gap:
                    if not self.server_data_obj.mqtt_send_get_obj.b_start:
                        b_log_points = 0
                        continue
                # 计算总里程
                for index, gaode_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points):
                    if index == 0:
                        distance_p = lng_lat_calculate.distanceFromCoordinate(
                            pi_current_gaode_lng_lat[0],
                            pi_current_gaode_lng_lat[1],
                            gaode_lng_lat[0],
                            gaode_lng_lat[1])
                        self.totle_distance += distance_p
                    else:
                        distance_p = lng_lat_calculate.distanceFromCoordinate(
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points[index - 1][0],
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points[index - 1][1],
                            gaode_lng_lat[0],
                            gaode_lng_lat[1])
                        self.totle_distance += distance_p
                self.logger.info({'全部距离': self.totle_distance})
                # 如果使用寻点但是没有点击确定
                self.path_info = [0, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
                while self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) > 0:
                    print(self.server_data_obj.mqtt_send_get_obj.sampling_points_status)
                    for index, gaode_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.sampling_points):
                        # 判断该点是否已经到达
                        if self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] == 1:
                            continue
                        # 更新目标点提示消息

                        # 计算下一个目标点经纬度
                        next_lng_lat = self.calc_target_lng_lat()
                        # 如果当前点靠近采样点指定范围就停止并采样
                        sample_distance = lng_lat_calculate.distanceFromCoordinate(
                            next_lng_lat[0],
                            next_lng_lat[1],
                            gaode_lng_lat[0],
                            gaode_lng_lat[1])
                        if sample_distance < config.forward_see_distance:
                            b_arrive_sample = self.points_arrive_control(gaode_lng_lat, gaode_lng_lat, b_force_arrive=True)
                        else:
                            b_arrive_sample = self.points_arrive_control(next_lng_lat, gaode_lng_lat, b_force_arrive=False)
                        # print('sample_distance b_arrive_sample',sample_distance,b_arrive_sample)
                        if b_arrive_sample:
                            self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] = 1
                            self.path_info = [index+1, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
                            if not config.home_debug:
                                # 开始抽水并等待
                                self.server_data_obj.mqtt_send_get_obj.b_draw = 1
                                self.pi_main_obj.stop()
                                self.draw()
                                time.sleep(config.draw_time)
                                self.b_draw_over_send_data = True
                                self.draw()
                        # break
                        if self.b_stop_path_track:
                            break
                    if self.b_stop_path_track:
                        break
                # 全部结束后停止
                end_distance = self.run_distance
                try:
                    self.distance_move_score = round(100 * self.totle_distance/(end_distance - start_distance), 1)
                except Exception as e:
                    self.logger.error({'error': e})
                # 清除状态
                self.totle_distance = 0
                self.clear_status()
                # 检查是否需要返航
                if self.ship_status == ShipStatus.backhome:
                    self.back_home()

    # 读取函数会阻塞 必须使用线程发送mqtt状态数据和检测数据
    def send_mqtt_data(self):
        last_read_time = time.time()
        last_runtime = None
        last_run_distance = None
        while True:
            time.sleep(config.pi2mqtt_interval)
            if self.server_data_obj.mqtt_send_get_obj.pool_code:
                self.data_define_obj.pool_code = copy.deepcopy(self.server_data_obj.mqtt_send_get_obj.pool_code)
            status_data = copy.deepcopy(self.data_define_obj.status)
            status_data.update({'mapId': self.data_define_obj.pool_code})
            detect_data = copy.deepcopy(self.data_define_obj.detect)
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            # 更新经纬度为高德经纬度
            if config.home_debug:
                if self.lng_lat is None:
                    self.lng_lat = config.ship_gaode_lng_lat
            if self.lng_lat is not None and self.use_true_gps:
                current_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
            elif self.lng_lat is not None and not self.use_true_gps:
                current_lng_lat = self.lng_lat
            else:
                current_lng_lat = None

            if current_lng_lat is not None:
                status_data.update({'current_lng_lat': current_lng_lat})
            elif current_lng_lat is None and self.second_lng_lat is not None:
                current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.second_lng_lat)
                status_data.update({'current_lng_lat': current_gaode_lng_lat})
            if self.home_lng_lat is not None:
                if config.home_debug:
                    home_gaode_lng_lat = self.home_lng_lat
                else:
                    home_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.home_lng_lat)
                status_data.update({'home_lng_lat': home_gaode_lng_lat})
            # 更新速度  更新里程
            if self.speed is not None:
                status_data.update({'speed': self.speed})
            status_data.update({"runtime": round(time.time() - self.start_time)})
            status_data.update({"run_distance": round(self.run_distance, 1)})
            if last_runtime is None:
                last_runtime = 0
            if last_run_distance is None:
                last_run_distance = 0
            if os.path.exists(config.run_distance_time_path):
                try:
                    with open(config.run_distance_time_path, 'r') as f:
                        run_distance_time_data = json.load(f)
                        save_distance = run_distance_time_data.get('save_distance')
                        save_time = run_distance_time_data.get('save_time')
                except Exception as e:
                    save_distance = 0
                    save_time = 0
                    self.logger.error({'error': e})
            else:
                save_distance = 0
                save_time = 0
            with open(config.run_distance_time_path, 'w') as f:
                save_distance = save_distance + round(self.run_distance, 1) - last_run_distance
                save_time = save_time + round(time.time() - self.start_time) - last_runtime
                json.dump({'save_distance': save_distance,
                           'save_time': save_time}, f)
            last_runtime = round(time.time() - self.start_time)
            last_run_distance = round(self.run_distance, 1)
            status_data.update({"totle_distance": round(save_distance,1)})
            status_data.update({"totle_time": round(save_time)})

            # 更新船头方向
            if self.theta is not None:
                status_data.update({"direction": round(self.theta)})
            # 更新经纬度误差
            if self.lng_lat_error is not None:
                status_data.update({"lng_lat_error": self.lng_lat_error})

            # 更新真实数据
            if self.b_draw_over_send_data:
                mqtt_send_detect_data = copy.deepcopy(detect_data)
                mqtt_send_detect_data['water'].update(self.water_data_dict)
                mqtt_send_status_data = copy.deepcopy(status_data)
            # 更新模拟数据
            else:
                mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
                mqtt_send_status_data = data_define.fake_status_data(status_data)
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                    mqtt_send_detect_data[k_all].update({new_key: pop_value})
            if self.dump_energy is not None:
                self.dump_energy_deque.append(self.dump_energy)
                mqtt_send_status_data.update({'dump_energy': self.dump_energy})
            # 向mqtt发送数据
            self.send(method='mqtt', topic='status_data_%s' % (config.ship_code), data=mqtt_send_status_data,
                      qos=0)
            if time.time() - last_read_time > 10:
                last_read_time = time.time()
                self.data_save_logger.info({"发送状态数据": mqtt_send_status_data})
            if config.home_debug:
                if time.time() % 10 < 1:
                    self.send(method='mqtt', topic='detect_data_%s' % (config.ship_code), data=mqtt_send_detect_data,
                              qos=0)
                    self.logger.info({'fakedate': mqtt_send_detect_data})

            if self.b_draw_over_send_data:
                self.send(method='mqtt', topic='detect_data_%s' % (config.ship_code), data=mqtt_send_detect_data,
                          qos=0)
                # 添加真实经纬度
                save_detect_data = copy.deepcopy(mqtt_send_detect_data)
                save_detect_data.update({'lng_lat': self.lng_lat})
                self.logger.info({"本地保存检测数据": save_detect_data})
                if len(self.data_define_obj.pool_code) > 0:
                    self.send(method='http', data=mqtt_send_detect_data,
                              url=config.http_data_save,
                              http_type='POST')
                    self.data_save_logger.info({"发送检测数据": mqtt_send_detect_data})
                # 发送结束改为False
                self.b_draw_over_send_data = False

            # 客户端获取基础设置数据
            if self.server_data_obj.mqtt_send_get_obj.base_setting_data_info in [1, 4]:
                if self.server_data_obj.mqtt_send_get_obj.base_setting_data is None:
                    self.logger.error(
                        {'base_setting_data is None': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
                else:
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data.update({'info_type': 3})
                    self.send(method='mqtt', topic='base_setting_%s' % (config.ship_code),
                              data=self.server_data_obj.mqtt_send_get_obj.base_setting_data,
                              qos=0)
                    self.logger.info({'base_setting': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data = None
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data_info = 0

            # 客户端获取高级设置数据
            if self.server_data_obj.mqtt_send_get_obj.height_setting_data_info in [1, 4]:
                if self.server_data_obj.mqtt_send_get_obj.height_setting_data is None:
                    self.logger.error(
                        {'height_setting_data is None': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                else:
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data.update({'info_type': 3})
                    self.send(method='mqtt', topic='height_setting_%s' % (config.ship_code),
                              data=self.server_data_obj.mqtt_send_get_obj.height_setting_data,
                              qos=0)
                    self.logger.info({'height_setting': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data = None
                    # 改为0位置状态，不再重复发送
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data_info = 0

    def send(self, method, data, topic='test', qos=0, http_type='POST', url=''):
        """
        :param url:
        :param http_type:
        :param qos:
        :param topic:
        :param data: 发送数据
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type, data, url)
            self.logger.debug({'请求 url': url, 'status_code': return_data.status_code})
            # 如果是POST返回的数据，添加数据到地图数据保存文件中
            if http_type == 'POST' and r'map/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.info({'map/save content_data success': content_data["success"]})
                if not content_data["success"]:
                    self.logger.error('POST请求发送地图数据失败')
                # POST 返回湖泊ID
                pool_id = content_data['data']['id']
                return pool_id
            # http发送检测数据给服务器
            elif http_type == 'POST' and r'data/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.debug({'data/save content_data success': content_data["success"]})
                if not content_data["success"]:
                    self.logger.error('POST发送检测请求失败')
            elif http_type == 'GET' and r'device/binding' in url:
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                save_data = content_data["data"]
                return save_data
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                save_data = content_data["data"]["mapList"]
                return save_data
        elif method == 'mqtt':
            self.server_data_obj.send_server_mqtt_data(data=data, topic=topic, qos=qos)

    # 状态检查函数，检查自身状态发送对应消息
    def check_status(self):
        while True:
            # 循环等待一定时间
            time.sleep(config.check_status_interval)
            # 检查当前状态
            if config.home_debug:
                if config.b_play_audio:
                    audios_manager.play_audio(5, b_backend=False)
            # 检查网络
            if config.b_check_network:
                if not check_network.check_network():
                    if config.b_play_audio:
                        audios_manager.play_audio(2, b_backend=False)
                    self.logger.error('当前无网络信号')
                else:
                    self.logger.info('当前网络正常...')
            if self.last_lng_lat:
                ship_theta = lng_lat_calculate.angleFromCoordinate(self.last_lng_lat[0],
                                                                   self.last_lng_lat[1],
                                                                   self.lng_lat[0],
                                                                   self.lng_lat[1])
            else:
                ship_theta = 0
            # 求船头与目标角度偏差角度
            # 都为空直接不要误差
            if config.use_shape_theta_type == 1:
                self.current_theta = self.theta
            elif config.use_shape_theta_type == 2:
                self.current_theta = self.theta1
            else:
                self.current_theta = ship_theta

            # 检查电量 如果连续20次检测电量平均值低于电量阈值就报警
            if config.energy_backhome:
                if sum(list(self.dump_energy_deque)) / len(list(self.dump_energy_deque)) < config.energy_backhome:
                    self.low_dump_energy_warnning = 1
                else:
                    self.low_dump_energy_warnning = 0
            # 接收到重置湖泊按钮
            if self.server_data_obj.mqtt_send_get_obj.reset_pool_click:
                self.data_define_obj.pool_code = ''
                self.server_data_obj.mqtt_send_get_obj.pool_code = ''
                self.server_data_obj.mqtt_send_get_obj.reset_pool_click = 0
            # 船状态提示消息
            notice_info_data = {
                "distance": str(round(self.distance_p,2)) + ' s ' + str(self.distance_move_score),
                # // 路径规划提示消息
                "path_info": '当前目标点:%d 目标点总数: %d' % (int(self.path_info[0]), int(self.path_info[1])),
                # 船执行手动控制信息
                "control_info": self.control_info,
                # 水泵开关状态消息
                "draw_info": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 自动巡航下角度偏差
                "theta_error": round(self.theta_error,2),
            }
            notice_info_data.update({"mapId": self.data_define_obj.pool_code})
            # 遥控器是否启用
            if config.current_platform == config.CurrentPlatform.pi:
                notice_info_data.update({"b_start_remote": self.pi_main_obj.b_start_remote})
            # 罗盘提示消息
            if len(self.compass_notice_info) > 3:
                notice_info_data.update({"compass_notice_info": self.compass_notice_info+self.compass_notice_info1})
            # 使用超声波时候更新超声波提示消息
            if config.b_use_ultrasonic:
                notice_info_data.update({"ultrasonic_distance": str(self.l_distance) + '  ' + str(self.r_distance)})
            # 使用电量告警是提示消息
            if self.low_dump_energy_warnning:
                notice_info_data.update({"low_dump_energy_warnning": self.low_dump_energy_warnning})
            self.send(
                method='mqtt',
                topic='notice_info_%s' % (config.ship_code),
                data=notice_info_data,
                qos=0)
            if time.time() % 10 < config.check_status_interval:
                self.logger.info({'notice_info_': notice_info_data})

            # 保存数据与发送刷新后提示消息
            if len(self.data_define_obj.pool_code) > 0:
                save_plan_path_data = {
                    "mapId": self.data_define_obj.pool_code,
                    'sampling_points': self.server_data_obj.mqtt_send_get_obj.sampling_points,
                    'path_points': self.server_data_obj.mqtt_send_get_obj.path_planning_points,
                }
                save_data.set_data(save_plan_path_data, config.save_plan_path)
                if self.server_data_obj.mqtt_send_get_obj.refresh_info_type == 1:
                    save_plan_path_data.update({"info_type": 2})
                    self.send(method='mqtt',
                              topic='refresh_%s' % (config.ship_code),
                              data=save_plan_path_data,
                              qos=0)


if __name__ == '__main__':
    obj = DataManager()
    obj.send_mqtt_data()
    while True:
        # move_direction = obj.server_data_obj.mqtt_send_get_obj.control_move_direction
        # obj.logger.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位
