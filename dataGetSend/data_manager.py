"""
管理数据收发
"""
import time
import json
import copy
import random
import math
from dataGetSend.data_define import DataDefine
from dataGetSend import data_define
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData
from dataGetSend import drone_kit_control
from utils import check_network
from utils.log import LogHandler
from baiduMap import baidu_map
from audios import audios_manager
from utils import lng_lat_calculate
import config
from piControl import pi_main

class DataManager:
    def __init__(self):
        self.data_define_obj = DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log', level=20)
        self.data_save_logger = LogHandler('data_save_log', level=20)
        self.com_data_read_logger = LogHandler('com_data_read_logger', level=20)
        self.com_data_send_logger = LogHandler('com_data_send_logger', level=20)
        self.server_log = LogHandler('server_data')
        self.map_log = LogHandler('map_log')
        # self.drone_log = LogHandler('drone_log')

        if config.current_platform == 'l':
            # 串口数据收发对象
            self.com_data_obj = self.get_serial_obj(port=config.port, baud=config.baud,
                                                    time_out=1.0 / config.com2pi_interval,
                                                    log=self.com_data_read_logger)
            self.pi_main_obj = pi_main.PiMain()

            if config.b_use_pix:
                self.drone_obj = drone_kit_control.DroneKitControl(config.pix_port)
                self.drone_obj.download_mission(True)
                self.drone_obj.arm()

        # mqtt服务器数据收发对象
        self.server_data_obj = ServerData(self.server_log, topics=self.data_define_obj.topics)

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
        self.current_target_gaode_lng_lats = None

        # 记录当前发送给单片机目标点经纬度（避免重复发送）
        self.send_target_gaode_lng_lat = None

        # 路径跟踪是否被终止
        self.b_stop_path_track = False

        # 返回状态消息中提示消息
        self.notice_info = ''

        # 记录里程与时间数据
        self.totle_distance = 0
        self.plan_start_time = None

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
        self.current_b_draw=None
        # 抽水开始时间
        self.draw_start_time = None
        # 抽水结束发送数据
        self.b_draw_over_send_data = False
        # 单片机串口读取到的数据
        self.second_lng_lat = None
        self.water_data_dict = {}
        # 剩余电量
        self.dump_energy = None

    def get_serial_obj(self, port, baud, time_out=None, log=None):
        return SerialData(port, baud, timeout=1 / config.com2pi_interval,
                          logger=self.com_data_read_logger)

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
                    self.second_lng_lat = [lng,lat]
                    if time.time() - last_read_time > 3:
                        last_read_time = time.time()
                        self.com_data_read_logger.info({'second_lng_lat': self.second_lng_lat})
                elif com_data_read.startswith('BB'):
                    com_data_list = com_data_read.split(',')
                    self.water_data_dict.update({'EC':float(com_data_list[1].split(':')[1])/math.pow(10,int(com_data_list[7][3:]))})
                    self.water_data_dict.update({'DO':float(com_data_list[0][2:].split(':')[1])/math.pow(10,int(com_data_list[6][3:]))})
                    self.water_data_dict.update({'TD':float(com_data_list[2].split(':')[1])/math.pow(10,int(com_data_list[8][3:]))})
                    self.water_data_dict.update({'pH':float(com_data_list[3].split(':')[1])/math.pow(10,int(com_data_list[9][3:]))})
                    self.water_data_dict.update({'wt':float(com_data_list[4].split(':')[1])})
                    self.dump_energy = float(com_data_list[5].split(':')[1])
                    if time.time() - last_read_time > 3:
                        last_read_time = time.time()
                        self.com_data_read_logger.info({'str com_data_read': com_data_read})
                        self.com_data_read_logger.info({'water_data_dict': self.water_data_dict})
                    # self.com_data_read_logger.info({'dump_energy': self.dump_energy})
            except Exception as e:
                self.com_data_read_logger.error({'串口数据解析错误': e})

    def draw(self):
        # 判断是否抽水
        if self.server_data_obj.mqtt_send_get_obj.b_draw:
            if self.draw_start_time is None or self.current_b_draw == 0:
                if self.draw_start_time is not None:
                    # 超时中断抽水
                    if time.time()-self.draw_start_time>config.draw_time:
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
                if time.time() - self.draw_start_time > config.draw_time:
                    self.b_draw_over_send_data = True
                    self.com_data_obj.send_data('A0Z')
                    self.current_b_draw = 0
                    self.server_data_obj.mqtt_send_get_obj.b_draw=0
                    self.draw_start_time = None
        else:
            if self.current_b_draw is None or self.current_b_draw == 1:
                self.com_data_obj.send_data('A0Z')
                self.current_b_draw = 0
                self.draw_start_time = None


                # 发送函数会阻塞 必须使用线程

    def clear_status(self):
        """
        按了暂停或者不满足开始状态时候清除状态
        :return:
        """
        self.pi_main_obj.pi_obj.stop()
        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
        # 清空目标点和状态
        self.server_data_obj.mqtt_send_get_obj.path_planning_points = []
        self.server_data_obj.mqtt_send_get_obj.path_planning_points_status = []
        # 清空里程和时间
        self.totle_distance = 0
        self.plan_start_time = None
        # 清空提醒
        self.path_info = [0, 0]
        self.distance_p = 0

    def send_com_data(self):
        # 0 自动  1手动
        manul_or_auto = 1
        # 记录上次手动发送
        last_control = None

        try:
            while True:
                time.sleep(config.pi2com_interval)
                if config.b_use_ultrasonic:
                    self.logger.info({'left_distance':self.pi_main_obj.left_distance,'right_distance':self.pi_main_obj.left_distance})
                # 判断当前是手动控制还是自动控制
                d = int(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
                if d in [-1, 0, 90, 180, 270]:
                    manul_or_auto = 1
                # 使用路径规划
                if len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                    manul_or_auto = 0
                    # 此时清除d
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2

                if manul_or_auto == 1:
                    # 收到停止终止所有路径规划点
                    if d == -1:
                        self.server_data_obj.mqtt_send_get_obj.path_planning_points = []
                        self.server_data_obj.mqtt_send_get_obj.path_planning_points_status = []
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
                            self.pi_main_obj.pi_obj.forward()
                        elif d == 90:
                            self.control_info = '向左'
                            self.pi_main_obj.pi_obj.left()
                        elif d == 180:
                            self.control_info = '向后'
                            self.pi_main_obj.pi_obj.backword()
                        elif d == 270:
                            self.control_info = '向右'
                            self.pi_main_obj.pi_obj.right()
                        elif d == -1:
                            self.control_info = '停止'
                            self.pi_main_obj.pi_obj.stop()

                        # 改变状态不再重复发送指令
                        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    # 使用单片机
                    else:
                        if temp_com_data is not None:
                            if last_control is None or last_control != temp_com_data:
                                last_control = temp_com_data
                                com_data_send = 'A5A5%d,0,0,0,0,0,0,0,0,0#' % temp_com_data
                                self.com_data_obj.send_data(com_data_send)
                                self.com_data_send_logger.info('com_data_send' + com_data_send)
                    # 手动模式下判断是否抽水
                    self.draw()

                # 自动模式计算角度
                if manul_or_auto == 0:
                    self.control_info = ''
                    # 使用树莓派
                    if config.b_use_pi:
                        if self.pi_main_obj.lng_lat is None and self.second_lng_lat is None:
                            self.logger.error('无当前GPS，不能自主巡航')
                            time.sleep(0.5)
                            self.clear_status()
                            continue
                        # 第一次进入路径规划时候的点设置为返航点
                        if self.pi_main_obj.home_lng_lat is None:
                            self.pi_main_obj.set_home_location()
                        if self.plan_start_time is None:
                            self.plan_start_time = time.time()
                        # 设置自动路径搜索为False
                        self.b_stop_path_track = False
                        self.logger.info({'点击地点': self.server_data_obj.mqtt_send_get_obj.path_planning_points})
                        if self.pi_main_obj.lng_lat is None and self.second_lng_lat is not None:
                            pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.second_lng_lat)
                        else:
                            pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.pi_main_obj.lng_lat)
                        start_distance = self.pi_main_obj.run_distance
                        # 计算总里程
                        for index, gaode_lng_lat in enumerate(
                                self.server_data_obj.mqtt_send_get_obj.path_planning_points):
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
                        for index, gaode_lng_lat in enumerate(
                                self.server_data_obj.mqtt_send_get_obj.path_planning_points):
                            self.path_info = [index, len(self.server_data_obj.mqtt_send_get_obj.path_planning_points)]
                            # 判断该点是否已经到达
                            if self.server_data_obj.mqtt_send_get_obj.path_planning_points_status[index] == 1:
                                continue
                            # 计算目标真实经纬度
                            # 将目标经纬度转换为真实经纬度
                            pi_current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.pi_main_obj.lng_lat)
                            path_planning_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.pi_main_obj.lng_lat,
                                                                                         pi_current_gaode_lng_lat,
                                                                                         gaode_lng_lat)
                            distance = lng_lat_calculate.distanceFromCoordinate(
                                self.pi_main_obj.lng_lat[0],
                                self.pi_main_obj.lng_lat[1],
                                path_planning_point_gps[0],
                                path_planning_point_gps[1])
                            self.logger.info({'目标真实GPS': path_planning_point_gps, 'distance': distance})
                            while distance > config.arrive_distance:
                                distance = lng_lat_calculate.distanceFromCoordinate(
                                    self.pi_main_obj.lng_lat[0],
                                    self.pi_main_obj.lng_lat[1],
                                    path_planning_point_gps[0],
                                    path_planning_point_gps[1])
                                # 更新提醒的距离信息
                                self.distance_p = distance
                                if int(time.time()) % 1 == 0:
                                    self.logger.debug({'距离目标点距离: ': distance})
                                left_pwm, right_pwm = self.pi_main_obj.point_control(path_planning_point_gps)
                                # print('left_pwm, right_pwm',left_pwm, right_pwm)
                                self.pi_main_obj.pi_obj.set_pwm(left_pwm, right_pwm,pwm_timeout=config.pid_interval)
                                # 按了暂停按钮 清空规划点
                                # print(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
                                if int(self.server_data_obj.mqtt_send_get_obj.control_move_direction) == -1:
                                    self.clear_status()
                                    # 记录是因为按了暂停按钮而终止
                                    self.b_stop_path_track = True
                                    break
                                # time.sleep(config.pid_interval)
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points_status[index] = 1
                            if self.b_stop_path_track:
                                break
                            # 开始抽水并等待
                            self.server_data_obj.mqtt_send_get_obj.b_draw=1
                            self.pi_main_obj.pi_obj.stop()
                            self.draw()
                            time.sleep(config.draw_time)
                            self.b_draw_over_send_data = True
                            self.draw()

                        # 全部结束后停止
                        end_distance = self.pi_main_obj.run_distance
                        try:
                            self.distance_move_score = round(self.distance_p/(end_distance-start_distance),2)
                        except Exception as e:
                            self.logger.error({'error':e})
                        self.clear_status()

                    else:
                        for index, value in enumerate(self.plan_path_points_status):
                            if self.plan_path_points_status[index] == 1:
                                continue
                            self.current_index = index
                            self.current_index = 1

                        # 计算改目标点是否已达
                        # 无GPS信号
                        if self.data_define_obj.status['current_lng_lat'] is None:
                            self.com_data_send_logger.info('data_define_obj.status current_lng_lat is None')

                        if self.send_target_gaode_lng_lat is None or self.send_target_gaode_lng_lat != self.plan_path[
                            self.current_index]:
                            current_lng_lat = self.data_define_obj.status['current_lng_lat']
                            current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(current_lng_lat)
                            # 计算目标真实经纬度
                            path_planning_point_gps = lng_lat_calculate.gps_gaode_to_gps(current_lng_lat,
                                                                                         current_gaode_lng_lat,
                                                                                         self.plan_path[1])
                            # 使用飞控
                            if config.b_use_pix:
                                # 计算目标点相对于当前点的距离
                                x, y = lng_lat_calculate.get_x_y_distance(current_gaode_lng_lat,
                                                                          self.plan_path[1])
                                # self.drone_obj.goto(x, y)
                                self.com_data_send_logger.info({'target_x': x, 'target_y': y})

                            else:
                                self.com_data_send_logger.info(
                                    {'send_target_lng_lat_gps': self.send_target_gaode_lng_lat,
                                     'path_planning_point_gps': path_planning_point_gps})

                                com_data_send = 'A5A50,0,%f,%f,0,0,0,0,0,0#' % (
                                    path_planning_point_gps[0], path_planning_point_gps[1])
                                self.com_data_obj.send_data(com_data_send)
                                self.com_data_send_logger.info('com_data_send: ' + str(com_data_send))
                                # 记录上次发送经纬度 如果一样则不发送
                                self.send_target_gaode_lng_lat = copy.deepcopy(self.plan_path[self.current_index])
                        # 重置手动控制
                        self.server_data_obj.mqtt_send_get_obj.control_move_direction = str(360)
                        last_control = None
        except KeyboardInterrupt:
            self.com_data_obj.send_data('A5A55,0,0,0,0,0,0,0,0,0#')
        except Exception as e:
            self.com_data_send_logger.error({'error': e})

    # 读取函数会阻塞 必须使用线程
    # 发送mqtt状态数据和检测数据
    def send_mqtt_data(self):
        while True:
            time.sleep(config.pi2mqtt_interval)
            if self.server_data_obj.mqtt_send_get_obj.pool_id is not None:
                self.data_define_obj.pool_code = copy.deepcopy(self.server_data_obj.mqtt_send_get_obj.pool_id)
            status_data = copy.deepcopy(self.data_define_obj.status)
            status_data.update({'mapId': self.data_define_obj.pool_code})

            detect_data = copy.deepcopy(self.data_define_obj.detect)
            detect_data.update({'mapId': self.data_define_obj.pool_code})

            # 更新经纬度为高德经纬度
            if not config.home_debug:
                current_lng_lat = self.pi_main_obj.lng_lat
                if current_lng_lat is not None:
                    current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(current_lng_lat)
                    status_data.update({'current_lng_lat': current_gaode_lng_lat})
                elif current_lng_lat is None and self.second_lng_lat is not None:
                    current_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.second_lng_lat)
                    status_data.update({'current_lng_lat': current_gaode_lng_lat})

                if self.pi_main_obj.home_lng_lat is not None:
                    home_gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.pi_main_obj.home_lng_lat)
                    status_data.update({'home_lng_lat': home_gaode_lng_lat})

                # 更新速度  更新里程
                if self.pi_main_obj.speed is not None:
                    status_data.update({'speed': self.pi_main_obj.speed})
                status_data.update({"totle_time": round(self.totle_distance/(2.0))})
                status_data.update({"run_distance": round(self.totle_distance, 1)})
                status_data.update({"totle_distance": round(self.pi_main_obj.run_distance, 1)})
                if self.plan_start_time is not None:
                    status_data.update({"runtime": round(time.time() - self.plan_start_time)})
                # 更新船头方向
                if self.pi_main_obj.theta is not None:
                    status_data.update({"direction": round(self.pi_main_obj.theta)})
                # 更新经纬度误差
                if self.pi_main_obj.lng_lat_error is not None:
                    status_data.update({"lng_lat_error": self.pi_main_obj.lng_lat_error})

                # 更新真实数据
                if config.ship_code.endswith('a') and self.b_draw_over_send_data:
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
                    mqtt_send_status_data.update({'dump_energy': self.dump_energy})
                # 向mqtt发送数据
                self.send(method='mqtt', topic='status_data_%s' % (config.ship_code), data=mqtt_send_status_data,
                          qos=0)
                # self.data_save_logger.info({"发送状态数据": mqtt_send_status_data})
                if self.b_draw_over_send_data:
                    # self.pi_main_obj.lng_lat 添加真实经纬度
                    self.send(method='mqtt', topic='detect_data_%s' % (config.ship_code), data=mqtt_send_detect_data,
                              qos=0)
                    save_detect_data = copy.deepcopy(mqtt_send_detect_data)
                    save_detect_data.update({'lng_lat':self.pi_main_obj.lng_lat})
                    self.logger.info({"本地保存检测数据":save_detect_data})
                    if len(self.data_define_obj.pool_code) > 0:
                        self.send(method='http', data=mqtt_send_detect_data,
                                  url=config.http_data_save,
                                  http_type='POST')
                        self.data_save_logger.info({"发送检测数据": mqtt_send_detect_data})
                    # 发送结束改为False
                    self.b_draw_over_send_data = False

            # 客户端获取基础设置数据
            if self.server_data_obj.mqtt_send_get_obj.base_setting_data_info in [1,4]:
                if self.server_data_obj.mqtt_send_get_obj.base_setting_data is None:
                    self.logger.error(
                        {'base_setting_data is None': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
                else:
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data.update({'info_type':3})
                    self.send(method='mqtt', topic='base_setting_%s' % (config.ship_code), data=self.server_data_obj.mqtt_send_get_obj.base_setting_data,
                              qos=0)
                    self.logger.info({'base_setting':self.server_data_obj.mqtt_send_get_obj.base_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data = None
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data_info=0
            # 客户端获取高级设置数据
            if self.server_data_obj.mqtt_send_get_obj.height_setting_data_info in [1,4]:
                if self.server_data_obj.mqtt_send_get_obj.height_setting_data is None:
                    self.logger.error({'height_setting_data is None':self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                else:
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data.update({'info_type': 3})
                    self.send(method='mqtt', topic='height_setting_%s' % (config.ship_code), data=self.server_data_obj.mqtt_send_get_obj.height_setting_data,
                              qos=0)
                    self.logger.info({'height_setting': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data = None
                    # 改为0位置状态，不再重复发送
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data_info=0

    def send(self, method, data, topic='test', qos=0, http_type='POST', url=''):
        """
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
                self.data_define_obj.status['current_lng_lat'] = config.init_gaode_gps

                if config.b_play_audio:
                    audios_manager.play_audio(5, b_backend=False)
                # self.logger.error('当前GPS信号弱')
            # if not check_network.check_network():
            #     if config.b_play_audio:
            #         audios_manager.play_audio(2, b_backend=False)
            #     self.logger.error('当前无网络信号')
            notice_info_data = {
                # // 船状态提示消息
                "distance": str(self.distance_p)+ '  '+str(self.distance_move_score),
                # // 路径规划提示消息
                "path_info": '当前目标点:%d 目标点总数: %d' % (int(self.path_info[0]), int(self.path_info[1])),
                # 船执行手动控制信息
                "control_info": self.control_info,
                # 水泵开关状态消息
                "draw_info": self.server_data_obj.mqtt_send_get_obj.b_draw,
            }

            self.send(
                method='mqtt',
                topic='notice_info_%s' % (config.ship_code),
                data=notice_info_data,
                qos=0)

            # 检查确认航线
            if config.b_check_path_planning:
                if self.server_data_obj.mqtt_send_get_obj.path_id is not None and self.server_data_obj.mqtt_send_get_obj.confirm_index is not None:
                    try:
                        self.plan_path_status[self.server_data_obj.mqtt_send_get_obj.path_id] = 1
                        # 确认航线就启动
                        if self.server_data_obj.mqtt_send_get_obj.confirm_index == 1:
                            pass
                    except Exception as e:
                        self.logger.error({'非法的航线确认ID': self.server_data_obj.mqtt_send_get_obj.path_id})

            # 获取初始地点GPS
            if self.server_data_obj.mqtt_send_get_obj.b_start:
                pass

    # 定时发送给单片机数据
    def send_com_heart_data(self):
        if self.baidu_map_obj is None:
            time.sleep(config.com_heart_time)
        else:
            if self.baidu_map_obj.init_ship_gps is not None:
                self.com_data_obj.send_data(
                    'B6B6,%f,%f#' % (self.baidu_map_obj.init_ship_gps[0], self.baidu_map_obj.init_ship_gps[1]))
                time.sleep(config.com_heart_time)

if __name__ == '__main__':
    obj = DataManager()
    obj.send_mqtt_data()
    while True:
        # move_direction = obj.server_data_obj.mqtt_send_get_obj.control_move_direction
        # obj.logger.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位
