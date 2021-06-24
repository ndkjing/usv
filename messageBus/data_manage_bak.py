# """
# 管理数据收发
# """
# import time
# import json
# import copy
# import os
# import math
# import enum
# import numpy as np
# import random
# from collections import deque
#
# from messageBus import data_define
# from externalConnect import server_data
# from drivers import com_data
# from utils.log import LogHandler
# from externalConnect import baidu_map
# from drivers import audios_manager, drone_kit_control, pi_main
# from utils import lng_lat_calculate
# from utils import check_network
# from utils import resolve_gps_data
# from utils import data_valid
# from storage import save_data
# import config
# from moveControl.pathTrack import simple_pid
# from dataAnalyze import utils
#
#
# class RelayType(enum.Enum):
#     """
#     控制继电器种类
#     headlight 前大灯
#     audio_light 声光报警器
#     side_light 舷灯
#     """
#     headlight = 0
#     audio_light = 1
#     side_light = 2
#
#
# class ShipStatus(enum.Enum):
#     """
#     船状态
#     idle  等待状态
#     remote_control 遥控器控制
#     computer_control 页面手动控制
#     auto 自动控制
#     """
#     idle = 0
#     remote_control = 1
#     computer_control = 2
#     auto = 3
#     backhome = 4
#
#
# class LightStatus(enum.Enum):
#     """
#     当前状态灯的状态
#     """
#     red = 0
#     green = 1
#     blue = 2
#     buzzer = 3
#
#
# class DataManager:
#     def __init__(self):
#         self.data_define_obj = data_define.DataDefine()
#         # 日志对象
#         self.logger = LogHandler('data_manager_log', level=20)
#         self.data_save_logger = LogHandler('data_save_log', level=20)
#         self.com_data_read_logger = LogHandler('com_data_read_logger', level=20)
#         self.com_data_send_logger = LogHandler('com_data_send_logger', level=20)
#         self.server_log = LogHandler('server_data')
#         self.map_log = LogHandler('map_log')
#         self.gps_log = LogHandler('gps_log')
#         self.compass_log = LogHandler('compass_log')
#         self.compass_log1 = LogHandler('compass_log1')
#         self.path_track_obj = simple_pid.SimplePid()
#
#         # mqtt服务器数据收发对象
#         self.server_data_obj = server_data.ServerData(self.server_log, topics=self.data_define_obj.topics)
#         # 路径跟踪是否被终止
#         self.b_stop_path_track = False
#         # 记录里程与时间数据
#         self.totle_distance = 0
#         self.plan_start_time = None
#         # 开机时间
#         self.start_time = time.time()
#         # 提示消息
#         # 距离下一个目标点距离
#         self.distance_p = 0
#         # 路径结束计算得分
#         self.distance_move_score = 0
#         # 目标点信息
#         self.path_info = [0, 0]
#         # 当前是否抽水
#         self.current_b_draw = 0
#         # 抽水开始时间
#         self.draw_start_time = None
#         # 抽水结束发送数据
#         self.b_draw_over_send_data = False
#         # 单片机串口读取到的数据
#         self.second_lng_lat = None
#         self.water_data_dict = {}
#         # 剩余电量
#         self.dump_energy = None
#         # 求剩余电量均值
#         self.dump_energy_deque = deque(maxlen=20)
#         # 电量告警 是否是低电量返航
#         self.low_dump_energy_warnning = 0
#         # 是否是断网返航
#         self.b_network_backhome = 0
#         # 是否已经返航在家
#         self.b_at_home = 0
#         # 返航点
#         self.home_lng_lat = None
#         # 返航点高德经纬度
#         self.home_gaode_lng_lat = None
#         # 船速度
#         self.speed = None
#         # 船行驶里程
#         self.run_distance = 0
#         # 经纬度 和 船头角度 北为0 逆时针为正
#         self.lng_lat = None
#         # 船高德经纬度
#         self.gaode_lng_lat = None
#         self.lng_lat_error = None
#         # 罗盘角度
#         self.theta = None
#         # 船头角度
#         self.current_theta = None
#         # 偏差角度
#         self.theta_error = 0
#         # 罗盘提示消息
#         self.compass_notice_info = ''
#         self.compass_notice_info1 = ''
#         # 记录pwm调节时间和数值用于在家调试
#         self.last_left_pwm = 1500
#         self.last_right_pwm = 1500
#         self.last_change_pwm_time = time.time()
#         # 记录上一次经纬度
#         self.last_lng_lat = None
#         # 记录船状态
#         self.ship_status = ShipStatus.idle
#         # 记录平滑路径
#         self.smooth_path_lng_lat = None
#         self.smooth_path_lng_lat_index = []
#         # 船手动控制提示信息
#         self.control_info = ''
#         # 网络延时
#         self.ping = 0
#         # 避障提示消息
#         self.obstacle_info = ''
#         self.lng_lat_list = []
#         self.deep_list = []
#         # 记录上一次控制的单片机继电器状态
#         self.last_side_light = 0
#         self.last_headlight = 0
#         self.last_audio_light = 0
#         self.last_status_light = 0
#         # 状态灯
#         self.light_status = LightStatus.red
#         if config.current_platform == config.CurrentPlatform.pi:
#             # 串口数据收发对象
#             if os.path.exists(config.stc_port):
#                 self.com_data_obj = self.get_com_obj(port=config.stc_port, baud=config.stc_baud,
#                                                      timeout=config.stc2pi_timeout,
#                                                      logger=self.com_data_read_logger)
#             if os.path.exists(config.gps_port):
#                 self.gps_obj = self.get_com_obj(config.gps_port, config.gps_baud, self.gps_log)
#             if os.path.exists(config.compass_port):
#                 self.compass_obj = self.get_com_obj(config.compass_port, config.compass_baud, self.compass_log)
#             self.pi_main_obj = pi_main.PiMain()
#             if config.b_use_pix:
#                 self.drone_obj = drone_kit_control.DroneKitControl(config.pix_port)
#                 self.drone_obj.download_mission(True)
#                 self.drone_obj.arm()
#             self.pi_main_obj.init_motor()
#         # 使用真实GPS 还是 初始化高德GPS
#         self.use_true_gps = 0
#         if os.path.exists(config.gps_port) or config.b_pin_gps:
#             self.use_true_gps = 1
#         # 当前路径平滑搜索列表
#         self.search_list = []
#
#     # 读取函数会阻塞 必须使用线程
#     def get_com_data(self):
#         last_read_time = time.time()
#         while True:
#             # 水质数据 b''BBD:0,R:158,Z:36,P:0,T:16.1,V:92.08,x1:0,x2:2,x3:1,x4:0\r\n'
#             # 经纬度数据 b'AA2020.2354804,N,11425.41234568896,E\r\n'
#             try:
#                 time.sleep(0.5)
#                 row_com_data_read = self.com_data_obj.readline()
#                 print('row_com_data_read', row_com_data_read)
#                 # 如果数据不存在或者数据过短都跳过
#                 if row_com_data_read:
#                     if len(str(row_com_data_read)) < 7:
#                         continue
#                 else:
#                     continue
#                 com_data_read = str(row_com_data_read)[2:-5]
#                 if time.time() - last_read_time > 3:
#                     last_read_time = time.time()
#                     self.com_data_read_logger.info({'str com_data_read': com_data_read})
#                 # 解析串口发送过来的数据
#                 if com_data_read is None:
#                     continue
#                 # 读取数据过短跳过
#                 if len(com_data_read) < 5:
#                     continue
#                 if com_data_read.startswith('AA'):
#                     com_data_list = com_data_read.split(',')
#                     lng, lat = round(float(com_data_list[2][:3]) +
#                                      float(com_data_list[2][3:]) /
#                                      60, 6), round(float(com_data_list[0][2:4]) +
#                                                    float(com_data_list[0][4:]) /
#                                                    60, 6)
#                     self.second_lng_lat = [lng, lat]
#                     if time.time() - last_read_time > 10:
#                         self.com_data_read_logger.info({'second_lng_lat': self.second_lng_lat})
#                 elif com_data_read.startswith('BB'):
#                     com_data_list = com_data_read.split(',')
#                     ec_data = float(com_data_list[1].split(':')[1]) / math.pow(10, int(com_data_list[7][3:]))
#                     ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
#                     self.water_data_dict.update({'EC': ec_data})
#                     do_data = float(com_data_list[0][2:].split(':')[1]) / math.pow(10, int(com_data_list[6][3:]))
#                     do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
#                     self.water_data_dict.update({'DO': do_data})
#                     td_data = float(com_data_list[2].split(':')[1]) / math.pow(10, int(com_data_list[8][3:]))
#                     td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
#                     self.water_data_dict.update({'TD': td_data})
#                     ph_data = float(com_data_list[3].split(':')[1]) / math.pow(10, int(com_data_list[9][3:]))
#                     ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
#                     self.water_data_dict.update({'pH': ph_data})
#                     wt_data = float(com_data_list[4].split(':')[1])
#                     wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
#                     self.water_data_dict.update({'wt': wt_data})
#                     self.dump_energy = float(com_data_list[5].split(':')[1])
#                     if time.time() - last_read_time > 5:
#                         last_read_time = time.time()
#                         self.com_data_read_logger.info({'str com_data_read': com_data_read})
#                         self.com_data_read_logger.info({'water_data_dict': self.water_data_dict})
#             except Exception as e:
#                 self.com_data_read_logger.error({'串口数据解析错误': e})
#
#     # 获取串口对象
#     @staticmethod
#     def get_com_obj(port, baud, logger, timeout=0.4):
#         return com_data.ComData(
#             port,
#             baud,
#             timeout=timeout,
#             logger=logger)
#
#     # 在线程中读取 gps
#     def get_gps_data(self):
#         last_read_time = None
#         while True:
#             try:
#                 data = self.gps_obj.readline()
#                 str_data = data.decode('ascii')
#                 gps_dict = resolve_gps_data.resolve_gps(str_data)
#                 lng = gps_dict.get('lng')
#                 lat = gps_dict.get('lat')
#                 lng_lat_error = gps_dict.get('lng_lat_error')
#                 if lng and lat:
#                     if lng >= 1 and lat >= 1:
#                         # 替换上一次的值
#                         self.last_lng_lat = copy.deepcopy(self.lng_lat)
#                         self.lng_lat = [lng, lat]
#                         self.lng_lat_error = lng_lat_error
#                         if not last_read_time:
#                             last_read_time = time.time()
#                         if self.lng_lat_error and self.lng_lat_error < 2.5:
#                             if self.last_lng_lat:
#                                 # 计算当前行驶里程
#                                 speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
#                                                                                           self.last_lng_lat[1],
#                                                                                           self.lng_lat[0],
#                                                                                           self.lng_lat[1])
#                                 self.run_distance += speed_distance
#                                 # 计算速度
#                                 self.speed = round(speed_distance / (time.time() - last_read_time), 1)
#                                 last_read_time = time.time()
#             except Exception as e:
#                 self.logger.error({'error': e})
#                 time.sleep(0.5)
#
#     # 在线程中读取罗盘
#     def get_compass_data(self):
#         # 累计50次出错则记为None
#         count = 50
#         # 记录上一次发送数据
#         last_send_data = None
#         while True:
#             try:
#                 # 检查罗盘是否需要校准 # 开始校准
#                 if int(config.calibration_compass) == 1:
#                     if last_send_data != 'C0':
#                         self.compass_obj.send_data('C0', b_hex=True)
#                         time.sleep(0.05)
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         last_send_data = 'C0'
#                 # 结束校准
#                 elif int(config.calibration_compass) == 2:
#                     if last_send_data != 'C1':
#                         self.compass_notice_info = ''
#                         self.compass_obj.send_data('C1', b_hex=True)
#                         time.sleep(0.05)
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         data0 = self.compass_obj.readline()
#                         str_data0 = data0.decode('ascii')
#                         if len(str_data0) > 3:
#                             self.compass_notice_info += str_data0
#                         last_send_data = 'C1'
#                         # 发送完结束校准命令后将配置改为 0
#                         config.calibration_compass = 0
#                         config.write_setting(b_height=True)
#                 # 隐藏修改 设置为10 改为启用gps计算角度  设置为9 改为启用二号罗盘
#                 elif int(config.calibration_compass) in [9, 10]:
#                     self.theta = None
#                 else:
#                     self.compass_obj.send_data('31', b_hex=True)
#                     time.sleep(0.05)
#                     data0 = self.compass_obj.readline()
#                     # 角度
#                     str_data0 = data0.decode('ascii')[:-3]
#                     if len(str_data0) < 1:
#                         continue
#                     float_data0 = float(str_data0)
#                     self.theta = 360 - float_data0
#                     time.sleep(config.compass_timeout / 4)
#                     count = 50
#                     last_send_data = None
#                     # self.compass_notice_info = ''
#             except Exception as e:
#                 if count > 0:
#                     count = count - 1
#                 else:
#                     self.logger.error({'error': e})
#                     count = 50
#                 time.sleep(1)
#
#     # 抽水
#     def draw(self):
#         """
#         接受mqtt抽水开始
#             当前在抽水：
#                 已经超时：
#                     停止抽水并更新发送数据标志位
#                 未超时：
#                     继续抽水
#             未抽水：
#                 开始抽水并开始计时
#         接受mqtt停止抽水：
#             当前在抽水：
#                 发送停止
#         :return:无
#         """
#         # 判断开关是否需要打开或者关闭
#         if config.home_debug:
#             pass
#         else:
#             # 判断是否抽水
#             if self.server_data_obj.mqtt_send_get_obj.b_draw:
#                 if self.draw_start_time is None or self.current_b_draw == 0:
#                     if self.draw_start_time is not None:
#                         # 超时中断抽水
#                         if time.time() - self.draw_start_time > config.draw_time:
#                             self.b_draw_over_send_data = True
#                             self.com_data_obj.send_data('A0Z')
#                             print('A0Z')
#                             self.current_b_draw = 0
#                             self.draw_start_time = None
#                     else:
#                         self.com_data_obj.send_data('A1Z')
#                         print('A1Z')
#                         self.current_b_draw = 1
#                         self.draw_start_time = time.time()
#                 elif self.draw_start_time is not None and self.current_b_draw == 1:
#                     # 超时中断抽水
#                     print(time.time() - self.draw_start_time)
#                     if time.time() - self.draw_start_time > config.draw_time:
#                         self.b_draw_over_send_data = True
#                         self.com_data_obj.send_data('A0Z')
#                         print('A0Z')
#                         self.current_b_draw = 0
#                         self.server_data_obj.mqtt_send_get_obj.b_draw = 0
#                         self.draw_start_time = None
#             else:
#                 if self.current_b_draw == 1:
#                     self.com_data_obj.send_data('A0Z')
#                     print('A0Z')
#                     self.current_b_draw = 0
#                     self.draw_start_time = None
#
#     def control_relay(self):
#         """
#         控制继电器
#         :return:
#         """
#         if self.server_data_obj.mqtt_send_get_obj.side_light:
#             if self.last_side_light:
#                 pass
#             else:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('B1Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('B1Z')
#         else:
#             if self.last_side_light:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('B0Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('B0Z')
#             else:
#                 pass
#         self.last_side_light = self.server_data_obj.mqtt_send_get_obj.side_light
#
#         if self.server_data_obj.mqtt_send_get_obj.headlight:
#             if self.last_headlight:
#                 pass
#             else:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('C1Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('C1Z')
#         else:
#             if self.last_headlight:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('C0Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('C0Z')
#             else:
#                 pass
#         self.last_headlight = self.server_data_obj.mqtt_send_get_obj.headlight
#
#         if self.server_data_obj.mqtt_send_get_obj.audio_light:
#             if self.last_audio_light:
#                 pass
#             else:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('D1Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('D1Z')
#         else:
#             if self.last_audio_light:
#                 if config.b_pin_stc:
#                     self.pi_main_obj.stc_obj.send_stc_data('D0Z')
#                 elif os.path.exists(config.stc_port):
#                     self.com_data_obj.send_data('D0Z')
#             else:
#                 pass
#         self.last_audio_light = self.server_data_obj.mqtt_send_get_obj.audio_light
#
#         if self.server_data_obj.mqtt_send_get_obj.status_light != self.last_status_light:
#             send_stc_data = 'E%sZ' % (str(self.server_data_obj.mqtt_send_get_obj.status_light))
#             if config.b_pin_stc:
#                 self.pi_main_obj.stc_obj.send_stc_data(send_stc_data)
#             elif os.path.exists(config.stc_port):
#                 self.com_data_obj.send_data(send_stc_data)
#         self.last_status_light = self.server_data_obj.mqtt_send_get_obj.status_light
#         if random.random() > 0.8:
#             self.last_status_light = 4
#
#     # 清除状态
#     def clear_status(self):
#         """
#         按了暂停或者不满足开始状态时候清除状态
#         :return:
#         """
#         if not config.home_debug:
#             self.pi_main_obj.stop()
#         self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
#         # 清空目标点和状态
#         self.server_data_obj.mqtt_send_get_obj.sampling_points = []
#         self.server_data_obj.mqtt_send_get_obj.path_planning_points = []
#         self.server_data_obj.mqtt_send_get_obj.sampling_points_status = []
#         self.server_data_obj.mqtt_send_get_obj.sampling_points_gps = []
#         self.smooth_path_lng_lat_index = []
#         # 清空里程和时间
#         self.totle_distance = 0
#         self.plan_start_time = None
#         # 清空提醒
#         self.path_info = [0, 0]
#         self.distance_p = 0
#         # 清空路径
#         self.smooth_path_lng_lat = None
#         self.server_data_obj.mqtt_send_get_obj.b_start = 0
#         self.server_data_obj.mqtt_send_get_obj.back_home = 0
#
#     # 检查是否需要返航
#     def check_backhome(self):
#         if config.network_backhome:
#             if int(config.network_backhome) > 600:
#                 network_backhome_time = int(config.network_backhome)
#             else:
#                 network_backhome_time = 600
#             if time.time() - self.server_data_obj.mqtt_send_get_obj.last_command_time > network_backhome_time:
#                 self.b_network_backhome = 1
#                 self.ship_status = ShipStatus.backhome
#                 self.back_home()
#         if self.low_dump_energy_warnning:
#             # 记录是因为按了低电量判断为返航
#             self.back_home()
#
#     def back_home(self):
#         """
#         紧急情况下返航
#         :return:
#         """
#         # 有返航点下情况下返回返航点，没有则停止
#         if self.home_lng_lat is None:
#             if not config.home_debug:
#                 self.pi_main_obj.stop()
#         else:
#             print('back home')
#             self.points_arrive_control(self.home_lng_lat, self.home_lng_lat, True, True)
#             # self.pi_main_obj.stop()
#
#     def smooth_path(self):
#         """
#         平滑路径
#         :return:平滑路径线路
#         """
#         smooth_path_lng_lat = []
#         distance_matrix = []
#         for index, target_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps):
#             if index == 0:
#                 theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
#                                                               self.lng_lat[1],
#                                                               target_lng_lat[0],
#                                                               target_lng_lat[1])
#                 distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
#                                                                     self.lng_lat[1],
#                                                                     target_lng_lat[0],
#                                                                     target_lng_lat[1])
#                 if distance < config.smooth_path_ceil_size:
#                     smooth_path_lng_lat.append(target_lng_lat)
#                 else:
#                     for i in range(1, int((distance / config.smooth_path_ceil_size) + 1)):
#                         cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
#                                                                                   self.lng_lat[1],
#                                                                                   theta,
#                                                                                   config.smooth_path_ceil_size * i)
#                         smooth_path_lng_lat.append(cal_lng_lat)
#                     smooth_path_lng_lat.append(target_lng_lat)
#             else:
#                 theta = lng_lat_calculate.angleFromCoordinate(
#                     self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
#                     self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
#                     target_lng_lat[0],
#                     target_lng_lat[1])
#                 distance = lng_lat_calculate.distanceFromCoordinate(
#                     self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
#                     self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
#                     target_lng_lat[0],
#                     target_lng_lat[1])
#                 if distance < config.smooth_path_ceil_size:
#                     smooth_path_lng_lat.append(target_lng_lat)
#                 else:
#                     for i in range(1, int(distance / config.smooth_path_ceil_size + 1)):
#                         cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(
#                             self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
#                             self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
#                             theta,
#                             config.smooth_path_ceil_size * i)
#                         smooth_path_lng_lat.append(cal_lng_lat)
#                     smooth_path_lng_lat.append(target_lng_lat)
#         for smooth_lng_lat_i in smooth_path_lng_lat:
#             distance_list = []
#             for sampling_points_gps_i in self.server_data_obj.mqtt_send_get_obj.sampling_points_gps:
#                 s_d = lng_lat_calculate.distanceFromCoordinate(sampling_points_gps_i[0],
#                                                                sampling_points_gps_i[1],
#                                                                smooth_lng_lat_i[0],
#                                                                smooth_lng_lat_i[1])
#                 distance_list.append(s_d)
#             distance_matrix.append(distance_list)
#         a_d_m = np.asarray(distance_matrix)
#         for k in range(len(distance_matrix[0])):
#             temp_a = a_d_m[:, k]
#             temp_list = temp_a.tolist()
#             index_l = temp_list.index(min(temp_list))
#             self.smooth_path_lng_lat_index.append(index_l)
#         return smooth_path_lng_lat
#
#     def calc_target_lng_lat(self, index_):
#         """
#         根据当前点和路径计算下一个经纬度点
#         :return:
#         """
#         # 离散按指定间距求取轨迹点数量
#         if not self.smooth_path_lng_lat:
#             self.smooth_path_lng_lat = self.smooth_path()
#         # 搜索最临近的路点
#         distance_list = []
#         start_index = self.smooth_path_lng_lat_index[index_]
#         if index_ == 0:
#             self.search_list = copy.deepcopy(self.smooth_path_lng_lat[:start_index])
#         else:
#             self.search_list = copy.deepcopy(
#                 self.smooth_path_lng_lat[self.smooth_path_lng_lat_index[index_ - 1]:start_index])
#         for target_lng_lat in self.search_list:
#             distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
#                                                                 self.lng_lat[1],
#                                                                 target_lng_lat[0],
#                                                                 target_lng_lat[1])
#             distance_list.append(distance)
#         # 如果没有可以去路径
#         if len(distance_list) == 0:
#             return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
#         index = distance_list.index(min(distance_list))
#         # if index + 1 == len(self.search_list):
#         #     return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
#         lng_lat = self.search_list[index]
#         index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
#                                                                         self.lng_lat[1],
#                                                                         lng_lat[0],
#                                                                         lng_lat[1])
#         while config.smooth_path_ceil_size > index_point_distance and (index + 1) < len(
#                 self.search_list):
#             lng_lat = self.search_list[index]
#             index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
#                                                                             self.lng_lat[1],
#                                                                             lng_lat[0],
#                                                                             lng_lat[1])
#             index += 1
#         return self.search_list[index]
#
#     # 构建障碍物地图
#     def build_obstacle_map(self):
#         """
#         根据超声波距离构建障碍物地图
#         :return: 障碍物位置举证
#         """
#         method = 1
#         if method == 0:
#             map_size = int(20 / 0.5)
#             obstacle_map = np.zeros((map_size, map_size))
#             # 判断前方距离是否有障碍物，根据障碍物改变目标点
#             for k, v in self.pi_main_obj.distance_dict.items():
#                 v = min(v, 20)
#                 row = int(map_size - math.ceil(math.cos(math.radians(k)) * v / 0.5))
#                 col = int((map_size / 2) - 1 - math.ceil(math.sin(math.radians(k)) * v / 0.5))
#                 for row_index in range(row):
#                     obstacle_map[row_index, col] = 1
#         else:
#             obstacle_map = [0] * len(self.pi_main_obj.distance_dict.items())
#             for k, v in self.pi_main_obj.distance_dict.items():
#                 if v < 5:
#                     obstacle_map[10 + int(k / 0.9)] = 1
#         return obstacle_map
#
#     # 计算障碍物下目标点
#     def get_avoid_obstacle_point(self, path_planning_point_gps=None):
#         """
#         根据障碍物地图获取下一个运动点
#         :return: 下一个目标点，是否需要紧急停止
#         """
#         next_point_lng_lat = copy.deepcopy(path_planning_point_gps)
#         if config.b_millimeter_wave:
#             print('config.obstacle_avoid_type', config.obstacle_avoid_type)
#             # 不避障
#             if config.obstacle_avoid_type == 0:
#                 return path_planning_point_gps, False
#             # 避障停止
#             elif config.obstacle_avoid_type == 1:
#                 if 1 in self.pi_main_obj.obstacle_list[
#                         int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
#                     return next_point_lng_lat, True
#                 else:
#                     return path_planning_point_gps, False
#             # 避障绕行，根据障碍物计算下一个目标点
#             elif config.obstacle_avoid_type == 2:
#                 angle_point = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
#                                                                     self.lng_lat[1],
#                                                                     path_planning_point_gps[0],
#                                                                     path_planning_point_gps[1])
#                 if angle_point > 180:
#                     angle_point_temp = angle_point - 360
#                 else:
#                     angle_point_temp = angle_point
#                 point_angle_index = angle_point_temp // self.pi_main_obj.view_cell + 9
#                 # 目标区域超出避障范围，当前正在转弯不必进行避障
#                 if point_angle_index < 0 or point_angle_index >= len(self.pi_main_obj.obstacle_list):
#                     return next_point_lng_lat, False
#                 index_i = 0
#                 value_list = []
#                 while index_i < self.pi_main_obj.cell_size:
#                     kr = index_i
#                     index_j = index_i
#                     while index_j < self.pi_main_obj.cell_size and self.pi_main_obj.obstacle_list[index_j] == 0:
#                         kl = index_j
#                         if kl - kr >= config.ceil_max:  # 判断是否是宽波谷
#                             print(self.pi_main_obj.obstacle_list, round(kl - config.ceil_max // 2))
#                             v = round((kl + kr) / 2)
#                             value_list.append(v)
#                             break
#                         index_j = index_j + 1
#                     index_i += 1
#                 print('self.pi_main_obj.obstacle_list', self.pi_main_obj.obstacle_list, )
#                 # 没有可以通过通道
#                 if len(value_list) == 0:
#                     return next_point_lng_lat, True
#                 else:
#                     how = []
#                     for value_i in value_list:
#                         howtemp = abs(value_i - point_angle_index)
#                         how.append(howtemp)
#                     ft = how.index(min(how))
#                     kb = value_list[int(ft)]
#                     angle = kb * config.view_cell - config.field_of_view / 2
#                     if angle < 0:
#                         angle += 360
#                 next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
#                                                                                  self.lng_lat[1],
#                                                                                  angle,
#                                                                                  config.min_steer_distance)
#                 print('angle', angle)
#                 return next_point_lng_lat, False
#         else:
#             return path_planning_point_gps, False
#
#     # 控制到达目标点
#     def points_arrive_control(self, target_lng_lat_gps, sample_lng_lat_gps, b_force_arrive=False, b_back_home=False):
#         """
#         :param target_lng_lat_gps: 目标点真实经纬度
#         :param sample_lng_lat_gps: 下一个采样点真实经纬度
#         :param b_force_arrive: 是否约束一定要到达
#         :param b_back_home 是否是正在返航
#         :return:
#         """
#         distance = lng_lat_calculate.distanceFromCoordinate(
#             self.lng_lat[0],
#             self.lng_lat[1],
#             sample_lng_lat_gps[0],
#             sample_lng_lat_gps[1])
#         self.distance_p = distance
#         if distance < config.arrive_distance:
#             return True
#         while distance >= config.arrive_distance:
#             if not b_back_home and not self.b_at_home:
#                 self.check_backhome()
#             distance_sample = lng_lat_calculate.distanceFromCoordinate(
#                 self.lng_lat[0],
#                 self.lng_lat[1],
#                 sample_lng_lat_gps[0],
#                 sample_lng_lat_gps[1])
#             self.distance_p = distance_sample
#             # 避障判断下一个点
#             b_stop = False
#             if not config.home_debug:
#                 target_lng_lat_gps, b_stop = self.get_avoid_obstacle_point(target_lng_lat_gps)
#             all_distance = lng_lat_calculate.distanceFromCoordinate(
#                 self.lng_lat[0], self.lng_lat[1], target_lng_lat_gps[0],
#                 target_lng_lat_gps[1])
#             # 当前点到目标点角度
#             point_theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
#                                                                 self.lng_lat[1],
#                                                                 target_lng_lat_gps[0],
#                                                                 target_lng_lat_gps[1])
#             theta_error = point_theta - self.current_theta
#             if abs(theta_error) > 180:
#                 if theta_error > 0:
#                     theta_error = theta_error - 360
#                 else:
#                     theta_error = 360 + theta_error
#             self.theta_error = theta_error
#             left_pwm, right_pwm = self.path_track_obj.pid_pwm_2(distance=all_distance,
#                                                                 theta_error=theta_error)
#             self.last_left_pwm = left_pwm
#             self.last_right_pwm = right_pwm
#             # 在家调试模式下预测目标经纬度
#             if config.home_debug:
#                 time.sleep(0.1)
#                 # 计算当前行驶里程
#                 if self.last_lng_lat:
#                     speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
#                                                                               self.last_lng_lat[1],
#                                                                               self.lng_lat[0],
#                                                                               self.lng_lat[1])
#                     self.run_distance += speed_distance
#                 left_delta_pwm = int(self.last_left_pwm + left_pwm) / 2 - config.stop_pwm
#                 right_delta_pwm = int(self.last_right_pwm + right_pwm) / 2 - config.stop_pwm
#                 steer_power = left_delta_pwm - right_delta_pwm
#                 forward_power = left_delta_pwm + right_delta_pwm
#                 delta_distance = forward_power * 0.002
#                 delta_theta = steer_power * 0.08
#                 self.last_lng_lat = copy.deepcopy(self.lng_lat)
#                 if self.current_theta is not None:
#                     self.current_theta = (self.current_theta - delta_theta / 2) % 360
#                 self.lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
#                                                                            self.lng_lat[1],
#                                                                            self.current_theta,
#                                                                            delta_distance)
#             else:
#                 # 判断是否需要避障处理
#                 print('b_stop', b_stop)
#                 # self.pi_main_obj.set_pwm(left_pwm, right_pwm)
#                 if b_stop:
#                     self.obstacle_info = '1'
#                     self.pi_main_obj.stop()
#                     # 记录是因为按了暂停按钮而终止
#                     self.b_stop_path_track = True
#                     return False
#                 else:
#                     self.obstacle_info = '0'
#                     self.pi_main_obj.set_pwm(left_pwm, right_pwm)
#
#             # 清空规划点
#             if int(self.server_data_obj.mqtt_send_get_obj.control_move_direction) == -1:
#                 # 记录是因为按了暂停按钮而终止
#                 self.b_stop_path_track = True
#                 return False
#             if not config.home_debug and self.pi_main_obj.b_start_remote:
#                 # 记录是因为按了遥控而终止
#                 self.b_stop_path_track = True
#                 break
#             # 如果目标点改变并且不是强制到达 b_force_arrive
#             if not b_force_arrive:
#                 break
#             else:
#                 if distance_sample < config.arrive_distance:
#                     if b_back_home:
#                         self.b_at_home = 1
#                     return True
#
#     # 处理电机控制 必须使用线程
#     def move_control(self):
#         # 记录上次手动发送
#         last_control = None
#         # 记录发送的经纬度
#         b_log_points = 1
#         while True:
#             time.sleep(config.pi2com_timeout)
#             # 检查是否需要返航
#             if not self.b_at_home:
#                 self.check_backhome()
#             # 判断当前是手动控制还是自动控制
#             d = int(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
#             if d in [-2, -1, 0, 90, 180, 270]:
#                 self.ship_status = ShipStatus.computer_control
#                 # 改变状态不再重复发送指令
#                 self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
#                 if d in [-1, 0, 90, 180, 270]:
#                     # 手动控制下设置在家状态为0
#                     self.b_at_home = 0
#                     b_log_points = 1
#             # 使用路径规划
#             if len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
#                 # 此时为自动模式清除d控制状态
#                 self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
#                 self.ship_status = ShipStatus.auto
#             # 使用遥控器  调试模式下无法使用
#             if not config.home_debug and \
#                     config.current_platform == config.CurrentPlatform.pi and \
#                     self.pi_main_obj.b_start_remote:
#                 remote_left_pwm, remote_right_pwm = self.pi_main_obj.check_remote_pwm()
#                 self.pi_main_obj.set_pwm(set_left_pwm=remote_left_pwm, set_right_pwm=remote_right_pwm)
#             # 手动模式
#             elif self.ship_status == ShipStatus.computer_control:
#                 if config.obstacle_avoid_type == 3:
#                     if 1 in self.pi_main_obj.obstacle_list[
#                             int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
#                         print('self.pi_main_obj.obstacle_list', self.pi_main_obj.obstacle_list)
#                         d = -1
#                 # 使用飞控
#                 if config.b_use_pix:
#                     if d == 0:
#                         pwm_data = {'1': 1900, '3': 1900}
#                     elif d == 90:
#                         pwm_data = {'1': 1900, '3': 1100}
#                     elif d == 180:
#                         pwm_data = {'1': 1100, '3': 1100}
#                     elif d == 270:
#                         pwm_data = {'1': 1100, '3': 1900}
#                     elif d == -1:
#                         pwm_data = {'1': 1500, '3': 1500}
#                     else:
#                         pwm_data = None
#                     if last_control is None or last_control != pwm_data:
#                         last_control = pwm_data
#                         self.drone_obj.channel_control(pwm_data)
#                         self.com_data_send_logger.info({'com pwm data': pwm_data})
#                 # 使用树莓派
#                 elif config.b_use_pi:
#                     if not config.home_debug:
#                         if d == 0:
#                             self.control_info = '向前'
#                             self.pi_main_obj.forward()
#                         elif d == 90:
#                             self.control_info = '向左'
#                             self.pi_main_obj.left()
#                         elif d == 180:
#                             self.control_info = '向后'
#                             self.pi_main_obj.backword()
#                         elif d == 270:
#                             self.control_info = '向右'
#                             self.pi_main_obj.right()
#                         elif d == -1:
#                             self.control_info = '停止'
#                             self.pi_main_obj.stop()
#                 # 手动模式下判断是否抽水
#                 if not config.home_debug:
#                     if config.b_pin_stc or os.path.exists(config.stc_port):
#                         self.draw()
#             # 自动模式计算角度
#             elif self.ship_status == ShipStatus.auto:
#                 self.control_info = ''
#                 if self.lng_lat is None:
#                     self.logger.error('无当前GPS，不能自主巡航')
#                     time.sleep(0.5)
#                     self.clear_status()
#                     continue
#                 if self.plan_start_time is None:
#                     self.plan_start_time = time.time()
#                 # 设置自动路径搜索为False
#                 self.b_stop_path_track = False
#                 if b_log_points:
#                     self.logger.info({'点击地点': self.server_data_obj.mqtt_send_get_obj.path_planning_points})
#                 # 船起始运行距离
#                 start_distance = self.run_distance
#                 # 判断是否是寻点模式点了寻点但是还没点开始
#                 if self.server_data_obj.mqtt_send_get_obj.row_gap:
#                     if not self.server_data_obj.mqtt_send_get_obj.b_start:
#                         b_log_points = 0
#                         time.sleep(0.2)
#                         continue
#                 # 计算总里程
#                 for index, gaode_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points):
#                     if index == 0:
#                         distance_p = lng_lat_calculate.distanceFromCoordinate(
#                             self.gaode_lng_lat[0],
#                             self.gaode_lng_lat[1],
#                             gaode_lng_lat[0],
#                             gaode_lng_lat[1])
#                         self.totle_distance += distance_p
#                     else:
#                         distance_p = lng_lat_calculate.distanceFromCoordinate(
#                             self.server_data_obj.mqtt_send_get_obj.path_planning_points[index - 1][0],
#                             self.server_data_obj.mqtt_send_get_obj.path_planning_points[index - 1][1],
#                             gaode_lng_lat[0],
#                             gaode_lng_lat[1])
#                         self.totle_distance += distance_p
#                 self.logger.info({'全部距离': self.totle_distance})
#                 # 将目标点转换为真实经纬度
#                 self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps = []
#                 self.server_data_obj.mqtt_send_get_obj.sampling_points_gps = []
#                 # 如果勾选了返航且存在返航点
#                 if self.server_data_obj.mqtt_send_get_obj.back_home:
#                     if self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat:
#                         self.server_data_obj.mqtt_send_get_obj.path_planning_points.append(
#                             self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat)
#                         self.server_data_obj.mqtt_send_get_obj.sampling_points.append(
#                             self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat)
#                         self.server_data_obj.mqtt_send_get_obj.sampling_points_status.append(0)
#                 if config.home_debug:
#                     self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps = copy.deepcopy(
#                         self.server_data_obj.mqtt_send_get_obj.path_planning_points)
#                     self.server_data_obj.mqtt_send_get_obj.sampling_points_gps = copy.deepcopy(
#                         self.server_data_obj.mqtt_send_get_obj.sampling_points)
#                 else:
#                     for path_planning_point in self.server_data_obj.mqtt_send_get_obj.path_planning_points:
#                         path_planning_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
#                                                                                      self.gaode_lng_lat,
#                                                                                      path_planning_point)
#                         self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps.append(path_planning_point_gps)
#                     for sampling_point in self.server_data_obj.mqtt_send_get_obj.sampling_points:
#                         sampling_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
#                                                                                 self.gaode_lng_lat,
#                                                                                 sampling_point)
#                         self.server_data_obj.mqtt_send_get_obj.sampling_points_gps.append(sampling_point_gps)
#                 self.path_info = [0, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
#                 print('self.server_data_obj.mqtt_send_get_obj.sampling_points_status',
#                       self.server_data_obj.mqtt_send_get_obj.sampling_points_status)
#                 while self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) > 0:
#                     for index, sampling_point_gps in enumerate(
#                             self.server_data_obj.mqtt_send_get_obj.sampling_points_gps):
#                         # 如果状态清空则跳出
#                         if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) <= 0:
#                             break
#                         if self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] == 1:
#                             continue
#                         # 计算下一个目标点经纬度
#                         if config.path_plan_type:
#                             next_lng_lat = self.calc_target_lng_lat(index)
#                             # 如果当前点靠近采样点指定范围就停止并采样
#                             sample_distance = lng_lat_calculate.distanceFromCoordinate(
#                                 next_lng_lat[0],
#                                 next_lng_lat[1],
#                                 sampling_point_gps[0],
#                                 sampling_point_gps[1])
#                             # print('sample_distance', sample_distance)
#                             if sample_distance < config.forward_see_distance:
#                                 b_arrive_sample = self.points_arrive_control(sampling_point_gps, sampling_point_gps,
#                                                                              b_force_arrive=True)
#                                 print('b_arrive_sample', b_arrive_sample)
#                             else:
#                                 b_arrive_sample = self.points_arrive_control(next_lng_lat, sampling_point_gps,
#                                                                              b_force_arrive=False)
#                         else:
#                             b_arrive_sample = self.points_arrive_control(sampling_point_gps, sampling_point_gps,
#                                                                          b_force_arrive=True)
#                         if b_arrive_sample:
#                             # 更新目标点提示消息
#                             self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] = 1
#                             self.path_info = [index + 1, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
#                             if not config.home_debug and config.b_draw:
#                                 # 开始抽水并等待
#                                 self.server_data_obj.mqtt_send_get_obj.b_draw = 1
#                                 self.pi_main_obj.stop()
#                                 if config.b_pin_stc or os.path.exists(config.stc_port):
#                                     self.draw()
#                                     time.sleep(config.draw_time)
#                                     self.b_draw_over_send_data = True
#                                     self.draw()
#                             # elif not config.b_draw:
#                             #     if not config.home_debug:
#                             #         self.pi_main_obj.stop()
#                         else:
#                             break
#                     if self.b_stop_path_track:
#                         break
#                 # 全部结束后停止
#                 end_distance = self.run_distance
#                 try:
#                     self.distance_move_score = round(100 * self.totle_distance / (end_distance - start_distance), 1)
#                 except Exception as e:
#                     self.logger.error({' distance_move_score error': e})
#                 # 清除状态
#                 self.totle_distance = 0
#                 self.clear_status()
#
#     # 将经纬度转换为高德经纬度
#     def update_ship_gaode_lng_lat(self):
#         # 更新经纬度为高德经纬度
#         while True:
#             if config.home_debug:
#                 if self.lng_lat is None:
#                     self.lng_lat = config.ship_gaode_lng_lat
#             if self.lng_lat is not None:
#                 if self.use_true_gps:
#                     if not config.home_debug:
#                         try:
#                             gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
#                             if gaode_lng_lat:
#                                 self.gaode_lng_lat = gaode_lng_lat
#                         except Exception as e:
#                             self.logger.error({'error': e})
#                     else:
#                         self.gaode_lng_lat = self.lng_lat
#             # 调试模式下自身经纬度和高德经纬度一致
#             elif self.lng_lat is not None and not self.use_true_gps:
#                 self.gaode_lng_lat = self.lng_lat
#             time.sleep(config.pi2mqtt_interval)
#
#     # 更新经纬度
#     def update_lng_lat(self):
#         last_read_time = None
#         while True:
#             # print('self.pi_main_obj.lng_lat',self.pi_main_obj.lng_lat)
#             if not config.home_debug and \
#                     self.pi_main_obj.lng_lat and \
#                     self.pi_main_obj.lng_lat[0] > 1 and \
#                     self.pi_main_obj.lng_lat[1] > 1:
#                 self.lng_lat = copy.deepcopy(self.pi_main_obj.lng_lat)
#                 self.lng_lat_error = self.pi_main_obj.lng_lat_error
#                 if not last_read_time:
#                     last_read_time = time.time()
#                 if self.lng_lat_error and self.lng_lat_error < 2.5:
#                     if self.last_lng_lat:
#                         # 计算当前行驶里程
#                         speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
#                                                                                   self.last_lng_lat[1],
#                                                                                   self.lng_lat[0],
#                                                                                   self.lng_lat[1])
#                         self.run_distance += speed_distance
#                         # 计算速度
#                         self.speed = round(speed_distance / (time.time() - last_read_time), 1)
#                         # 替换上一次的值
#                         self.last_lng_lat = copy.deepcopy(self.lng_lat)
#                         last_read_time = time.time()
#                     else:
#                         self.last_lng_lat = copy.deepcopy(self.lng_lat)
#                         last_read_time = time.time()
#             time.sleep(1 / config.gps_frequency)
#
#     # 读取函数会阻塞 必须使用线程发送mqtt状态数据和检测数据
#     def send_mqtt_data(self):
#         last_read_time = time.time()
#         last_runtime = None
#         last_run_distance = None
#         while True:
#             time.sleep(config.pi2mqtt_interval)
#             if self.server_data_obj.mqtt_send_get_obj.pool_code:
#                 self.data_define_obj.pool_code = self.server_data_obj.mqtt_send_get_obj.pool_code
#             status_data = self.data_define_obj.status
#             status_data.update({'mapId': self.data_define_obj.pool_code})
#             detect_data = self.data_define_obj.detect
#             detect_data.update({'mapId': self.data_define_obj.pool_code})
#             status_data.update({'ping': round(self.ping, 1)})
#             status_data.update({'current_lng_lat': self.gaode_lng_lat})
#             if config.b_sonar:
#                 self.lng_lat_list.append(self.gaode_lng_lat)
#                 deep_ = self.pi_main_obj.get_sonar_data()
#                 deep = deep_ if deep_ else 0
#                 self.deep_list.append(deep)
#                 assert len(self.deep_list) == len(self.lng_lat_list)
#                 if len(self.deep_list) % 20 == 0:
#                     utils.generate_geojson(self.lng_lat_list, self.deep_list, b_save_data=True)
#             if self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat:
#                 status_data.update({'home_lng_lat': self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat})
#                 self.home_lng_lat = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
#                                                                        self.gaode_lng_lat,
#                                                                        self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat)
#             # 更新速度  更新里程
#             if self.speed is not None:
#                 status_data.update({'speed': self.speed})
#             status_data.update({"runtime": round(time.time() - self.start_time)})
#             status_data.update({"run_distance": round(self.run_distance, 1)})
#             if last_runtime is None:
#                 last_runtime = 0
#             if last_run_distance is None:
#                 last_run_distance = 0
#             if os.path.exists(config.run_distance_time_path):
#                 try:
#                     with open(config.run_distance_time_path, 'r') as f:
#                         run_distance_time_data = json.load(f)
#                         save_distance = run_distance_time_data.get('save_distance')
#                         save_time = run_distance_time_data.get('save_time')
#                 except Exception as e:
#                     save_distance = 0
#                     save_time = 0
#                     self.logger.error({'读取run_distance_time_path': e})
#             else:
#                 save_distance = 0
#                 save_time = 0
#             with open(config.run_distance_time_path, 'w') as f:
#                 save_distance = save_distance + round(self.run_distance, 1) - last_run_distance
#                 save_time = save_time + round(time.time() - self.start_time) - last_runtime
#                 json.dump({'save_distance': save_distance,
#                            'save_time': save_time}, f)
#             last_runtime = round(time.time() - self.start_time)
#             last_run_distance = round(self.run_distance, 1)
#             status_data.update({"totle_distance": round(save_distance, 1)})
#             status_data.update({"totle_time": round(save_time)})
#             # 更新船头方向
#             if self.current_theta:
#                 status_data.update({"direction": round(self.current_theta)})
#             # 更新经纬度误差
#             if self.lng_lat_error is not None:
#                 status_data.update({"lng_lat_error": self.lng_lat_error})
#             # 更新真实数据
#             if not config.home_debug:
#                 mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
#                 mqtt_send_detect_data['water'].update(self.water_data_dict)
#                 mqtt_send_status_data = data_define.fake_status_data(status_data)
#             # 更新模拟数据
#             else:
#                 mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
#                 mqtt_send_status_data = data_define.fake_status_data(status_data)
#             # 替换键
#             for k_all, v_all in data_define.name_mappings.items():
#                 for old_key, new_key in v_all.items():
#                     pop_value = mqtt_send_detect_data[k_all].pop(old_key)
#                     mqtt_send_detect_data[k_all].update({new_key: pop_value})
#             if self.dump_energy is not None:
#                 self.dump_energy_deque.append(self.dump_energy)
#                 mqtt_send_status_data.update({'dump_energy': self.dump_energy})
#             # 向mqtt发送数据
#             self.send(method='mqtt', topic='status_data_%s' % config.ship_code, data=mqtt_send_status_data,
#                       qos=0)
#             if time.time() - last_read_time > 10:
#                 last_read_time = time.time()
#                 self.data_save_logger.info({"发送状态数据": mqtt_send_status_data})
#                 if config.home_debug:
#                     self.send(method='mqtt', topic='detect_data_%s' % config.ship_code, data=mqtt_send_detect_data,
#                               qos=0)
#                     self.logger.info({'fakedate': mqtt_send_detect_data})
#                     # 调试时使用发送检测数据
#                     if config.debug_send_detect_data:
#                         mqtt_send_detect_data.update({'jwd': json.dumps(self.lng_lat)})
#                         mqtt_send_detect_data.update({'gjwd': json.dumps(self.gaode_lng_lat)})
#                         if len(self.data_define_obj.pool_code) > 0:
#                             self.send(method='http', data=mqtt_send_detect_data,
#                                       url=config.http_data_save,
#                                       http_type='POST')
#                             time.sleep(0.5)
#                             self.data_save_logger.info({"发送检测数据": mqtt_send_detect_data})
#             if self.b_draw_over_send_data:
#                 # 添加经纬度
#                 mqtt_send_detect_data.update({'jwd': json.dumps(self.lng_lat)})
#                 mqtt_send_detect_data.update({'gjwd': json.dumps(self.gaode_lng_lat)})
#                 self.send(method='mqtt', topic='detect_data_%s' % config.ship_code, data=mqtt_send_detect_data,
#                           qos=0)
#                 if len(self.data_define_obj.pool_code) > 0:
#                     self.send(method='http', data=mqtt_send_detect_data,
#                               url=config.http_data_save,
#                               http_type='POST')
#                     self.data_save_logger.info({"发送检测数据": mqtt_send_detect_data})
#                 save_detect_data = copy.deepcopy(mqtt_send_detect_data)
#                 # save_detect_data.update({'lng_lat': self.lng_lat})
#                 self.logger.info({"本地保存检测数据": save_detect_data})
#                 del save_detect_data
#                 # 发送结束改为False
#                 self.b_draw_over_send_data = False
#
#     # 配置更新
#     def update_config(self):
#         while True:
#             # 客户端获取基础设置数据
#             if self.server_data_obj.mqtt_send_get_obj.base_setting_data_info in [1, 4]:
#                 if self.server_data_obj.mqtt_send_get_obj.base_setting_data is None:
#                     self.logger.error(
#                         {'base_setting_data is None': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
#                 else:
#                     self.server_data_obj.mqtt_send_get_obj.base_setting_data.update({'info_type': 3})
#                     self.send(method='mqtt', topic='base_setting_%s' % config.ship_code,
#                               data=self.server_data_obj.mqtt_send_get_obj.base_setting_data,
#                               qos=0)
#                     self.logger.info({'base_setting': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
#                     self.server_data_obj.mqtt_send_get_obj.base_setting_data = None
#                     self.server_data_obj.mqtt_send_get_obj.base_setting_data_info = 0
#
#             # 客户端获取高级设置数据
#             if self.server_data_obj.mqtt_send_get_obj.height_setting_data_info in [1, 4]:
#                 if self.server_data_obj.mqtt_send_get_obj.height_setting_data is None:
#                     self.logger.error(
#                         {'height_setting_data is None': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
#                 else:
#                     self.server_data_obj.mqtt_send_get_obj.height_setting_data.update({'info_type': 3})
#                     self.send(method='mqtt', topic='height_setting_%s' % config.ship_code,
#                               data=self.server_data_obj.mqtt_send_get_obj.height_setting_data,
#                               qos=0)
#                     self.logger.info({'height_setting': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
#                     self.server_data_obj.mqtt_send_get_obj.height_setting_data = None
#                     # 改为0位置状态，不再重复发送
#                     self.server_data_obj.mqtt_send_get_obj.height_setting_data_info = 0
#             time.sleep(config.pi2mqtt_interval)
#
#     # 发送数据
#     def send(self, method, data, topic='test', qos=0, http_type='POST', url=''):
#         """
#         :param url:
#         :param http_type:
#         :param qos:
#         :param topic:
#         :param data: 发送数据
#         :param method 获取数据方式　http mqtt com
#         """
#         assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
#         if method == 'http':
#             return_data = self.server_data_obj.send_server_http_data(http_type, data, url)
#             self.logger.info({'请求 url': url, 'status_code': return_data.status_code})
#             # 如果是POST返回的数据，添加数据到地图数据保存文件中
#             if http_type == 'POST' and r'map/save' in url:
#                 content_data = json.loads(return_data.content)
#                 self.logger.info({'map/save content_data success': content_data["success"]})
#                 if not content_data["success"]:
#                     self.logger.error('POST请求发送地图数据失败')
#                 # POST 返回湖泊ID
#                 pool_id = content_data['data']['id']
#                 return pool_id
#             # http发送检测数据给服务器
#             elif http_type == 'POST' and r'data/save' in url:
#                 content_data = json.loads(return_data.content)
#                 self.logger.debug({'data/save content_data success': content_data["success"]})
#                 if not content_data["success"]:
#                     self.logger.error('POST发送检测请求失败')
#             elif http_type == 'GET' and r'device/binding' in url:
#                 content_data = json.loads(return_data.content)
#                 if not content_data["success"]:
#                     self.logger.error('GET请求失败')
#                 save_data_binding = content_data["data"]
#                 return save_data_binding
#             else:
#                 # 如果是GET请求，返回所有数据的列表
#                 content_data = json.loads(return_data.content)
#                 if not content_data["success"]:
#                     self.logger.error('GET请求失败')
#                 save_data_map = content_data["data"]["mapList"]
#                 return save_data_map
#         elif method == 'mqtt':
#             self.server_data_obj.send_server_mqtt_data(data=data, topic=topic, qos=qos)
#
#     # 状态检查函数，检查自身状态发送对应消息
#     def check_status(self):
#         while True:
#             # 循环等待一定时间
#             time.sleep(config.check_status_interval)
#             if config.home_debug:
#                 self.send_test_distance()
#             if not config.home_debug:
#                 self.control_relay()
#             if config.home_debug:
#                 if config.b_play_audio:
#                     audios_manager.play_audio(5, b_backend=False)
#             if self.last_lng_lat:
#                 ship_theta = lng_lat_calculate.angleFromCoordinate(self.last_lng_lat[0],
#                                                                    self.last_lng_lat[1],
#                                                                    self.lng_lat[0],
#                                                                    self.lng_lat[1])
#             else:
#                 ship_theta = 0
#             # 船头角度
#             if config.use_shape_theta_type == 1:
#                 if config.b_pin_compass:
#                     self.current_theta = self.pi_main_obj.theta
#                 else:
#                     self.current_theta = self.theta
#             else:
#                 self.current_theta = ship_theta
#             # 检查电量 如果连续20次检测电量平均值低于电量阈值就报警
#             if config.energy_backhome:
#                 if len(list(self.dump_energy_deque)) > 0 and sum(list(self.dump_energy_deque)) / len(
#                         list(self.dump_energy_deque)) < config.energy_backhome:
#                     self.low_dump_energy_warnning = 1
#                 else:
#                     self.low_dump_energy_warnning = 0
#             # 接收到重置湖泊按钮
#             if self.server_data_obj.mqtt_send_get_obj.reset_pool_click:
#                 self.data_define_obj.pool_code = ''
#                 self.server_data_obj.mqtt_send_get_obj.pool_code = ''
#                 self.server_data_obj.mqtt_send_get_obj.reset_pool_click = 0
#             # 船状态提示消息
#             notice_info_data = {
#                 "distance": str(round(self.distance_p, 2)) + ' s ' + str(self.distance_move_score),
#                 # // 路径规划提示消息
#                 "path_info": '当前目标点:%d 目标点总数: %d' % (int(self.path_info[0]), int(self.path_info[1])),
#                 # 船执行手动控制信息
#                 "control_info": self.control_info,
#                 # 水泵开关状态消息
#                 "draw_info": self.server_data_obj.mqtt_send_get_obj.b_draw,
#                 # 声光报警器
#                 "audio_light_info": self.server_data_obj.mqtt_send_get_obj.audio_light,
#                 # 大灯
#                 "headlight_info": self.server_data_obj.mqtt_send_get_obj.headlight,
#                 # 舷灯
#                 "side_light_info": self.server_data_obj.mqtt_send_get_obj.side_light,
#                 # 自动巡航下角度偏差
#                 "theta_error": round(self.theta_error, 2),
#             }
#             notice_info_data.update({"mapId": self.data_define_obj.pool_code})
#             # 遥控器是否启用
#             if config.current_platform == config.CurrentPlatform.pi:
#                 notice_info_data.update({"b_start_remote": self.pi_main_obj.b_start_remote})
#             # 罗盘提示消息
#             if len(self.compass_notice_info) > 3:
#                 notice_info_data.update({"compass_notice_info": self.compass_notice_info + self.compass_notice_info1})
#             if not config.home_debug and self.pi_main_obj.compass_notice_info:
#                 notice_info_data.update({"compass_notice_info": self.pi_main_obj.compass_notice_info})
#             # 使用电量告警是提示消息
#             if self.low_dump_energy_warnning:
#                 notice_info_data.update({"low_dump_energy_warnning": self.low_dump_energy_warnning})
#             self.send(
#                 method='mqtt',
#                 topic='notice_info_%s' % config.ship_code,
#                 data=notice_info_data,
#                 qos=0)
#             if time.time() % 10 < config.check_status_interval:
#                 self.logger.info({'notice_info_': notice_info_data})
#
#             # 保存数据与发送刷新后提示消息
#             if len(self.data_define_obj.pool_code) > 0:
#                 save_plan_path_data = {
#                     "mapId": self.data_define_obj.pool_code,
#                     'sampling_points': self.server_data_obj.mqtt_send_get_obj.sampling_points,
#                     'path_points': self.server_data_obj.mqtt_send_get_obj.path_planning_points,
#                 }
#                 save_data.set_data(save_plan_path_data, config.save_plan_path)
#                 if self.server_data_obj.mqtt_send_get_obj.refresh_info_type == 1:
#                     save_plan_path_data.update({"info_type": 2})
#                     self.send(method='mqtt',
#                               topic='refresh_%s' % config.ship_code,
#                               data=save_plan_path_data,
#                               qos=0)
#
#     # 检测网络延时
#     def check_ping_delay(self):
#         # 检查网络
#         while True:
#             if config.b_check_network:
#                 ping = check_network.get_ping_delay()
#                 if not check_network.get_ping_delay():
#                     if config.b_play_audio:
#                         audios_manager.play_audio(2, b_backend=False)
#                     self.logger.error('当前无网络信号')
#                 else:
#                     self.ping = ping
#             time.sleep(1)
#
#     # 测试发送障碍物数据
#     def send_test_distance(self):
#         direction = int(random.random() * 360)
#         distance_info_data = {
#             # 设备号
#             "deviceId": "3c50f4c3-a9c1-4872-9f18-883af014380a",
#             # 船头角度  以北为0度 ，逆时针方向为正
#             "direction": direction,
#             # 距离信息 内部为列表，列中中元素为字典，distance为距离单位米  angle为角度单位度，以船头角度为0度 左正右负
#             "distance_info": [{"distance": 4.5, "angle": 20},
#                               {"distance": 7.5, "angle": -20},
#                               {"distance": 6, "angle": 0}],
#         }
#
#         self.send(method='mqtt',
#                   topic='distance_info_%s' % config.ship_code,
#                   data=distance_info_data,
#                   qos=0)
#
#     # 发送障碍物信息
#     def send_distacne(self):
#         while True:
#             distance_info_data = {}
#             if len(self.pi_main_obj.distance_dict) > 0:
#                 distance_info_data.update({'deviceId': config.ship_code})
#                 distance_info_data.update({'distance_info': []})
#                 for k in self.pi_main_obj.distance_dict.copy():
#                     distance_info_data['distance_info'].append(
#                         {'distance': self.pi_main_obj.distance_dict[k][0],
#                          'angle': self.pi_main_obj.distance_dict[k][1]})
#                 if self.current_theta:
#                     distance_info_data.update({'direction': round(self.current_theta)})
#                 else:
#                     distance_info_data.update({'direction': 0})
#                 # print('distance_data',distance_info_data)
#                 self.send(method='mqtt',
#                           topic='distance_info_%s' % config.ship_code,
#                           data=distance_info_data,
#                           qos=0)
#             time.sleep(1)
#
#     # 检查开关相关信息
#     def check_switch(self):
#         """
#         对应继电器均需要高电平触发
#         :return:
#         """
#         if not config.home_debug:
#             if self.server_data_obj.mqtt_send_get_obj.headlight:
#                 self.pi_main_obj.set_gpio(control_headlight=1)
#             if self.server_data_obj.mqtt_send_get_obj.audio_light:
#                 self.pi_main_obj.set_gpio(control_alarm_light=1)
#             if self.server_data_obj.mqtt_send_get_obj.side_light:
#                 self.pi_main_obj.set_gpio(control_left_sidelight=1,
#                                           control_right_sidelight=1)
#
#
# if __name__ == '__main__':
#     obj = DataManager()
#     obj.send_mqtt_data()
#     # 114.523792, 30.506471;
#     # 114.524232, 30.506255;
#     # 114.523823, 30.505948
#     ############
#     while True:
#         # move_direction = obj.server_data_obj.mqtt_send_get_obj.control_move_direction
#         # obj.logger.info('move_direction: %f' % (float(move_direction)))
#         time.sleep(2)