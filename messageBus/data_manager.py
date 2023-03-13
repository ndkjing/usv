"""
管理数据收发
"""
import time
import math
import json
import copy
import enum
import numpy as np
from collections import deque
import threading
from messageBus import data_define
from messageBus import ship_type
from externalConnect import server_data
from utils.log import LogHandler
from externalConnect import baidu_map
from utils import lng_lat_calculate
from utils import check_network
from utils import data_valid
import config
from moveControl.obstacleAvoid import vfh


class ShipStatus(enum.Enum):
    """
    船状态
    idle  等待状态
    remote_control 遥控器控制
    computer_control 页面手动控制
    computer_auto 自动控制
    backhome_low_energy 低电量返航状态
    backhome_network 低电量返航状态
    at_home 在家
    tasking  执行检测/抽水任务中
    """
    idle = 1
    remote_control = 2
    computer_control = 3
    computer_auto = 4
    tasking = 5
    avoidance = 6
    backhome_low_energy = 7
    backhome_network = 8
    at_home = 9


class DataManager:
    def __init__(self, ship_id, tcp_server_obj):
        # 日志对象
        self.logger = LogHandler('data_manager_log_%d' % ship_id, level=20)
        self.server_log = LogHandler('server_data%d' % ship_id, level=20)
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id
        self.data_define_obj = data_define.DataDefine(self.ship_code)
        # mqtt服务器数据收发对象
        self.server_data_obj = server_data.ServerData(self.server_log,
                                                      topics=self.data_define_obj.topics,
                                                      ship_code=self.ship_code,
                                                      ship_id=ship_id)
        # 记录里程与时间数据
        self.totle_distance = 0
        # 船行驶里程
        self.run_distance = 0
        # 开机时间
        self.start_time = time.time()
        # 提示消息
        # 距离下一个目标点距离
        self.distance_p = 0
        # 目标点信息
        self.path_info = [0, 0]
        # 抽水结束发送数据
        self.b_draw_over_send_data = False
        # 采样检测船抽水结束发送检测数据
        self.b_draw_over_send_detect_data = False
        # 剩余电量
        self.dump_energy = None
        # 求剩余电量均值
        self.dump_energy_deque = deque(maxlen=20)
        # 电量告警 是否是低电量返航
        self.low_dump_energy_warnning = 0
        # 是否是断网返航
        self.b_network_backhome = 0
        # 是否已经返航在家
        self.b_at_home = 0
        # 返航点
        self.home_lng_lat = None
        # 返航点高德经纬度
        self.home_gaode_lng_lat = None
        # 船速度
        self.speed = None
        # 经纬度 和 船头角度 北为0 逆时针为正
        self.lng_lat = None
        # 船高德经纬度
        self.gaode_lng_lat = None
        # 记录上一次经纬度
        self.last_lng_lat = None
        self.last_get_gaode_lng_lat = None  # 上一次请求高德经纬度的经纬度
        # 记录船状态
        self.ship_status = ShipStatus.idle
        # 记录平滑路径
        self.smooth_path_lng_lat = None
        self.smooth_path_lng_lat_index = []
        self.control_info = ''  # 船控制提示信息
        self.ping = 0  # 网络延时
        # 当前路径平滑搜索列表
        self.search_list = []
        # 是否到达目标点
        self.b_arrive_point = 0
        # 当前运动方向 -1 空闲 0 前 90左 180 后 270 右     10 北  190 西 1180 南  1270东
        self.direction = -1
        # 切换到任务状态前的状态
        self.last_ship_status = ShipStatus.idle
        #  # 如果采样点长时间没到达则跳过
        self.point_arrive_start_time = None
        # 是否需要抓取水质数据
        self.b_check_get_water_data = 0
        self.area_id = None  # 船所在地区编号
        self.current_draw_bottle = 0  # 当前抽水瓶号
        self.current_draw_deep = 0  # 当前抽水深度
        self.current_draw_capacity = 0  # 当前抽水容量
        self.draw_over_bottle_info = []
        self.http_save_distance = None  # http 记录保存距离
        self.http_save_time = None  # http记录保存时间
        self.http_save_id = None  # http记录保存id
        self.record_path = []  # 记录手动轨迹点
        self.is_wait = 1  # 船只是否空闲
        # 调试模拟避障数据
        self.send_obstacle = False
        # 距离矩阵
        self.cell_size = int(config.field_of_view // config.view_cell)
        self.obstacle_list = [0] * self.cell_size  # 自动避障列表
        self.task_list = []  # 获取存储的任务  经纬度，采样深度，采样量数据样式([lng,lat],[bottle_id,deep,capacity],[bottle_id,deep,capacity])
        self.sort_task_list = []  # 获取存储的任务  经纬度，采样深度，采样量数据样式[[lng,lat],[bottle_id,deep,capacity],[bottle_id,deep,capacity]]
        self.sort_task_done_list = []  # 总共有多少个点抽水完成存储0/1  单个长度等于任务点数量[[0,0],[0,0,0,0]
        self.current_arriver_index = None  # 当前到达预存储任务点索引
        self.draw_points_list = []  # 记录抽水点数据[[lng,lat,bottle_id,deep,amount],]
        self.has_task = 0  # 当前是否有任务在执行
        self.arrive_all_task = 0  # 是否完成所有任务
        self.action_id = None  # 行动id
        self.action_type = 2  # 行动是否在执行1：在执行  2：没有执行  3 继续执行
        self.need_action_id = 0  # 是否需要发送请求获取action_id
        self.is_need_update_plan = 0  # 是否需要更新任务状态
        self.is_plan_all_arrive = 0  # 任务点全部执行成功
        self.throttle = 1  # 避障油门比例
        self.obstacle_avoid_time_distance = []  # 记录避障开始时间和距离 如果避障10秒到达目标点距离减少值小于10米则认为遇到无法避障岸边了
        self.b_avoid = 0  # 当前运动是否因为避障
        self.sample_index = []  # 任务数据中记录采样点位置 0:路径点 1:监测点 [0,0,0,1,0,1,0]
        self.bank_stop = []  # 记录当出现距离》0但是位置不变角度不变则认为挂在岸边了
        self.token = None  # 记录登录后token
        self.dump_draw_list = [0, 0]
        self.creator = ""  # 行动人
        self.save_direction_angle = None  # 当使用方位时记录开始的角度值
        self.ship_type_obj = ship_type.ShipType(ship_id=self.ship_id)
        self.deep = 0  # adcp检测深度
        self.tcp_server_obj = tcp_server_obj  # tcp发送数据对象
        self.tcp_send_data = ""  # tcp需要发送数据
        self.tcp_current_data = ""  # tcp目前正在发送de数据
        self.tcp_receive_data = ""  # tcp接受到的确定数据
        self.b_need_stop_draw = 0  # 是否需要发送停止抽水
        self.pre_control_data = None  # 记录上一次发送的运动控制消息防重发(运动控制消息不发送确认)
        self.server_data_obj_dict = {}  # 船号对应mqtt服务字典
        self.ceil_go_throw = 1  # 船只通行可行驶角度值单元大小
        self.deep = 0  # 深度 单位米
        self.send_data_list = ["", "", "", "", "", "", "", "", "S8Z"]  # 空位 经纬度 开关  手动控制 设置 pid 罗盘校准 抽水杆 心跳
        self.tcp_server_obj.ship_id_send_dict.update({ship_id: self.send_data_list})
        self.pre_draw_info = 'S2,0,0,0Z'
        self.control_data = ''

    def thread_control(self):
        # 通用调用函数
        common_func_list = [self.move_control,
                            self.check_status,
                            self.send_mqtt_status_data,
                            self.update_ship_gaode_lng_lat,
                            self.update_lng_lat,
                            self.update_config,
                            self.check_ping_delay,
                            self.change_status,
                            self.check_switch,
                            self.connect_mqtt_server,
                            self.control_draw_thread,
                            self.send_distacne,
                            self.loop_check_task,
                            self.send_high_f_status_data,
                            self.send_record_point_data,
                            self.scan_cal,
                            self.loop_send_http,
                            self.loop_send_tcp_data
                            ]
        common_thread_list = []
        for common_func in common_func_list:
            common_thread_list.append(threading.Thread(target=common_func))
        for common_thread in common_thread_list:
            common_thread.setDaemon(True)
        for common_thread in common_thread_list:
            common_thread.start()
        while True:
            for index_common_thread, common_thread in enumerate(common_thread_list):
                if common_thread is not None and not common_thread.is_alive():
                    self.logger.error({'线程重启': index_common_thread})
                    common_thread_list[index_common_thread] = threading.Thread(
                        target=common_func_list[index_common_thread])
                    common_thread_list[index_common_thread].start()
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出thread_control线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出thread_control线程": self.ship_id})
                return

    # 重连mqtt服务器
    def connect_mqtt_server(self):
        while True:
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出connect_mqtt_server线程": self.ship_id})
                return
            if not self.server_data_obj.mqtt_send_get_obj.is_connected and self.ship_id in self.tcp_server_obj.client_dict:
                self.server_data_obj.mqtt_send_get_obj.mqtt_connect()
                time.sleep(2)
            elif self.server_data_obj.mqtt_send_get_obj.is_connected and self.ship_id not in self.tcp_server_obj.client_dict:
                self.server_data_obj.mqtt_send_get_obj.break_connect()
                self.logger.info("船断线mqtt主动断开connect_mqtt_server连接。。。")
                time.sleep(2)

    def set_send_data(self, data, index):
        self.tcp_server_obj.ship_id_send_dict.get(self.ship_id)[index] = data

    def loop_send_tcp_data(self):
        # 不同消息发送间隔 每个循环20ms  控制数次每次都发  其他数据20次发一次
        init_count = 20
        count = 0
        control_count = 10  # 同样的控制数据最多发十次
        while True:
            time.sleep(0.05)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出loop_send_tcp_data线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出loop_send_tcp_data线程": self.ship_id})
                return
            if self.tcp_server_obj.ship_id_send_dict.get(self.ship_id):
                for info in self.tcp_server_obj.ship_id_send_dict.get(self.ship_id):
                    if info:
                        if 'S3' in info or count == init_count:
                            # 在指定状态才能发送指定状态消息
                            if self.ship_status not in [ShipStatus.computer_auto, ShipStatus.backhome_network,
                                                        ShipStatus.backhome_low_energy] and 'S1' in info:
                                continue
                            # if self.ship_status != ShipStatus.computer_control and 'S3' in info:
                            #     continue
                            if info.startswith('S3'):
                                if self.control_data == info:
                                    control_count -= 1
                                    control_count = max(control_count, -1)
                                else:
                                    control_count = 10
                                if control_count > 0:
                                    self.tcp_server_obj.write_data(self.ship_id, info)
                                self.control_data = info
                            else:
                                self.tcp_server_obj.write_data(self.ship_id, info)
                            # self.tcp_server_obj.write_data(self.ship_id, info)
                            # if info.startswith('S1'):
                            #     control_data=''
                            time.sleep(0.05)
                count += 1
                if count > init_count:
                    count = 0

    # 抽水排水控制
    def control_draw_thread(self):
        while True:
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出control_draw_thread线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出control_draw_thread线程": self.ship_id})
                return
            time.sleep(0.2)
            # 船号11 为采样检测测深船 深度数据从mqtt接受
            if self.ship_id == 11:
                self.deep = self.server_data_obj.mqtt_send_get_obj.deep
            else:
                if self.tcp_server_obj.ship_id_deep_dict.get(self.ship_id):
                    self.deep = self.tcp_server_obj.ship_id_deep_dict.get(self.ship_id)
            self.ship_type_obj.ship_obj.draw(self)
            # 定时发送心跳数据
            # if int(time.time()) % 3 == 2:
            #     self.tcp_send_data = 'S8Z'

    # 清楚所有状态
    def clear_all_status(self):
        self.logger.info('清除自动状态数据')
        self.server_data_obj.mqtt_send_get_obj.sampling_points.clear()
        self.server_data_obj.mqtt_send_get_obj.path_planning_points.clear()
        self.server_data_obj.mqtt_send_get_obj.sampling_points_status.clear()
        self.server_data_obj.mqtt_send_get_obj.sampling_points_gps.clear()
        self.smooth_path_lng_lat_index.clear()
        self.totle_distance = 0
        self.path_info = [0, 0]  # 清空提醒
        self.distance_p = 0
        self.smooth_path_lng_lat = None  # 清空路径
        self.server_data_obj.mqtt_send_get_obj.b_start = 0
        # self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
        self.server_data_obj.mqtt_send_get_obj.keep_point = 0
        self.point_arrive_start_time = None  # 清楚记录长期不到时间
        self.obstacle_avoid_time_distance = []
        self.b_at_home = 0

    # 处理状态切换
    def change_status(self):
        while True:
            # 删除任务模式，将抽水单独控制
            time.sleep(0.1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出change_status线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出change_status线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.client_dict and \
                    self.tcp_server_obj.ship_status_data_dict.get(self.ship_id) and \
                    self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[4] == 1:
                b_remote_control = 1
            else:
                b_remote_control = 0
            # 判断是否需要返航
            return_ship_status = None
            if self.ship_status != ShipStatus.at_home:
                return_ship_status = self.check_backhome()
            # 判断空闲状态切换到其他状态
            if self.ship_status == ShipStatus.idle:
                # 切换到遥控器控制模式
                if b_remote_control:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = ShipStatus.remote_control
                # 切换到电脑手动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction in [-1, 0, 90, 180, 270, 10, 190,
                                                                                       1180, 1270]:
                    self.ship_status = ShipStatus.computer_control
                # 摇杆开启导致切换到手动模式
                elif 0 <= self.server_data_obj.mqtt_send_get_obj.rocker_angle <= 360:
                    self.ship_status = ShipStatus.computer_control
                # 切换到自动模式
                elif len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    if self.lng_lat is None:
                        self.logger.error('无当前GPS，不能自主巡航')
                        time.sleep(0.5)
                    else:
                        self.ship_status = ShipStatus.computer_auto
                # 切换到抽水模式
                elif self.server_data_obj.mqtt_send_get_obj.b_draw == 1:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.last_ship_status = ShipStatus.computer_control
                    self.ship_status = ShipStatus.tasking
            # 判断电脑手动状态切换到其他状态
            if self.ship_status == ShipStatus.computer_control:
                # 切换到遥控器控制
                if b_remote_control:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    # 此时为遥控器控制模式 清除d控制状态
                    self.ship_status = ShipStatus.remote_control
                # 切换到自动巡航模式
                elif len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    if self.lng_lat is None:
                        self.logger.error('无当前GPS，不能自主巡航')
                        time.sleep(0.5)
                    else:
                        self.ship_status = ShipStatus.computer_auto
                # 点击抽水
                elif self.server_data_obj.mqtt_send_get_obj.b_draw:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.last_ship_status = ShipStatus.computer_control
                    self.ship_status = ShipStatus.tasking
                # 切换到返航
                elif return_ship_status is not None:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = return_ship_status
                    self.last_ship_status = ShipStatus.computer_control

            # 判断电脑自动切换到其他状态情况
            if self.ship_status == ShipStatus.computer_auto:
                # 切换到遥控器控制  此时等于暂停自动
                if b_remote_control:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    if self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:  # 清除暂停标记
                        self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type = 2
                    self.ship_status = ShipStatus.remote_control
                # 切换到返航
                elif return_ship_status is not None:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = return_ship_status
                    self.last_ship_status = ShipStatus.computer_auto
                # 取消自动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                    # self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.control_data = ''
                    self.clear_all_status()  # 取消自动时清楚所有自动信息标记
                    self.ship_status = ShipStatus.computer_control
                # 被暂停切换到手动
                elif self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = ShipStatus.computer_control
                # 到点
                elif self.b_arrive_point:
                    self.last_ship_status = ShipStatus.computer_auto
                    # if config.current_ship_type == config.ShipType.water_detect:
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 1
                    self.b_arrive_point = 0
                    self.ship_status = ShipStatus.tasking
                # 点击抽水
                elif self.server_data_obj.mqtt_send_get_obj.b_draw:
                    self.last_ship_status = ShipStatus.computer_auto
                    self.ship_status = ShipStatus.tasking

            # 判断任务模式切换到其他状态情况
            if self.ship_status == ShipStatus.tasking:
                # 切换到电脑自动模式  切换到电脑手动模式
                if self.server_data_obj.mqtt_send_get_obj.b_draw == 0:
                    # 如果自动每个点均已经到达
                    if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0 and \
                            all(self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                        print('self.server_data_obj.mqtt_send_get_obj.sampling_points_status',
                              self.server_data_obj.mqtt_send_get_obj.sampling_points_status)
                        self.clear_all_status()  # 最后一个任务点也到达后清楚状态
                        self.ship_status = ShipStatus.computer_control
                        # 自动模式下到达最后一个点切换为电脑手动状态
                        self.last_ship_status = ShipStatus.computer_control
                    # self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                    self.ship_status = self.last_ship_status

            # 遥控器状态切换到其他状态
            if self.ship_status == ShipStatus.remote_control:
                # 切换到电脑控制状态
                if not b_remote_control:
                    self.ship_status = ShipStatus.computer_control

            # 返航状态切换到其他状态
            if self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 判断是否返航到家
                if self.b_at_home:
                    self.ship_status = ShipStatus.at_home
                # 切换到遥控器模式 使能遥控器
                if b_remote_control:
                    self.ship_status = ShipStatus.remote_control
                # 切换到电脑手动控制
                if self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                    self.ship_status = ShipStatus.computer_control
                # 返航途中发现不满足返航条件 时退出返航状态
                if not return_ship_status:
                    self.ship_status = self.last_ship_status

            # 返航到家状态切换到其他状态
            if self.ship_status == ShipStatus.at_home:
                if self.server_data_obj.mqtt_send_get_obj.control_move_direction in [-1, 0, 90, 180, 270, 10, 190, 1180,
                                                                                     1270]:
                    self.ship_status = ShipStatus.computer_control
                # 切换到遥控器模式 使能遥控器
                if b_remote_control:
                    self.ship_status = ShipStatus.remote_control

    # 平滑路径
    def smooth_path(self):
        """
        平滑路径
        :return:平滑路径线路
        """
        smooth_path_lng_lat = []
        distance_matrix = []
        for index, target_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps):
            if index == 0:
                theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
                                                              self.lng_lat[1],
                                                              target_lng_lat[0],
                                                              target_lng_lat[1])
                distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                    self.lng_lat[1],
                                                                    target_lng_lat[0],
                                                                    target_lng_lat[1])
                if distance < config.smooth_path_ceil_size:
                    smooth_path_lng_lat.append(target_lng_lat)
                else:
                    for i in range(1, int((distance / config.smooth_path_ceil_size) + 1)):
                        cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                  self.lng_lat[1],
                                                                                  theta,
                                                                                  config.smooth_path_ceil_size * i)
                        smooth_path_lng_lat.append(cal_lng_lat)
                    smooth_path_lng_lat.append(target_lng_lat)
            else:
                theta = lng_lat_calculate.angleFromCoordinate(
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
                    target_lng_lat[0],
                    target_lng_lat[1])
                distance = lng_lat_calculate.distanceFromCoordinate(
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
                    target_lng_lat[0],
                    target_lng_lat[1])
                if distance < config.smooth_path_ceil_size:
                    smooth_path_lng_lat.append(target_lng_lat)
                else:
                    for i in range(1, int(distance / config.smooth_path_ceil_size + 1)):
                        cal_lng_lat = lng_lat_calculate.one_point_diatance_to_end(
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][0],
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps[index - 1][1],
                            theta,
                            config.smooth_path_ceil_size * i)
                        smooth_path_lng_lat.append(cal_lng_lat)
                    smooth_path_lng_lat.append(target_lng_lat)
        for smooth_lng_lat_i in smooth_path_lng_lat:
            distance_list = []
            for sampling_points_gps_i in self.server_data_obj.mqtt_send_get_obj.sampling_points_gps:
                s_d = lng_lat_calculate.distanceFromCoordinate(sampling_points_gps_i[0],
                                                               sampling_points_gps_i[1],
                                                               smooth_lng_lat_i[0],
                                                               smooth_lng_lat_i[1])
                distance_list.append(s_d)
            distance_matrix.append(distance_list)
        a_d_m = np.asarray(distance_matrix)
        for k in range(len(distance_matrix[0])):
            temp_a = a_d_m[:, k]
            temp_list = temp_a.tolist()
            index_l = temp_list.index(min(temp_list))
            self.smooth_path_lng_lat_index.append(index_l)
        return smooth_path_lng_lat

    # 根据当前点和路径计算下一个经纬度点
    def calc_target_lng_lat(self, index_):
        """
        根据当前点和路径计算下一个经纬度点
        :return:
        """
        # 离散按指定间距求取轨迹点数量
        if not self.smooth_path_lng_lat:
            self.smooth_path_lng_lat = self.smooth_path()
        # 搜索最临近的路点
        distance_list = []
        start_index = self.smooth_path_lng_lat_index[index_]
        cal_lng_lat = self.lng_lat
        # print('self.smooth_path_lng_lat, index_,', self.smooth_path_lng_lat_index, len(self.smooth_path_lng_lat),
        #       index_)
        # 限制后面路径点寻找时候不能找到之前采样点路径上
        if index_ == 0:
            self.search_list = copy.deepcopy(self.smooth_path_lng_lat[:start_index])
        else:
            self.search_list = copy.deepcopy(
                self.smooth_path_lng_lat[self.smooth_path_lng_lat_index[index_ - 1]:start_index])
        for target_lng_lat in self.search_list:
            distance = lng_lat_calculate.distanceFromCoordinate(cal_lng_lat[0],
                                                                cal_lng_lat[1],
                                                                target_lng_lat[0],
                                                                target_lng_lat[1])
            distance_list.append(distance)
        # print('distance_list',distance_list)
        # 如果没有可以去路径
        if len(distance_list) == 0:
            return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
        index = distance_list.index(min(distance_list))
        # if index + 1 == len(self.search_list):
        #     return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
        lng_lat = self.search_list[index]
        index_point_distance = lng_lat_calculate.distanceFromCoordinate(cal_lng_lat[0],
                                                                        cal_lng_lat[1],
                                                                        lng_lat[0],
                                                                        lng_lat[1])
        while config.forward_target_distance > index_point_distance and (index + 1) < len(
                self.search_list):
            lng_lat = self.search_list[index]
            index_point_distance = lng_lat_calculate.distanceFromCoordinate(cal_lng_lat[0],
                                                                            cal_lng_lat[1],
                                                                            lng_lat[0],
                                                                            lng_lat[1])
            if config.forward_target_distance * 1.2 < index_point_distance:
                break
            index += 1
        # 超过第一个点后需要累积之前计数
        if index_ > 0:
            self.path_info = [self.smooth_path_lng_lat_index[index_ - 1] + index, len(self.smooth_path_lng_lat)]
        else:
            self.path_info = [index, len(self.smooth_path_lng_lat)]
        # print('index_point_distance', index_point_distance)
        return self.search_list[index]

    def get_avoid_obstacle_point(self, path_planning_point_gps=None, sampling_point_gps=None):
        """
        根据障碍物地图获取下一个运动点
        :return: 下一个目标点，是否需要紧急停止【True为需要停止，False为不需要停止】
        """
        next_point_lng_lat = copy.deepcopy(path_planning_point_gps)
        angle = vfh.vfh_func(self.obstacle_list, self.ceil_go_throw)
        # print('避障角度：', angle)
        distance_sample = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0],
            self.lng_lat[1],
            sampling_point_gps[0],
            sampling_point_gps[1])
        # 记录当前时间和到目标点距离，当前角度
        self.obstacle_avoid_time_distance.append(
            [time.time(), distance_sample, self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[3]])
        # 判断指定时间 内避障行走距离是否小于指定距离米 或者避障角度小于指定度数
        if len(self.obstacle_avoid_time_distance) > 2:
            if abs(self.obstacle_avoid_time_distance[0][0] - self.obstacle_avoid_time_distance[-1][0]) >= 5:
                if abs(self.obstacle_avoid_time_distance[0][1] - self.obstacle_avoid_time_distance[-1][1]) < 2 and \
                        abs(self.obstacle_avoid_time_distance[0][2] - self.obstacle_avoid_time_distance[-1][2]) < 50:
                    self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type = 1
                    print('############################无法避障暂停 距离 度数#############################',
                          abs(self.obstacle_avoid_time_distance[0][1] - self.obstacle_avoid_time_distance[-1][1]),
                          abs(self.obstacle_avoid_time_distance[0][2] - self.obstacle_avoid_time_distance[-1][2]))
                    self.obstacle_avoid_time_distance = []
                else:
                    del self.obstacle_avoid_time_distance[:-1]  # 在移动，保留最新一条数据删除其他所有
        if angle == -1:  # 没有可通行区域
            abs_angle = (self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[
                             3] + config.field_of_view / 2) % 360
            next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                             self.lng_lat[1],
                                                                             abs_angle,
                                                                             1)
            self.b_avoid = 1
            return next_point_lng_lat, False
        elif angle == 0:  # 当前船头角度可通行
            self.b_avoid = 0
            return next_point_lng_lat, False
        else:  # 船头角度不能通过但是传感器检测其他角度可以通过
            abs_angle = (self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[3] + angle) % 360
            next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                             self.lng_lat[1],
                                                                             abs_angle,
                                                                             2)
            self.b_avoid = 1
            # print('绕行角度:', abs_angle)
            return next_point_lng_lat, False

    # 处理电机控制
    def move_control(self):
        while True:
            time.sleep(0.01)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出move_control线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出move_control线程": self.ship_id})
                return
            self.direction = self.server_data_obj.mqtt_send_get_obj.control_move_direction
            control_info_dict = {
                ShipStatus.computer_control: '手动',
                ShipStatus.remote_control: '遥控',
                ShipStatus.computer_auto: '自动',
                ShipStatus.tasking: '抽水中',
                ShipStatus.backhome_network: '返航',
                ShipStatus.backhome_low_energy: '返航',
                ShipStatus.at_home: '返航点',
                ShipStatus.idle: '等待',
            }
            # 判断船是否能给用户点击再次运动
            if self.ship_status in [ShipStatus.idle, ShipStatus.remote_control, ShipStatus.computer_control,
                                    ShipStatus.at_home]:
                # 暂停状态设定为非空闲
                if self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                    self.is_wait = 2  # 非空闲
                else:
                    self.is_wait = 1  # 是空闲
            else:
                self.is_wait = 2  # 非空闲
            self.control_info = ''
            if self.ship_status in control_info_dict:
                if self.ship_status == ShipStatus.tasking:
                    if self.tcp_server_obj.ship_draw_dict.get(self.ship_id):
                        if self.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 2:
                            self.control_info += control_info_dict[self.ship_status]
                        elif self.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 1:
                            self.control_info += '调节深度'
                else:
                    self.control_info += control_info_dict[self.ship_status]
            # 电脑手动
            if self.ship_status == ShipStatus.computer_control or self.ship_status == ShipStatus.tasking:
                if 0 <= self.server_data_obj.mqtt_send_get_obj.rocker_angle < 360:
                    #         90                0
                    # 将  180       0  ->   90        270
                    #         270               180
                    rocker_angle = (self.server_data_obj.mqtt_send_get_obj.rocker_angle + 270) % 360
                    # 左右容易推到左后 右后 造成转向误差 在左后 右后增加一定范围映射到左上右上
                    threshold = 45
                    if 90 < rocker_angle < 90 + threshold:
                        rocker_angle = 90
                    if 270 - threshold < rocker_angle < 270:
                        rocker_angle = 270
                    control_data = 'S3,%d,1Z' % rocker_angle
                    self.set_send_data(control_data, 3)
                    # if self.pre_control_data is None or self.pre_control_data != control_data:
                    #     self.tcp_send_data = control_data
                    #     self.pre_control_data = self.tcp_send_data
                if self.server_data_obj.mqtt_send_get_obj.rocker_angle == -1:
                    if self.direction == -1:
                        stop_pause = 3
                    elif self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                        stop_pause = 4
                    else:
                        stop_pause = 1
                    # print('self.direction  pause_continue_data_type',self.direction,self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type)
                    if stop_pause:
                        control_data = 'S3,%d,%dZ' % (
                            self.server_data_obj.mqtt_send_get_obj.rocker_angle, stop_pause)
                        self.set_send_data(control_data, 3)
                        # if self.pre_control_data is None or self.pre_control_data != control_data:
                        #     self.tcp_send_data = control_data
                        #     self.pre_control_data = self.tcp_send_data
            # 电脑自动
            elif self.ship_status == ShipStatus.computer_auto:
                # 计算总里程 和其他需要在巡航开始前计算数据
                if self.totle_distance == 0:
                    for index, gaode_lng_lat in enumerate(self.server_data_obj.mqtt_send_get_obj.path_planning_points):
                        if index == 0:
                            distance_p = lng_lat_calculate.distanceFromCoordinate(
                                self.gaode_lng_lat[0],
                                self.gaode_lng_lat[1],
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
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps = []
                    self.server_data_obj.mqtt_send_get_obj.sampling_points_gps = []
                    # 将目标点转换为真实经纬度
                    for path_planning_point in self.server_data_obj.mqtt_send_get_obj.path_planning_points:
                        path_planning_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
                                                                                     self.gaode_lng_lat,
                                                                                     path_planning_point)
                        self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps.append(
                            path_planning_point_gps)
                    for sampling_point in self.server_data_obj.mqtt_send_get_obj.sampling_points:
                        sampling_point_gps = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
                                                                                self.gaode_lng_lat,
                                                                                sampling_point)
                        self.server_data_obj.mqtt_send_get_obj.sampling_points_gps.append(sampling_point_gps)
                    self.path_info = [0, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
                while self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) > 0:
                    # 清空经纬度不让船移动
                    # control_data = ''
                    # self.set_send_data(control_data, 3)
                    # 被暂停
                    if self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                        if self.ship_status != ShipStatus.computer_auto:  # 暂停时允许使用遥控器取消暂停状态
                            break
                    # 判断是否接受到开始行动
                    if self.server_data_obj.mqtt_send_get_obj.task_id and self.server_data_obj.mqtt_send_get_obj.action_type != 1:
                        # 还没点击开始行动就点结束则取消行动
                        if self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                            self.server_data_obj.mqtt_send_get_obj.cancel_action = 1
                        continue
                    if self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) <= 0:
                        break
                    index = self.server_data_obj.mqtt_send_get_obj.sampling_points_status.index(0)
                    sampling_point_gps = self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index]
                    # 计算下一个目标点经纬度
                    try:
                        next_lng_lat = self.calc_target_lng_lat(index)
                    except Exception as e:
                        self.logger.error({'平滑路径报错': e})
                        next_lng_lat = sampling_point_gps
                    # next_lng_lat = sampling_point_gps
                    # 当前位置到采样点距离
                    arrive_sample_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                                      self.lng_lat[1],
                                                                                      sampling_point_gps[0],
                                                                                      sampling_point_gps[1])

                    # 调试用 10秒后认为到达目的点
                    # if config.current_platform == config.CurrentPlatform.windows:
                    #     if self.point_arrive_start_time is None:
                    #         self.point_arrive_start_time = time.time()
                    #     if time.time() - self.point_arrive_start_time > 10:
                    #         arrive_sample_distance = 1
                    # 如果该点已经到达目的地
                    # print('arrive_sample_distance',arrive_sample_distance,self.lng_lat,sampling_point_gps)
                    if arrive_sample_distance < config.arrive_distance:
                        # 清空经纬度不让船移动
                        if index != 0:
                            control_data = 'S3,-1,3Z'
                            self.set_send_data(control_data, 3)
                        # 判断是否是行动抽水
                        if self.action_id and self.sample_index and self.sample_index[index]:
                            self.b_arrive_point = 1  # 到点了用于通知抽水  暂时修改为不抽水
                            self.current_arriver_index = index  # 当前到达点下标
                            print('######################################到达下标点################',
                                  self.current_arriver_index)
                        if self.ship_type_obj.ship_type == config.ShipType.water_detect and index != 0:
                            self.b_arrive_point = 1  # 到点了用于通知抽水
                        self.point_arrive_start_time = None
                        self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] = 1
                        # 全部点到达后清除自动状态
                        if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) == sum(
                                self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                            time.sleep(2)
                            self.is_plan_all_arrive = 1
                            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
                    else:
                        if arrive_sample_distance < config.forward_target_distance:
                            send_lng_lat = sampling_point_gps
                        else:
                            send_lng_lat = next_lng_lat
                        if self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_type != 0:
                            send_lng_lat, b_stop = self.get_avoid_obstacle_point(send_lng_lat, sampling_point_gps)
                        send_data = 'S1,%d,%dZ' % (
                            round(send_lng_lat[0], 6) * 1000000, round(send_lng_lat[1], 6) * 1000000)
                        self.set_send_data(send_data, 1)
                        arrive_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                                         self.lng_lat[1],
                                                                                         send_lng_lat[0],
                                                                                         send_lng_lat[1])
                        # print('##############发送经纬度', arrive_sample_distance, arrive_point_distance, self.lng_lat,
                        #       send_lng_lat)
                    if self.ship_status != ShipStatus.computer_auto:
                        break

            # 返航 断网返航 低电量返航
            elif self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 有返航点下情况下返回返航点，没有则停止
                if self.home_lng_lat:
                    send_lng_lat = self.home_lng_lat
                    send_data = 'S1,%d,%dZ' % (
                        round(send_lng_lat[0], 6) * 1000000, round(send_lng_lat[1], 6) * 1000000)
                    print('########设置返航', send_data)
                    # self.tcp_send_data = send_data
                    self.set_send_data(send_data, 1)
                else:
                    pass  # 没有返航点

    def update_ship_gaode_lng_lat(self):
        # 更新经纬度为高德经纬度
        while True:
            time.sleep(1)  # 间隔指定时间
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出update_ship_gaode_lng_lat线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出update_ship_gaode_lng_lat线程": self.ship_id})
                return
            if self.lng_lat is not None:
                try:
                    b_request_gaode = 0  # 是否请求高德
                    if self.last_get_gaode_lng_lat:
                        # 如果移动距离超过1m才请求不然不发送请求
                        speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_get_gaode_lng_lat[0],
                                                                                  self.last_get_gaode_lng_lat[1],
                                                                                  self.lng_lat[0],
                                                                                  self.lng_lat[1])
                        if speed_distance > 1:
                            b_request_gaode = 1
                    else:
                        b_request_gaode = 1
                    if b_request_gaode:
                        gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
                        if gaode_lng_lat:
                            self.gaode_lng_lat = gaode_lng_lat
                            self.last_get_gaode_lng_lat = copy.deepcopy(self.lng_lat)
                            if not self.home_lng_lat:
                                self.home_lng_lat = copy.deepcopy(self.lng_lat)
                                self.home_gaode_lng_lat = copy.deepcopy(self.gaode_lng_lat)
                except Exception as e:
                    self.logger.error({'error': e})

    # 更新经纬度
    def update_lng_lat(self):
        last_read_time = None
        while True:
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出update_lng_lat线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出update_lng_lat线程": self.ship_id})
                return
            if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id) and \
                    180 > self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[0] > 10 and \
                    90 > self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[1] > 10:
                self.lng_lat = copy.deepcopy(self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[:2])
                if not last_read_time:
                    last_read_time = time.time()
                if self.last_lng_lat and last_read_time:
                    # 计算当前行驶里程
                    speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
                                                                              self.last_lng_lat[1],
                                                                              self.lng_lat[0],
                                                                              self.lng_lat[1])
                    if speed_distance > 100:
                        self.logger.error(
                            {"经纬度计算报错": [speed_distance, self.last_lng_lat[0], self.last_lng_lat[1], self.lng_lat[0],
                                         self.lng_lat[1]]})
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                    else:
                        self.run_distance += speed_distance
                        if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id):
                            speed_scale = 1.1  # 速度放大比例
                            self.speed = round(
                                speed_scale * self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[6], 1)
                            # print('sudu',self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[6],self.speed)
                        # self.speed = round(speed_distance / (time.time() - last_read_time), 1)
                        # 替换上一次的值
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                        last_read_time = time.time()
                else:
                    self.last_lng_lat = copy.deepcopy(self.lng_lat)
                    last_read_time = time.time()
            time.sleep(0.2)

    # 必须使用线程发送mqtt状态数据
    def send_mqtt_status_data(self):
        last_runtime = None
        last_run_distance = None
        while True:
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出send_mqtt_status_data线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出send_mqtt_status_data线程": self.ship_id})
                return
            if self.ship_id not in self.tcp_server_obj.client_dict:
                continue
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            if self.ship_id not in self.tcp_server_obj.client_dict:
                continue
            if self.server_data_obj.mqtt_send_get_obj.pool_code:
                self.data_define_obj.pool_code = self.server_data_obj.mqtt_send_get_obj.pool_code
            status_data = self.data_define_obj.status
            status_data.update({'mapId': self.data_define_obj.pool_code})
            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            status_data.update({'ping': round(self.ping, 1)})
            status_data.update({'current_lng_lat': self.gaode_lng_lat})
            status_data.update({'draw_time': self.dump_draw_list})
            status_data.update({'action_type': self.server_data_obj.mqtt_send_get_obj.action_type})
            status_data.update({'pause_continue': self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type})
            # 更新返航点
            if self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat:
                status_data.update({'home_lng_lat': self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat})
                self.home_gaode_lng_lat = copy.deepcopy(self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat)
                self.home_lng_lat = lng_lat_calculate.gps_gaode_to_gps(self.lng_lat,
                                                                       self.gaode_lng_lat,
                                                                       self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat)
                self.server_data_obj.mqtt_send_get_obj.set_home_gaode_lng_lat = None
            elif self.home_gaode_lng_lat:
                status_data.update({'home_lng_lat': self.home_gaode_lng_lat})
            # 更新速度  更新里程
            if self.speed is not None:
                status_data.update({'speed': str(self.speed)})
            status_data.update({"runtime": round(time.time() - self.start_time)})
            status_data.update({"run_distance": round(self.run_distance, 1)})
            if last_runtime is None:
                last_runtime = 0
            if last_run_distance is None:
                last_run_distance = 0
            status_data.update({"totle_distance": 0 if not self.http_save_distance else self.http_save_distance})
            status_data.update({"totle_time": 0 if not self.http_save_time else self.http_save_time})
            # 发送GPS和毫米波在线状态
            if self.tcp_server_obj.gps_millimeter_wave_online.get(self.ship_id):
                if self.tcp_server_obj.gps_millimeter_wave_online.get(self.ship_id).get("gps"):
                    status_data.update({"gps": 1})
                if self.tcp_server_obj.gps_millimeter_wave_online.get(self.ship_id).get("millimeter_wave"):
                    status_data.update({"millimeter_wave": 1})
            # 更新船头方向
            if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id):
                status_data.update({"direction": self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[3]})
            # 更新电量
            if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id):
                self.dump_energy = self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[2]
                self.dump_energy_deque.append(self.dump_energy)
                status_data.update({'dump_energy': self.dump_energy})
            if self.server_data_obj.mqtt_send_get_obj.b_record_point:
                status_data.update({'is_record': 1})
            else:
                status_data.update({'is_record': 2})
            # 更新船是否能运动状态
            status_data.update({'is_wait': self.is_wait})
            # 向mqtt发送数据
            self.send(method='mqtt', topic='status_data_%s' % self.ship_code, data=status_data,
                      qos=0)
            if time.time() % 20 < 1:
                self.logger.info({'status_data_': status_data})

    # 配置更新
    def update_config(self):
        while True:
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出update_config线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出update_config线程": self.ship_id})
                return
            # 客户端获取基础设置数据
            if self.server_data_obj.mqtt_send_get_obj.base_setting_data_info in [1, 4]:
                if self.server_data_obj.mqtt_send_get_obj.base_setting_data is None:
                    self.logger.error(
                        {'base_setting_data is None': self.server_data_obj.mqtt_send_get_obj.base_setting_data})
                else:
                    self.server_data_obj.mqtt_send_get_obj.base_setting_data.update({'info_type': 3})
                    # 删除湖泊名称和安全距离 这两个值放到服务器上
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('pool_name') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['pool_name']
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('secure_distance') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['secure_distance']
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('keep_point') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['keep_point']
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('video_url') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['video_url']
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('row') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['row']
                    if self.server_data_obj.mqtt_send_get_obj.base_setting_data.get('col') is not None:
                        del self.server_data_obj.mqtt_send_get_obj.base_setting_data['col']
                    self.send(method='mqtt', topic='base_setting_%s' % self.ship_code,
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
                    self.send(method='mqtt', topic='height_setting_%s' % self.ship_code,
                              data=self.server_data_obj.mqtt_send_get_obj.height_setting_data,
                              qos=0)
                    self.logger.info({'height_setting': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data = None
                    # 改为0位置状态，不再重复发送
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data_info = 0

    # 快速发送船头角度和误差角度
    def send_high_f_status_data(self):
        high_f_status_data = {}
        while 1:
            time.sleep(0.16)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出send_high_f_status_data线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出send_high_f_status_data线程": self.ship_id})
                return
            if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id):
                high_f_status_data.update(
                    {"direction": self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[3]})
                if self.ship_status != ShipStatus.computer_auto:
                    high_f_status_data.update(
                        {"theta_error": "0"})
                else:
                    high_f_status_data.update(
                        {"theta_error": str(self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[5])})
                self.send(method='mqtt', topic='high_f_status_data_%s' % self.ship_code, data=high_f_status_data,
                          qos=0)

    # 检查任务
    def loop_check_task(self):
        while True:
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出loop_check_task线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出loop_check_task线程": self.ship_id})
                return
            self.ship_type_obj.ship_obj.task(self)

    # 状态检查函数，检查自身状态发送对应消息
    def check_status(self):
        while True:
            # 循环等待一定时间
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出change_status线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出check_status线程": self.ship_id})
                return
            time.sleep(1)
            if self.ship_id not in self.tcp_server_obj.client_dict:
                continue
            # 5：水泵长时间空抽，6：水泵堵抽，7：出现舵机堵转
            pump = 0
            steer = 0
            # 检测到异常且还一次都没有发送
            if self.tcp_server_obj.ship_draw_dict.get(self.ship_id):
                if self.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 5:
                    pump = 1
                elif self.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 6:
                    pump = 2
                elif self.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 7:
                    steer = 1
                # 需要发送消息提示前端并停止抽水
                if pump or steer:
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0  # 设置停止抽水
                    if self.server_data_obj.mqtt_send_get_obj.abnormal_confirm == 0:
                        self.send(
                            method='mqtt',
                            topic='pump_steer_abnormal_%s' % self.ship_code,
                            data={"pump": pump,
                                  "steer": steer},
                            qos=0)
            # 检查电量 如果连续20次检测电量平均值低于电量阈值就报警
            if self.server_data_obj.mqtt_send_get_obj.energy_backhome:
                energy_backhome_threshold = max(self.server_data_obj.mqtt_send_get_obj.energy_backhome, 10)
                if len(self.dump_energy_deque) > 0 and sum(self.dump_energy_deque) / len(
                        self.dump_energy_deque) < energy_backhome_threshold:
                    self.low_dump_energy_warnning = 1
                else:
                    self.low_dump_energy_warnning = 0
            else:
                self.low_dump_energy_warnning = 0
            # if len(self.dump_energy_deque) > 0:
            #     print('self.dump_energy_deque',self.dump_energy_deque,self.server_data_obj.mqtt_send_get_obj.energy_backhome,sum(self.dump_energy_deque) / len(
            #                 self.dump_energy_deque))
            if self.server_data_obj.mqtt_send_get_obj.energy_backhome != self.server_data_obj.mqtt_send_get_obj.pre_energy_backhome or \
                    self.server_data_obj.mqtt_send_get_obj.network_backhome != self.server_data_obj.mqtt_send_get_obj.pre_network_backhome or \
                    self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_type != self.server_data_obj.mqtt_send_get_obj.pre_obstacle_avoid_type or \
                    self.server_data_obj.mqtt_send_get_obj.max_pwm_grade != self.server_data_obj.mqtt_send_get_obj.pre_max_pwm_grade:
                if self.server_data_obj.mqtt_send_get_obj.network_backhome != 0:
                    n = 1
                else:
                    n = 0
                if self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_type != 0:
                    o = 1
                else:
                    o = 0
                send_data = 'S4,%d,%d,%d,3,%dZ' % (n, self.server_data_obj.mqtt_send_get_obj.energy_backhome, o,
                                                   self.server_data_obj.mqtt_send_get_obj.max_pwm_grade)
                print('######## 设置改变##############', send_data)
                # self.tcp_send_data = 'S4,%d,%d,%d,3,3Z' % (n, e, o)
                self.set_send_data(send_data, 4)
                self.server_data_obj.mqtt_send_get_obj.pre_energy_backhome = self.server_data_obj.mqtt_send_get_obj.energy_backhome
                self.server_data_obj.mqtt_send_get_obj.pre_network_backhome = self.server_data_obj.mqtt_send_get_obj.network_backhome
                self.server_data_obj.mqtt_send_get_obj.pre_obstacle_avoid_type = self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_type
                self.server_data_obj.mqtt_send_get_obj.pre_max_pwm_grade = self.server_data_obj.mqtt_send_get_obj.max_pwm_grade
            if self.server_data_obj.mqtt_send_get_obj.calibration_compass != self.server_data_obj.mqtt_send_get_obj.pre_calibration_compass:
                send_data = 'S6,%dZ' % self.server_data_obj.mqtt_send_get_obj.calibration_compass
                print('######## 改变校准罗盘##############', send_data)
                self.set_send_data(send_data, 6)
                self.server_data_obj.mqtt_send_get_obj.pre_calibration_compass = self.server_data_obj.mqtt_send_get_obj.calibration_compass
            # 接收到重置湖泊按钮
            if self.server_data_obj.mqtt_send_get_obj.reset_pool_click:
                self.data_define_obj.pool_code = ''
                self.server_data_obj.mqtt_send_get_obj.pool_code = ''
                self.server_data_obj.mqtt_send_get_obj.reset_pool_click = 0
            # 船状态提示消息
            if int(self.path_info[1]) == 0:
                progress = 0
            else:
                progress = int((float(self.path_info[0]) / float(self.path_info[1])) * 100)
            notice_info_data = {
                "distance": str(round(self.distance_p, 1)),  # 船到下一点距离
                "bank_distance": self.server_data_obj.mqtt_send_get_obj.bank_distance,
                "progress": progress,
                "control_info": self.control_info,  # 船状态提示信息
                # 水泵开关状态消息
                "draw_info": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 声光报警器
                "audio_light_info": self.server_data_obj.mqtt_send_get_obj.audio_light,
                # 大灯
                "headlight_info": self.server_data_obj.mqtt_send_get_obj.headlight,
                # 舷灯
                "side_light_info": self.server_data_obj.mqtt_send_get_obj.side_light,
                # adcp开关信息
                "adcp_info": self.server_data_obj.mqtt_send_get_obj.adcp
            }
            notice_info_data.update({"mapId": self.data_define_obj.pool_code})
            # 遥控器是否启用
            if self.tcp_server_obj.ship_status_data_dict.get(self.ship_id) and \
                    self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[4] == 1:
                notice_info_data.update({"b_start_remote": "1"})
            else:
                notice_info_data.update({"b_start_remote": "0"})
            # 使用电量告警是提示消息
            if self.low_dump_energy_warnning:
                notice_info_data.update({"low_dump_energy_warnning": self.low_dump_energy_warnning})
            self.send(
                method='mqtt',
                topic='notice_info_%s' % self.ship_code,
                data=notice_info_data,
                qos=0)
            if time.time() % 10 < 1:
                self.logger.info({'notice_info_': notice_info_data})
            # 保存数据与发送刷新后提示消息
            if len(self.data_define_obj.pool_code) > 0:
                save_plan_path_data = {
                    "mapId": self.data_define_obj.pool_code,
                    'sampling_points': self.server_data_obj.mqtt_send_get_obj.sampling_points,
                    'path_points': self.server_data_obj.mqtt_send_get_obj.path_planning_points,
                }
                if self.server_data_obj.mqtt_send_get_obj.draw_bottle_id \
                        and self.server_data_obj.mqtt_send_get_obj.draw_deep and \
                        self.server_data_obj.mqtt_send_get_obj.draw_capacity:
                    save_plan_path_data.update({'bottle_info': [self.server_data_obj.mqtt_send_get_obj.draw_bottle_id,
                                                                self.server_data_obj.mqtt_send_get_obj.draw_deep,
                                                                self.server_data_obj.mqtt_send_get_obj.draw_capacity]})
                # save_data.set_data(save_plan_path_data, config.save_plan_path)
                if self.server_data_obj.mqtt_send_get_obj.refresh_info_type == 1:
                    save_plan_path_data.update({"info_type": 2})
                    # self.send_obstacle = True  # 调试时发送障碍物图标
                    self.send(method='mqtt',
                              topic='refresh_%s' % self.ship_code,
                              data=save_plan_path_data,
                              qos=0)
            else:
                self.server_data_obj.mqtt_send_get_obj.refresh_info_type = 2

    # 检测网络延时
    def check_ping_delay(self):
        # 检查网络
        while True:
            time.sleep(3)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出check_ping_delay线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接check_ping_delay退出线程": self.ship_id})
                return
            if not self.b_check_get_water_data and self.gaode_lng_lat is not None and \
                    self.ship_type_obj.ship_type == config.ShipType.water_detect:
                adcode = baidu_map.BaiduMap.get_area_code(self.gaode_lng_lat)
                if adcode:
                    self.area_id = data_valid.adcode_2_area_id(adcode)
                    try:
                        data_valid.get_current_water_data(area_id=self.area_id)
                    except Exception as e:
                        self.logger.error({'请求数据报错': e})
                    self.b_check_get_water_data = 1
            # 发送模拟障碍物
            if self.send_obstacle:
                obstacle_points = {"lng_lat": config.obstacle_points}
                self.send(method='mqtt',
                          data=obstacle_points,
                          topic='obstacle_points_%s' % self.ship_code,
                          qos=0
                          )
                self.logger.info({"obstacle_points": obstacle_points})
                self.send_obstacle = False
            ping = check_network.get_ping_delay()
            if ping:
                self.ping = round(ping, 1)
            else:
                self.logger.error('当前无网络信号')
                # self.server_data_obj.mqtt_send_get_obj.is_connected = 0
            self.logger.info({'ping': self.ping})

    # 发送障碍物信息线程
    def send_distacne(self):
        min_distance = 40
        while True:
            time.sleep(0.2)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出send_distacne线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出send_distacne线程": self.ship_id})
                return
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            distance_info_data = {}
            # print('避障距离:', self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_distance)
            self.obstacle_list = [0] * self.cell_size
            if self.tcp_server_obj.ship_obstacle_data_dict.get(self.ship_id):
                distance_info_data.update({'deviceId': self.ship_code})
                distance_info_data.update({'distance_info': []})
                for k in self.tcp_server_obj.ship_obstacle_data_dict.get(self.ship_id):
                    # 将原来左负右正改为左正右负，然后将负数角度转为正
                    angle = -1 * self.tcp_server_obj.ship_obstacle_data_dict.get(self.ship_id)[k][0]
                    distance = self.tcp_server_obj.ship_obstacle_data_dict.get(self.ship_id)[k][1]
                    obstacle_index = angle // config.view_cell + self.cell_size // 2
                    if distance > self.server_data_obj.mqtt_send_get_obj.obstacle_avoid_distance:
                        b_obstacle = 0
                    else:
                        b_obstacle = 1
                    # 不要超过检测范围角度的值
                    if obstacle_index < 0 or obstacle_index >= self.cell_size:
                        print("obstacle_index", obstacle_index, self.obstacle_list)
                        continue
                    self.obstacle_list[obstacle_index] = b_obstacle
                    if angle < 0:
                        angle = 360 + angle
                    angle = 360 - angle
                    min_distance = min(distance, min_distance)
                    distance_info_data['distance_info'].append(
                        {'distance': distance,
                         'angle': angle})
                # 计算前视野范围可通行区域  障碍物距离不同可通行单元格不同  障碍物距离按检测到最近的障碍物计算
                # 设定船宽度为0.7米
                ship_width = 0.5
                for i in range(1, config.field_of_view // config.view_cell):
                    angle = i * config.view_cell
                    if min_distance * math.sin(math.radians(angle)) < ship_width:
                        self.ceil_go_throw = i
                if self.ship_id in self.tcp_server_obj.ship_status_data_dict:
                    distance_info_data.update(
                        {'direction': round(self.tcp_server_obj.ship_status_data_dict.get(self.ship_id)[3], 1)})
                else:
                    distance_info_data.update({'direction': 0})
                self.send(method='mqtt',
                          topic='distance_info_%s' % self.ship_code,
                          data=distance_info_data,
                          qos=0)
            else:
                distance_info_data.update({'deviceId': self.ship_code})
                distance_info_data.update({'distance_info': []})
                self.send(method='mqtt',
                          topic='distance_info_%s' % self.ship_code,
                          data=distance_info_data,
                          qos=0)
                time.sleep(0.05)  # 发送数据后延时一点时间
                min_distance = 40
                self.ceil_go_throw = 1

    # 检查开关相关信息
    def check_switch(self):
        """
        检查开关信息发送到mqtt
        :return:
        """
        while True:
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出check_switch线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出check_switch线程": self.ship_id})
                return
            switch_data = {
                "info_type": 2,  # 树莓派发给前端
                # 抽水 1 抽水 没有该键或者0表示不抽水
                "b_draw": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 前大灯 1 打开前大灯 没有该键或者0表示不打开
                # "headlight": self.server_data_obj.mqtt_send_get_obj.headlight,
                # 声光报警器 1 打开声光报警器 没有该键或者0表示不打开
                # "audio_light": self.server_data_obj.mqtt_send_get_obj.audio_light,
                # 舷灯 1 允许打开舷灯 没有该键或者0表示不打开
                "side_light": self.server_data_obj.mqtt_send_get_obj.side_light,
                # adcp
                "adcp": self.server_data_obj.mqtt_send_get_obj.adcp
            }
            self.send(method='mqtt',
                      topic='switch_%s' % self.ship_code,
                      data=switch_data,
                      qos=0)

    # 检查手动记录路径点
    def send_record_point_data(self):
        """
        检查开关信息发送到mqtt
        :return:
        """
        pre_record_lng_lat = [10.0, 10.0]  # 随便设置的初始值
        while True:
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出send_record_point_data线程": self.ship_id})
                return
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出send_record_point_data线程": self.ship_id})
                return
            if self.server_data_obj.mqtt_send_get_obj.b_record_point:
                if self.gaode_lng_lat is None:
                    time.sleep(3)
                    continue
                record_distance = lng_lat_calculate.distanceFromCoordinate(pre_record_lng_lat[0],
                                                                           pre_record_lng_lat[1],
                                                                           self.gaode_lng_lat[0],
                                                                           self.gaode_lng_lat[1],
                                                                           )
                if record_distance > self.server_data_obj.mqtt_send_get_obj.record_distance:
                    record_point_data = {
                        "lng_lat": self.gaode_lng_lat
                    }
                    self.send(method='mqtt',
                              topic='record_point_data_%s' % self.ship_code,
                              data=record_point_data,
                              qos=0)
                    self.logger.info({"发送手动记录点存储": record_point_data})
                    self.record_path.append(self.gaode_lng_lat)
                    pre_record_lng_lat = self.gaode_lng_lat
                    time.sleep(1)
            else:
                # 是否存在记录点
                if len(self.record_path) > 0:
                    # 湖泊轮廓id是否存在
                    if self.server_data_obj.mqtt_send_get_obj.pool_code:
                        send_record_path = {
                            "deviceId": self.ship_code,
                            "mapId": self.server_data_obj.mqtt_send_get_obj.pool_code,
                            "route": json.dumps(self.record_path),
                            "routeName": self.server_data_obj.mqtt_send_get_obj.record_name
                        }
                        return_data = self.server_data_obj.send_server_http_data('POST', send_record_path,
                                                                                 config.http_record_path,
                                                                                 token=self.token)
                        if return_data:
                            content_data = json.loads(return_data.content)
                            if content_data.get("code") != 200 and content_data.get("code") != 20000:
                                self.logger.error('发送手动记录点存储数据请求失败')
                            else:
                                self.logger.info({"发送手动记录点存储数据": send_record_path})
                                self.record_path = []
                                pre_record_lng_lat = [10.0, 10.0]  # 随便设置的初始值
                        else:
                            self.logger.info({"发送手动记录点存储数据error": 11})
            time.sleep(3)

    # 检测扫描点
    def scan_cal(self):
        scan_points = []
        while True:
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出scan_cal线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出scan_cal线程": self.ship_id})
                return
            if self.server_data_obj.mqtt_send_get_obj.surrounded_points is not None and \
                    len(self.server_data_obj.mqtt_send_get_obj.surrounded_points) > 0 and \
                    config.scan_gap:
                print('surrounded_points,config.scan_gap', self.server_data_obj.mqtt_send_get_obj.surrounded_points,
                      self.server_data_obj.mqtt_send_get_obj.scan_gap)
                scan_points = baidu_map.BaiduMap.scan_area(self.server_data_obj.mqtt_send_get_obj.surrounded_points,
                                                           self.server_data_obj.mqtt_send_get_obj.scan_gap)
                if scan_points:
                    self.send(method='mqtt',
                              topic='surrounded_path_%s' % self.ship_code,
                              data={
                                  "lng_lat": scan_points
                              },
                              qos=0)
                self.server_data_obj.mqtt_send_get_obj.surrounded_points = None
            # 收到开始命令将路径点赋值到需要行驶路劲中
            if self.server_data_obj.mqtt_send_get_obj.surrounded_start == 1 and len(scan_points) > 0:
                # 对点进行排序
                if self.gaode_lng_lat:
                    dis_start = lng_lat_calculate.distanceFromCoordinate(
                        self.gaode_lng_lat[0],
                        self.gaode_lng_lat[1],
                        scan_points[0][0],
                        scan_points[0][1])
                    dis_end = lng_lat_calculate.distanceFromCoordinate(
                        self.gaode_lng_lat[0],
                        self.gaode_lng_lat[1],
                        scan_points[-1][0],
                        scan_points[-1][1])
                    if dis_start > dis_end:
                        scan_points.reverse()
                    self.server_data_obj.mqtt_send_get_obj.path_planning_points = scan_points
                    self.server_data_obj.mqtt_send_get_obj.sampling_points = scan_points
                    self.server_data_obj.mqtt_send_get_obj.sampling_points_status = [0] * len(scan_points)
                    self.server_data_obj.mqtt_send_get_obj.surrounded_start = 0
            else:
                time.sleep(1)
            # 判断是否需要请求轨迹
            if self.server_data_obj.mqtt_send_get_obj.path_id:
                url = config.http_record_get + "?id=%s" % self.server_data_obj.mqtt_send_get_obj.path_id
                return_data = self.server_data_obj.send_server_http_data('GET', '', url, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    print('轨迹记录点', content_data)
                    if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                        self.logger.error('device/getRoute GET请求失败')
                    task_data = content_data.get("data")
                    if task_data:
                        self.logger.info({'轨迹记录点': task_data.get('records')})
                        return_data_list = task_data.get('records')
                        self.server_data_obj.mqtt_send_get_obj.path_id = None
                        # 解析返回到的路径id
                        record_points = json.loads(return_data_list[0].get('route'))
                        # 收到开始命令将路径点赋值到需要行驶路劲中
                        if len(record_points) > 0:
                            # 对点进行排序
                            if self.gaode_lng_lat:
                                dis_start = lng_lat_calculate.distanceFromCoordinate(
                                    self.gaode_lng_lat[0],
                                    self.gaode_lng_lat[1],
                                    record_points[0][0],
                                    record_points[0][1])
                                dis_end = lng_lat_calculate.distanceFromCoordinate(
                                    self.gaode_lng_lat[0],
                                    self.gaode_lng_lat[1],
                                    record_points[-1][0],
                                    record_points[-1][1])
                                if dis_start > dis_end:
                                    record_points.reverse()
                                self.server_data_obj.mqtt_send_get_obj.path_planning_points = record_points
                                self.server_data_obj.mqtt_send_get_obj.sampling_points = record_points
                                self.server_data_obj.mqtt_send_get_obj.sampling_points_status = [0] * len(record_points)
            time.sleep(1)

    # 将要向服务器发送HTTP请求的移动到此函数中
    def loop_send_http(self):
        last_runtime = 0
        last_run_distance = 0
        http_get_time = True
        while True:
            time.sleep(1)
            if self.tcp_server_obj.main_obj.is_close == 1:
                self.logger.info({"人为主动断开退出loop_send_http线程": self.ship_id})
                return
            if self.ship_id in self.tcp_server_obj.disconnect_client_list:
                self.logger.info({"船只断开连接退出loop_send_http线程": self.ship_id})
                return
            # 登录获取值
            if not self.token:
                login_data = {"deviceId": self.ship_code}
                print('登录数据:', login_data)
                return_login_data = self.server_data_obj.send_server_http_data('POST', login_data,
                                                                               config.http_get_token, token=self.token)
                if return_login_data:
                    return_login_data_json = json.loads(return_login_data.content)
                    if return_login_data_json.get("code") == 200 and return_login_data_json.get("data"):
                        self.logger.info({'登录返回token': return_login_data_json.get("data").get("token")})
                        self.token = return_login_data_json.get("data").get("token")
                    else:
                        self.logger.error({'return_login_data': return_login_data_json})
                else:
                    self.logger.error({'return_login_data': return_login_data})
            if not self.token:
                continue
            if http_get_time:
                if self.http_save_distance is None or self.http_save_time is None:
                    url = config.http_mileage_get + "?deviceId=%s" % self.ship_code
                    return_data = self.server_data_obj.send_server_http_data('GET', '', url, token=self.token)
                    if return_data:
                        return_data_json = json.loads(return_data.content)
                        print('获取里程返回数据', return_data_json)
                        if return_data_json.get('data') and return_data_json.get('data').get('mileage'):
                            self.http_save_distance = int(return_data_json.get('data').get('mileage').get("total"))
                            # self.http_save_distance = 0
                            self.http_save_time = int(return_data_json.get('data').get('mileage').get("totalTime"))
                            # self.http_save_time = 0
                            self.http_save_id = return_data_json.get('data').get('mileage').get('id')
                            http_get_time = False
                        # 能返回数据但是里面不包含里程就更新里程
                        elif not return_data_json.get('data') or return_data_json.get('data').get('mileage') is None:
                            send_mileage_data = {
                                "deviceId": self.ship_code,
                                "total": str(0),
                                "totalTime": str(0)
                            }
                            return_data = self.server_data_obj.send_server_http_data('POST',
                                                                                     send_mileage_data,
                                                                                     config.http_mileage_save,
                                                                                     token=self.token)
                            if return_data:
                                # 如果是GET请求，返回所有数据的列表
                                content_data = json.loads(return_data.content)
                                if content_data.get("code") != 200 and content_data.get("code") != 20000:
                                    self.logger.error('保存新里程请求失败')
                                    print('保存新里程请求content_data', content_data)
                                else:
                                    self.logger.info("保存里程成功")
                    else:
                        self.logger.error("请求里程失败")
                        print('请求里程返回数据', return_data)
            # 更新总时间和总里程
            if time.time() % 10 < 1 and self.ship_id in self.tcp_server_obj.client_dict:
                if self.http_save_distance is not None and self.http_save_time is not None and self.http_save_id:
                    self.http_save_distance = self.http_save_distance + int(self.run_distance) - last_run_distance
                    # self.http_save_distance = 0
                    self.http_save_time = self.http_save_time + int(time.time() - self.start_time) - last_runtime
                    send_mileage_data = {
                        "deviceId": self.ship_code,
                        "id": self.http_save_id,
                        "total": str(self.http_save_distance),
                        "totalTime": str(self.http_save_time)
                    }
                    return_data = self.server_data_obj.send_server_http_data('POST',
                                                                             send_mileage_data,
                                                                             config.http_mileage_update,
                                                                             token=self.token)
                    if return_data:
                        content_data = json.loads(return_data.content)
                        if content_data.get("code") not in [200, 20000]:
                            self.logger.error('更新里程GET请求失败')
                            # 获取到401后重新请求token
                            if content_data.get("code") == 401:
                                self.token = None
                                continue
                            print('content_data', content_data)
                        else:
                            self.logger.info({'更新里程和时间成功': send_mileage_data})
                last_runtime = int(time.time() - self.start_time)
                last_run_distance = int(self.run_distance)
            # 请求行动id
            if not self.action_id and self.server_data_obj.mqtt_send_get_obj.action_name:  # 到点了没有行动id则获取行动id
                self.need_action_id = 1
            if self.need_action_id:
                data = {"deviceId": self.ship_code,
                        "mapId": self.data_define_obj.pool_code,
                        "taskId": self.server_data_obj.mqtt_send_get_obj.task_id,
                        "planName": self.server_data_obj.mqtt_send_get_obj.action_name,
                        "creator": self.creator
                        }
                print('行动请求数据', data)
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         data,
                                                                         config.http_action_get, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    self.logger.info({'获取行动id返回数据': content_data})
                    if not content_data.get("data"):
                        self.logger.error('获取行动id失败')
                    else:
                        self.action_id = content_data.get("data")
                        self.need_action_id = False
            # 上传采样数据
            self.ship_type_obj.ship_obj.send_data(self)

    # 发送http mqtt数据
    def send(self, method, data, topic='test', qos=0, http_type='POST', url='', parm_type=1):
        """
        :param url:
        :param http_type:
        :param qos:
        :param topic:
        :param data: 发送数据
        :param method 获取数据方式　http mqtt com
        """
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type, data, url, parm_type=parm_type,
                                                                     token=self.token)
            if not return_data:
                return False
            self.logger.info({'请求 url': url, 'status_code': return_data.status_code})
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
                save_data_binding = content_data["data"]
                return save_data_binding
            elif r'mileage/getOne' in url or r'mileage/get' in url and http_type == 'GET':
                if return_data:
                    return json.loads(return_data.content)
                else:
                    return False
            elif r'device/getRoute' in url or r'route/list' in url and http_type == 'GET':
                if return_data:
                    content_data = json.loads(return_data.content)
                    print('轨迹记录点', content_data)
                    if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                        self.logger.error('device/getRoute GET请求失败')
                    task_data = content_data.get("data")
                    if task_data:
                        self.logger.info({'轨迹记录点': task_data.get('records')})
                        return task_data.get('records')
                    return False
            elif http_type == 'GET' and r'task/list' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    print('获取任务数据', content_data)
                    if not content_data.get('code'):
                        self.logger.info({'获取任务 GET请求失败': content_data})
                    if content_data.get('data') and content_data.get('data').get('records') and len(
                            content_data.get('data').get('records')) == 1:
                        task_data = content_data.get('data').get('records')[0].get('task')
                        temp_task_data = content_data.get('data').get('records')[0].get('taskTem')
                        if temp_task_data:
                            temp_task_data = json.loads(temp_task_data)
                        task_data = json.loads(task_data)
                        print('task_data', task_data)
                        print('temp_task_data', temp_task_data)
                        time.sleep(10)
                        if temp_task_data and content_data.get('data').get('records')[0].get('planId'):
                            self.action_id = content_data.get('data').get('records')[0].get('planId')
                            self.server_data_obj.mqtt_send_get_obj.action_type = 3
                        if temp_task_data and len(temp_task_data) > 0:
                            return temp_task_data
                        else:
                            return task_data
                        # return task_data
            elif http_type == 'POST' and r'task/upDataTask' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    self.logger.info({'content_data': content_data})
                    if not content_data["success"]:
                        self.logger.error('upDataTask请求失败')
                    else:
                        return True
            elif http_type == 'POST' and r'task/delTask' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    self.logger.info({'content_data': content_data})
                    if not content_data["success"]:
                        self.logger.error('delTask请求失败')
                    else:
                        return True
            elif http_type == 'POST' and r'plan/save' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    self.logger.info({'获取行动id返回数据': content_data})
                    if not content_data.get("data"):
                        self.logger.error('获取行动id失败')
                    else:
                        return content_data.get("data")
            elif http_type == 'POST' and r'task/update' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    self.logger.info({'更新任务': content_data})
                    if content_data.get("code") != 200:
                        self.logger.error('更新任务失败')
                    else:
                        return True
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                print('content_data', content_data)
                if content_data.get("code") != 200 and content_data.get("code") != 20000:
                    self.logger.error('GET请求失败')
                else:
                    return True
        elif method == 'mqtt':
            self.server_data_obj.send_server_mqtt_data(data=data, topic=topic, qos=qos)

    # 检查是否需要返航
    def check_backhome(self):
        """
        返回返航状态或者None
        :return:返回None为不需要返航，返回低电量返航或者断网返航
        """
        return_ship_status = None
        if config.network_backhome:
            #  最大值设置为半小时
            if int(config.network_backhome) > 1800:
                network_backhome_time = 1800
            else:
                network_backhome_time = int(config.network_backhome)
            # 使用过电脑端按键操作过才能进行断网返航(防止启动时长时间没连上网判断为返航)
            if self.server_data_obj.mqtt_send_get_obj.b_receive_mqtt:
                if time.time() - self.server_data_obj.mqtt_send_get_obj.last_command_time > network_backhome_time:
                    return_ship_status = ShipStatus.backhome_network
        if self.low_dump_energy_warnning:
            # 记录是因为按了低电量判断为返航
            return_ship_status = ShipStatus.backhome_low_energy
        if return_ship_status is not None and self.ship_status not in [ShipStatus.backhome_network,
                                                                       ShipStatus.backhome_low_energy,
                                                                       ShipStatus.at_home]:
            pass
            # self.logger.info({"正在返航": return_ship_status})
        return return_ship_status
