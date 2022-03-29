"""
管理数据收发
"""
import time
import json
import copy
import os
import enum
import numpy as np
import random
from collections import deque

from messageBus import data_define
from externalConnect import server_data
from utils.log import LogHandler
from externalConnect import baidu_map
from drivers import audios_manager, pi_main
from utils import lng_lat_calculate
from utils import check_network
from utils import data_valid
from storage import save_data
import config
from moveControl.pathTrack import simple_pid
from moveControl.obstacleAvoid import vfh
from dataAnalyze import utils


class RelayType(enum.Enum):
    """
    控制继电器种类
    headlight 前大灯
    audio_light 声光报警器
    side_light 舷灯
    """
    headlight = 0
    audio_light = 1
    side_light = 2


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


class LightStatus(enum.Enum):
    """
    当前状态灯的状态
    """
    red = 0
    green = 1
    yellow = 2
    buzzer = 3


class Nwse(enum.Enum):
    """
    北东南西方向
    """
    north = 0
    west = 1
    south = 2
    east = 3


class DataManager:
    def __init__(self):
        self.data_define_obj = data_define.DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log', level=20)
        self.data_save_logger = LogHandler('data_save_log', level=20)
        self.com_data_read_logger = LogHandler('com_data_read_logger', level=20)
        self.com_data_send_logger = LogHandler('com_data_send_logger', level=20)
        self.server_log = LogHandler('server_data', level=20)
        self.gps_log = LogHandler('gps_log', level=20)
        self.path_track_obj = simple_pid.SimplePid()
        # mqtt服务器数据收发对象
        self.server_data_obj = server_data.ServerData(self.server_log, topics=self.data_define_obj.topics)
        # 路径跟踪是否被终止
        self.b_stop_path_track = False
        # 记录里程与时间数据
        self.totle_distance = 0
        # 开机时间
        self.start_time = time.time()
        # 提示消息
        # 距离下一个目标点距离
        self.distance_p = 0
        # 目标点信息
        self.path_info = [0, 0]
        # 抽水开始时间
        self.draw_start_time = None
        # 抽水结束发送数据
        self.b_draw_over_send_data = False
        # 水质数据
        self.water_data_dict = {}
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
        # 船行驶里程
        self.run_distance = 0
        # 经纬度 和 船头角度 北为0 逆时针为正
        self.lng_lat = None
        # 船高德经纬度
        self.gaode_lng_lat = None
        self.lng_lat_error = None
        # 船头角度
        self.current_theta = None
        # 偏差角度
        self.theta_error = 0
        # 罗盘提示消息
        self.compass_notice_info = ''
        # 记录pwm调节时间和数值用于在家调试
        self.last_left_pwm = 1500
        self.last_right_pwm = 1500
        self.last_change_pwm_time = time.time()
        # 记录上一次经纬度
        self.last_lng_lat = None
        # 记录船状态
        self.ship_status = ShipStatus.idle
        # 记录平滑路径
        self.smooth_path_lng_lat = None
        self.smooth_path_lng_lat_index = []
        # 船手动控制提示信息
        self.control_info = ''
        # 网络延时
        self.ping = 0
        # 避障提示消息
        self.obstacle_info = ''
        self.lng_lat_list = []
        self.deep_list = []
        # 记录上一次控制的单片机继电器状态
        self.last_side_light = 0  # 舷灯
        self.last_headlight = 0  # 大灯
        self.last_audio_light = 0  # 声光报警器
        self.last_status_light = 0  # 状态灯
        self.last_drain = 0  # 0没有在排水 1 在排水
        self.pi_main_obj = None
        if config.current_platform == config.CurrentPlatform.pi:
            self.pi_main_obj = pi_main.PiMain()
        # 使用真实GPS 还是 初始化高德GPS
        self.use_true_gps = 0
        if config.current_platform == config.CurrentPlatform.pi:
            if config.b_pin_gps:
                self.use_true_gps = 1
        # 当前路径平滑搜索列表
        self.search_list = []
        self.b_sampling = 0  # 在抽水中  0没有在抽水  1 在抽水中   2 抽水结束切换到其他状态
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
        # 是否已经初始化电机
        self.is_init_motor = 0
        self.area_id = None
        # 检测完成收回杆子标志
        self.is_need_drain = False  # 判断是否需要排水
        self.drain_start_time = None  # 记录排水时间
        self.last_send_stc_log_data = None  # 记录上一次发送给单片机数据重复就不用记录日志
        # 自动排水标志位
        self.is_auto_drain = 0
        self.http_save_distance = None  # http 记录保存距离
        self.http_save_time = None  # http记录保存时间
        self.http_save_id = None  # http记录保存id
        self.last_read_time_debug = None  # 调试用计算速度
        self.dump_draw_time = 0  # 剩余抽水时间
        self.record_path = []  # 记录手动轨迹点
        self.is_wait = 1  # 船只是否空闲
        # 调试模拟避障数据
        self.send_obstacle = True
        # 距离矩阵
        self.distance_dict = {}
        self.cell_size = int(config.field_of_view / config.view_cell)
        self.obstacle_list = [0] * self.cell_size  # 自动避障列表
        self.control_obstacle_list = [0] * self.cell_size  # 手动避障障碍物列表
        # 测试新的运动控制方式
        self.kp_v = self.server_data_obj.mqtt_send_get_obj.kp_v
        self.is_start_step = 1  # 是否是起步阶段
        self.want_v = 0  # 期望速度
        self.current_v = 0  # 当前速度
        self.pre_v = 0  # 之前速度

    def connect_mqtt_server(self):
        while True:
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                self.server_data_obj.mqtt_send_get_obj.mqtt_connect()
            time.sleep(5)

    # 抽水
    def draw(self):
        """
        """
        # time.sleep(1)
        # 判断开关是否需要打开或者关闭
        if config.home_debug or not config.b_draw:
            if self.server_data_obj.mqtt_send_get_obj.b_draw:
                if self.draw_start_time is None:
                    self.draw_start_time = time.time()
                else:
                    self.dump_draw_time = config.draw_time - int(time.time() - self.draw_start_time)
                    if time.time() - self.draw_start_time > config.draw_time:
                        self.dump_draw_time = 0
                        self.draw_start_time = None
                        self.b_draw_over_send_data = True
                        self.b_sampling = 2
                        self.server_data_obj.mqtt_send_get_obj.b_draw = 0
            else:
                self.draw_start_time = None
                self.dump_draw_time = 0
        else:
            # 开启了遥控器
            if self.pi_main_obj.b_start_remote:
                # 判断遥控器控制抽水
                # 正在抽水时不能让排水发送A0Z
                if self.pi_main_obj.remote_draw_status == 1:
                    if self.draw_start_time is None:
                        self.draw_start_time = time.time()
                    self.send_stc_data('A1Z')
                else:
                    if self.draw_start_time is not None:
                        # 超过抽水时间发送数据
                        self.dump_draw_time = config.draw_time - int(time.time() - self.draw_start_time)
                        if time.time() - self.draw_start_time > config.draw_time:
                            self.dump_draw_time = 0
                            self.b_draw_over_send_data = True
                    # 将时间置空
                    self.draw_start_time = None
                    # 正在排水时不能发送结束抽水
                    if not self.pi_main_obj.remote_drain_status:
                        self.send_stc_data('A0Z')
                # 排水
                if self.pi_main_obj.remote_drain_status:
                    self.send_stc_data('A2Z')
                    time.sleep(0.1)
                else:
                    # 正在抽水时不能发送停止
                    if self.draw_start_time is None:
                        self.send_stc_data('A0Z')
                    time.sleep(0.1)
            # 没有开启遥控器
            else:
                # 判断是否抽水  点击抽水情况
                if self.server_data_obj.mqtt_send_get_obj.b_draw:
                    # 判断是否有杆子放下杆子
                    if config.b_control_deep:
                        self.pi_main_obj.set_draw_deep(config.min_deep_steer_pwm)
                        if self.pi_main_obj.draw_steer_pwm == self.pi_main_obj.target_draw_steer_pwm:
                            self.send_stc_data('A1Z')
                            # 如果有抽水杆需要先放到下面再计算开始时间
                            if self.draw_start_time is None:
                                self.draw_start_time = time.time()
                                # 触发一次停止
                                self.pi_main_obj.stop()
                            else:
                                # 超时中断抽水
                                self.dump_draw_time = config.draw_time - int(time.time() - self.draw_start_time)
                                if time.time() - self.draw_start_time > config.draw_time:
                                    self.dump_draw_time = 0
                                    self.b_draw_over_send_data = True
                                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                                    self.is_need_drain = True
                                    self.b_sampling = 2
                                    self.send_stc_data('A0Z')
                    else:
                        self.send_stc_data('A1Z')
                        if self.draw_start_time is None:
                            self.draw_start_time = time.time()
                            # 触发一次停止
                            self.pi_main_obj.stop()
                        else:
                            # 超时中断抽水
                            self.dump_draw_time = config.draw_time - int(time.time() - self.draw_start_time)
                            if time.time() - self.draw_start_time > config.draw_time:
                                self.dump_draw_time = 0
                                self.b_draw_over_send_data = True
                                self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                                self.is_need_drain = True
                                self.b_sampling = 2
                                self.send_stc_data('A0Z')
                else:
                    self.dump_draw_time = 0
                    # 没有在排水才能发送停止
                    if not self.is_auto_drain:
                        self.send_stc_data('A0Z')
                    # 没有抽水的情况下杆子都要收回来
                    if config.b_control_deep and self.drain_start_time is None:
                        self.pi_main_obj.set_draw_deep(config.max_deep_steer_pwm)
                    self.draw_start_time = None

                # 判断没有排水则先排水再收杆子
                if self.is_need_drain:
                    if self.drain_start_time is None:
                        self.drain_start_time = time.time()
                        self.send_stc_data('A2Z')
                        self.is_auto_drain = 1
                    else:
                        # 超时中断排水
                        if time.time() - self.drain_start_time > config.draw_time:
                            self.send_stc_data('A0Z')
                            self.is_auto_drain = 0
                            # 收回杆子
                            if config.b_control_deep:
                                self.pi_main_obj.set_draw_deep(config.max_deep_steer_pwm)
                            self.is_need_drain = False
                            self.drain_start_time = None

    # 控制继电器
    def control_relay(self):
        """
        控制继电器
        :return:
        """
        # 改为一直发送模式
        is_continue_send = 1
        # 舷灯
        if self.pi_main_obj.remote_side_light_status == 1:
            self.server_data_obj.mqtt_send_get_obj.side_light = 1
        elif self.pi_main_obj.remote_side_light_status == 0:
            self.server_data_obj.mqtt_send_get_obj.side_light = 0
        if self.server_data_obj.mqtt_send_get_obj.side_light:
            if is_continue_send:
                self.send_stc_data('B1Z')
                time.sleep(0.1)
            else:
                if self.last_side_light:
                    pass
                else:
                    self.send_stc_data('B1Z')
        else:
            if is_continue_send:
                self.send_stc_data('B0Z')
                time.sleep(0.1)
            else:
                if self.last_side_light:
                    self.send_stc_data('B0Z')
                else:
                    pass
        self.last_side_light = self.server_data_obj.mqtt_send_get_obj.side_light
        # 大灯
        if self.pi_main_obj.remote_head_light_status == 1:
            self.server_data_obj.mqtt_send_get_obj.headlight = 1
        elif self.pi_main_obj.remote_head_light_status == 0:
            self.server_data_obj.mqtt_send_get_obj.headlight = 0
        if self.server_data_obj.mqtt_send_get_obj.headlight:
            if is_continue_send:
                self.send_stc_data('C1Z')
                time.sleep(0.1)
            else:
                if self.last_headlight:
                    pass
                else:
                    self.send_stc_data('C1Z')
        else:
            if is_continue_send:
                self.send_stc_data('C0Z')
                time.sleep(0.1)
            else:
                if self.last_headlight:
                    self.send_stc_data('C0Z')
                else:
                    pass
        self.last_headlight = self.server_data_obj.mqtt_send_get_obj.headlight
        # 声光报警器
        if self.server_data_obj.mqtt_send_get_obj.audio_light:
            if is_continue_send:
                self.send_stc_data('D1Z')
                time.sleep(0.1)
            else:
                if self.last_audio_light:
                    pass
                else:
                    self.send_stc_data('D1Z')
        else:
            if is_continue_send:
                self.send_stc_data('D0Z')
                time.sleep(0.1)
            else:
                if self.last_audio_light:
                    self.send_stc_data('D0Z')
                else:
                    pass
        self.last_audio_light = self.server_data_obj.mqtt_send_get_obj.audio_light

        # 舵机
        if self.is_init_motor:
            if self.pi_main_obj.b_start_remote:
                if self.pi_main_obj.target_draw_steer_pwm != self.pi_main_obj.draw_steer_pwm:
                    self.pi_main_obj.set_draw_deep(self.pi_main_obj.target_draw_steer_pwm)
        # 状态灯
        # if self.server_data_obj.mqtt_send_get_obj.status_light != self.last_status_light:
        # 启动后mqtt连接上亮绿灯
        if self.server_data_obj.mqtt_send_get_obj.is_connected:
            self.server_data_obj.mqtt_send_get_obj.status_light = 3
        # 使能遥控器就亮黄灯
        if self.pi_main_obj.b_start_remote:
            self.server_data_obj.mqtt_send_get_obj.status_light = 2
        # 低电量蜂鸣改为红灯
        if self.low_dump_energy_warnning:
            self.server_data_obj.mqtt_send_get_obj.status_light = 1
        # 断网红灯
        if self.b_network_backhome:
            self.server_data_obj.mqtt_send_get_obj.status_light = 1
        send_stc_data = 'E%sZ' % (str(self.server_data_obj.mqtt_send_get_obj.status_light))
        time.sleep(0.1)
        self.send_stc_data(send_stc_data)
        self.last_status_light = self.server_data_obj.mqtt_send_get_obj.status_light
        if random.random() > 0.98:
            self.last_status_light = 1

    # 外围设备控制线程函数
    def control_peripherals(self):
        while True:
            time.sleep(0.1)
            if not config.home_debug and self.pi_main_obj:
                self.control_relay()

    # 抽水排水控制
    def control_draw_thread(self):
        while True:
            time.sleep(1)
            # 当状态不是遥控器控制,不在执行任务状态且不在抽水过程中收回抽水杆子
            self.draw()

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
        self.server_data_obj.mqtt_send_get_obj.back_home = 0
        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
        self.server_data_obj.mqtt_send_get_obj.keep_point = 0
        self.point_arrive_start_time = None  # 清楚记录长期不到时间
        self.theta_error = 0

    # 当模式改变是改变保存的状态消息
    def change_status_info(self, target_status, b_clear_status=False):
        """
        当模式改变是改变保存的状态消息
        :return:
        """
        # 状态切换表 当前状态与目标状态
        status_change_list = {
            (ShipStatus.idle, ShipStatus.computer_control): 0,
            (ShipStatus.idle, ShipStatus.computer_auto): 28,
            (ShipStatus.idle, ShipStatus.backhome_network): 26,
            (ShipStatus.idle, ShipStatus.backhome_low_energy): 27,
            (ShipStatus.idle, ShipStatus.remote_control): 1,
            (ShipStatus.idle, ShipStatus.tasking): 31,
            (ShipStatus.computer_control, ShipStatus.tasking): 2,
            (ShipStatus.computer_control, ShipStatus.remote_control): 3,
            (ShipStatus.computer_control, ShipStatus.computer_auto): 4,
            (ShipStatus.computer_control, ShipStatus.backhome_network): 5,
            (ShipStatus.computer_control, ShipStatus.backhome_low_energy): 6,
            (ShipStatus.computer_control, ShipStatus.idle): 25,
            (ShipStatus.remote_control, ShipStatus.idle): 7,
            (ShipStatus.remote_control, ShipStatus.tasking): 8,
            (ShipStatus.tasking, ShipStatus.remote_control): 9,
            (ShipStatus.tasking, ShipStatus.computer_control): 10,
            (ShipStatus.tasking, ShipStatus.computer_auto): 11,
            (ShipStatus.tasking, ShipStatus.idle): 24,
            (ShipStatus.computer_auto, ShipStatus.computer_control): 12,
            (ShipStatus.computer_auto, ShipStatus.remote_control): 13,
            (ShipStatus.computer_auto, ShipStatus.backhome_network): 14,
            (ShipStatus.computer_auto, ShipStatus.backhome_low_energy): 15,
            (ShipStatus.computer_auto, ShipStatus.tasking): 16,
            (ShipStatus.backhome_network, ShipStatus.at_home): 17,
            (ShipStatus.backhome_low_energy, ShipStatus.at_home): 18,
            (ShipStatus.backhome_network, ShipStatus.computer_control): 19,
            (ShipStatus.backhome_low_energy, ShipStatus.computer_control): 20,
            (ShipStatus.backhome_network, ShipStatus.remote_control): 21,
            (ShipStatus.backhome_low_energy, ShipStatus.remote_control): 22,
            (ShipStatus.at_home, ShipStatus.idle): 23,
            (ShipStatus.at_home, ShipStatus.computer_control): 29,
            (ShipStatus.at_home, ShipStatus.remote_control): 30,
        }
        status_tuple = (self.ship_status, target_status)
        self.logger.info({'status change': status_tuple})
        if status_tuple not in status_change_list:
            self.logger.error({'status change error': status_tuple})
            return None
        status_change_index = status_change_list.get(status_tuple)
        # 从空闲到电脑手动
        if status_change_index == 0:
            # 改变状态不再重复发送指令
            pass
        # 从空闲到遥控器控制
        elif status_change_index == 1:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到执行任务
        elif status_change_index == 2:
            # 将船设置停下
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到遥控器手动
        elif status_change_index == 3:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到空闲   只有任务模式最后一点到达时发声
        elif status_change_index == 25:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到电脑自动
        elif status_change_index == 4:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到断网返航
        elif status_change_index == 5:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑手动到低电量返航
        elif status_change_index == 6:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从遥控器控制到空闲
        elif status_change_index == 7:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从遥控器控制到任务执行
        elif status_change_index == 8:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从任务执行到遥控器控制
        elif status_change_index == 9:
            pass
        # 从任务执行到电脑手动
        elif status_change_index == 10:
            pass
        # 从任务执行到电脑自动
        elif status_change_index == 11:
            pass
        # 从电脑自动到电脑手动
        elif status_change_index == 12:
            # 此时区分是否取消之前的航点
            # 按暂停后按后退则取消
            # 清空目标点和状态
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑自动到遥控器控制
        elif status_change_index == 13:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        # 从电脑自动到网络返航
        elif status_change_index == 14:
            pass
        # 从电脑自动到低电量返航
        elif status_change_index == 15:
            pass
        # 从电脑自动到任务模式 水质检测船才抽水 采样船不抽水
        elif status_change_index == 16:
            if config.current_ship_type == config.ShipType.water_detect:
                self.server_data_obj.mqtt_send_get_obj.b_draw = 1
            self.b_arrive_point = 0

        elif status_change_index == 24:  # 任务到等待
            pass
        # 断网返航到返航到家
        elif status_change_index == 17:
            pass
        # 低电量返航到返航到家
        elif status_change_index == 18:
            pass
        # 断网返航到电脑手动
        elif status_change_index == 19:
            pass
        # 低电量返航到电脑手动
        elif status_change_index == 20:
            pass
        # 断网返航到遥控器控制
        elif status_change_index == 21:
            pass
        # 低电量返航到电脑手动
        elif status_change_index == 22:
            pass
        # 返航到家到空闲
        elif status_change_index == 23:
            pass
        elif status_change_index == 26:  #
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        elif status_change_index == 27:
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        elif status_change_index == 28:  # 空闲到自动
            self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
        if b_clear_status:
            self.clear_all_status()
        self.ship_status = target_status

    # 检查是否需要返航
    def check_backhome(self):
        """
        返回返航状态或者None
        :return:返回None为不需要返航，返回低电量返航或者断网返航
        """
        return_ship_status = None
        if config.network_backhome and self.server_data_obj.mqtt_send_get_obj.is_connected:
            if int(config.network_backhome) > 600:
                network_backhome_time = int(config.network_backhome)
            else:
                network_backhome_time = 600
            # 使用过电脑端按键操作过才能进行断网返航
            if self.server_data_obj.mqtt_send_get_obj.b_receive_mqtt:
                if time.time() - self.server_data_obj.mqtt_send_get_obj.last_command_time > network_backhome_time:
                    return_ship_status = ShipStatus.backhome_network
        if self.low_dump_energy_warnning:
            # 记录是因为按了低电量判断为返航
            return_ship_status = ShipStatus.backhome_low_energy
        return return_ship_status

    # 处理状态切换
    def change_status(self):
        while True:
            # 删除任务模式，将抽水单独控制
            time.sleep(0.1)
            # 判断是否需要返航
            return_ship_status = None
            if self.ship_status != ShipStatus.at_home:
                return_ship_status = self.check_backhome()

            # 判断空闲状态切换到其他状态
            if self.ship_status == ShipStatus.idle:
                # 切换到遥控器控制模式
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    self.change_status_info(target_status=ShipStatus.remote_control)
                # 切换到电脑手动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction in [-1, 0, 90, 180, 270, 10, 190,
                                                                                       1180, 1270]:
                    self.change_status_info(target_status=ShipStatus.computer_control)
                # 切换到返航
                elif return_ship_status is not None:
                    self.change_status_info(target_status=return_ship_status)
                # 切换到自动模式
                elif len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                    if self.lng_lat is None:
                        self.logger.error('无当前GPS，不能自主巡航')
                        time.sleep(0.5)
                    else:
                        self.change_status_info(target_status=ShipStatus.computer_auto)
                # 切换到抽水模式
                elif self.server_data_obj.mqtt_send_get_obj.b_draw == 1:
                    self.change_status_info(target_status=ShipStatus.tasking)

            # 判断电脑手动状态切换到其他状态
            if self.ship_status == ShipStatus.computer_control:
                # 切换到遥控器控制
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    # 此时为遥控器控制模式 清除d控制状态
                    self.change_status_info(target_status=ShipStatus.remote_control)
                # 切换到自动巡航模式
                elif len(self.server_data_obj.mqtt_send_get_obj.path_planning_points) > 0:
                    if self.lng_lat is None:
                        self.logger.error('无当前GPS，不能自主巡航')
                        time.sleep(0.5)
                    else:
                        self.change_status_info(target_status=ShipStatus.computer_auto)
                # 点击抽水
                elif self.server_data_obj.mqtt_send_get_obj.b_draw:
                    self.last_ship_status = ShipStatus.computer_control
                    self.change_status_info(target_status=ShipStatus.tasking)
                # 切换到返航
                elif return_ship_status is not None:
                    self.change_status_info(target_status=return_ship_status)

            # 判断电脑自动切换到其他状态情况
            if self.ship_status == ShipStatus.computer_auto:
                # 切换到遥控器控制
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    self.change_status_info(target_status=ShipStatus.remote_control, b_clear_status=True)
                # 切换到返航
                elif return_ship_status is not None:
                    self.change_status_info(target_status=return_ship_status)
                # 取消自动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                    self.change_status_info(target_status=ShipStatus.computer_control, b_clear_status=True)
                # 切换到手动
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction in [0, 90, 180, 270, 10, 190, 1180,
                                                                                       1270]:
                    self.change_status_info(target_status=ShipStatus.computer_control)
                # 到点
                elif self.b_arrive_point:
                    self.last_ship_status = ShipStatus.computer_auto
                    self.change_status_info(target_status=ShipStatus.tasking)
                # 点击抽水
                elif self.server_data_obj.mqtt_send_get_obj.b_draw:
                    self.last_ship_status = ShipStatus.computer_auto
                    self.change_status_info(target_status=ShipStatus.tasking)

            # 判断任务模式切换到其他状态情况
            if self.ship_status == ShipStatus.tasking:
                # 切换到电脑自动模式  切换到电脑手动模式
                if self.b_sampling == 2 or self.server_data_obj.mqtt_send_get_obj.b_draw == 0:
                    if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0 and \
                            all(self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                        self.change_status_info(target_status=ShipStatus.computer_control, b_clear_status=True)
                        # 自动模式下到达最后一个点切换为空闲状态
                        self.last_ship_status = ShipStatus.idle
                    self.b_sampling = 0
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                    self.change_status_info(self.last_ship_status)
                if self.server_data_obj.mqtt_send_get_obj.b_draw == 0:
                    self.change_status_info(target_status=ShipStatus.idle)

            # 遥控器状态切换到其他状态
            if self.ship_status == ShipStatus.remote_control:
                # 切换到返航
                if return_ship_status is not None:
                    if len(self.pi_main_obj.remote_control_data) == 14 and \
                            abs(self.pi_main_obj.remote_control_data[2] - 50) < 30:
                        self.change_status_info(target_status=return_ship_status)
                # 切换到空闲状态
                elif not config.home_debug and self.pi_main_obj.b_start_remote == 0:
                    self.change_status_info(target_status=ShipStatus.idle)
                # 切换到任务模式
                elif not config.home_debug and self.pi_main_obj.remote_draw_status == 1:
                    self.last_ship_status = ShipStatus.remote_control
                    self.change_status_info(target_status=ShipStatus.tasking)

            # 返航状态切换到其他状态
            if self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 判断是否返航到家
                if self.b_at_home:
                    self.change_status_info(target_status=ShipStatus.at_home)
                # 切换到遥控器模式 使能遥控器
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    self.change_status_info(target_status=ShipStatus.remote_control)
                # 切换到电脑手动控制
                if self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                    self.change_status_info(target_status=ShipStatus.computer_control)

            # 返航到家状态切换到其他状态
            if self.ship_status == ShipStatus.at_home:
                if self.server_data_obj.mqtt_send_get_obj.control_move_direction in [-1, 0, 90, 180, 270, 10, 190, 1180,
                                                                                     1270]:
                    self.change_status_info(target_status=ShipStatus.computer_control)
                # 切换到遥控器模式 使能遥控器
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    self.change_status_info(target_status=ShipStatus.remote_control)

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
        # print('self.smooth_path_lng_lat, index_,', self.smooth_path_lng_lat_index, index_)
        # 限制后面路径点寻找时候不能找到之前采样点路径上
        if index_ == 0:
            self.search_list = copy.deepcopy(self.smooth_path_lng_lat[:start_index])
        else:
            self.search_list = copy.deepcopy(
                self.smooth_path_lng_lat[self.smooth_path_lng_lat_index[index_ - 1]:start_index])
        for target_lng_lat in self.search_list:
            distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                target_lng_lat[0],
                                                                target_lng_lat[1])
            distance_list.append(distance)
        # 如果没有可以去路径
        if len(distance_list) == 0:
            return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
        index = distance_list.index(min(distance_list))
        # if index + 1 == len(self.search_list):
        #     return self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index_]
        lng_lat = self.search_list[index]
        index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                        self.lng_lat[1],
                                                                        lng_lat[0],
                                                                        lng_lat[1])
        while config.smooth_path_ceil_size > index_point_distance and (index + 1) < len(
                self.search_list):
            lng_lat = self.search_list[index]
            index_point_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                            self.lng_lat[1],
                                                                            lng_lat[0],
                                                                            lng_lat[1])
            index += 1
        # 超过第一个点后需要累积之前计数
        if index_ > 0:
            self.path_info = [self.smooth_path_lng_lat_index[index_ - 1] + index, len(self.smooth_path_lng_lat)]
        else:
            self.path_info = [index, len(self.smooth_path_lng_lat)]
        return self.search_list[index]

    # 往东南西北运动控制
    def nesw_control(self, nest):
        if nest == Nwse.north:
            angle = 0
        elif nest == Nwse.west:
            angle = 90
        elif nest == Nwse.south:
            angle = 180
        else:
            angle = 270
        point = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                            self.lng_lat[1],
                                                            angle,
                                                            config.min_steer_distance * 5)
        self.points_arrive_control(point, point, False, False)

    # 调试模式下往前后左右运动
    def debug_lrfb_control(self, lrfb):
        "lrfb 表示前后左右left right forward backward"
        if self.current_theta is None:
            return
        if lrfb == 'f':
            angle = self.current_theta
            point = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                angle,
                                                                config.min_steer_distance / 30)
            self.lng_lat = point
            if random.random() > 0.5:
                self.current_theta += 0.1
            else:
                self.current_theta -= 0.1
        elif lrfb == 'b':
            angle = (self.current_theta + 180) % 360
            point = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                angle,
                                                                config.min_steer_distance / 30)
            self.lng_lat = point
            if random.random() > 0.5:
                self.current_theta += 0.1
            else:
                self.current_theta -= 0.1
        elif lrfb == 'l':
            self.current_theta += 3
        else:
            self.current_theta -= 3
        self.current_theta %= 360

    # 计算障碍物下目标点
    def get_avoid_obstacle_point(self, path_planning_point_gps=None):
        """
        根据障碍物地图获取下一个运动点
        :return: 下一个目标点，是否需要紧急停止【True为需要停止，False为不需要停止】
        """
        next_point_lng_lat = copy.deepcopy(path_planning_point_gps)
        if config.b_millimeter_wave:
            print('config.obstacle_avoid_type', config.obstacle_avoid_type)
            # 不避障
            if config.obstacle_avoid_type == 0:
                return path_planning_point_gps, False
            # 避障停止
            elif config.obstacle_avoid_type == 1:
                # 船头角度左右3个扇区有障碍物
                if 1 in self.pi_main_obj.obstacle_list[
                        int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                    return path_planning_point_gps, False
                    # 下面为避免传感器误差只有看到障碍物且船到岸边距离小于最小转弯距离才停止
                    # if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < config.min_steer_distance:
                    #     return path_planning_point_gps, True
                else:
                    return path_planning_point_gps, False
            # 避障绕行，根据障碍物计算下一个目标点
            elif config.obstacle_avoid_type in [2, 3]:
                angle = vfh.vfh_func(9, self.pi_main_obj.obstacle_list)
                print('避障角度：', angle)
                if angle == -1:  # 没有可通行区域
                    # 如果是离岸边太近就直接认为到达
                    if 1 in self.pi_main_obj.obstacle_list[
                            int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                        # 不够转弯距离返回停止  够转弯距离返回新的目标点
                        if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < config.min_steer_distance:
                            return next_point_lng_lat, True
                        else:
                            abs_angle = (self.pi_main_obj.theta + 180) % 360
                            next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                             self.lng_lat[1],
                                                                                             abs_angle,
                                                                                             config.min_steer_distance)
                            print('新方向角度：', abs_angle)
                            return next_point_lng_lat, False
                elif angle == 0:  # 当前船头角度可通行
                    if config.obstacle_avoid_type == 3:  # 避障方式为3是对河内避障对岸边停止
                        if 1 in self.pi_main_obj.obstacle_list[
                                int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                            if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < config.min_steer_distance:
                                return next_point_lng_lat, True
                    # 为0表示原始路径可以通行此时不跳过
                    return next_point_lng_lat, False
                else:  # 船头角度不能通过但是传感器检测其他角度可以通过
                    if config.obstacle_avoid_type == 3:  # 避障方式为3是对河内避障对岸边停止
                        if 1 in self.pi_main_obj.obstacle_list[
                                int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                            if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < config.min_steer_distance:
                                return next_point_lng_lat, True
                    abs_angle = (self.pi_main_obj.theta + angle) % 360
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     abs_angle,
                                                                                     config.min_steer_distance)
                    print('绕行角度:', abs_angle)
                    return next_point_lng_lat, False
        else:
            return path_planning_point_gps, False

    # 构建模拟避障数据
    def build_obstacle_data(self):
        # 角度限制
        while True:
            # 计算船到每个障碍物的距离和角度
            for index, point in enumerate(config.obstacle_points):
                if not self.gaode_lng_lat:
                    continue
                distance = lng_lat_calculate.distanceFromCoordinate(self.gaode_lng_lat[0],
                                                                    self.gaode_lng_lat[1],
                                                                    point[0],
                                                                    point[1]
                                                                    )
                angle = lng_lat_calculate.angleFromCoordinate(self.gaode_lng_lat[0],
                                                              self.gaode_lng_lat[1],
                                                              point[0],
                                                              point[1])
                ori_angle = angle - self.current_theta
                if ori_angle < 0:
                    ori_angle = 360 + ori_angle
                self.distance_dict.update({index: [distance, ori_angle]})
            # print('self.distance_dict', self.distance_dict)
            self.obstacle_list = [0] * self.cell_size  # 自动避障列表
            self.control_obstacle_list = [0] * self.cell_size  # 手动避障障碍物列表
            for obj_id in self.distance_dict:
                distance_average = self.distance_dict.get(obj_id)[0]
                angle_average = self.distance_dict.get(obj_id)[1]
                if config.field_of_view / 2 <= angle_average < 360 - config.field_of_view / 2:
                    b_obstacle = 0
                else:
                    if angle_average >= 360 - config.field_of_view / 2:
                        angle_average = angle_average - (360 - config.field_of_view / 2)
                        obstacle_index = int(angle_average // config.view_cell + self.cell_size // 2)
                    else:
                        angle_average = - angle_average
                        obstacle_index = int(angle_average // config.view_cell + self.cell_size // 2)
                        # print('obstacle_index', obstacle_index)
                    if distance_average > config.min_steer_distance or abs(angle_average) >= (config.field_of_view / 2):
                        b_obstacle = 0
                    else:
                        # print('self.distance_dict', self.distance_dict)
                        b_obstacle = 1
                    try:
                        self.obstacle_list[obstacle_index] = b_obstacle
                    except Exception as e:
                        print('#######################obstacle_index', obstacle_index)
                    if distance_average > config.control_obstacle_distance:
                        b_control_obstacle = 0
                    else:
                        b_control_obstacle = 1
                    self.control_obstacle_list[obstacle_index] = b_control_obstacle
            # print(time.time(), 'self.obstacle_list', self.obstacle_list)
            time.sleep(0.1)

    # 计算障碍物下目标点
    def get_avoid_obstacle_point_debug(self, path_planning_point_gps=None):
        """
        调试用根据障碍物地图获取下一个运动点
        :return: 下一个目标点，是否需要紧急停止【True为需要停止，False为不需要停止】
        """
        next_point_lng_lat = copy.deepcopy(path_planning_point_gps)
        ceil_size = 4
        if config.b_millimeter_wave:
            # print('config.obstacle_avoid_type', config.obstacle_avoid_type)
            # 不避障
            if config.obstacle_avoid_type == 0:
                return path_planning_point_gps, False
            # 避障停止
            elif config.obstacle_avoid_type == 1:
                # 船头角度左右3个扇区有障碍物
                if 1 in self.obstacle_list[
                        int(self.cell_size / 2) - ceil_size:int(self.cell_size / 2) + ceil_size]:
                    return path_planning_point_gps, False
                    # 下面为避免传感器误差只有看到障碍物且船到岸边距离小于最小转弯距离才停止
                    # if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < config.min_steer_distance:
                    #     return path_planning_point_gps, True
                else:
                    return path_planning_point_gps, False
            # 避障绕行，根据障碍物计算下一个目标点
            elif config.obstacle_avoid_type in [2, 3]:
                angle = vfh.vfh_func(9, self.obstacle_list)
                print('避障角度：', angle)
                if angle == -1:  # 没有可通行区域
                    # 如果是离岸边太近就直接认为到达
                    if 1 in self.obstacle_list[
                            int(self.cell_size / 2) - ceil_size:int(self.cell_size / 2) + ceil_size]:
                        # 不够转弯距离返回停止  够转弯距离返回新的目标点
                        if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < 6:
                            return next_point_lng_lat, True
                        else:
                            abs_angle = (self.current_theta + 180) % 360
                            next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                             self.lng_lat[1],
                                                                                             abs_angle,
                                                                                             config.min_steer_distance / 10)
                            print('新方向角度：', abs_angle)
                            return next_point_lng_lat, False
                elif angle == 0:  # 当前船头角度可通行
                    if config.obstacle_avoid_type == 3:  # 避障方式为3是对河内避障对岸边停止
                        if 1 in self.obstacle_list[
                                int(self.cell_size / 2) - ceil_size:int(self.cell_size / 2) + ceil_size]:
                            if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < 6:
                                return next_point_lng_lat, True
                    # 为0表示原始路径可以通行此时不跳过
                    return next_point_lng_lat, False
                else:  # 船头角度不能通过但是传感器检测其他角度可以通过
                    if config.obstacle_avoid_type == 3:  # 避障方式为3是对河内避障对岸边停止
                        if 1 in self.obstacle_list[
                                int(self.cell_size / 2) - ceil_size:int(self.cell_size / 2) + ceil_size]:
                            if -100 < self.server_data_obj.mqtt_send_get_obj.bank_distance < 6:
                                return next_point_lng_lat, True
                    abs_angle = (self.current_theta + angle) % 360
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     abs_angle,
                                                                                     config.min_steer_distance)
                    print('绕行角度:', abs_angle)
                    return next_point_lng_lat, False
        else:
            return path_planning_point_gps, False

    # 控制到达目标点
    def points_arrive_control(self, target_lng_lat_gps, sample_lng_lat_gps, b_force_arrive=False, b_back_home=False):
        """
        :param target_lng_lat_gps: 目标点真实经纬度
        :param sample_lng_lat_gps: 下一个采样点真实经纬度
        :param b_force_arrive: 是否约束一定要到达
        :param b_back_home 是否是正在返航
        :return:
        """
        distance = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0],
            self.lng_lat[1],
            sample_lng_lat_gps[0],
            sample_lng_lat_gps[1])
        self.distance_p = distance
        if distance < config.arrive_distance:
            return True
        # 超时不到则跳过 达到30米且60秒不到则跳过
        if distance < 30 and self.point_arrive_start_time is None:
            self.point_arrive_start_time = time.time()
        elif self.point_arrive_start_time and time.time() - self.point_arrive_start_time > 60:
            return True
        while distance >= config.arrive_distance:
            # 超时不到则跳过 达到30米且60秒不到则跳过
            if distance < 30 and self.point_arrive_start_time is None:
                self.point_arrive_start_time = time.time()
            elif self.point_arrive_start_time and time.time() - self.point_arrive_start_time > 60:
                return True
            distance_sample = lng_lat_calculate.distanceFromCoordinate(
                self.lng_lat[0],
                self.lng_lat[1],
                sample_lng_lat_gps[0],
                sample_lng_lat_gps[1])
            self.distance_p = distance_sample
            if distance_sample < config.arrive_distance:
                return True
            # 避障判断下一个点
            if not config.home_debug:
                target_lng_lat_gps, b_stop = self.get_avoid_obstacle_point(target_lng_lat_gps)
            else:
                target_lng_lat_gps, b_stop = self.get_avoid_obstacle_point_debug(target_lng_lat_gps)
            if b_stop:  # 避障判断是否需要停止
                print('避障停止', b_stop)
            # 计算偏差距离
            all_distance = lng_lat_calculate.distanceFromCoordinate(
                self.lng_lat[0], self.lng_lat[1], target_lng_lat_gps[0],
                target_lng_lat_gps[1])
            # 当前点到目标点角度
            point_theta = lng_lat_calculate.angleFromCoordinate(self.lng_lat[0],
                                                                self.lng_lat[1],
                                                                target_lng_lat_gps[0],
                                                                target_lng_lat_gps[1])
            # 计算偏差角度
            if config.home_debug:
                if self.current_theta is not None:
                    theta_error = point_theta - self.current_theta
                else:
                    theta_error = 0
            else:
                if self.pi_main_obj.theta is not None:
                    theta_error = point_theta - self.pi_main_obj.theta
                else:
                    theta_error = 0
            if abs(theta_error) > 180:
                if theta_error > 0:
                    theta_error = theta_error - 360
                else:
                    theta_error = 360 + theta_error
            self.theta_error = theta_error
            method = 1  # 测试新的控制方式
            if method == 2:
                # 分为起步阶段  中间恒速阶段  到点减速阶段/避障减速
                # 起步阶段用偏差角度小于指定阈值判断
                if self.is_start_step:
                    if abs(theta_error) >= 60:
                        self.want_v = 180 - abs(theta_error)
                    else:
                        self.want_v = 180 - abs(theta_error)
                        self.is_start_step = 0
                # 恒速阶段用距离和偏差角度计算
                if distance_sample >= 10:
                    self.want_v = max(100, 250 - 2 * abs(theta_error))
                # 到点减速阶段用距离计算期望速度
                else:
                    self.want_v = max(70, 25 * distance_sample)
                left_pwm, right_pwm = self.path_track_obj.pid_pwm_3(distance=self.want_v,
                                                                    theta_error=theta_error)
            else:
                left_pwm, right_pwm = self.path_track_obj.pid_pwm_2(distance=all_distance,
                                                                    theta_error=theta_error)
            self.last_left_pwm = left_pwm
            self.last_right_pwm = right_pwm
            # 在家调试模式下预测目标经纬度
            if config.home_debug:
                time.sleep(0.1)
                if not self.last_read_time_debug:
                    self.last_read_time_debug = time.time()
                # 计算当前行驶里程
                if self.last_lng_lat:
                    speed_distance = lng_lat_calculate.distanceFromCoordinate(self.last_lng_lat[0],
                                                                              self.last_lng_lat[1],
                                                                              self.lng_lat[0],
                                                                              self.lng_lat[1])
                    try:
                        self.speed = round(speed_distance / (time.time() - self.last_read_time_debug), 1)
                    except Exception as e:
                        print('error', e)
                    self.run_distance += speed_distance
                left_delta_pwm = int(self.last_left_pwm + left_pwm) / 2 - config.stop_pwm
                right_delta_pwm = int(self.last_right_pwm + right_pwm) / 2 - config.stop_pwm
                steer_power = left_delta_pwm - right_delta_pwm
                forward_power = left_delta_pwm + right_delta_pwm
                delta_distance = forward_power * 0.001
                delta_theta = steer_power * 0.08
                if self.last_lng_lat:
                    ship_theta = lng_lat_calculate.angleFromCoordinate(self.last_lng_lat[0],
                                                                       self.last_lng_lat[1],
                                                                       self.lng_lat[0],
                                                                       self.lng_lat[1])
                else:
                    ship_theta = 0
                # 船头角度
                self.current_theta = ship_theta
                if self.current_theta is not None:
                    self.current_theta = (self.current_theta - delta_theta / 2) % 360
                    self.current_theta += 0.1
                self.last_lng_lat = copy.deepcopy(self.lng_lat)
                self.last_read_time_debug = time.time()
                self.lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                           self.lng_lat[1],
                                                                           self.current_theta,
                                                                           delta_distance)
            else:
                # 判断是否需要避障处理
                print('b_stop', b_stop)
                if b_stop:
                    self.obstacle_info = '1'
                    self.pi_main_obj.stop()
                    # 记录是因为按了暂停按钮而终止
                    self.b_stop_path_track = True
                    return False
                else:
                    self.obstacle_info = '0'
                    self.pi_main_obj.set_pwm(left_pwm, right_pwm)
            if self.ship_status != ShipStatus.computer_auto:
                self.b_stop_path_track = True
                return False
            # 如果目标点改变并且不是强制到达 b_force_arrive
            if not b_force_arrive:
                return False
            if b_back_home:
                if distance_sample < config.arrive_distance:
                    return True

    # 处理电机控制
    def move_control(self):
        b_log_points = 1  # 防止寻点模式下等待用户点击开始前一直记录日志
        while True:
            time.sleep(0.1)
            self.direction = self.server_data_obj.mqtt_send_get_obj.control_move_direction
            control_info_dict = {
                ShipStatus.computer_control: '手动',
                ShipStatus.remote_control: '遥控',
                ShipStatus.computer_auto: '自动',
                ShipStatus.tasking: '抽水中',
                ShipStatus.backhome_network: '返航',
                ShipStatus.backhome_low_energy: '返航',
                ShipStatus.at_home: '在返航点',
                ShipStatus.idle: '等待',
            }
            self.direction = int(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
            # 判断船是否能给用户点击再次运动
            if self.ship_status in [ShipStatus.idle, ShipStatus.remote_control, ShipStatus.computer_control,
                                    ShipStatus.at_home]:
                self.is_wait = 1  # 是空闲
            else:
                self.is_wait = 2  # 非空闲
            self.control_info = ''
            if self.ship_status in control_info_dict:
                self.control_info += control_info_dict[self.ship_status]
            # 电脑手动
            if self.ship_status == ShipStatus.computer_control or self.ship_status == ShipStatus.tasking:
                # 手动模式避障距离
                # if config.obstacle_avoid_type == 3:
                #     if 1 in self.pi_main_obj.control_obstacle_list[
                #             int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                #         if self.direction == 0:
                #             self.direction = -1
                if not config.home_debug:
                    if self.direction == 0:
                        self.control_info += ' 向前'
                        self.pi_main_obj.forward()
                    elif self.direction == 90:
                        self.control_info += ' 向左'
                        self.pi_main_obj.left()
                    elif self.direction == 180:
                        self.control_info += ' 向后'
                        self.pi_main_obj.backword()
                    elif self.direction == 270:
                        self.control_info += ' 向右'
                        self.pi_main_obj.right()
                    elif self.direction == 10:
                        self.control_info += ' 向北'
                        self.nesw_control(nest=Nwse.north)
                    elif self.direction == 190:
                        self.control_info += ' 向西'
                        self.nesw_control(nest=Nwse.west)
                    elif self.direction == 1180:
                        self.control_info += ' 向南'
                        self.nesw_control(nest=Nwse.south)
                    elif self.direction == 1270:
                        self.control_info += ' 向东'
                        self.nesw_control(nest=Nwse.east)
                    elif self.direction == -1:
                        self.control_info += ' 停止'
                        self.point_arrive_start_time = None
                        self.pi_main_obj.stop()
                else:
                    if self.direction == 0:
                        self.control_info += ' 向前'
                        self.debug_lrfb_control(lrfb='f')
                    elif self.direction == 90:
                        self.control_info += ' 向左'
                        self.debug_lrfb_control(lrfb='l')
                    elif self.direction == 180:
                        self.control_info += ' 向后'
                        self.debug_lrfb_control(lrfb='b')
                    elif self.direction == 270:
                        self.control_info += ' 向右'
                        self.debug_lrfb_control(lrfb='r')
                    elif self.direction == 10:
                        self.control_info += ' 向北'
                        self.nesw_control(nest=Nwse.north)
                    elif self.direction == 190:
                        self.control_info += ' 向西'
                        self.nesw_control(nest=Nwse.west)
                    elif self.direction == 1180:
                        self.control_info += ' 向南'
                        self.nesw_control(nest=Nwse.south)
                    elif self.direction == 1270:
                        self.control_info += ' 向东'
                        self.nesw_control(nest=Nwse.east)
                    elif self.direction == -1:
                        self.control_info += ' 停止'
            # 遥控器控制
            elif self.ship_status == ShipStatus.remote_control or self.ship_status == ShipStatus.tasking:
                # lora遥控器
                if config.b_lora_remote_control:
                    remote_left_pwm, remote_right_pwm = self.pi_main_obj.check_remote_pwm(b_lora_remote_control=True)
                    self.pi_main_obj.set_pwm(set_left_pwm=remote_left_pwm, set_right_pwm=remote_right_pwm)
                # 2.4g遥控器
                elif config.b_use_remote_control:
                    remote_left_pwm, remote_right_pwm = self.pi_main_obj.check_remote_pwm(b_lora_remote_control=False)
                    self.pi_main_obj.set_pwm(set_left_pwm=remote_left_pwm, set_right_pwm=remote_right_pwm)
                else:
                    continue
            # 电脑自动
            elif self.ship_status == ShipStatus.computer_auto:
                if b_log_points:
                    self.logger.info({'点击地点': self.server_data_obj.mqtt_send_get_obj.path_planning_points})
                # 判断是否是寻点模式点了寻点但是还没点开始
                # if self.server_data_obj.mqtt_send_get_obj.row_gap:
                #     if not self.server_data_obj.mqtt_send_get_obj.b_start:
                #         b_log_points = 0
                #         time.sleep(0.5)
                #         self.logger.info('等待点击开始寻点')
                #         continue
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
                    # 如果勾选了返航且存在返航点
                    if self.server_data_obj.mqtt_send_get_obj.back_home:
                        if self.home_gaode_lng_lat:
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points.append(self.home_gaode_lng_lat)
                            self.server_data_obj.mqtt_send_get_obj.sampling_points.append(self.home_gaode_lng_lat)
                            self.server_data_obj.mqtt_send_get_obj.sampling_points_status.append(0)
                    # 将目标点转换为真实经纬度
                    if config.home_debug:
                        self.server_data_obj.mqtt_send_get_obj.path_planning_points_gps = copy.deepcopy(
                            self.server_data_obj.mqtt_send_get_obj.path_planning_points)
                        self.server_data_obj.mqtt_send_get_obj.sampling_points_gps = copy.deepcopy(
                            self.server_data_obj.mqtt_send_get_obj.sampling_points)
                    else:
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
                    # print('self.server_data_obj.mqtt_send_get_obj.sampling_points_status',
                    #       self.server_data_obj.mqtt_send_get_obj.sampling_points_status,
                    #       self.server_data_obj.mqtt_send_get_obj.sampling_points_gps)
                while self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) > 0:
                    index = self.server_data_obj.mqtt_send_get_obj.sampling_points_status.index(0)
                    # print('index', index)
                    sampling_point_gps = self.server_data_obj.mqtt_send_get_obj.sampling_points_gps[index]
                    # 计算下一个目标点经纬度
                    if config.path_plan_type:
                        next_lng_lat = self.calc_target_lng_lat(index)
                        # 如果当前点靠近采样点距离小于指定距离就开启强制到达模式否者只是尽量到达
                        sample_distance = lng_lat_calculate.distanceFromCoordinate(
                            next_lng_lat[0],
                            next_lng_lat[1],
                            sampling_point_gps[0],
                            sampling_point_gps[1])
                        if sample_distance < config.forward_see_distance:
                            b_arrive_sample = self.points_arrive_control(sampling_point_gps, sampling_point_gps,
                                                                         b_force_arrive=True)
                        else:
                            b_arrive_sample = self.points_arrive_control(next_lng_lat, sampling_point_gps,
                                                                         b_force_arrive=False)
                    else:
                        b_arrive_sample = self.points_arrive_control(sampling_point_gps, sampling_point_gps,
                                                                     b_force_arrive=True)
                    if b_arrive_sample:
                        if not config.home_debug:
                            self.pi_main_obj.stop()
                        self.path_track_obj.adjust_p_list.clear()
                        print('b_arrive_sample', b_arrive_sample)
                        # 更新目标点提示消息
                        # self.b_arrive_point = 1  # 到点了用于通知抽水  暂时修改为不抽水
                        self.point_arrive_start_time = None
                        self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] = 1
                        self.path_info = [index + 1, len(self.server_data_obj.mqtt_send_get_obj.sampling_points)]
                        time.sleep(1)
                    # 全部点到达后清除自动状态
                    if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) == sum(
                            self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
                    if self.ship_status != ShipStatus.computer_auto:
                        break

            # 返航 断网返航 低电量返航
            elif self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 有返航点下情况下返回返航点，没有则停止
                print('back home')
                if self.home_lng_lat:
                    back_home_flag = self.points_arrive_control(self.home_lng_lat, self.home_lng_lat, True, True)
                    if back_home_flag:
                        self.b_at_home = 1
                else:
                    if not config.home_debug:
                        self.pi_main_obj.stop()
            if config.home_debug and self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                self.speed = 0

    def update_ship_gaode_lng_lat(self):
        # 更新经纬度为高德经纬度
        while True:
            time.sleep(1)
            if config.home_debug:
                if self.lng_lat is None:
                    self.lng_lat = config.ship_gaode_lng_lat
            if self.lng_lat is not None:
                if self.use_true_gps:
                    if not config.home_debug:
                        try:
                            gaode_lng_lat = baidu_map.BaiduMap.gps_to_gaode_lng_lat(self.lng_lat)
                            if gaode_lng_lat:
                                self.gaode_lng_lat = gaode_lng_lat
                                if not self.home_lng_lat:
                                    self.home_lng_lat = copy.deepcopy(self.lng_lat)
                                    self.home_gaode_lng_lat = copy.deepcopy(gaode_lng_lat)
                        except Exception as e:
                            self.logger.error({'error': e})
                else:
                    self.gaode_lng_lat = self.lng_lat
                    if not self.home_lng_lat:
                        self.home_lng_lat = copy.deepcopy(self.lng_lat)
                        self.home_gaode_lng_lat = self.lng_lat
            # 调试模式下自身经纬度和高德经纬度一致
            elif self.lng_lat is not None and not self.use_true_gps:
                self.gaode_lng_lat = self.lng_lat

    # 更新经纬度
    def update_lng_lat(self):
        last_read_time = None
        while True:
            if not config.home_debug and \
                    self.pi_main_obj.lng_lat and \
                    self.pi_main_obj.lng_lat[0] > 1 and \
                    self.pi_main_obj.lng_lat[1] > 1:
                self.lng_lat = copy.deepcopy(self.pi_main_obj.lng_lat)
                self.lng_lat_error = self.pi_main_obj.lng_lat_error
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
                        # 判断是使用GPS数据中速度还是使用自己计算速度
                        if self.pi_main_obj.speed is not None:
                            self.speed = self.pi_main_obj.speed
                        # 计算速度
                        else:
                            self.speed = round(speed_distance / (time.time() - last_read_time), 1)
                        # self.speed = round(speed_distance / (time.time() - last_read_time), 1)
                        # 替换上一次的值
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                        # self.gps_log.info({'lng_lat': self.lng_lat})
                        last_read_time = time.time()
                    else:
                        self.last_lng_lat = copy.deepcopy(self.lng_lat)
                        last_read_time = time.time()
            time.sleep(0.5)

    # 必须使用线程发送发送检测数据
    def send_mqtt_detect_data(self):
        while True:
            time.sleep(1)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            if self.server_data_obj.mqtt_send_get_obj.pool_code:
                self.data_define_obj.pool_code = self.server_data_obj.mqtt_send_get_obj.pool_code

            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            # 更新真实数据
            if not config.home_debug:
                mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
                mqtt_send_detect_data['water'].update(self.pi_main_obj.water_data_dict)
            # 更新模拟数据
            else:
                mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                    mqtt_send_detect_data[k_all].update({new_key: pop_value})

            if self.b_draw_over_send_data and config.b_draw:
                # 添加经纬度
                mqtt_send_detect_data.update({'jwd': json.dumps(self.lng_lat)})
                mqtt_send_detect_data.update({'gjwd': json.dumps(self.gaode_lng_lat)})
                self.send(method='mqtt', topic='detect_data_%s' % config.ship_code, data=mqtt_send_detect_data,
                          qos=0)
                if len(self.data_define_obj.pool_code) > 0:
                    mqtt_send_detect_data.update({'mapId': self.data_define_obj.pool_code})
                    try:
                        self.send(method='http', data=mqtt_send_detect_data,
                                  url=config.http_data_save,
                                  http_type='POST')
                    except Exception as e:
                        self.data_save_logger.info({"发送检测数据error": e})
                    self.data_save_logger.info({"发送检测数据": mqtt_send_detect_data})
                    self.logger.info({"本地保存检测数据": mqtt_send_detect_data})
                # 发送结束改为False
                self.b_draw_over_send_data = False

    # 必须使用线程发送mqtt状态数据
    def send_mqtt_status_data(self):
        last_runtime = None
        last_run_distance = None
        while True:
            time.sleep(1)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            if self.server_data_obj.mqtt_send_get_obj.pool_code:
                self.data_define_obj.pool_code = self.server_data_obj.mqtt_send_get_obj.pool_code
            status_data = self.data_define_obj.status
            status_data.update({'mapId': self.data_define_obj.pool_code})
            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            status_data.update({'ping': round(self.ping, 1)})
            status_data.update({'current_lng_lat': self.gaode_lng_lat})
            status_data.update({'draw_time': [self.dump_draw_time, config.draw_time]})
            if config.b_sonar:
                self.lng_lat_list.append(self.gaode_lng_lat)
                deep_ = self.pi_main_obj.get_sonar_data()
                deep = deep_ if deep_ else 0
                self.deep_list.append(deep)
                assert len(self.deep_list) == len(self.lng_lat_list)
                if len(self.deep_list) % 20 == 0:
                    utils.generate_geojson(self.lng_lat_list, self.deep_list, b_save_data=True)
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
                status_data.update({'speed': self.speed})
            status_data.update({"runtime": round(time.time() - self.start_time)})
            status_data.update({"run_distance": round(self.run_distance, 1)})
            if last_runtime is None:
                last_runtime = 0
            if last_run_distance is None:
                last_run_distance = 0
            if time.time() % 10 < 1 and self.dump_draw_time == 0:
                # if os.path.exists(config.run_distance_time_path):
                #     try:
                #         with open(config.run_distance_time_path, 'r') as f:
                #             run_distance_time_data = json.load(f)
                #             save_distance = run_distance_time_data.get('save_distance')
                #             save_time = run_distance_time_data.get('save_time')
                #     except Exception as e:
                #         save_distance = 0
                #         save_time = 0
                #         self.logger.error({'读取run_distance_time_path': e})
                # else:
                #     save_distance = 0
                #     save_time = 0
                # with open(config.run_distance_time_path, 'w') as f:
                #     save_distance = save_distance + round(self.run_distance, 1) - last_run_distance
                #     # print('############################读取保存距离save_distance',save_distance)
                #     save_time = save_time + round(time.time() - self.start_time) - last_runtime
                #     json.dump({'save_distance': save_distance,
                #                'save_time': save_time}, f)
                if self.http_save_distance is not None and self.http_save_time is not None and self.http_save_id:
                    self.http_save_distance = self.http_save_distance + int(self.run_distance) - last_run_distance
                    self.http_save_time = self.http_save_time + int(time.time() - self.start_time) - last_runtime
                    send_mileage_data = {
                        "deviceId": config.ship_code,
                        "id": self.http_save_id,
                        "total": str(self.http_save_distance),
                        "totalTime": str(self.http_save_time)
                    }
                    return_data = self.send(method='http', data=send_mileage_data,
                                            url=config.http_mileage_update,
                                            http_type='POST',
                                            )
                    if return_data:
                        self.logger.info({'更新里程和时间成功': send_mileage_data})
                else:
                    if self.http_save_distance is None or self.http_save_time is None:
                        return_data = self.send(method='http',
                                                data='',
                                                url=config.http_mileage_get + "?deviceId=%s" % config.ship_code,
                                                http_type='GET'
                                                )
                        if return_data:
                            print('self.http_save_distance,self.http_save_time,self.http_save_id',
                                  self.http_save_distance,
                                  self.http_save_time, self.http_save_id)
                            self.http_save_distance = int(return_data.get("total"))
                            self.http_save_time = int(return_data.get("totalTime"))
                            self.http_save_id = return_data.get("id")
                    if self.http_save_distance and self.http_save_time:
                        self.http_save_distance = self.http_save_distance + int(self.run_distance) - last_run_distance
                        self.http_save_time = self.http_save_time + int(time.time() - self.start_time) - last_runtime
                        send_mileage_data = {
                            "id": config.ship_code,
                            "deviceId": config.ship_code,
                            "total": str(self.http_save_distance),
                            "totalTime": str(self.http_save_time)
                        }
                        return_data = self.send(method='http', data=send_mileage_data,
                                                url=config.http_mileage_update,
                                                http_type='POST',
                                                )
                        if return_data:
                            self.logger.info({'更新里程和时间成功': send_mileage_data})
                        # self.logger.info({'status_data_': })
                last_runtime = int(time.time() - self.start_time)
                last_run_distance = int(self.run_distance)
                status_data.update({"totle_distance": self.http_save_distance})
                status_data.update({"totle_time": self.http_save_time})
            # if os.path.exists(config.run_distance_time_path):
            #     try:
            #         with open(config.run_distance_time_path, 'r') as f:
            #             run_distance_time_data = json.load(f)
            #             save_distance = run_distance_time_data.get('save_distance')
            #             save_time = run_distance_time_data.get('save_time')
            #     except Exception as e:
            #         save_distance = 0
            #         save_time = 0
            #         self.logger.error({'读取run_distance_time_path': e})
            # else:
            #     save_distance = 0
            #     save_time = 0
            # with open(config.run_distance_time_path, 'w') as f:
            #     save_distance = save_distance + round(self.run_distance, 1) - last_run_distance
            #     save_time = save_time + round(time.time() - self.start_time) - last_runtime
            #     json.dump({'save_distance': save_distance,
            #                'save_time': save_time}, f)
            # last_runtime = round(time.time() - self.start_time)
            # last_run_distance = round(self.run_distance, 1)
            # status_data.update({"totle_distance": round(save_distance, 1)})
            # status_data.update({"totle_time": round(save_time)})
            # 更新船头方向
            if config.home_debug and self.current_theta is not None:
                status_data.update({"direction": round(self.current_theta, 1)})
            elif not config.home_debug and self.pi_main_obj.theta:
                status_data.update({"direction": round(self.pi_main_obj.theta, 1)})
            # 更新经纬度误差
            if self.lng_lat_error is not None:
                status_data.update({"lng_lat_error": self.lng_lat_error})
            # 更新真实数据
            if not config.home_debug:
                mqtt_send_status_data = data_define.fake_status_data(status_data)
            # 更新模拟数据
            else:
                mqtt_send_status_data = data_define.fake_status_data(status_data)
            if self.dump_energy is not None:
                self.dump_energy_deque.append(self.dump_energy)
                mqtt_send_status_data.update({'dump_energy': self.dump_energy})
            if not config.home_debug and self.pi_main_obj.dump_energy is not None:
                self.dump_energy_deque.append(self.pi_main_obj.dump_energy)
                mqtt_send_status_data.update({'dump_energy': self.pi_main_obj.dump_energy})
            if self.server_data_obj.mqtt_send_get_obj.b_record_point:
                mqtt_send_status_data.update({'is_record': 1})
            else:
                mqtt_send_status_data.update({'is_record': 2})
            # 更新船是否能运动状态
            mqtt_send_status_data.update({'is_wait': self.is_wait})
            # 向mqtt发送数据
            self.send(method='mqtt', topic='status_data_%s' % config.ship_code, data=mqtt_send_status_data,
                      qos=0)
            if time.time() % 10 < 1:
                self.logger.info({'status_data_': mqtt_send_status_data})

    # 配置更新
    def update_config(self):
        while True:
            time.sleep(1)
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
                    self.send(method='mqtt', topic='base_setting_%s' % config.ship_code,
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
                    self.send(method='mqtt', topic='height_setting_%s' % config.ship_code,
                              data=self.server_data_obj.mqtt_send_get_obj.height_setting_data,
                              qos=0)
                    self.logger.info({'height_setting': self.server_data_obj.mqtt_send_get_obj.height_setting_data})
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data = None
                    # 改为0位置状态，不再重复发送
                    self.server_data_obj.mqtt_send_get_obj.height_setting_data_info = 0

    def send_high_f_status_data(self):
        high_f_status_data = {}
        while 1:
            time.sleep(0.16)
            # print('send high_f')
            if config.home_debug and self.current_theta is None:
                self.current_theta = 1
            if config.home_debug and self.current_theta is not None:
                high_f_status_data.update({"direction": round(self.current_theta, 1)})
            elif not config.home_debug and self.pi_main_obj.theta:
                high_f_status_data.update({"direction": round(self.pi_main_obj.theta, 1)})
            high_f_status_data.update({"theta_error": round(self.theta_error, 1)})
            self.send(method='mqtt', topic='high_f_status_data_%s' % config.ship_code, data=high_f_status_data,
                      qos=0)

    # 状态检查函数，检查自身状态发送对应消息
    def check_status(self):
        while True:
            # 循环等待一定时间
            time.sleep(1)

            # 检查电量 如果连续20次检测电量平均值低于电量阈值就报警
            if config.energy_backhome:
                try:
                    energy_backhome_threshold = 20 if config.energy_backhome < 20 else config.energy_backhome
                except Exception as e_e:
                    print({'e_e': e_e})
                    energy_backhome_threshold = 30
                if len(self.dump_energy_deque) > 0 and sum(self.dump_energy_deque) / len(
                        self.dump_energy_deque) < energy_backhome_threshold:
                    self.low_dump_energy_warnning = 1
                else:
                    self.low_dump_energy_warnning = 0
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
                "distance": str(round(self.distance_p, 2)),
                # // 路径规划提示消息
                "path_info": '当前:%d 总共:%d 离岸:%.1f ' % (
                    int(self.path_info[0]), int(self.path_info[1]),
                    self.server_data_obj.mqtt_send_get_obj.bank_distance),
                "bank_distance": self.server_data_obj.mqtt_send_get_obj.bank_distance,
                "progress": progress,
                # 船执行手动控制信息
                "control_info": self.control_info,
                # 水泵开关状态消息
                "draw_info": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 声光报警器
                "audio_light_info": self.server_data_obj.mqtt_send_get_obj.audio_light,
                # 大灯
                "headlight_info": self.server_data_obj.mqtt_send_get_obj.headlight,
                # 舷灯
                "side_light_info": self.server_data_obj.mqtt_send_get_obj.side_light,
                # 自动巡航下角度偏差
                "theta_error": round(self.theta_error, 2),
            }
            notice_info_data.update({"mapId": self.data_define_obj.pool_code})
            # 遥控器是否启用
            if config.current_platform == config.CurrentPlatform.pi:
                notice_info_data.update({"b_start_remote": str(self.pi_main_obj.b_start_remote)})
            else:
                notice_info_data.update({"b_start_remote": "0"})
            # 罗盘提示消息
            if len(self.compass_notice_info) > 3:
                notice_info_data.update({"compass_notice_info": self.compass_notice_info})
            if not config.home_debug and self.pi_main_obj.compass_notice_info:
                notice_info_data.update({"compass_notice_info": self.pi_main_obj.compass_notice_info})
            # 使用电量告警是提示消息
            if self.low_dump_energy_warnning:
                notice_info_data.update({"low_dump_energy_warnning": self.low_dump_energy_warnning})
            self.send(
                method='mqtt',
                topic='notice_info_%s' % config.ship_code,
                data=notice_info_data,
                qos=0)
            if time.time() % 10 < 1:
                self.logger.info({'notice_info_': notice_info_data})
            # 调试时使用刷新角度用来更新地图上船
            # if self.server_data_obj.mqtt_send_get_obj.refresh_info_type == 1:
            #     if config.home_debug and self.current_theta is not None:
            #         self.current_theta += 0.1
            #     print('self.current_theta', self.current_theta)
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
                    self.send_obstacle = True
                    self.send(method='mqtt',
                              topic='refresh_%s' % config.ship_code,
                              data=save_plan_path_data,
                              qos=0)
            # 调试时更新角度后不能重复刷新
            # self.server_data_obj.mqtt_send_get_obj.refresh_info_type = 0

    # 检测网络延时
    def check_ping_delay(self):
        # 检查网络
        while True:
            time.sleep(5)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            if config.network_backhome:
                ping = check_network.get_ping_delay()
                if ping:
                    self.ping = ping
                else:
                    self.logger.error('当前无网络信号')
                    self.server_data_obj.mqtt_send_get_obj.is_connected = 0

    # 发送障碍物信息线程
    def send_distacne(self):
        while True:
            time.sleep(1.1)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            # 调试情况下
            if config.home_debug:
                distance_info_data = {}
                if len(self.distance_dict) > 0:
                    distance_info_data.update({'deviceId': config.ship_code})
                    distance_info_data.update({'distance_info': []})
                    for k in self.distance_dict.copy():
                        distance_info_data['distance_info'].append(
                            {'distance': self.distance_dict[k][0],
                             'angle': self.distance_dict[k][1]})
                    if self.current_theta:
                        distance_info_data.update({'direction': round(self.current_theta)})
                    else:
                        distance_info_data.update({'direction': 0})
                    # distance_info_data = {
                    #     # 船头角度  以北为0度 ，逆时针方向为正
                    #     "direction": self.current_theta,
                    #     # 距离信息 内部为列表，列中中元素为字典，distance为距离单位米  angle为角度单位度，以船头角度为0度 左正右负
                    #     "distance_info": [{"distance": 4.5, "angle": 20},
                    #                       {"distance": 7.5, "angle": -20},
                    #                       {"distance": 6, "angle": 0}],
                    # }
                    distance_info_data.update({'deviceId': config.ship_code})
                    self.send(method='mqtt',
                              topic='distance_info_%s' % config.ship_code,
                              data=distance_info_data,
                              qos=0)
                    print('distance_info_data', distance_info_data)
            # 真实情况
            else:
                distance_info_data = {}
                if len(self.pi_main_obj.distance_dict) > 0:
                    distance_info_data.update({'deviceId': config.ship_code})
                    distance_info_data.update({'distance_info': []})
                    for k in self.pi_main_obj.distance_dict.copy():
                        distance_info_data['distance_info'].append(
                            {'distance': self.pi_main_obj.distance_dict[k][0],
                             'angle': self.pi_main_obj.distance_dict[k][1]})
                    if self.current_theta:
                        distance_info_data.update({'direction': round(self.current_theta)})
                    else:
                        distance_info_data.update({'direction': 0})
                    self.send(method='mqtt',
                              topic='distance_info_%s' % config.ship_code,
                              data=distance_info_data,
                              qos=0)

    # 检查开关相关信息
    def check_switch(self):
        """
        检查开关信息发送到mqtt
        :return:
        """
        while True:
            time.sleep(0.5)
            switch_data = {
                "info_type": 2,  # 树莓派发给前端
                # 检测 1 检测 没有该键表示不检测
                "b_sampling": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 抽水 1 抽水 没有该键或者0表示不抽水
                "b_draw": self.server_data_obj.mqtt_send_get_obj.b_draw,
                # 前大灯 1 打开前大灯 没有该键或者0表示不打开
                "headlight": self.server_data_obj.mqtt_send_get_obj.headlight,
                # 声光报警器 1 打开声光报警器 没有该键或者0表示不打开
                "audio_light": self.server_data_obj.mqtt_send_get_obj.audio_light,
                # 舷灯 1 允许打开舷灯 没有该键或者0表示不打开
                "side_light": self.server_data_obj.mqtt_send_get_obj.side_light,
            }
            if not config.home_debug:
                switch_data.update({'b_draw': self.server_data_obj.mqtt_send_get_obj.b_draw})
                switch_data.update({'headlight': self.server_data_obj.mqtt_send_get_obj.headlight})
                switch_data.update({'audio_light': self.server_data_obj.mqtt_send_get_obj.audio_light})
                switch_data.update({'side_light': self.server_data_obj.mqtt_send_get_obj.side_light})
            self.send(method='mqtt',
                      topic='switch_%s' % config.ship_code,
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
                              topic='record_point_data_%s' % config.ship_code,
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
                    print('self.server_data_obj.mqtt_send_get_obj.pool_code',
                          self.server_data_obj.mqtt_send_get_obj.pool_code)
                    if self.server_data_obj.mqtt_send_get_obj.pool_code:
                        send_record_path = {
                            "deviceId": config.ship_code,
                            "mapId": self.server_data_obj.mqtt_send_get_obj.pool_code,
                            "route": json.dumps(self.record_path),
                            "routeName": self.server_data_obj.mqtt_send_get_obj.record_name
                        }
                        try:
                            print('send_record_path', send_record_path)
                            self.send(method='http', data=send_record_path,
                                      url=config.http_record_path,
                                      http_type='POST')
                        except Exception as e:
                            self.data_save_logger.info({"发送手动记录点存储数据error": e})
                        self.data_save_logger.info({"发送手动记录点存储数据": send_record_path})
                        self.record_path = []
                        pre_record_lng_lat = [10.0, 10.0]  # 随便设置的初始值
                time.sleep(3)

    def scan_cal(self):
        scan_points = []
        while True:
            if self.server_data_obj.mqtt_send_get_obj.surrounded_points is not None and \
                    len(self.server_data_obj.mqtt_send_get_obj.surrounded_points) > 0 and \
                    self.server_data_obj.mqtt_send_get_obj.surrounded_distance:
                scan_points = baidu_map.BaiduMap.scan_area(self.server_data_obj.mqtt_send_get_obj.surrounded_points,
                                                           self.server_data_obj.mqtt_send_get_obj.surrounded_distance)
                if scan_points:
                    self.send(method='mqtt',
                              topic='surrounded_path_%s' % config.ship_code,
                              data={
                                  "lng_lat": scan_points
                              },
                              qos=0)
                print('scan_points', scan_points)
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
                return_data = self.send(method='http',
                                        data='',
                                        url=config.http_record_get + "?id=%s" % self.server_data_obj.mqtt_send_get_obj.path_id,
                                        http_type='GET'
                                        )
                if len(return_data) > 0:
                    print(return_data)
                    self.server_data_obj.mqtt_send_get_obj.path_id = None
                    # 解析返回到的路径id
                    record_points = json.loads(return_data[0].get('route'))
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
                    time.sleep(3)

    # 开机启动一次函数
    def start_once_func(self):
        http_get_time = True
        while True:
            if not config.home_debug and not self.is_init_motor:
                self.pi_main_obj.init_motor()
                self.pi_main_obj.set_draw_deep(deep_pwm=config.max_deep_steer_pwm, b_slow=False)
                self.is_init_motor = 1
            if not self.b_check_get_water_data and self.gaode_lng_lat is not None:
                adcode = baidu_map.BaiduMap.get_area_code(self.gaode_lng_lat)
                self.area_id = data_valid.adcode_2_area_id(adcode)
                data_valid.get_current_water_data(area_id=self.area_id)
                self.b_check_get_water_data = 1
            if http_get_time:
                if self.http_save_distance is None or self.http_save_time is None:
                    return_data = self.send(method='http',
                                            data='',
                                            url=config.http_mileage_get + "?deviceId=%s" % config.ship_code,
                                            http_type='GET'
                                            )
                    print('return_data http_mileage_get', return_data)
                    if return_data:
                        self.http_save_distance = int(return_data.get("total"))
                        self.http_save_time = int(return_data.get("totalTime"))
                        self.http_save_id = return_data.get('id')
                        print('self.http_save_distance,self.http_save_time,self.http_save_id', self.http_save_distance,
                              self.http_save_time, self.http_save_id)
                    if return_data is None or not return_data:
                        send_mileage_data = {
                            "deviceId": config.ship_code,
                            "total": str(0),
                            "totalTime": str(0)
                        }
                        self.send(method='http',
                                  data=send_mileage_data,
                                  url=config.http_mileage_save,
                                  http_type='POST',
                                  )
                http_get_time = False
            # 发送模拟障碍物
            if self.send_obstacle:
                obstacle_points = {"lng_lat": config.obstacle_points}
                self.send(method='mqtt',
                          data=obstacle_points,
                          topic='obstacle_points_%s' % config.ship_code,
                          qos=0
                          )
                self.logger.info({"obstacle_points": obstacle_points})
                self.send_obstacle = False
            time.sleep(3)

    # 发送数据
    def send(self, method, data, topic='test', qos=0, http_type='POST', url='', parm_type=1):
        """
        :param url:
        :param http_type:
        :param qos:
        :param topic:
        :param data: 发送数据
        :param method 获取数据方式　http mqtt com
        """
        # assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
        # self.logger.info({'method':method,'url':url})
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type, data, url, parm_type=parm_type)
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
            elif http_type == 'GET' and r'mileage/getOne' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data["success"]:
                        self.logger.error('mileage/getOne GET请求失败')
                    task_data = content_data.get("data")
                    if task_data:
                        self.logger.info({'mileage/getOne': task_data.get('items')})
                        return task_data.get('items')
                    return False
                else:
                    return False
            elif http_type == 'GET' and r'device/getRoute' in url:
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data["success"]:
                        self.logger.error('device/getRoute GET请求失败')
                    task_data = content_data.get("data")
                    if task_data:
                        self.logger.info({'device/getRoute': task_data.get('items')})
                        return task_data.get('items')
                    return False
                else:
                    return False
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                else:
                    return True
        elif method == 'mqtt':
            self.server_data_obj.send_server_mqtt_data(data=data, topic=topic, qos=qos)

    def send_stc_data(self, data):
        """
        发送给单片机数据
        :param data:
        :return:
        """
        if self.last_send_stc_log_data is not None and self.last_send_stc_log_data != data:
            self.com_data_send_logger.info(data)
            self.last_send_stc_log_data = data
        if config.b_pin_stc:
            self.pi_main_obj.stc_obj.send_stc_data(data)
        elif config.b_com_stc:
            self.pi_main_obj.com_data_obj.send_data(data)
