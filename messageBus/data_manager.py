"""
管理数据收发
"""
import math
import time
import json
import copy
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
        self.control_info = ''  # 船控制提示信息
        self.ping = 0  # 网络延时
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
        self.send_obstacle = False
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
        self.task_list = []  # 获取存储的任务  经纬度，采样深度，采样量数据样式([lng,lat],[bottle_id,deep,capacity],[bottle_id,deep,capacity])
        self.sort_task_list = []  # 获取存储的任务  经纬度，采样深度，采样量数据样式[[lng,lat],[bottle_id,deep,capacity],[bottle_id,deep,capacity]]
        self.sort_task_done_list = []  # 总共有多少个点抽水完成存储0/1  单个长度等于任务点数量[[0,0],[0,0,0,0]
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

    # 重连mqtt服务器
    def connect_mqtt_server(self):
        while True:
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                self.server_data_obj.mqtt_send_get_obj.mqtt_connect()
            time.sleep(5)

    # 抽水
    def draw(self):
        """
        抽水
        """
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
                # 如果电脑控制抽水时候打开了遥控  将电脑抽水置位0
                if self.server_data_obj.mqtt_send_get_obj.b_draw:
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0
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
                            self.dump_draw_time = config.draw_time - int(
                                time.time() - self.draw_start_time)  # 提示用户抽水剩余时间
                            if time.time() - self.draw_start_time > config.draw_time:
                                self.dump_draw_time = 0
                                self.b_draw_over_send_data = True
                                self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                                self.is_need_drain = True  # 抽水完成需要排水
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
        # 低电量蜂鸣改红灯  断网为红灯
        if self.low_dump_energy_warnning or self.b_network_backhome:
            self.server_data_obj.mqtt_send_get_obj.status_light = 1
        send_stc_data = 'E%sZ' % (str(self.server_data_obj.mqtt_send_get_obj.status_light))
        self.send_stc_data(send_stc_data)
        self.last_status_light = self.server_data_obj.mqtt_send_get_obj.status_light

    # 外围设备控制线程函数
    def control_peripherals(self):
        while True:
            time.sleep(0.2)
            if not config.home_debug and self.pi_main_obj:
                self.control_relay()

    # 抽水排水控制
    def control_draw_thread(self):
        while True:
            time.sleep(0.2)
            # 当状态不是遥控器控制,不在执行任务状态且不在抽水过程中收回抽水杆子
            self.draw()

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
        self.server_data_obj.mqtt_send_get_obj.back_home = 0
        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
        self.server_data_obj.mqtt_send_get_obj.keep_point = 0
        self.point_arrive_start_time = None  # 清楚记录长期不到时间
        self.theta_error = 0
        self.b_stop_path_track = False
        self.obstacle_avoid_time_distance = []
        self.b_at_home =0

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
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = ShipStatus.remote_control
                # 切换到电脑手动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction in [-1, 0, 90, 180, 270, 10, 190,
                                                                                       1180, 1270]:
                    self.ship_status = ShipStatus.computer_control
                # 切换到返航   --还没有执行任何操作不能切换到返航
                # elif return_ship_status is not None:
                #     self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                #     self.ship_status = return_ship_status
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
                if not config.home_debug and self.pi_main_obj.b_start_remote:
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
                if not config.home_debug and self.pi_main_obj.b_start_remote:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    if self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:  # 清楚暂停标记
                        self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type = 2
                    self.ship_status = ShipStatus.remote_control
                # 切换到返航
                elif return_ship_status is not None:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = return_ship_status
                    self.last_ship_status = ShipStatus.computer_auto
                # 取消自动模式
                elif self.server_data_obj.mqtt_send_get_obj.control_move_direction == -1:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.clear_all_status()  # 取消自动时清楚所有自动信息标记
                    self.ship_status = ShipStatus.computer_control
                # 切换到手动
                elif self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                    self.server_data_obj.mqtt_send_get_obj.control_move_direction = -2
                    self.ship_status = ShipStatus.computer_control
                # 到点
                elif self.b_arrive_point:
                    self.last_ship_status = ShipStatus.computer_auto
                    if config.current_ship_type == config.ShipType.water_detect:
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
                if self.b_sampling == 2 or self.server_data_obj.mqtt_send_get_obj.b_draw == 0:
                    # 如果自动每个点均已经到达
                    if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0 and \
                            all(self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                        self.clear_all_status()  # 最后一个任务点也到达后清楚状态
                        self.ship_status = ShipStatus.computer_control
                        # 自动模式下到达最后一个点切换为电脑手动状态
                        self.last_ship_status = ShipStatus.computer_control
                    self.b_sampling = 0
                    self.server_data_obj.mqtt_send_get_obj.b_draw = 0
                    self.ship_status = self.last_ship_status
                # if self.server_data_obj.mqtt_send_get_obj.b_draw == 0:
                #     if not config.home_debug and not self.pi_main_obj.b_start_remote:
                #         self.ship_status = ShipStatus.computer_control

            # 遥控器状态切换到其他状态
            if self.ship_status == ShipStatus.remote_control:
                # 切换到电脑控制状态
                if not config.home_debug and self.pi_main_obj.b_start_remote == 0:
                    self.pi_main_obj.stop()  # 遥控器不使能是先停止一下，防止遥控器推着摇杆关机
                    self.ship_status = ShipStatus.computer_control
                # 切换到任务模式
                elif not config.home_debug and self.pi_main_obj.remote_draw_status == 1:
                    self.last_ship_status = ShipStatus.remote_control
                    self.ship_status = ShipStatus.tasking

            # 返航状态切换到其他状态
            if self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 判断是否返航到家
                if self.b_at_home:
                    self.ship_status = ShipStatus.at_home
                # 切换到遥控器模式 使能遥控器
                if not config.home_debug and self.pi_main_obj.b_start_remote:
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
                if not config.home_debug and self.pi_main_obj.b_start_remote:
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
                                                            config.min_steer_distance)
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
                                                                config.min_steer_distance / 100)
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
                                                                config.min_steer_distance / 100)
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

    def get_avoid_obstacle_point(self, path_planning_point_gps=None):
        """
        根据障碍物地图获取下一个运动点
        :return: 下一个目标点，是否需要紧急停止【True为需要停止，False为不需要停止】
        """
        next_point_lng_lat = copy.deepcopy(path_planning_point_gps)
        if config.b_millimeter_wave:
            # 不避障
            if config.obstacle_avoid_type == 0:
                return path_planning_point_gps, False
            # 避障停止
            elif config.obstacle_avoid_type == 1:
                # 船头角度左右3个扇区有障碍物
                if 1 in self.pi_main_obj.obstacle_list[
                        int(self.pi_main_obj.cell_size / 2) - 3:int(self.pi_main_obj.cell_size / 2) + 3]:
                    return path_planning_point_gps, True
                else:
                    return path_planning_point_gps, False
            # 避障绕行，根据障碍物计算下一个目标点
            elif config.obstacle_avoid_type in [2, 3]:
                angle = vfh.vfh_func(self.pi_main_obj.obstacle_list, self.pi_main_obj.ceil_go_throw)
                # print('避障角度：', angle)
                if angle == -1:  # 没有可通行区域
                    abs_angle = (self.pi_main_obj.theta + config.field_of_view / 2) % 360
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
                    abs_angle = (self.pi_main_obj.theta + angle) % 360
                    next_point_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
                                                                                     self.lng_lat[1],
                                                                                     abs_angle,
                                                                                     2)
                    self.b_avoid = 1
                    # print('绕行角度:', abs_angle)
                    return next_point_lng_lat, False
        else:
            return path_planning_point_gps, False

    # 构建模拟避障数据
    def build_obstacle_data(self):
        # 角度限制
        while True:
            # 计算船到每个障碍物的距离和角度
            for index, point in enumerate(config.obstacle_points):
                if not self.gaode_lng_lat or not self.current_theta:
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
                        self.throttle = max(min(distance_average / config.min_steer_distance, 1), 0)  # 按照障碍物距离等比例减少油门
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
            # print('self.obstacle_list',self.obstacle_list)
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
                angle = vfh.vfh_func(self.obstacle_list)
                # print('angle',angle)
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
        if not config.home_debug and \
                self.pi_main_obj.lng_lat and \
                self.pi_main_obj.lng_lat[0] > 1 and \
                self.pi_main_obj.lng_lat[1] > 1:
            cal_lng_lat = copy.deepcopy(self.pi_main_obj.lng_lat)
        else:
            cal_lng_lat = copy.deepcopy(self.lng_lat)
        distance = lng_lat_calculate.distanceFromCoordinate(
            cal_lng_lat[0],
            cal_lng_lat[1],
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
            if not config.home_debug and \
                    self.pi_main_obj.lng_lat and \
                    self.pi_main_obj.lng_lat[0] > 1 and \
                    self.pi_main_obj.lng_lat[1] > 1:
                cal_lng_lat = copy.deepcopy(self.pi_main_obj.lng_lat)
            else:
                cal_lng_lat = copy.deepcopy(self.lng_lat)
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
            # print('避障等级', config.obstacle_avoid_type)
            if config.obstacle_avoid_type in [2, 3] and self.b_avoid:
                self.obstacle_avoid_time_distance.append([time.time(), distance_sample])  # 记录当前时间和到目标点距离
                # 判断指定时间内避障行走距离是否大于指定距离米
                if len(self.obstacle_avoid_time_distance) > 2:
                    if abs(self.obstacle_avoid_time_distance[0][0] - self.obstacle_avoid_time_distance[-1][0]) >= 5:
                        if abs(self.obstacle_avoid_time_distance[0][1] - self.obstacle_avoid_time_distance[-1][1]) < 2:
                            self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type = 1
                            print('############################无法避障暂停#############################')
                        else:
                            del self.obstacle_avoid_time_distance[:-1]
            # 计算偏差距离
            all_distance = lng_lat_calculate.distanceFromCoordinate(
                self.lng_lat[0], self.lng_lat[1], target_lng_lat_gps[0],
                target_lng_lat_gps[1])
            # 当前点到目标点角度
            point_theta = lng_lat_calculate.angleFromCoordinate(cal_lng_lat[0],
                                                                cal_lng_lat[1],
                                                                target_lng_lat_gps[0],
                                                                target_lng_lat_gps[1])
            # 计算偏差角度  目标在左侧为正在右侧为负
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
            self.bank_stop.append([time.time(), all_distance, theta_error])  # 记录当前时间和到目标点距离
            # 判断指定时间内避障行走距离是否大于指定距离米
            if not config.home_debug:
                if len(self.bank_stop) > 2:
                    # 3秒 距离变化小于1米  角度变化小于2度 则暂停
                    if abs(self.bank_stop[0][0] - self.bank_stop[-1][0]) >= 3:
                        if abs(self.bank_stop[0][1] - self.bank_stop[-1][1]) < 1 and abs(
                                self.bank_stop[0][2] - self.bank_stop[-1][2]) < 2:
                            print(self.bank_stop[0], self.bank_stop[-1])
                            self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type = 1
                            print('############################无法躲避岸边暂停#############################')
                        else:
                            print('self.bank_stop', len(self.bank_stop), self.bank_stop)
                            del self.bank_stop[:-1]
            if abs(theta_error) > 180:
                if theta_error > 0:
                    theta_error = theta_error - 360
                else:
                    theta_error = 360 + theta_error
            self.theta_error = theta_error
            # 计算距离余弦值 将超过90度的余弦距离置位0
            if abs(theta_error) <= 90:
                distance_cos = all_distance * math.cos(math.radians(abs(theta_error)))
            else:
                distance_cos = 0
            # print('point_theta,self.current_theta', point_theta, self.current_theta)
            # print('theta_error', theta_error)
            # print('distance_cos', distance_cos)
            if config.obstacle_avoid_type in [2, 3]:
                if config.home_debug:
                    throttle = self.throttle
                else:
                    throttle = self.pi_main_obj.throttle
            else:
                throttle = 1
            # print('油门', throttle)
            left_pwm, right_pwm = self.path_track_obj.pid_pwm_4(distance=distance_cos,
                                                                theta_error=theta_error,
                                                                throttle=throttle)
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
                # delta_distance = forward_power * 0.0006
                # delta_theta = steer_power * 0.04
                delta_distance = forward_power * 0.0012
                delta_theta = steer_power * 0.06
                # print('delta_theta', delta_theta, )
                if self.current_theta is not None:
                    self.current_theta = (self.current_theta - delta_theta / 2) % 360
                    self.current_theta += 0.1
                self.last_lng_lat = copy.deepcopy(self.lng_lat)
                self.last_read_time_debug = time.time()
                self.lng_lat = lng_lat_calculate.one_point_diatance_to_end(cal_lng_lat[0],
                                                                           cal_lng_lat[1],
                                                                           self.current_theta,
                                                                           delta_distance)
            else:
                # 判断是否需要避障处理
                # print('b_stop', b_stop)
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

    # 处理电机控制
    def move_control(self):
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
                ShipStatus.at_home: '返航点',
                ShipStatus.idle: '等待',
            }
            self.direction = self.server_data_obj.mqtt_send_get_obj.control_move_direction
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
                while self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(0) > 0:
                    # 被暂停
                    if self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type == 1:
                        if self.ship_status != ShipStatus.computer_auto:  # 暂停时允许使用遥控器取消暂停状态
                            break
                        if not config.home_debug:
                            self.pi_main_obj.stop()
                        continue
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
                    if config.path_plan_type:
                        next_lng_lat = self.calc_target_lng_lat(index)
                        # 距离从下一路径点到采样点距离改为当前位置到采样点距离
                        arrive_sample_distance = lng_lat_calculate.distanceFromCoordinate(self.lng_lat[0],
                                                                                          self.lng_lat[1],
                                                                                          sampling_point_gps[0],
                                                                                          sampling_point_gps[1])
                        if arrive_sample_distance < config.forward_see_distance:
                            b_arrive_sample = self.points_arrive_control(sampling_point_gps, sampling_point_gps,
                                                                         b_force_arrive=False)
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
                        print('b_arrive_sample', b_arrive_sample, 'self.action_id', self.action_id)
                        # 判断是否是行动抽水
                        if self.action_id and self.sample_index[index]:
                            self.b_arrive_point = 1  # 到点了用于通知抽水  暂时修改为不抽水
                        self.is_need_update_plan = 1  # 到点后需要更新任务状态
                        self.point_arrive_start_time = None
                        self.server_data_obj.mqtt_send_get_obj.sampling_points_status[index] = 1
                        time.sleep(1)
                    # 全部点到达后清除自动状态
                    if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) == sum(
                            self.server_data_obj.mqtt_send_get_obj.sampling_points_status):
                        self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
                        if sum(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0:
                            self.is_plan_all_arrive = 1
                    if self.ship_status != ShipStatus.computer_auto:
                        break

            # 返航 断网返航 低电量返航
            elif self.ship_status in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
                # 有返航点下情况下返回返航点，没有则停止
                if self.home_lng_lat:
                    back_home_flag = self.points_arrive_control(self.home_lng_lat, self.home_lng_lat, False, False)
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
            status_data.update({'action_type': self.server_data_obj.mqtt_send_get_obj.action_type})
            status_data.update({'pause_continue': self.server_data_obj.mqtt_send_get_obj.pause_continue_data_type})
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
            status_data.update({"totle_distance": 0 if not self.http_save_distance else self.http_save_distance})
            status_data.update({"totle_time": 0 if not self.http_save_time else self.http_save_time})
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
                mqtt_send_status_data = status_data
            # 更新模拟数据
            else:
                self.dump_energy = data_define.get_dump_energy()
                mqtt_send_status_data = status_data
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
            if config.home_debug and self.current_theta is None:
                self.current_theta = 1
            if config.home_debug and self.current_theta is not None:
                self.current_theta += 0.1 if random.random() > 0.5 else -0.1
                if self.current_theta <= 0:
                    self.current_theta = 0.1
                high_f_status_data.update({"direction": round(self.current_theta, 1)})
            elif not config.home_debug and self.pi_main_obj.theta:
                high_f_status_data.update({"direction": round(self.pi_main_obj.theta, 1)})
            high_f_status_data.update({"theta_error": round(self.theta_error, 1)})
            self.send(method='mqtt', topic='high_f_status_data_%s' % config.ship_code, data=high_f_status_data,
                      qos=0)

    # 预先存储任务   计算距离并排序
    def check_task(self):
        if self.server_data_obj.mqtt_send_get_obj.get_task == 1 and self.server_data_obj.mqtt_send_get_obj.task_id:
            print("获取任务task_id", self.server_data_obj.mqtt_send_get_obj.task_id)
            url = config.http_get_task + "?taskId=%s" % self.server_data_obj.mqtt_send_get_obj.task_id
            return_data = self.server_data_obj.send_server_http_data('GET', '', url, token=self.token)
            task_data_list = None
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
                    # 上次任务还没有完成继续任务
                    if temp_task_data and content_data.get('data').get('records')[0].get('planId'):
                        self.action_id = content_data.get('data').get('records')[0].get('planId')
                        self.server_data_obj.mqtt_send_get_obj.action_type = 3
                        task_data_list = temp_task_data
                    else:
                        task_data_list = task_data
            if not task_data_list:
                print('############ 没有任务数据')
                return
            self.server_data_obj.mqtt_send_get_obj.get_task = 0
            self.task_list = task_data_list
            self.sort_task_list = task_data_list
            self.has_task = 1

    def loop_check_task(self):
        while True:
            time.sleep(1)
            if len(self.sort_task_list) == 0:
                self.check_task()  # 检查是否需要发送预先存储任务
            # 有任务发送任务状态 更新任务为正在执行
            if self.has_task == 1:
                # 任务模式自己规划路径不再重新规划路径
                # 存放路径点和监测点
                path_planning_data = {"sampling_points": [],
                                      "path_points": []
                                      }
                for i in self.sort_task_list:
                    if i.get("type") == 1:  # 检测点添加到监测点轨迹中
                        self.sample_index.append(1)
                    else:
                        self.sample_index.append(0)
                    path_planning_data.get("sampling_points").append(i.get("lnglat"))
                    path_planning_data.get("path_points").append(i.get("lnglat"))
                self.send(method='mqtt',
                          topic='path_planning_%s' % config.ship_code,
                          data=path_planning_data,
                          qos=0)
                print('mqtt任务经纬度数据', path_planning_data)
                print("task_id", self.server_data_obj.mqtt_send_get_obj.task_id)
                self.has_task = 0
            if self.server_data_obj.mqtt_send_get_obj.cancel_task and self.server_data_obj.mqtt_send_get_obj.task_id:
                print("task_id", self.server_data_obj.mqtt_send_get_obj.task_id)
                data = {"taskId": self.server_data_obj.mqtt_send_get_obj.task_id, "state": "0"},
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         config.http_update_task,
                                                                         data, parm_type=2, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data.get("success"):
                        self.logger.error('更新任务状态请求失败')
                        print('更新任务状态请求content_data', content_data)
                    else:
                        self.server_data_obj.mqtt_send_get_obj.cancel_task = 0
                        self.server_data_obj.mqtt_send_get_obj.task_id = ''
            if self.arrive_all_task:
                data = {'taskId': self.server_data_obj.mqtt_send_get_obj.task_id}
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         config.http_delete_task,
                                                                         data, parm_type=2, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data.get("success"):
                        self.logger.error('取消任务请求失败')
                        print('取消任务content_data', content_data)
                    else:
                        self.arrive_all_task = 0
                        # self.server_data_obj.mqtt_send_get_obj.task_id = ''

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
                    print({'设置返航电量错误:': e_e})
                    energy_backhome_threshold = 20
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
            if not config.home_debug:
                notice_info_data.update({"b_start_remote": str(self.pi_main_obj.b_start_remote)})
            else:
                notice_info_data.update({"b_start_remote": "0"})
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
            # 保存数据与发送刷新后提示消息
            if len(self.data_define_obj.pool_code) > 0:
                save_plan_path_data = {
                    "mapId": self.data_define_obj.pool_code,
                    'sampling_points': self.server_data_obj.mqtt_send_get_obj.sampling_points,
                    'path_points': self.server_data_obj.mqtt_send_get_obj.path_planning_points,
                }
                # save_data.set_data(save_plan_path_data, config.save_plan_path)
                if self.server_data_obj.mqtt_send_get_obj.refresh_info_type == 1:
                    save_plan_path_data.update({"info_type": 2})
                    # self.send_obstacle = True  # 调试时发送障碍物图标
                    self.send(method='mqtt',
                              topic='refresh_%s' % config.ship_code,
                              data=save_plan_path_data,
                              qos=0)
            else:
                self.server_data_obj.mqtt_send_get_obj.refresh_info_type = 2

    # 检测网络延时
    def check_ping_delay(self):
        # 检查网络
        while True:
            time.sleep(5)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            print('config.network_backhome',config.network_backhome)
            if config.network_backhome:
                ping = check_network.get_ping_delay()
                if ping:
                    self.ping = ping
                else:
                    self.logger.error('当前无网络信号')
                    self.server_data_obj.mqtt_send_get_obj.is_connected = 0

    # 发送障碍物信息线程
    def send_distacne(self):
        last_send_time = time.time()  # 超过两秒没有障碍物就清空
        b_clear = False
        while True:
            time.sleep(0.01)
            if not self.server_data_obj.mqtt_send_get_obj.is_connected:
                continue
            # 调试情况下
            if config.home_debug and self.send_obstacle:
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
                    distance_info_data.update({'deviceId': config.ship_code})
                    self.send(method='mqtt',
                              topic='distance_info_%s' % config.ship_code,
                              data=distance_info_data,
                              qos=0)
                    last_send_time = time.time()
                    b_clear = True
                    time.sleep(0.4)
            # 真实情况
            elif not config.home_debug:
                distance_info_data = {}
                if len(self.pi_main_obj.distance_dict) > 0:
                    # print('障碍物列表',self.pi_main_obj.distance_dict)
                    distance_info_data.update({'deviceId': config.ship_code})
                    distance_info_data.update({'distance_info': []})
                    for k in self.pi_main_obj.distance_dict.copy():
                        # 将原来左负右正改为左正右负，然后将负数角度转为正
                        angle = -1 * self.pi_main_obj.distance_dict[k][1]
                        if angle < 0:
                            angle = 360 + angle
                        distance_info_data['distance_info'].append(
                            {'distance': self.pi_main_obj.distance_dict[k][0],
                             'angle': angle})
                    if self.pi_main_obj.theta:
                        distance_info_data.update({'direction': round(self.pi_main_obj.theta, 1)})
                    else:
                        distance_info_data.update({'direction': 0})
                    self.send(method='mqtt',
                              topic='distance_info_%s' % config.ship_code,
                              data=distance_info_data,
                              qos=0)
                    last_send_time = time.time()
                    b_clear = True
                    time.sleep(0.05)  # 发送数据后延时一点时间
            if time.time() - last_send_time > 2 and b_clear:
                distance_info_data_clear = {}
                distance_info_data_clear.update({'deviceId': config.ship_code})
                distance_info_data_clear.update({'distance_info': []})
                distance_info_data_clear.update({'direction': 0})
                self.send(method='mqtt',
                          topic='distance_info_%s' % config.ship_code,
                          data=distance_info_data_clear,
                          qos=0)
                b_clear = False

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
                        return_data = self.server_data_obj.send_server_http_data('POST',
                                                                                 config.http_record_path,
                                                                                 send_record_path, token=self.token)
                        if return_data:
                            content_data = json.loads(return_data.content)
                            if content_data.get("code") != 200 and content_data.get("code") != 20000:
                                self.logger.error('发送手动记录点存储数据请求失败')
                                print('发送手动记录点存储返回数据', content_data)
                            else:
                                self.data_save_logger.info({"发送手动记录点存储数据": send_record_path})
                                self.record_path = []
                                pre_record_lng_lat = [10.0, 10.0]  # 随便设置的初始值
                        else:
                            self.data_save_logger.info({"发送手动记录点存储数据error": 11})

                time.sleep(3)

    def scan_cal(self):
        scan_points = []
        while True:
            if self.server_data_obj.mqtt_send_get_obj.surrounded_points is not None and \
                    len(self.server_data_obj.mqtt_send_get_obj.surrounded_points) > 0 and \
                    self.server_data_obj.mqtt_send_get_obj.surrounded_distance:
                print(
                    'self.server_data_obj.mqtt_send_get_obj.surrounded_points,self.server_data_obj.mqtt_send_get_obj.surrounded_distance',
                    self.server_data_obj.mqtt_send_get_obj.surrounded_points,
                    self.server_data_obj.mqtt_send_get_obj.surrounded_distance)
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

    # 开机启动一次函数
    def start_once_func(self):
        while True:
            if not config.home_debug and not self.is_init_motor:
                self.pi_main_obj.init_motor()
                self.pi_main_obj.set_draw_deep(deep_pwm=config.max_deep_steer_pwm, b_slow=False)
                self.is_init_motor = 1
            if not self.b_check_get_water_data and self.gaode_lng_lat is not None:
                adcode = baidu_map.BaiduMap.get_area_code(self.gaode_lng_lat)
                if adcode:
                    self.area_id = data_valid.adcode_2_area_id(adcode)
                    data_valid.get_current_water_data(area_id=self.area_id)
                    self.b_check_get_water_data = 1
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
            # 获取token
            # if not self.server_data_obj.mqtt_send_get_obj.token:
            #     self.send(method='mqtt',
            #               topic='token_%s' % config.ship_code,
            #               data={"type": 1},
            #               qos=0)
            time.sleep(3)

    # 将要向服务器发送HTTP请求的移动到此函数中
    def loop_send_http(self):
        last_runtime = 0
        last_run_distance = 0
        http_get_time = True
        while True:
            time.sleep(1)
            # 登录获取值
            if not self.token:
                login_data = {"deviceId": config.ship_code}
                return_login_data = self.server_data_obj.send_server_http_data('POST', login_data,
                                                                               config.http_get_token, token=self.token)
                if return_login_data:
                    return_login_data_json = json.loads(return_login_data.content)
                    if return_login_data_json.get("code") == 200 and return_login_data_json.get("data"):
                        print('登录返回token', return_login_data_json.get("data").get("token"))
                        self.token = return_login_data_json.get("data").get("token")
                    else:
                        print('return_login_data', return_login_data_json)
                else:
                    print('return_login_data', return_login_data)
            if not self.token:
                continue
            if http_get_time:
                if self.http_save_distance is None or self.http_save_time is None:
                    url = config.http_mileage_get + "?deviceId=%s" % config.ship_code
                    return_data = self.server_data_obj.send_server_http_data('GET', '', url, token=self.token)
                    if return_data:
                        return_data_json = json.loads(return_data.content)
                        print('获取里程返回数据', return_data_json)
                        if return_data_json.get('data') and return_data_json.get('data').get('mileage'):
                            self.http_save_distance = int(return_data_json.get('data').get('mileage').get("total"))
                            self.http_save_time = int(return_data_json.get('data').get('mileage').get("totalTime"))
                            self.http_save_id = return_data_json.get('data').get('mileage').get('id')
                            http_get_time = False
                        # 能返回数据但是里面不包含里程就更新里程
                        elif not return_data.get('data') or return_data.get('data').get('mileage') is None:
                            send_mileage_data = {
                                "deviceId": config.ship_code,
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
            if last_runtime is None:
                last_runtime = 0
            if last_run_distance is None:
                last_run_distance = 0
            # 更新总时间和总里程
            if time.time() % 10 < 1 and self.dump_draw_time == 0:
                if self.http_save_distance is not None and self.http_save_time is not None and self.http_save_id:
                    self.http_save_distance = self.http_save_distance + int(self.run_distance) - last_run_distance
                    self.http_save_time = self.http_save_time + int(time.time() - self.start_time) - last_runtime
                    send_mileage_data = {
                        "deviceId": config.ship_code,
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
                data = {"deviceId": config.ship_code,
                        "mapId": self.data_define_obj.pool_code,
                        "taskId": self.server_data_obj.mqtt_send_get_obj.task_id,
                        "planName": self.server_data_obj.mqtt_send_get_obj.action_name,
                        }
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
            # 上传检测数据
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
                mqtt_send_detect_data.update(mqtt_send_detect_data.get('water'))
                if self.action_id:
                    mqtt_send_detect_data.update({'planId': self.action_id})
                else:
                    mqtt_send_detect_data.update({'planId': ''})
                self.send(method='mqtt', topic='detect_data_%s' % config.ship_code, data=mqtt_send_detect_data,
                          qos=0)
                if len(self.data_define_obj.pool_code) > 0:
                    mqtt_send_detect_data.update({'mapId': self.data_define_obj.pool_code})
                    return_data = self.server_data_obj.send_server_http_data('POST',
                                                                             mqtt_send_detect_data,
                                                                             config.http_data_save, token=self.token)
                    if return_data:
                        content_data = json.loads(return_data.content)
                        if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                            self.logger.error({'POST发送检测请求失败': content_data})
                        else:
                            # 发送结束改为False
                            self.b_draw_over_send_data = False
                        self.logger.info({"本地保存检测数据": mqtt_send_detect_data})
            # 更新剩余任务点 到点减一 所有点到达后设置任务状态为0
            if self.server_data_obj.mqtt_send_get_obj.cancel_action == 1:  # 取消行动
                print('self.server_data_obj.mqtt_send_get_obj.cancel_action',
                      self.server_data_obj.mqtt_send_get_obj.cancel_action)
                self.server_data_obj.mqtt_send_get_obj.cancel_action = 0
                self.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
                update_plan_data = {"id": self.server_data_obj.mqtt_send_get_obj.task_id,
                                    # "taskTem": '[]',
                                    "state": 0,
                                    "deviceId": config.ship_code,
                                    "planId": self.action_id
                                    }
                print('更新任务消息', update_plan_data)
                self.sort_task_list = []
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         update_plan_data,
                                                                         config.http_plan_update, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if content_data.get("code") != 200:
                        self.logger.error('更新任务失败')
                    else:
                        self.logger.info({'更新任务': content_data})
                        self.server_data_obj.mqtt_send_get_obj.cancel_action = 0
                        self.server_data_obj.mqtt_send_get_obj.task_id = ''
                        self.action_id = None
                        self.server_data_obj.mqtt_send_get_obj.action_name = ""
            if self.is_plan_all_arrive:
                print('self.server_data_obj.mqtt_send_get_obj.task_id', self.server_data_obj.mqtt_send_get_obj.task_id)
                print('self.is_plan_all_arrive', self.is_plan_all_arrive)
                update_plan_data = {"id": self.server_data_obj.mqtt_send_get_obj.task_id,
                                    "taskTem": '[]',
                                    "state": 0,
                                    "deviceId": config.ship_code,
                                    "planId": ""
                                    }
                print('更新任务消息', update_plan_data)
                self.sort_task_list = []
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         update_plan_data,
                                                                         config.http_plan_update, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if content_data.get("code") != 200:
                        self.logger.error('更新任务失败')
                    else:
                        self.logger.info({'更新任务': content_data})
                        self.is_need_update_plan = 0
                        self.is_plan_all_arrive = 0
                        self.server_data_obj.mqtt_send_get_obj.task_id = ''
                        self.action_id = None
                        self.server_data_obj.mqtt_send_get_obj.action_name = ""
                self.server_data_obj.mqtt_send_get_obj.action_type = 2
            if self.is_need_update_plan == 1 and not self.is_plan_all_arrive and self.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(
                    0) > 0 and self.server_data_obj.mqtt_send_get_obj.task_id:
                print('#################self.is_need_update_plan', self.is_need_update_plan)
                print('#################self.server_data_obj.mqtt_send_get_obj.sampling_points_status',
                      self.server_data_obj.mqtt_send_get_obj.sampling_points_status)
                if len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0:
                    index = self.server_data_obj.mqtt_send_get_obj.sampling_points_status.index(0)
                    sampling_point_gps_list = []
                    for i in range(index, len(self.server_data_obj.mqtt_send_get_obj.sampling_points_status)):
                        sampling_point_gps = self.server_data_obj.mqtt_send_get_obj.sampling_points[i]
                        sampling_point_gps_list.append({"lnglat": sampling_point_gps, "type": self.sample_index[i]})
                else:
                    sampling_point_gps_list = []
                update_plan_data = {"id": self.server_data_obj.mqtt_send_get_obj.task_id,
                                    "taskTem": json.dumps(sampling_point_gps_list),
                                    "state": 1,
                                    "deviceId": config.ship_code,
                                    "planId": self.action_id
                                    }
                print('更新任务消息', update_plan_data)
                return_data = self.server_data_obj.send_server_http_data('POST',
                                                                         update_plan_data,
                                                                         config.http_plan_update, token=self.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if content_data.get("code") != 200:
                        self.logger.error('更新任务失败')
                    else:
                        self.logger.info({'更新任务': content_data})
                        self.is_need_update_plan = 0

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

    # 发送给单片机数据
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

    # 检查是否需要返航
    def check_backhome(self):
        """
        返回返航状态或者None
        :return:返回None为不需要返航，返回低电量返航或者断网返航
        """
        return_ship_status = None
        if config.network_backhome:
            #
            if int(config.network_backhome) > 600:
                network_backhome_time = 600
            else:
                network_backhome_time = int(config.network_backhome)
            # 使用过电脑端按键操作过才能进行断网返航
            if self.server_data_obj.mqtt_send_get_obj.b_receive_mqtt:
                if time.time() - self.server_data_obj.mqtt_send_get_obj.last_command_time > network_backhome_time:
                    return_ship_status = ShipStatus.backhome_network
                    # print('return_ship_status',return_ship_status)
        if self.low_dump_energy_warnning:
            # 记录是因为按了低电量判断为返航
            return_ship_status = ShipStatus.backhome_low_energy
        if return_ship_status is not None and self.ship_status not in [ShipStatus.backhome_network, ShipStatus.backhome_low_energy]:
            self.logger.info({"正在返航":return_ship_status})
        return return_ship_status
