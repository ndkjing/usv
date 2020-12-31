"""
管理数据收发
"""
import time
import json
import numpy as np
import os
import copy

from dataGetSend.data_define import DataDefine
from dataGetSend import data_define
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData
from utils import check_network
from utils.log import LogHandler
from baiduMap import baidu_map
from audios import audios_manager
from pathPlanning import a_star
from obstacleAvoid import basic_obstacle_avoid
from utils import lng_lat_calculate
import config

class DataManager:
    def __init__(self):
        self.data_define_obj = DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log')
        self.server_log = LogHandler('server_data')
        self.com_log = LogHandler('com_data')
        self.map_log = LogHandler('map_log')
        # mqtt服务器数据收发对象
        self.server_data_obj = ServerData(self.server_log, topics=self.data_define_obj.topics)
        # 串口数据收发对象
        self.com_data_obj = SerialData(config.port, config.baud, timeout=1 / config.com2pi_interval,
                                       logger=self.com_log)

        # 规划路径和
        self.plan_path = []
        # 规划路径状态
        self.plan_path_status = []
        # 规划路径点状态
        self.plan_path_points_status = []

        # 船当前正确前往哪个目的地
        self.current_ststus_index = -1
        self.start = False
        self.baidu_map_obj = None

        # 船最终控制移动方向
        self.ship_move_direction = str(360)

        # 船当前朝向
        self.ship_current_direction = -1

        # 左右侧超声波距离
        self.l_distance = None
        self.r_distance = None

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        while True:
            com_data_read = self.com_data_obj.readline()
            ## 解析串口发送过来的数据
            if com_data_read is None:
                continue
            self.logger.debug({'com_data_read': com_data_read})
            com_data_list = com_data_read.split(',')

            # 角度，TDS，温度，经度，纬度，左侧距离1，右侧距离2，距离目标点距离
            # 当前朝向
            self.ship_current_direction = com_data_list[0].split('AAA')[1]
            # TDO
            self.data_define_obj.water['TD'] = com_data_list[1]
            ## 水质数据
            # 水温
            self.data_define_obj.water['wt'] = com_data_list[2]
            # 经纬度
            if float(com_data_list[3]) < 0 or float(com_data_list[4]) < 0:
                self.data_define_obj.status['current_lng_lat'] = None
            elif float(com_data_list[3]) > 180 or float(com_data_list[4]) > 80:
                self.data_define_obj.status['current_lng_lat'] = None
            else:
                self.data_define_obj.status['current_lng_lat'] = [float(com_data_list[3]), float(com_data_list[4])]

            # 左右侧的超声波检测距离 转化为单位米
            self.l_distance, self.r_distance = float(com_data_list[5]) / 1000, float(com_data_list[6]) / 1000
            if self.l_distance < 0.5:
                self.l_distance = None
            if self.r_distance < 0.5:
                self.r_distance = None
            # 判断距离是否已达
            target_distance = float(com_data_list[7])
            if target_distance <= config.arrive_distance:
                # 还没开始
                if self.current_ststus_index == -1:
                    pass
                # 到达终点
                elif self.current_ststus_index == len(self.plan_path):
                    pass
                # 正常到达一点开始下一点
                else:
                    self.plan_path_status[self.current_ststus_index] = 1

            self.logger.info({'ship_current_direction': self.ship_current_direction,
                              'TD': self.data_define_obj.water['TD'],
                              'water_temperature': self.data_define_obj.water['wt'],
                              'current_lng_lat': self.data_define_obj.status['current_lng_lat'],
                              'r_distance': self.r_distance,
                              'l_distance': self.l_distance,
                              'target_distance': target_distance})

    # 发送函数会阻塞 必须使用线程
    def send_com_data(self):
        # 0 自动  1手动
        manul_or_auto = 1
        try:
            while True:
                # 判断当前是手动控制还是自动控制
                if len(self.plan_path) == 0:
                    manul_or_auto = 1
                elif self.data_define_obj.status['current_lng_lat'] == None:
                    manul_or_auto = 1
                elif self.start:
                    for i in self.plan_path_points_status[self.server_data_obj.mqtt_send_get_obj.path_id]:
                        if i == 0:
                            manul_or_auto = 0
                            break
                        manul_or_auto = 1
                # 手动模式使用用户给定角度
                if manul_or_auto == 1:
                    d = int(self.server_data_obj.mqtt_send_get_obj.control_move_direction)
                    if d == 0:
                        temp_com_data = 1
                    elif d == 90:
                        temp_com_data = 3
                    elif d == 180:
                        temp_com_data = 2
                    elif d == 270:
                        temp_com_data = 4
                    else:
                        temp_com_data = 5
                    com_data_send = 'A5A5%d,0,0,0,0,0,0,0,0,0#' % temp_com_data
                    self.com_data_obj.send_data(com_data_send)
                    if not int(self.server_data_obj.mqtt_send_get_obj.control_move_direction) == 360:
                        self.logger.debug('com_data_send' + com_data_send)

                # 自动模式计算角度
                elif manul_or_auto == 0:
                    # 计算发送给单片机数据
                    for index, value in enumerate(self.plan_path_points_status[self.server_data_obj.mqtt_send_get_obj.path_id]):
                        if self.plan_path_status[index] == 1:
                            continue
                        self.current_index = index
                        ## 计算改目标点是否已达
                        # 无GPS信号
                        if self.data_define_obj.status['current_lng_lat'] is None:
                            continue
                        current_lng_lat = self.data_define_obj.status['current_lng_lat']
                        current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(current_lng_lat)

                        # 计算目标真实经纬度
                        target_lng_lat_gps = lng_lat_calculate.gps_gaode_to_gps(current_lng_lat,
                                                                                current_gaode_lng_lat,
                                                                                self.plan_path[index])
                        com_data_send = 'A5A50,0,%f,%f,0,0,0,0,0,0#' % (target_lng_lat_gps[0], target_lng_lat_gps[1])
                        self.com_data_obj.send_data(com_data_send)
                        self.logger.info('com_data_send: ' + str(com_data_send))
                        """
                        # 该点已达  判断小于5米为已经到达
                        if lng_lat_calculate.distanceFromCoordinate(current_gaode_lng_lat[0],
                                                                    current_gaode_lng_lat[1],
                                                                    self.plan_path[index][0],
                                                                    self.plan_path[index][1], ) < 5:
                            self.plan_path_status[index] = 1
                            continue

                        target_degree = lng_lat_calculate.angleFromCoordinate(
                            current_gaode_lng_lat[0],
                            current_gaode_lng_lat[1],
                            self.plan_path[index][0],
                            self.plan_path[index][1])
                        # 偏差角度
                        delta_degree = target_degree - self.ship_current_direction
                        auto_move_direction = 360
                        # 判断移动方向
                        if delta_degree >= 0:
                            if delta_degree < 45 or delta_degree >= 315:
                                auto_move_direction = 0
                            elif delta_degree >= 45 and delta_degree < 135:
                                auto_move_direction = 90
                            elif delta_degree >= 135 and delta_degree < 225:
                                auto_move_direction = 180
                            elif delta_degree >= 225 and delta_degree < 315:
                                auto_move_direction = 270
                        else:
                            if abs(delta_degree) < 45 or abs(delta_degree) >= 315:
                                auto_move_direction = 0
                            elif abs(delta_degree) >= 45 and abs(delta_degree) < 135:
                                auto_move_direction = 270
                            elif abs(delta_degree) >= 135 and abs(delta_degree) < 225:
                                auto_move_direction = 180
                            elif abs(delta_degree) >= 225 and abs(delta_degree) < 315:
                                auto_move_direction = 90
                        obstacle_direction = basic_obstacle_avoid.move(self.l_distance, self.r_distance)
                        if self.l_distance is not None and self.l_distance is not None:
                            # 超声波避障
                            obstacle_direction = basic_obstacle_avoid.move(self.l_distance, self.r_distance)
                            if obstacle_direction != int(self.server_data_obj.mqtt_send_get_obj.control_move_direction):
                                self.com_data_obj.send_data('A%sZ' % (obstacle_direction))
                            else:
                                self.com_data_obj.send_data(
                                    'A%sZ' % (self.server_data_obj.mqtt_send_get_obj.control_move_direction))
                            if not int(self.server_data_obj.mqtt_send_get_obj.control_move_direction) == 360:
                                self.logger.debug('control_move_direction: ' + str(
                                    self.server_data_obj.mqtt_send_get_obj.control_move_direction))
                                                if obstacle_direction != int(auto_move_direction):
                            self.com_data_obj.send_data('A%sZ' % (obstacle_direction))
                        else:
                            self.com_data_obj.send_data('A%sZ' % (str(auto_move_direction)))
                        self.logger.info('auto_move_direction: ' + str(auto_move_direction))
                        """

                time.sleep(1 / config.pi2com_interval)
        except KeyboardInterrupt:
            self.com_data_obj.send_data('A5A55,0,0,0,0,0,0,0,0,0#')

    # 读取函数会阻塞 必须使用线程
    # 发送mqtt状态数据和检测数据
    def send_mqtt_data(self):
        while True:
            status_data = self.data_define_obj.status
            status_data.update({'mapId': self.data_define_obj.pool_code})
            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})

            # 更新经纬度为高德经纬度
            if config.home_debug:
                status_data.update({'current_lng_lat': config.init_gaode_gps})
            else:
                current_lng_lat = status_data['current_lng_lat']
                if self.baidu_map_obj is not None and current_lng_lat is not None:
                    current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(current_lng_lat)
                    status_data.update({'current_lng_lat': current_gaode_lng_lat})

            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    # pop_value = detect_data[k_all].pop(old_key)
                    if detect_data[k_all].get(old_key) == None:
                        continue
                    detect_data[k_all].update({new_key: detect_data[k_all].pop(old_key)})

            # 更新模拟数据
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            mqtt_send_status_data = data_define.fake_status_data(status_data)

            # 向mqtt发送数据
            if self.data_define_obj.pool_code == '':
                self.send(method='mqtt', topic='status_data_%s' % (config.ship_code), data=mqtt_send_status_data,
                          qos=1)
                self.logger.info({"发送状态数据": mqtt_send_status_data})
            else:
                self.send(method='mqtt', topic='status_data_%s' % (config.ship_code), data=mqtt_send_status_data,
                          qos=1)
                self.send(method='mqtt', topic='detect_data_%s' % (config.ship_code), data=mqtt_send_detect_data,
                          qos=1)
                self.logger.debug({"发送检测数据": mqtt_send_detect_data})
                self.logger.debug({"发送状态数据": mqtt_send_status_data})
                # http发送检测数据给服务器
                self.send(method='http', data=mqtt_send_detect_data,
                          url=config.http_data_save,
                          http_type='POST')
            time.sleep(1 / config.pi2mqtt_interval)

    def send(self, method, data, topic='test', qos=0, http_type='POST', url=''):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type, data, url)
            self.logger.info({'请求 url': url})
            self.logger.info({'status_code': return_data.status_code})
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
                self.logger.info({'data/save content_data success': content_data["success"]})
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
            if self.data_define_obj.status['current_lng_lat'] == None:
                audios_manager.play_audio(5, b_backend=False)
                self.logger.error('当前GPS信号弱')
            if not check_network.check_network():
                audios_manager.play_audio(2, b_backend=False)
                self.logger.error('当前无网络信号')

            # 若是用户没有点击点
            if self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat is None or self.server_data_obj.mqtt_send_get_obj.pool_click_zoom is None:
                self.logger.info('没有点击湖，等待用户执行操作')
                continue

            self.logger.info({'init map user_lng_lat': self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat, 'zoom': self.server_data_obj.mqtt_send_get_obj.pool_click_zoom})
            # 查找与更新湖泊id
            save_img_dir = os.path.join(config.root_path, 'baiduMap', 'imgs')
            if not os.path.exists(save_img_dir):
                os.mkdir(save_img_dir)
            save_img_path = os.path.join(
                    save_img_dir, 'gaode_%f_%f_%i_%i.png' %
                                  (self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat[0],
                                   self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat[1],
                                   self.server_data_obj.mqtt_send_get_obj.pool_click_zoom,
                                   1))

            ## 创建于查找湖泊
            if len(self.data_define_obj.pool_code) <= 0 or not os.path.exists(save_img_path) :
                # 创建地图对象
                self.baidu_map_obj = baidu_map.BaiduMap(
                    lng_lat=self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat,
                    zoom=self.server_data_obj.mqtt_send_get_obj.pool_click_zoom,
                    logger=self.map_log)

                pool_cnts, (pool_cx, pool_cy) = self.baidu_map_obj.get_pool_pix()
                if pool_cnts is None or pool_cx == -2:
                    # 若返回为None表示没找到湖 定义错误代码
                    is_collision = 1
                    pool_info_data = {
                        'deviceId': config.ship_code,
                        'lng_lat': self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat,
                        'is_collision': is_collision,
                        'zoom': self.server_data_obj.mqtt_send_get_obj.pool_click_zoom,
                    }
                    self.send(method='mqtt', topic='pool_info_%s' % (config.ship_code), data=pool_info_data, qos=1)
                    self.logger.debug({'pool_info_data':pool_info_data})
                    continue
                # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用
                _, self.baidu_map_obj.pool_lng_lats = self.baidu_map_obj.pix_to_gps(pool_cnts)
                _, self.baidu_map_obj.pool_center_lng_lat = self.baidu_map_obj.pix_to_gps([[pool_cx, pool_cy]])
                self.logger.info({'pool_center_lng_lat': self.baidu_map_obj.pool_center_lng_lat})

                # 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
                send_data = {"longitudeLatitude": str(self.baidu_map_obj.pool_center_lng_lat),
                             "mapData": str(self.baidu_map_obj.pool_lng_lats),
                             "deviceId": config.ship_code,
                             "pixData": str(pool_cnts)}

                # 本地保存经纬度信息，放大1000000倍 用来只保存整数
                save_pool_lng_lats = [[int(i[0] * 1000000), int(i[1] * 1000000)] for i in
                                      self.baidu_map_obj.pool_lng_lats]

                if not os.path.exists(config.local_map_data_path):
                    # 发送请求获取湖泊ID
                    self.logger.debug({'send_data': send_data})
                    if config.home_debug:
                        pool_id='123123213'
                    else:
                        pool_id = self.send(
                            method='http', data=send_data, url=config.http_save, http_type='POST')

                    if isinstance(self.baidu_map_obj.pool_cnts, np.ndarray):
                        save_pool_cnts = self.baidu_map_obj.pool_cnts.tolist()
                    else:
                        save_pool_cnts = self.baidu_map_obj.pool_cnts
                    save_data = {"mapList": [{"id": pool_id,
                                              "pool_center_lng_lat": self.baidu_map_obj.pool_center_lng_lat,
                                              "pool_lng_lats": save_pool_lng_lats,
                                              "pool_cnts": save_pool_cnts}]}
                    self.logger.info({'pool_id': pool_id})
                    with open(config.local_map_data_path, 'w') as f:
                        json.dump(save_data, f)
                else:
                    with open(config.local_map_data_path, 'r') as f:
                        local_map_data = json.load(f)
                        # pool_id = baidu_map.is_in_contours(pool_gps_center, local_map_data)
                        pool_id = baidu_map.is_in_contours(
                            (self.baidu_map_obj.lng_lat[0], self.baidu_map_obj.lng_lat[1]), local_map_data)

                    if pool_id is not None:
                        self.logger.info({'在本地找到湖泊 poolid': pool_id})

                    # 不存在获取新的id
                    else:
                        if config.home_debug:
                            pool_id = '123123213'
                        else:
                            pool_id = self.send(
                                method='http', data=send_data, url=config.http_save, http_type='POST')

                        self.logger.info({'新的湖泊 poolid': pool_id})
                        with open(config.local_map_data_path, 'w') as f:
                            # 以前存储键值
                            # local_map_data["mapList"].append({"id": pool_id,
                            #                                   "longitudeLatitude": self.baidu_map_obj.pool_center_lng_lat,
                            #                                   "mapData": self.baidu_map_obj.pool_lng_lat,
                            #                                   "pool_cnt": pool_cnts.tolist()})
                            if isinstance(self.baidu_map_obj.pool_cnts, np.ndarray):
                                save_pool_cnts = self.baidu_map_obj.pool_cnts.tolist()
                            local_map_data["mapList"].append({"id": pool_id,
                                                              "pool_center_lng_lat": self.baidu_map_obj.pool_center_lng_lat,
                                                              "pool_lng_lats": save_pool_lng_lats,
                                                              "pool_cnts": save_pool_cnts})
                            json.dump(local_map_data, f)

                self.data_define_obj.pool_code = pool_id

                pool_info_data = {
                    'deviceId': config.ship_code,
                    'mapId': self.data_define_obj.pool_code,
                    'lng_lat': self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat,
                    'pool_lng_lats':self.baidu_map_obj.pool_lng_lats
                }
                # pool_info_data = json.dumps(pool_info_data)
                self.send(method='mqtt', topic='pool_info_%s' % (config.ship_code), data=pool_info_data, qos=2)
                self.logger.debug({'pool_info_data':pool_info_data})

            ## 配置判断
            len_target_lng_lat = len(self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
            self.logger.debug({'target_lng_lat':self.server_data_obj.mqtt_send_get_obj.target_lng_lat})

            if len_target_lng_lat<=0:
                pass
            # 等待确认就不执行检测
            elif self.plan_path_status.count(0)>0:
                pass
            else:
                # 单点航行模式
                if len_target_lng_lat==1:
                    target_lng_lats = copy.deepcopy(self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                    # 是否返航
                    if int(self.server_data_obj.mqtt_send_get_obj.back_home)==1:
                        if config.home_debug:
                            target_lng_lats.append(config.init_gaode_gps)
                        else:
                            current_lng_lat = self.data_define_obj.status['current_lng_lat']
                            if self.baidu_map_obj is not None and current_lng_lat is not None:
                                current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(current_lng_lat)
                                target_lng_lats.append(current_gaode_lng_lat)
                    self.logger.debug({'path_planning': target_lng_lats})

                    self.path_planning(mode=0, target_lng_lats=target_lng_lats)

                # 多点巡航模式
                elif len_target_lng_lat > 1 :
                    target_lng_lats = copy.deepcopy(self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                    # 是否返航
                    if int(self.server_data_obj.mqtt_send_get_obj.back_home) == 1:
                        if config.home_debug:
                            target_lng_lats.append(config.init_gaode_gps)
                        else:
                            current_lng_lat = self.data_define_obj.status['current_lng_lat']
                            if self.baidu_map_obj is not None and current_lng_lat is not None:
                                current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(current_lng_lat)
                                target_lng_lats.append(current_gaode_lng_lat)
                    self.path_planning(mode=1, target_lng_lats=target_lng_lats)

                # 自动搜索模式
                if self.server_data_obj.mqtt_send_get_obj.row_gap is not None and self.server_data_obj.mqtt_send_get_obj.col_gap is not None and self.server_data_obj.mqtt_send_get_obj.safe_gap is not None:
                    # 根据用户指定米数距离计算像素距离
                    meter_pix = self.server_data_obj.mqtt_send_get_obj.meter_pix.get(self.server_data_obj.mqtt_send_get_obj.zoom[0])
                    row_pix_gap =  int(self.server_data_obj.mqtt_send_get_obj.self.row_gap/meter_pix)
                    col_pix_gap = int(self.server_data_obj.mqtt_send_get_obj.self.col_gap/meter_pix)
                    safe_pix_gap = self.server_data_obj.mqtt_send_get_obj.self.safe_gap
                    self.baidu_map_obj.scan_pool(self.baidu_map_obj.pool_cnts, pix_gap=row_pix_gap,safe_distance=safe_pix_gap)
                    self.path_planning(mode=2)

                ## TODO 巡逻绕湖模式
                elif self.server_data_obj.mqtt_send_get_obj.round_pool_gap is not None:
                    pass
                        # self.baidu_map_obj.scan_pool(self.baidu_map_obj.pool_cnts, pix_gap=row_pix_gap,safe_distance=safe_pix_gap)

            ## 检查确认航线
            if self.server_data_obj.mqtt_send_get_obj.path_id is not None and self.server_data_obj.mqtt_send_get_obj.confirm_index is not None:
                try:
                    self.plan_path_status[self.server_data_obj.mqtt_send_get_obj.path_id]=1
                    # 确认航线就启动
                    if self.server_data_obj.mqtt_send_get_obj.confirm_index == 1:
                        self.start = True
                except:
                    self.logger.error({'非法的航线确认ID':self.server_data_obj.mqtt_send_get_obj.path_id})

            # 获取初始地点GPS
            if not self.start:
                pass
            else:
                if config.home_debug:
                    self.baidu_map_obj.init_ship_gps = config.init_gaode_gps
                    self.baidu_map_obj.init_ship_gaode_lng_lat = config.init_gaode_gps
                else:
                    self.baidu_map_obj.init_ship_gps = self.data_define_obj.status['current_lng_lat']

    # 路径规划
    def path_planning(self, target_lng_lats=None, mode=5,back_home=False):
        """
        :param mode
        return path points
        """
        if config.home_debug:
            self.baidu_map_obj.ship_pix = self.baidu_map_obj.gaode_lng_lat_to_pix(config.init_gaode_gps)
            print('self.baidu_map_obj.ship_pix',self.baidu_map_obj.ship_pix)
            self.baidu_map_obj.ship_gps = config.init_gaode_gps
            self.baidu_map_obj.init_ship_gps = config.init_gaode_gps
            self.baidu_map_obj.init_ship_gaode_lng_lat = config.init_gaode_gps
        else:
            self.baidu_map_obj.ship_gps = self.data_define_obj.status['current_lng_lat']
        self.logger.debug({'path_planning mode':mode})
        return_gaode_lng_lat_path = a_star.get_path(baidu_map_obj=self.baidu_map_obj,
                                                    mode=mode,
                                                    target_lng_lats=target_lng_lats,
                                                    b_show=False,
                                                    back_home=back_home,
                                                    map_connect=1)
        print('return_gaode_lng_lat_path',return_gaode_lng_lat_path)
        if isinstance(return_gaode_lng_lat_path, str):
            self.logger.error(return_gaode_lng_lat_path)
            return
        path_id = len(self.plan_path)
        # 路径点
        self.plan_path.append(return_gaode_lng_lat_path)
        # 路径点状态
        self.plan_path_points_status.append([0] * len(self.plan_path))
        # 路径确认与取消状态
        self.plan_path_status.append(0)

        mqtt_send_path_planning_data = {
            "deviceId": config.ship_code,
            "mapId": self.data_define_obj.pool_code,
            "sampling_points": target_lng_lats,
            "path_points": self.plan_path,
            "path_id": path_id
        }

        self.send(method='mqtt', topic='path_planning_%s' % (config.ship_code), data=mqtt_send_path_planning_data,
                  qos=2)
        self.logger.debug({'mqtt_send_path_planning_data':mqtt_send_path_planning_data})



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

    while True:
        move_direction = obj.server_data_obj.mqtt_send_get_obj.control_move_direction
        obj.send(method='com', data=move_direction)
        obj.logger.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位
