"""
寻找地图上湖泊，路径规划
"""

import threading
import time
import json
import numpy as np
import os
import sys
import copy

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
sys.path.append(
    os.path.join(
        root_path,
        'baiduMap'))
sys.path.append(
    os.path.join(
        root_path,
        'dataGetSend'))
sys.path.append(
    os.path.join(
        root_path,
        'utils'))
sys.path.append(
    os.path.join(
        root_path,
        'pathPlanning'))
import config
from utils import lng_lat_calculate
from pathPlanning import a_star
from baiduMap import baidu_map
from utils.log import LogHandler
from dataGetSend.server_data import ServerData
from dataGetSend import data_define
from dataGetSend.data_define import DataDefine

class WebServer:
    def __init__(self):
        self.data_define_obj = DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log', level=20)
        self.server_log = LogHandler('server_data')
        self.map_log = LogHandler('map_log')
        self.drone_log = LogHandler('drone_log')
        # mqtt服务器数据收发对象
        self.server_data_obj = ServerData(
            self.server_log, topics=self.data_define_obj.topics)

        # 记录目标点击地点
        self.current_target_gaode_lng_lats=None
        # 记录路径规划地点
        self.plan_path = None

    def send(
            self,
            method,
            data,
            topic='test',
            qos=0,
            http_type='POST',
            url=''):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt',
                          'com'], 'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(
                http_type, data, url)
            self.logger.info({'请求 url': url})
            self.logger.info({'status_code': return_data.status_code})
            # 如果是POST返回的数据，添加数据到地图数据保存文件中
            if http_type == 'POST' and r'map/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.info(
                    {'map/save content_data success': content_data["success"]})
                if not content_data["success"]:
                    self.logger.error('POST请求发送地图数据失败')
                # POST 返回湖泊ID
                pool_id = content_data['data']['id']
                return pool_id
            # http发送检测数据给服务器
            elif http_type == 'POST' and r'data/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.info(
                    {'data/save content_data success': content_data["success"]})
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
            self.server_data_obj.send_server_mqtt_data(
                data=data, topic=topic, qos=qos)

    # 状态检查函数，检查自身状态发送对应消息
    def find_pool(self):
        while True:
            # 循环等待一定时间
            time.sleep(config.check_status_interval)
            # 若是用户没有点击点
            if self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat is None or self.server_data_obj.mqtt_send_get_obj.pool_click_zoom is None:
                self.logger.info('没有点击湖，等待用户执行操作')
                continue

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

            # 创建于查找湖泊
            if len(self.data_define_obj.pool_code) <= 0 or not os.path.exists(
                    save_img_path):
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
                    self.send(
                        method='mqtt',
                        topic='pool_info_%s' %
                        (config.ship_code),
                        data=pool_info_data,
                        qos=1)
                    self.logger.debug({'pool_info_data': pool_info_data})
                    continue
                # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用
                _, self.baidu_map_obj.pool_lng_lats = self.baidu_map_obj.pix_to_gps(
                    pool_cnts)
                _, self.baidu_map_obj.pool_center_lng_lat = self.baidu_map_obj.pix_to_gps([
                                                                                          [pool_cx, pool_cy]])
                self.logger.info(
                    {'pool_center_lng_lat': self.baidu_map_obj.pool_center_lng_lat})

                # 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
                if isinstance(self.baidu_map_obj.pool_cnts, np.ndarray):
                    save_pool_cnts = self.baidu_map_obj.pool_cnts.tolist()
                else:
                    save_pool_cnts = self.baidu_map_obj.pool_cnts
                send_data = {
                    "longitudeLatitude": json.dumps(
                        self.baidu_map_obj.pool_center_lng_lat),
                    "mapData": json.dumps(
                        self.baidu_map_obj.pool_lng_lats),
                    "deviceId": config.ship_code,
                    "pixData": json.dumps(save_pool_cnts)}

                # 本地保存经纬度信息，放大1000000倍 用来只保存整数
                save_pool_lng_lats = [[int(i[0] * 1000000), int(i[1] * 1000000)]
                                      for i in self.baidu_map_obj.pool_lng_lats]

                if not os.path.exists(config.local_map_data_path):
                    # 发送请求获取湖泊ID
                    self.logger.debug({'send_data': send_data})
                    if config.home_debug:
                        pool_id = '123123213'
                    else:
                        pool_id = self.send(
                            method='http',
                            data=send_data,
                            url=config.http_save,
                            http_type='POST')

                    if isinstance(self.baidu_map_obj.pool_cnts, np.ndarray):
                        save_pool_cnts = self.baidu_map_obj.pool_cnts.tolist()
                    else:
                        save_pool_cnts = self.baidu_map_obj.pool_cnts
                    save_data = {
                        "mapList": [
                            {
                                "id": pool_id,
                                "pool_center_lng_lat": self.baidu_map_obj.pool_center_lng_lat,
                                "pool_lng_lats": save_pool_lng_lats,
                                "pool_cnts": save_pool_cnts}]}
                    self.logger.info({'pool_id': pool_id})
                    with open(config.local_map_data_path, 'w') as f:
                        json.dump(save_data, f)
                else:
                    with open(config.local_map_data_path, 'r') as f:
                        local_map_data = json.load(f)
                        pool_id = baidu_map.is_in_contours(
                            (self.baidu_map_obj.lng_lat[0] * 1000000,
                             self.baidu_map_obj.lng_lat[1] * 1000000),
                            local_map_data)

                    if pool_id is not None:
                        self.logger.info({'在本地找到湖泊 poolid': pool_id})

                    # 不存在获取新的id
                    else:
                        if config.home_debug:
                            pool_id = '123123213'
                        else:
                            pool_id = self.send(
                                method='http',
                                data=send_data,
                                url=config.http_save,
                                http_type='POST')

                        self.logger.info({'新的湖泊 poolid': pool_id})
                        with open(config.local_map_data_path, 'w') as f:
                            # 以前存储键值
                            # local_map_data["mapList"].append({"id": pool_id,
                            #                                   "longitudeLatitude": self.baidu_map_obj.pool_center_lng_lat,
                            #                                   "mapData": self.baidu_map_obj.pool_lng_lat,
                            #                                   "pool_cnt": pool_cnts.tolist()})
                            if isinstance(
                                    self.baidu_map_obj.pool_cnts, np.ndarray):
                                save_pool_cnts = self.baidu_map_obj.pool_cnts.tolist()
                            local_map_data["mapList"].append(
                                {
                                    "id": pool_id,
                                    "pool_center_lng_lat": self.baidu_map_obj.pool_center_lng_lat,
                                    "pool_lng_lats": save_pool_lng_lats,
                                    "pool_cnts": save_pool_cnts})
                            json.dump(local_map_data, f)

                self.data_define_obj.pool_code = pool_id

                pool_info_data = {
                    'deviceId': config.ship_code,
                    'mapId': self.data_define_obj.pool_code,
                    'lng_lat': self.server_data_obj.mqtt_send_get_obj.pool_click_lng_lat,
                    # 'pool_lng_lats':self.baidu_map_obj.pool_lng_lats
                }
                self.send(
                    method='mqtt',
                    topic='pool_info_%s' %
                    (config.ship_code),
                    data=pool_info_data,
                    qos=2)
                self.logger.info({'pool_info_data': pool_info_data})

    def get_plan_path(self):
        while True:
            time.sleep(0.5)
            # 配置判断
            len_target_lng_lat = len(
                self.server_data_obj.mqtt_send_get_obj.target_lng_lat)

            if len_target_lng_lat <= 0:
                pass
            else:
                # 单点航行模式
                if len_target_lng_lat == 1:
                    target_lng_lats = copy.deepcopy(
                        self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                    # 是否返航
                    if int(
                            self.server_data_obj.mqtt_send_get_obj.back_home) == 1:
                        if config.home_debug:
                            target_lng_lats.append(config.init_gaode_gps)
                        else:
                            # 添加当前经纬度作为返航点
                            current_lng_lat = self.data_define_obj.status['current_lng_lat']
                            if self.baidu_map_obj is not None and current_lng_lat is not None:
                                current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(
                                    current_lng_lat)
                                target_lng_lats.append(current_gaode_lng_lat)

                    # 判断是否是上次的点
                    if self.current_target_gaode_lng_lats is not None and self.current_target_gaode_lng_lats == self.server_data_obj.mqtt_send_get_obj.target_lng_lat:
                        pass
                    else:
                        self.logger.info(
                            {'target_lng_lat': self.server_data_obj.mqtt_send_get_obj.target_lng_lat})
                        self.current_target_gaode_lng_lats = copy.deepcopy(
                            self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                        self.path_planning(
                            mode=0, target_lng_lats=self.current_target_gaode_lng_lats)

                # 多点巡航模式
                elif len_target_lng_lat > 1:
                    if self.current_target_gaode_lng_lats is not None and self.current_target_gaode_lng_lats == self.server_data_obj.mqtt_send_get_obj.target_lng_lat:
                        pass
                    else:
                        target_lng_lats = copy.deepcopy(
                            self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                        # 是否返航
                        if int(
                                self.server_data_obj.mqtt_send_get_obj.back_home) == 1:
                            if config.home_debug:
                                target_lng_lats.append(config.init_gaode_gps)
                            else:
                                current_lng_lat = self.data_define_obj.status['current_lng_lat']
                                if self.baidu_map_obj is not None and current_lng_lat is not None:
                                    current_gaode_lng_lat = self.baidu_map_obj.gps_to_gaode_lng_lat(
                                        current_lng_lat)
                                    target_lng_lats.append(current_gaode_lng_lat)
                        self.current_target_gaode_lng_lats = copy.deepcopy(
                            self.server_data_obj.mqtt_send_get_obj.target_lng_lat)
                        self.path_planning(mode=1, target_lng_lats=self.current_target_gaode_lng_lats)
                # 自动搜索模式
                if self.server_data_obj.mqtt_send_get_obj.row_gap is not None and self.server_data_obj.mqtt_send_get_obj.col_gap is not None and self.server_data_obj.mqtt_send_get_obj.safe_gap is not None:
                    # 根据用户指定米数距离计算像素距离
                    meter_pix = self.server_data_obj.mqtt_send_get_obj.meter_pix.get(
                        self.server_data_obj.mqtt_send_get_obj.zoom[0])
                    row_pix_gap = int(
                        self.server_data_obj.mqtt_send_get_obj.self.row_gap / meter_pix)
                    col_pix_gap = int(
                        self.server_data_obj.mqtt_send_get_obj.self.col_gap / meter_pix)
                    safe_pix_gap = self.server_data_obj.mqtt_send_get_obj.self.safe_gap
                    self.baidu_map_obj.scan_pool(
                        self.baidu_map_obj.pool_cnts,
                        pix_gap=row_pix_gap,
                        safe_distance=safe_pix_gap)
                    self.path_planning(mode=2)

                # TODO 巡逻绕湖模式
                elif self.server_data_obj.mqtt_send_get_obj.round_pool_gap is not None:
                    pass
                    # self.baidu_map_obj.scan_pool(self.baidu_map_obj.pool_cnts, pix_gap=row_pix_gap,safe_distance=safe_pix_gap)
            # 检查确认航线
            # if self.server_data_obj.mqtt_send_get_obj.path_id is not None and self.server_data_obj.mqtt_send_get_obj.confirm_index is not None:
            #     try:
            #         self.plan_path_status[self.server_data_obj.mqtt_send_get_obj.path_id]=1
            #         # 确认航线就启动
            #         if self.server_data_obj.mqtt_send_get_obj.confirm_index == 1:
            #             self.start = True
            #     except:
            #         self.logger.error({'非法的航线确认ID':self.server_data_obj.mqtt_send_get_obj.path_id})

            # 获取初始地点GPS
            # if not self.start:
            #     pass
            # else:
            #     if config.home_debug:
            #         self.baidu_map_obj.init_ship_gps = config.init_gaode_gps
            #         self.baidu_map_obj.init_ship_gaode_lng_lat = config.init_gaode_gps
            #     else:
            #         self.baidu_map_obj.init_ship_gps = self.data_define_obj.status['current_lng_lat']

    # 路径规划
    def path_planning(self, target_lng_lats=None, mode=5, back_home=False):
        """
        :param mode
        return path points
        """
        if config.home_debug:
            self.baidu_map_obj.ship_pix = self.baidu_map_obj.gaode_lng_lat_to_pix(
                config.init_gaode_gps)
            print('self.baidu_map_obj.ship_pix', self.baidu_map_obj.ship_pix)
            self.baidu_map_obj.ship_gps = config.init_gaode_gps
            self.baidu_map_obj.init_ship_gps = config.init_gaode_gps
            self.baidu_map_obj.init_ship_gaode_lng_lat = config.init_gaode_gps
        else:
            # 在服务器上运行不考虑船的位置
            self.baidu_map_obj.ship_gps = config.init_gaode_gps
            self.baidu_map_obj.ship_gaode_lng_lat = config.init_gaode_gps
        self.logger.debug({'path_planning mode': mode})
        # 进行路径规划
        if config.b_use_path_planning:
            return_gaode_lng_lat_path = a_star.get_path(
                baidu_map_obj=self.baidu_map_obj,
                mode=mode,
                target_lng_lats=target_lng_lats,
                b_show=False,
                back_home=back_home)
            self.logger.info(
                {'return_gaode_lng_lat_path': return_gaode_lng_lat_path})
            if isinstance(return_gaode_lng_lat_path, str):
                self.logger.error(return_gaode_lng_lat_path)
                return
            # 路径点
            self.plan_path = return_gaode_lng_lat_path
            # 路径点状态
            self.plan_path_points_status = [0] * len(self.plan_path)
            # 路径确认与取消状态
            # self.plan_path_status = 0
            # self.start = True
            # self.b_manul = 0

            mqtt_send_path_planning_data = {
                "deviceId": config.ship_code,
                "mapId": self.data_define_obj.pool_code,
                "sampling_points": target_lng_lats,
                "path_points": self.plan_path,
                "path_id": len(self.plan_path)
            }
        # 不进行路径规划直接到达
        else:
            self.plan_path = copy.deepcopy(target_lng_lats)
            mqtt_send_path_planning_data = {
                "deviceId": config.ship_code,
                "mapId": self.data_define_obj.pool_code,
                "sampling_points": target_lng_lats,
                "path_points": self.plan_path,
                "path_id": len(target_lng_lats)
            }
        self.send(
            method='mqtt',
            topic='path_planning_%s' %
            (config.ship_code),
            data=mqtt_send_path_planning_data,
            qos=2)
        self.logger.info(
            {'mqtt_send_path_planning_data': mqtt_send_path_planning_data})

if __name__ == '__main__':
    web_server_obj = WebServer()
    find_pool_thread = threading.Thread(target= web_server_obj.find_pool)
    get_plan_path_thread = threading.Thread(target= web_server_obj.get_plan_path)
    find_pool_thread.start()
    get_plan_path_thread.start()
    while True:
        time.sleep(1)

    # find_pool_thread.start()
    # find_pool_thread.start()

