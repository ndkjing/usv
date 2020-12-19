"""
管理数据收发
"""
import time
import json
import numpy as np
import os

from dataGetSend.data_define import DataDefine
from dataGetSend import data_define
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData
from utils import check_network
from utils.log import LogHandler
from baiduMap import baidu_map
from audios import audios_manager
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

        self.baidu_map_obj = None
        # 船最终控制移动方向
        self.ship_move_direction = str(360)
        # 船当前朝向
        self.ship_current_direction = -1

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        while True:
            com_data_read = self.com_data_obj.readline()
            ## 解析串口发送过来的数据
            if com_data_read is None:
                continue
            com_data_list = com_data_read.split(',')
            # 角度，TDS，温度，经度，纬度，距离1，距离2
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
            # 左右侧的超声波检测距离
            l_distance, r_distance = com_data_list[5], com_data_list[6]
            self.logger.info({'ship_current_direction': self.ship_current_direction,
                              'TD': self.data_define_obj.water['TD'],
                              'water_temperature': self.data_define_obj.water['wt'],
                              'current_lng_lat': self.data_define_obj.status['current_lng_lat'],
                              'r_distance': r_distance,
                              'l_distance': l_distance})

    # 发送函数会阻塞 必须使用线程
    def send_com_data(self):
        # 0 自动  1手动
        manul_or_auto = 0
        while True:
            # 判断当前是手动控制还是自动控制
            if len(self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status) == 0:
                manul_or_auto = 1
            elif self.data_define_obj.status['current_lng_lat'] == None:
                manul_or_auto = 1
            else:
                for i in self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status:
                    if i == 0:
                        manul_or_auto = 0
                        break
                    manul_or_auto = 1
            # 手动模式使用用户给定角度
            if manul_or_auto == 1:
                self.com_data_obj.send_data('A%sZ' % (self.server_data_obj.mqtt_send_get_obj.control_move_direction))
                # self.logger.debug('control_move_direction: '+str(self.server_data_obj.mqtt_send_get_obj.control_move_direction))
            # 自动模式计算角度
            elif manul_or_auto == 0:
                # 计算目标角度
                target_degree = baidu_map.get_degree(self.data_define_obj.status['current_lng_lat'][0],
                                                     self.data_define_obj.status['current_lng_lat'][1],
                                                     self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status[-1][
                                                         0],
                                                     self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status[-1][
                                                         1])
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
                self.com_data_obj.send_data('A%sZ' % (str(auto_move_direction)))
                self.logger.info('auto_move_direction: ' + str(auto_move_direction))

            time.sleep(1 / config.pi2com_interval)

    # 读取函数会阻塞 必须使用线程
    # 发送mqtt状态数据和检测数据
    def send_mqtt_data(self):
        while True:
            status_data = self.data_define_obj.status
            status_data.update({'mapId': self.data_define_obj.pool_code})
            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    # pop_value = detect_data[k_all].pop(old_key)
                    if detect_data[k_all].get(old_key) == None:
                        continue
                    detect_data[k_all].update({new_key: detect_data[k_all].pop(old_key)})

            # 向mqtt发送数据
            if self.data_define_obj.pool_code == '':
                self.send(method='mqtt', topic='status_data_%s' % (self.data_define_obj.ship_code), data=status_data,
                          qos=1)
                self.logger.info({"发送状态数据": status_data})
            else:
                self.send(method='mqtt', topic='status_data_%s' % (self.data_define_obj.ship_code), data=status_data,
                          qos=1)
                self.send(method='mqtt', topic='detect_data_%s' % (self.data_define_obj.ship_code), data=detect_data,
                          qos=1)
                self.logger.info({"发送检测数据": detect_data})
                self.logger.info({"发送状态数据": status_data})
                # http发送检测数据给服务器
                self.send(method='http', data=detect_data,
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
            # 检查当前状态
            if self.data_define_obj.status['current_lng_lat'] == None:
                audios_manager.play_audio('gps.mp3',b_backend=False)
                self.logger.error('当前GPS信号弱')
            if not check_network.check_network():
                audios_manager.play_audio('network.mp3', b_backend=False)
                self.logger.error('当前无网络信号')

            # 创建地图对象
            assert config.mod in ['manual', 'auto']
            if config.mod == 'auto':
                # 获取话题user_lng_lat数据
                # with open(config.usr_lng_lat_path,'r') as f:
                #     user_lng_lat = json.load(f)
                if len(self.server_data_obj.mqtt_send_get_obj.target_lng_lat) <= 0:
                    self.logger.info('没有点击湖，等待用户执行操作')
                    continue
                else:
                    # 检查目标经纬度和状态
                    user_lng_lat = -1
                    zoom = -1
                    for index_i, status_list in enumerate(self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status):
                        for index_j, status in enumerate(status_list):
                            if int(status) == 0:
                                user_lng_lat = self.server_data_obj.mqtt_send_get_obj.target_lng_lat[index_i][index_j]
                                zoom = self.server_data_obj.mqtt_send_get_obj.target_lng_lat.zoom[index_i]
                                self.server_data_obj.mqtt_send_get_obj.current_lng_lat_index = [index_i, index_j]
                                break
                        break
                    if user_lng_lat == -1 or zoom == -1:
                        self.logger.error('user_lng_lat== -1 or zoom == -1')
                        continue
                self.baidu_map_obj = baidu_map.BaiduMap(lng_lat=user_lng_lat, zoom=zoom, logger=self.map_log)
            else:
                user_lng_lat = [116.99868, 40.511224]
                zoom = 12
                # baidu_map_obj = BaiduMap([114.393142, 31.558981], zoom=14)
                # baidu_map_obj = BaiduMap([114.710639,30.656827], zoom=13)
                # baidu_map_obj = BaiduMap([117.574294,31.539694], zoom=11)
                self.baidu_map_obj = baidu_map.BaiduMap(lng_lat=user_lng_lat, zoom=zoom, logger=self.map_log)

            pool_cnts, (pool_cx, pool_cy) = self.baidu_map_obj.get_pool_pix()
            if pool_cnts is None:
                # 若返回为None表示没找到湖 定义错误代码
                is_collision = [1]
                index_i, index_j = self.server_data_obj.mqtt_send_get_obj.current_lng_lat_index
                data = {
                       'deviceId': config.ship_code,
                       'lng_lat': self.server_data_obj.mqtt_send_get_obj.target_lng_lat[index_i],
                       'is_collision': is_collision,
                       'zoom':self.server_data_obj.mqtt_send_get_obj.zoom[index_i],
                        'mode':self.server_data_obj.mqtt_send_get_obj.mode[index_i]
                }
                self.send(method='mqtt',topic='pool_info_%s' % (config.ship_code),data=data, qos = 1)
                continue
            pool_cnts = np.squeeze(pool_cnts)

            # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用
            _, pool_gps_points = self.baidu_map_obj.pix_to_gps(pool_cnts)
            _, pool_gps_center = self.baidu_map_obj.pix_to_gps([[pool_cx, pool_cy]])
            self.logger.info({'pool_gps_center': pool_gps_center})
            self.baidu_map_obj.pool_cnts = pool_cnts
            self.baidu_map_obj.pool_lng_lat = pool_gps_points
            self.baidu_map_obj.center_lng_lat = pool_gps_center

            # 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
            send_data = {"longitudeLatitude": str(pool_gps_center),
                         "mapData": str(pool_gps_points),
                         "deviceId": config.ship_code,
                         "pixData": str(pool_cnts)}
            if not os.path.exists(config.local_map_data_path):
                with open(config.local_map_data_path, 'w') as f:
                    # 发送请求获取湖泊ID
                    pool_id = self.send(
                        method='http', data=send_data, url=config.http_save, http_type='POST')
                    save_data = {"mapList": [{"id": pool_id,
                                              "longitudeLatitude": pool_gps_center,
                                              "mapData": pool_gps_points,
                                              "pool_cnt": pool_cnts.tolist()}]}
                    self.logger.info({'pool_id': pool_id})
                    json.dump(save_data, f)
            else:
                with open(config.local_map_data_path, 'r') as f:
                    local_map_data = json.load(f)

                    # TODO 判断是否在曾经出现的湖泊中
                    # pool_id = baidu_map.is_in_contours(pool_gps_center, local_map_data)
                    pool_id = baidu_map.is_in_contours((pool_cx, pool_cy), local_map_data)

                if pool_id is not None:
                    self.logger.info({'在本地找到湖泊 poolid': pool_id})
                # 不存在获取新的id
                else:
                    pool_id = self.send(
                        method='http', data=send_data, url=config.http_save, http_type='POST')
                    self.logger.info({'新的湖泊 poolid': pool_id})
                    with open(config.local_map_data_path, 'w') as f:
                        local_map_data["mapList"].append({"id": pool_id,
                                                          "longitudeLatitude": str(pool_gps_center),
                                                          "mapData": str(pool_gps_points),
                                                          "pool_cnt": pool_cnts.tolist()})
                        json.dump(local_map_data, f)
            self.data_define_obj.pool_code = pool_id
            time.sleep(config.check_status_interval)


if __name__ == '__main__':
    obj = DataManager()

    while True:
        move_direction = obj.server_data_obj.mqtt_send_get_obj.control_move_direction
        obj.send(method='com', data=move_direction)
        obj.logger.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位
