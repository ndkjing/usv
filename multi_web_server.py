"""
寻找地图上湖泊，路径规划 检查在线船只和其他需要再服务器上运行的功能
"""
import threading
import time
import json
import numpy as np
import os
import copy

from moveControl.pathPlanning import a_star
import server_baidu_map as baidu_map
from utils.log import LogHandler
import ship_state_utils
from web_server_data import ServerData
import server_data_define
import server_config
import draw_img
import config
import delete_mapid


class WebServer:
    def __init__(self):
        self.data_define_obj_dict = {}  # 每个船的数据定义类
        for ship_code in config.ship_code_type_dict.keys():
            self.data_define_obj_dict.update({ship_code: server_data_define.DataDefine(ship_code=ship_code)})
        self.baidu_map_obj_dict = {}
        # 日志对象
        self.logger = LogHandler('web_server', level=20)
        self.server_log = LogHandler('server_data')
        self.map_log = LogHandler('map_log')
        # mqtt服务器数据收发对象
        self.server_data_obj_dict = {}
        for ship_code in config.ship_code_type_dict.keys():
            self.server_data_obj_dict.update({ship_code: ServerData(self.server_log,
                                                                    topics=self.data_define_obj_dict.get(
                                                                        ship_code).topics,
                                                                    ship_code=ship_code)})
        # 记录目标点击地点
        self.current_target_gaode_lng_lats_dict = {}
        # 记录路径规划地点
        self.plan_path = None
        self.current_map_type = baidu_map.MapType.gaode  # 当前地图类型

    def send(
            self,
            method,
            data,
            ship_code,
            topic='test',
            qos=0,
            http_type='POST',
            url='',
            token=None):
        """
        获取数据方式　http mqtt com
        :param ship_code:
        :param url:
        :param http_type:
        :param qos:
        :param data:
        :param topic:
        :param method
        :param token
        """
        assert method in ['http', 'mqtt',
                          'com'], 'method error not in http mqtt com'
        if method == 'http':
            print('####################token', token)
            return_data = self.server_data_obj_dict.get(ship_code).send_server_http_data(
                http_type, data, url, token)
            self.logger.info({'请求 url': url})
            self.logger.info({'status_code': return_data.status_code})
            # 如果是POST返回的数据，添加数据到地图数据保存文件中
            if http_type == 'POST' and r'map/save' in url:
                content_data = json.loads(return_data.content)
                print('请求湖泊轮廓id数据', content_data)
                pool_id = content_data.get('data')
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
            elif http_type == 'POST' and r'upData' in url:
                content_data = json.loads(return_data.content)
                if content_data.get("success") and content_data.get("code") not in [200, 20000]:
                    self.logger.error('GET请求失败')
                    return False
                else:
                    return True
            elif http_type == 'POST' and r'sampling/save' in url:
                content_data = json.loads(return_data.content)
                if content_data.get('code') != 200 and content_data.get('code') != 20000:
                    self.logger.error('GET请求失败')
                    return False
                else:
                    return True
            elif http_type == 'POST' and r'sampling/save' in url:
                content_data = json.loads(return_data.content)
                if content_data.get('code') != 200 and content_data.get('code') != 20000:
                    self.logger.error('GET请求失败')
                    return False
                else:
                    return True
            elif http_type == 'POST' and r'sampling/save' in url:
                content_data = json.loads(return_data.content)
                if content_data.get('code') != 200 and content_data.get('code') != 20000:
                    self.logger.error('GET请求失败')
                    return False
                else:
                    return True
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if content_data.get("success") and content_data.get("code") not in [200, 20000]:
                    self.logger.error('请求失败')
                    return False
                else:
                    return True
        elif method == 'mqtt':
            self.server_data_obj_dict.get(ship_code).send_server_mqtt_data(
                data=data, topic=topic, qos=qos)

    # 状态检查函数，检查自身状态发送对应消息
    def find_pool(self):
        pre_click_lng_lat = None
        while True:
            # 循环等待一定时间
            time.sleep(0.1)
            save_img_dir1 = os.path.join(server_config.root_path, 'statics')  # 计算存储图片路径和判断路径是否存在
            if not os.path.exists(save_img_dir1):
                os.mkdir(save_img_dir1)
            save_img_dir = os.path.join(server_config.root_path, 'statics', 'imgs')  # 计算存储图片路径和判断路径是否存在
            if not os.path.exists(save_img_dir):
                os.mkdir(save_img_dir)
            save_img_name_list = os.listdir(save_img_dir)  # 超过1000张图片时候删除图片
            if len(save_img_name_list) > 1000:
                for i in save_img_name_list:
                    os.remove(os.path.join(save_img_dir, i))
            for ship_code in server_config.ship_code_list:
                # 删除湖泊
                try:
                    if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_mapId:
                        delete_mapid.delete_map(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_deviceId,
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_mapId)
                        delete_pool_data = {
                            # 设备号
                            "deviceId": self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_deviceId,
                            # 湖泊编号
                            "mapId": self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_mapId,
                            # 算法那边是否删除完成时候返回 1:删除完成
                            "a_delete_status": 1
                        }
                        self.send(method='mqtt',
                                  topic='delete_pool_%s' % self.server_data_obj_dict.get(
                                      ship_code).mqtt_send_get_obj.delete_pool_deviceId,
                                  ship_code=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_deviceId,
                                  data=delete_pool_data,
                                  qos=0)
                        print("删除湖泊发送mqtt消息:", delete_pool_data)
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_mapId = ""
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.delete_pool_deviceId = ""
                except Exception as e_pool :
                    self.logger.error({"删除湖泊数据错误:":e_pool})
                save_map_path = os.path.join(server_config.save_map_dir, 'map_%s.json' % ship_code)
                # 两种方式点击湖泊 新增和修改
                if self.server_data_obj_dict.get(
                        ship_code).mqtt_send_get_obj.pool_click_lng_lat and self.server_data_obj_dict.get(
                    ship_code).mqtt_send_get_obj.pool_click_zoom:
                    click_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_lng_lat
                    click_zoom = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_zoom
                    if pre_click_lng_lat is not None and pre_click_lng_lat != click_lng_lat:
                        self.data_define_obj_dict.get(ship_code).pool_code = ''
                elif self.server_data_obj_dict.get(
                        ship_code).mqtt_send_get_obj.update_pool_click_lng_lat and self.server_data_obj_dict.get(
                    ship_code).mqtt_send_get_obj.update_pool_click_zoom:
                    click_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_lng_lat
                    click_zoom = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_zoom
                    if pre_click_lng_lat is not None and pre_click_lng_lat != click_lng_lat:
                        self.data_define_obj_dict.get(ship_code).pool_code = ''
                else:
                    continue  # 都没有点击就跳过查询
                # 判断当前使用地图类型 循环检查3中类型地图 【 高德  腾讯 百度】
                if self.current_map_type == baidu_map.MapType.gaode:
                    title = 'gaode'
                elif self.current_map_type == baidu_map.MapType.tecent:
                    title = 'tecent'
                else:
                    title = 'baidu'
                # 计算保存图片名称和路径
                save_img_path = os.path.join(
                    save_img_dir, '%s_%f_%f_%i_%i.png' %
                                  (title, click_lng_lat[0],
                                   click_lng_lat[1],
                                   click_zoom,
                                   1))
                # 创建于查找湖泊
                if not self.data_define_obj_dict.get(ship_code).pool_code:
                    # print('self.data_define_obj_dict.get(ship_code).pool_code',
                    #       self.data_define_obj_dict.get(ship_code).pool_code, os.path.exists(save_img_path),
                    #       save_img_path)
                    # 创建地图对象
                    if os.path.exists(save_img_path) and self.baidu_map_obj_dict.get(ship_code) is not None:
                        continue
                    else:
                        baidu_map_obj = baidu_map.BaiduMap(
                            lng_lat=click_lng_lat,
                            zoom=click_zoom,
                            logger=self.map_log,
                            map_type=self.current_map_type)
                        self.baidu_map_obj_dict.update({ship_code: baidu_map_obj})
                        pre_click_lng_lat = copy.deepcopy(click_lng_lat)
                    # print('#########', 111111111111111111)
                    pool_cnts, (pool_cx, pool_cy) = self.baidu_map_obj_dict.get(ship_code).get_pool_pix()  # 获取点击处湖泊范围
                    # 为None表示没有找到湖泊 继续换地图找
                    if pool_cnts is None:
                        if self.current_map_type == baidu_map.MapType.gaode:
                            self.current_map_type = baidu_map.MapType.tecent
                            continue
                        if self.current_map_type == baidu_map.MapType.tecent:
                            self.current_map_type = baidu_map.MapType.baidu
                            continue
                        if self.current_map_type == baidu_map.MapType.baidu:
                            self.current_map_type = baidu_map.MapType.gaode
                            # 若返回为None表示没找到湖 定义错误代码
                            is_collision = 1
                            pool_info_data = {
                                'deviceId': ship_code,
                                'lng_lat': self.server_data_obj_dict.get(
                                    ship_code).mqtt_send_get_obj.pool_click_lng_lat,
                                'is_collision': is_collision,
                                'zoom': self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_zoom,
                            }
                            # 发送mqtt话题给前端此处没有湖泊
                            self.send(
                                method='mqtt',
                                topic='pool_info_%s' % ship_code,
                                ship_code=ship_code,
                                data=pool_info_data,
                                qos=1)
                            self.logger.debug({'pool_info_data': pool_info_data})
                            continue
                    # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用
                    _, self.baidu_map_obj_dict.get(ship_code).pool_lng_lats = self.baidu_map_obj_dict.get(
                        ship_code).pix_to_gps(pool_cnts)
                    _, self.baidu_map_obj_dict.get(ship_code).pool_center_lng_lat = self.baidu_map_obj_dict.get(
                        ship_code).pix_to_gps([[pool_cx, pool_cy]])
                    self.logger.info(
                        {'pool_center_lng_lat': self.baidu_map_obj_dict.get(ship_code).pool_center_lng_lat})
                    # 获取湖泊名称 不存在名称就用位置代替
                    self.baidu_map_obj_dict.get(ship_code).get_pool_name()
                    # 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
                    if isinstance(self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                        save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts.tolist()
                    else:
                        save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts
                    send_data = {
                        "longitudeLatitude": json.dumps(
                            self.baidu_map_obj_dict.get(ship_code).pool_center_lng_lat),
                        "mapData": json.dumps(
                            self.baidu_map_obj_dict.get(ship_code).pool_lng_lats),
                        "deviceId": ship_code,
                        "pixData": json.dumps(save_pool_cnts),
                        "province": self.baidu_map_obj_dict.get(ship_code).province,
                        "city": self.baidu_map_obj_dict.get(ship_code).city,
                        "area": self.baidu_map_obj_dict.get(ship_code).district,
                        "street": self.baidu_map_obj_dict.get(ship_code).township,
                    }
                    if self.baidu_map_obj_dict.get(ship_code).pool_name is not None:
                        server_config.pool_name = self.baidu_map_obj_dict.get(ship_code).pool_name
                        send_data.update({"name": server_config.pool_name, })
                    else:
                        server_config.pool_name = ""
                    self.logger.info({'server_config.pool_name': server_config.pool_name})
                    # print('发送湖泊轮廓数据',send_data)
                    # 本地保存经纬度信息，放大1000000倍 用来只保存整数
                    save_pool_lng_lats = [[int(i[0] * 1000000), int(i[1] * 1000000)]
                                          for i in self.baidu_map_obj_dict.get(ship_code).pool_lng_lats]
                    if not os.path.exists(save_map_path):
                        if isinstance(self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                            save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts.tolist()
                        else:
                            save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts
                        try:
                            pool_id = self.send(
                                method='http',
                                data=send_data,
                                ship_code=ship_code,
                                url=server_config.http_save,
                                http_type='POST',
                                token=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token)
                        except Exception as e_http_save:
                            self.logger.error({'e_http_save': e_http_save})
                            continue
                        # 保存数据到本地
                        save_data = {
                            "mapList": [
                                {
                                    "id": pool_id,
                                    "pool_center_lng_lat": self.baidu_map_obj_dict.get(ship_code).pool_center_lng_lat,
                                    "pool_lng_lats": save_pool_lng_lats,
                                    "pool_cnts": save_pool_cnts}]}
                        self.logger.info({'pool_id': pool_id})
                        with open(save_map_path, 'w') as f:
                            json.dump(save_data, f)
                        # sum_circle = self.baidu_map_obj_dict.get(ship_code).cal_map_circle(save_pool_lng_lats)
                        # sum_circle = baidu_map.cal_map_circle(save_pool_lng_lats)
                        # self.logger.info({'周长为': sum_circle})
                    else:
                        with open(save_map_path, 'r') as f:
                            local_map_data = json.load(f)
                            # 判断是否存在本地
                            pool_id = baidu_map.is_in_contours(
                                (self.baidu_map_obj_dict.get(ship_code).lng_lat[0] * 1000000,
                                 self.baidu_map_obj_dict.get(ship_code).lng_lat[1] * 1000000),
                                local_map_data)
                            print('查找本地湖泊 pool_id', pool_id)
                        if pool_id is not None:
                            self.logger.info({'在本地找到湖泊 poolid': pool_id})
                            # 判断是否是用的更新湖泊
                            if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_lng_lat and \
                                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_map_id:
                                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_map_id == pool_id:
                                    self.logger.info({'更新湖泊 poolid': pool_id})
                                    send_data.update({"id": pool_id})
                                    try:
                                        update_flag = self.send(
                                            method='http',
                                            data=send_data,
                                            ship_code=ship_code,
                                            url=server_config.http_update_map,
                                            http_type='POST',
                                            token=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token)
                                    except Exception as e_http_update_map:
                                        self.logger.error({'http_update_map': e_http_update_map})
                                        update_flag = False
                                    if update_flag:
                                        self.logger.info({'更新湖泊成功': pool_id})
                                    else:
                                        self.logger.info({'更新湖泊失败': pool_id})
                                else:
                                    self.logger.info({'湖泊 poolid不相等 本地id ': pool_id,
                                                      "传入id": self.server_data_obj_dict.get(
                                                          ship_code).mqtt_send_get_obj.update_map_id})
                                    try:
                                        pool_id = self.send(
                                            method='http',
                                            data=send_data,
                                            ship_code=ship_code,
                                            url=server_config.http_save,
                                            http_type='POST',
                                            token=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token)
                                    except Exception as e_http_save:
                                        self.logger.error({'保存湖泊失败config.http_save:': e_http_save})
                                    self.logger.info({'生成新的湖泊 poolid': pool_id})
                                    # 更新本地保存数据
                                    with open(save_map_path, 'w') as f:
                                        # 以前存储键值
                                        if isinstance(self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                                            save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts.tolist()
                                        local_map_data["mapList"].append(
                                            {
                                                "id": pool_id,
                                                "pool_center_lng_lat": self.baidu_map_obj_dict.get(
                                                    ship_code).pool_center_lng_lat,
                                                "pool_lng_lats": save_pool_lng_lats,
                                                "pool_cnts": save_pool_cnts})
                                        json.dump(local_map_data, f)
                                # 寻找一次后清空
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_map_id = None
                        # 不存在获取新的id
                        else:
                            try:
                                # print('请求湖泊id数据', send_data)
                                pool_id = self.send(method='http',
                                                    data=send_data,
                                                    ship_code=ship_code,
                                                    url=server_config.http_save,
                                                    http_type='POST',
                                                    token=self.server_data_obj_dict.get(
                                                        ship_code).mqtt_send_get_obj.token)
                                if pool_id is None:
                                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token = None
                                    os.remove(save_img_path)
                                    continue
                            except Exception as e1:
                                self.logger.error({'server_config.http_save:': e1})
                            self.logger.info({'新的湖泊 poolid': pool_id})
                            with open(save_map_path, 'w') as f:
                                if isinstance(self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                                    save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts.tolist()
                                local_map_data["mapList"].append(
                                    {
                                        "id": pool_id,
                                        "pool_center_lng_lat": self.baidu_map_obj_dict.get(
                                            ship_code).pool_center_lng_lat,
                                        "pool_lng_lats": save_pool_lng_lats,
                                        "pool_cnts": save_pool_cnts})
                                json.dump(local_map_data, f)
                    self.data_define_obj_dict.get(ship_code).pool_code = pool_id
                    pool_info_data = {
                        'deviceId': ship_code,
                        'mapId': self.data_define_obj_dict.get(ship_code).pool_code,
                        'lng_lat': self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_lng_lat,
                        # 'pool_lng_lats':self.baidu_map_obj.pool_lng_lats
                    }
                    self.send(
                        method='mqtt',
                        topic='pool_info_%s' % ship_code,
                        ship_code=ship_code,
                        data=pool_info_data,
                        qos=1)
                    # 将数据保存到本地设置中并且更新设置
                    server_config.write_ship_code_setting(ship_code)
                    server_config.update_base_setting(ship_code)
                    # 在基础设置中发送湖泊名称
                    if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.b_pool_click:
                        if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data is None:
                            self.logger.error({'base_setting_data is None': self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.server_base_setting_data})
                        else:
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data.update(
                                {'info_type': 3})
                            send_data = self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.server_base_setting_data
                            send_data.update({'pool_name': server_config.pool_name})
                            self.send(method='mqtt',
                                      topic='server_base_setting_%s' % ship_code,
                                      ship_code=ship_code,
                                      data=send_data,
                                      qos=0)
                            self.logger.info({'base_setting': send_data})
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.b_pool_click = 0

    def get_plan_path(self):
        while True:
            for ship_code in server_config.ship_code_list:
                time.sleep(0.1)
                # 配置判断
                len_target_lng_lat = len(self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat)
                if len_target_lng_lat > 0:
                    # 单点航行模式
                    if len_target_lng_lat == 1:
                        target_lng_lats = copy.deepcopy(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat)
                        self.logger.info({'target_lng_lats': target_lng_lats})
                        self.current_target_gaode_lng_lats_dict.update({ship_code: copy.deepcopy(target_lng_lats)})
                        try:
                            self.path_planning(target_lng_lats=target_lng_lats, ship_code=ship_code)
                            # 成功后将路径点清空
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat = []
                        except Exception as e:
                            self.logger.error({'单点路径规划错误': e})
                    # 多点巡航模式
                    elif len_target_lng_lat > 1:
                        target_lng_lats = copy.deepcopy(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat)
                        self.logger.info({'target_lng_lats': target_lng_lats})
                        self.current_target_gaode_lng_lats_dict.update({ship_code: copy.deepcopy(target_lng_lats)})
                        try:
                            self.path_planning(target_lng_lats=target_lng_lats, ship_code=ship_code)
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat = []
                        except Exception as e:
                            self.logger.error({'多点路径规划错误': e})
                # 客户端获取基础设置数据
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data_info in [1, 4]:
                    if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data is None:
                        self.logger.error(
                            {'base_setting_data is None': self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.server_base_setting_data})
                    else:
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data.update(
                            {'info_type': 3})
                        self.send(method='mqtt',
                                  ship_code=ship_code,
                                  topic='server_base_setting_%s' % ship_code,
                                  data=self.server_data_obj_dict.get(
                                      ship_code).mqtt_send_get_obj.server_base_setting_data,
                                  qos=0)
                        self.logger.info({'server_base_setting_': self.server_data_obj_dict.get(
                            ship_code).mqtt_send_get_obj.server_base_setting_data})
                        # self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.base_setting_data = None
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data_info = 0

                # 判断是否上传了间距使用自动生成采样点
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.row_gap:
                    try:
                        if self.baidu_map_obj_dict.get(ship_code) is None:
                            self.logger.error('地图对象还没有初始化，不能自动设置')
                            time.sleep(1)
                            continue
                        scan_point_cnts = self.baidu_map_obj_dict.get(ship_code).scan_pool(
                            meter_gap=int(self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.row_gap),
                            col_meter_gap=int(self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.row_gap),
                            safe_meter_distance=int(
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.safe_gap),
                            b_show=False)
                        _, scan_point_gaode_list = self.baidu_map_obj_dict.get(ship_code).pix_to_gps(scan_point_cnts)
                        self.path_planning(target_lng_lats=scan_point_gaode_list, ship_code=ship_code)
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.row_gap = None
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.col_gap = None
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.safe_gap = None
                    except Exception as e1:
                        self.logger.error({'error': e1})
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.row_gap = None
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.col_gap = None
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.safe_gap = None

    def path_planning(self, target_lng_lats, ship_code):
        """
        路径规划模块
        """
        b_plan_path = False
        if self.baidu_map_obj_dict.get(ship_code) is not None and self.server_data_obj_dict.get(
                ship_code).mqtt_send_get_obj.current_lng_lat is not None:
            self.baidu_map_obj_dict.get(ship_code).ship_gaode_lng_lat = self.server_data_obj_dict.get(
                ship_code).mqtt_send_get_obj.current_lng_lat
            b_plan_path = True
        # 进行路径规划
        if server_config.b_use_path_planning and b_plan_path:
            return_gaode_lng_lat_path = a_star.get_path(
                baidu_map_obj=self.baidu_map_obj_dict.get(ship_code),
                target_lng_lats=target_lng_lats,
                b_show=False,
            )
            print(return_gaode_lng_lat_path, 'return_gaode_lng_lat_path')
            # 当查找不成功时
            if isinstance(return_gaode_lng_lat_path, str) or return_gaode_lng_lat_path is None:
                self.logger.error(return_gaode_lng_lat_path)
                mqtt_send_path_planning_data = {
                    "deviceId": ship_code,
                    "mapId": self.data_define_obj_dict.get(ship_code).pool_code,
                    "sampling_points": target_lng_lats,
                    "path_points": target_lng_lats,
                    "path_id": len(target_lng_lats)
                }
                self.logger.error({'return_gaode_lng_lat_path': return_gaode_lng_lat_path})
            else:
                # 路径点
                self.plan_path = return_gaode_lng_lat_path
                # 路径点状态
                # self.plan_path_points_status = [0] * len(self.plan_path)
                mqtt_send_path_planning_data = {
                    "deviceId": ship_code,
                    "mapId": self.data_define_obj_dict.get(ship_code).pool_code,
                    "sampling_points": target_lng_lats,
                    "path_points": self.plan_path,
                    "path_id": len(self.plan_path)
                }
                self.logger.info({'len return_gaode_lng_lat_path': len(return_gaode_lng_lat_path)})
        # 不进行路径规划直接到达
        else:
            self.plan_path = copy.deepcopy(target_lng_lats)
            mqtt_send_path_planning_data = {
                "deviceId": ship_code,
                "mapId": self.data_define_obj_dict.get(ship_code).pool_code,
                "sampling_points": target_lng_lats,
                "path_points": target_lng_lats,
                "path_id": len(target_lng_lats)
            }

        # 发送路径规划数据
        self.send(
            method='mqtt',
            topic='path_planning_%s' % ship_code,
            ship_code=ship_code,
            data=mqtt_send_path_planning_data,
            qos=0)
        self.logger.info({'mqtt_send_path_planning_data': mqtt_send_path_planning_data})

    # 发送离岸距离
    def send_bank_distance(self):
        while True:
            for ship_code in server_config.ship_code_list:
                time.sleep(0.1)
                # 判断是否需要更新安全距离
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_safe_distance:
                    # 计算距离岸边距离
                    if self.baidu_map_obj_dict.get(ship_code) and isinstance(
                            self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                        current_pix = self.baidu_map_obj_dict.get(ship_code).gaode_lng_lat_to_pix(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.current_lng_lat)
                        bank_distance = self.baidu_map_obj_dict.get(ship_code).cal_bank_distance(
                            self.baidu_map_obj_dict.get(ship_code).pool_cnts,
                            current_pix,
                            self.baidu_map_obj_dict.get(ship_code).pix_2_meter)
                        bank_distance = round(bank_distance, 1)
                        send_data = {
                            # 设备号
                            "deviceId": ship_code,
                            # 距离 浮点数单位米
                            "bank_distance": bank_distance,
                        }
                        self.send(method='mqtt',
                                  topic='bank_distance_%s' % ship_code,
                                  ship_code=ship_code,
                                  data=send_data,
                                  qos=0)
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_safe_distance = False
                # 判断是否需要获取数据进行克里金插值
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.need_send_distribution is not None:
                    self.send(method='mqtt',
                              topic='distribution_map_%s' % ship_code,
                              ship_code=ship_code,
                              data={
                                  "info": self.server_data_obj_dict.get(
                                      ship_code).mqtt_send_get_obj.need_send_distribution,
                                  "width": 100,  # 图片宽度
                                  "height": self.server_data_obj_dict.get(
                                      ship_code).mqtt_send_get_obj.height_width  # 图片高度
                              },
                              qos=0)
                    print('self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.height_width',
                          self.server_data_obj_dict.get(
                              ship_code).mqtt_send_get_obj.height_width)
                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.need_send_distribution = None

    # 检查在线船只
    def check_online_ship(self):
        ship_status_dict = {}  # 船状态字典
        while True:
            time.sleep(0.5)
            # 判断是否需要更新在线消息
            for ship_code in server_config.ship_code_list:
                # 登录获取token值
                if not self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token:
                    login_data = {"deviceId": ship_code}
                    return_login_data = ship_state_utils.send_server_http_data('POST', login_data,
                                                                               server_config.http_get_token,
                                                                               )
                    if return_login_data:
                        return_login_data_json = json.loads(return_login_data.content)
                        if return_login_data_json.get("code") == 200 and return_login_data_json.get("data"):
                            print('登录返回token:', return_login_data_json.get("data").get("token"))
                            self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.token = return_login_data_json.get("data").get("token")
                        else:
                            print('登录返回失败 %s ' % ship_code, return_login_data_json)
                    else:
                        print('登录请求失败 %s ' % ship_code, return_login_data)
                if not self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token:
                    continue
                # 收到数据认为上线
                if time.time() - self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.receice_time[0] < 2:
                    if ship_status_dict.get(ship_code) == 0 or not ship_status_dict.get(ship_code):
                        current_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.current_lng_lat
                        data = {"deviceId": ship_code, "state": "1"}
                        if current_lng_lat:
                            data.update({'position': json.dumps(current_lng_lat)})
                        is_success = ship_state_utils.send_status(url=server_config.http_set_ship_status,
                                                                  data=data,
                                                                  token=self.server_data_obj_dict.get(
                                                                      ship_code).mqtt_send_get_obj.token)
                        if is_success == 1:
                            ship_status_dict.update({ship_code: 1})
                        # 上传失败需要重新获取token
                        elif is_success == 2:
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token = None
                            continue
                        else:
                            time.sleep(2)
                        print('ship_status_dict', ship_status_dict)
                # 超过指定时间没收到消息认为断开连接 下线时间5秒
                elif time.time() - self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.receice_time[0] > 5:
                    if ship_status_dict.get(ship_code) == 1:
                        current_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.current_lng_lat
                        data = {"deviceId": ship_code, "state": "0"}
                        if current_lng_lat:
                            data.update({'position': json.dumps(current_lng_lat)})
                        is_success = ship_state_utils.send_status(url=server_config.http_set_ship_status,
                                                                  data=data,
                                                                  token=self.server_data_obj_dict.get(
                                                                      ship_code).mqtt_send_get_obj.token)
                        print('is_success', is_success)
                        self.data_define_obj_dict.get(ship_code).pool_code = ''  # 下线后清除
                        if is_success == 1:
                            ship_status_dict.update({ship_code: 0})
                        # 上传失败需要重新获取token
                        elif is_success == 2:
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token = None
                            continue
                        else:
                            time.sleep(2)
                        print('ship_status_dict', ship_status_dict)

    # 判断是否需要重连mqtt
    def check_reconnrct(self):
        while True:
            time.sleep(1)
            # continue
            # 判断是否需要更新在线消息
            for ship_code in server_config.ship_code_list:
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.is_need_reconnect:
                    try:
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.mqtt_connect()  # 连接服务器
                        while not self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.is_reconnect_connected:  # 等待连上
                            time.sleep(0.2)
                        self.server_data_obj_dict.get(ship_code).resubscribe()  # 只有连上后再能订阅话题否则会订阅不上
                    except ConnectionRefusedError:
                        break
                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.is_need_reconnect = False  # 设置需要重连为否
                    self.server_data_obj_dict.get(
                        ship_code).mqtt_send_get_obj.is_reconnect_connected = False  # 设置等待连上为否

            # ADCP 行动id计算湖泊深度图数据 结束行动的时候调用页面单独点击也调用
            for ship_code in server_config.ship_code_list:
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_type == 2 and \
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id:
                    # 向服务器请求本行动数据
                    """
                    action_XXLJC4LCGSCSD1DA007
                    {
                    "action_type":2,
                    "action_id":1545614559804981250
                    }
                    """
                    request_deep_data = {
                        "action_id": self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id,
                        "python": "python",
                        "page": 1,
                        "limit": 1
                    }
                    url = 'http://%s/union/adcp/list/1/1?planId=%s&python=python' % (
                        server_config.http_domin, self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id)
                    return_deep_data = ship_state_utils.get_deep_date('GET', request_deep_data,
                                                                      url,
                                                                      token=self.server_data_obj_dict.get(
                                                                          ship_code).mqtt_send_get_obj.token
                                                                      )
                    return_deep_data_json = json.loads(return_deep_data.content)
                    print('深度数据请求return_deep_data', return_deep_data, return_deep_data_json)
                    if return_deep_data_json.get("code") == 401:
                        self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token = None
                        continue
                    lng_lat_list = []
                    deep_list = []
                    if return_deep_data_json.get('code') == 200 and return_deep_data_json.get('data'):
                        # 如果已经有了就不计算
                        if return_deep_data_json.get('container'):
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id = ""
                            continue
                        for i in return_deep_data_json.get('data'):
                            lng_lat = json.loads(i.get('gjwd'))
                            deep = float(i.get('deep'))
                            lng_lat_list.append(lng_lat)
                            deep_list.append(deep)
                    record_deep_data = {}
                    # 使用经纬度计算x轴坐标  对深度数据计算索引排序
                    x_list, sorted_id2, sum_area = self.gen_send_deep_data(lng_lat_list, deep_list)
                    deep_data_container = {
                        "deep": deep_list,
                        "coordinate": sorted_id2,
                        "xaxis": x_list,
                        "area": sum_area,
                    }
                    deep_data = {
                        "container": json.dumps(deep_data_container),
                        "id": self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id
                    }
                    try:
                        print('deep_data', deep_data)
                        is_success = ship_state_utils.send_deep_date('POST', data=deep_data,
                                                                     url=server_config.http_send_deep,
                                                                     token=self.server_data_obj_dict.get(
                                                                         ship_code).mqtt_send_get_obj.token
                                                                     )
                    except Exception as e:
                        is_success = False
                        self.logger.error({"发送测深绘图数据error": e})
                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.action_id = ""

    def gen_send_deep_data(self, lng_lat, y_list):
        """
        对测深数据深度计算索引排序，将经纬度转换为x轴数据
        @param lng_lat:经纬度数组
        @param y_list:深度数组
        @return:x轴数据  y轴数据 面积积分
        """
        x_List = [i for i in range(len(lng_lat))]
        sorted_id = sorted(range(len(y_list)), key=lambda k: y_list[k], reverse=False)
        # print('元素索引序列：', sorted_id)
        sorted_id2 = sorted(range(len(sorted_id)), key=lambda k: sorted_id[k], reverse=False)
        # print('元素索引序列2：', sorted_id2)
        dx = 1
        sum_area = round(float(np.sum(np.asarray(y_list) * dx)), 2)
        return x_List, sorted_id2, sum_area

    # 获取萤石云报警图片
    def get_ezviz_alarm_image(self):
        local_http = False
        if local_http:
            http_domin = '192.168.8.26:8008'
        else:
            http_domin = 'peri.xxlun.com'
        http_get_img_path = "http://%s/union/device/getPicUrl" % http_domin
        http_upload_img = "http://%s/union/user/uploadFile" % http_domin
        http_draw_save = 'http://%s/union/sampling/save' % http_domin
        ship_code = 'XXLJC4LCGSCSD1DA004'
        rerocd_data_list = []
        rerocd_time_list = []
        while True:
            if not self.server_data_obj_dict.get(ship_code):
                time.sleep(1)
                continue
            if not self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj:
                time.sleep(1)
                continue
            if not self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token:
                time.sleep(1)
                continue
            data_list = draw_img.get_img_url(http_get_img_path, {"deviceId": ship_code, 'type': 0},
                                             token=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token)
            # print('data_list', data_list)
            if data_list:
                for i in data_list:
                    if i.get('alarmType') != 15500:
                        continue
                    # 重复图片
                    if i.get('alarmTime') in rerocd_time_list:
                        continue
                    else:
                        rerocd_time_list.append(i.get('alarmTime'))
                        alarmTime = i.get('alarmTime')
                        alarmPicUrl = i.get('alarmPicUrl')
                        rerocd_data_list.append([alarmTime, alarmPicUrl])
                        # 上传图片
                        img_path = 'temp.jpg'
                        draw_img.save_img(alarmPicUrl, img_path)
                        server_save_img_path = draw_img.post_file(url=http_upload_img, file_path="temp.jpg",
                                                                  file_name=None, token=self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.token)
                        print('保存图片:', server_save_img_path)
                        if server_save_img_path:
                            draw_data = {}
                            draw_data.update(
                                {"pic": server_save_img_path, 'deviceId': ship_code, "seconds": alarmTime})
                            return_data = draw_img.send_server_http_data(request_type='POST', data=draw_data,
                                                                         url=http_draw_save,
                                                                         token=self.server_data_obj_dict.get(
                                                                             ship_code).mqtt_send_get_obj.token)
                            print('保存图片返回:', return_data)
                            # self.send(method='http', data=draw_data,
                            #           url=http_draw_save,
                            #           ship_code=ship_code,
                            #           http_type='POST',
                            #           token=self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.token)
            rerocd_data_list.sort(key=lambda x: (x[0], x[1]))
            if len(rerocd_data_list) > 0:
                for j in rerocd_data_list[-1:]:
                    # 判断是否收到前端确认图片
                    if self.server_data_obj_dict.get(ship_code):
                        if not self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.alarm_picture_data or \
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.alarm_picture_data.get(
                                    "picture_url") != j[1]:
                            alarm_data = {'type': 1,
                                          "picture_url": j[1]
                                          }
                            print('报警信息', alarm_data)
                            self.send(
                                method='mqtt',
                                topic='alarm_picture_%s' % ship_code,
                                ship_code=ship_code,
                                data=alarm_data,
                                qos=0)
            time.sleep(8)


if __name__ == '__main__':
    while True:
        try:
            web_server_obj = WebServer()
            find_pool_thread = threading.Thread(target=web_server_obj.find_pool)
            get_plan_path_thread = threading.Thread(target=web_server_obj.get_plan_path)
            send_bank_distance_thread = threading.Thread(target=web_server_obj.send_bank_distance)
            check_online_ship_thread = threading.Thread(target=web_server_obj.check_online_ship)
            check_reconnrct_thread = threading.Thread(target=web_server_obj.check_reconnrct)
            get_ezviz_thread = threading.Thread(target=web_server_obj.get_ezviz_alarm_image)
            find_pool_thread.start()
            get_plan_path_thread.start()
            send_bank_distance_thread.start()
            check_online_ship_thread.start()
            check_reconnrct_thread.start()
            # get_ezviz_thread.start()
            while True:
                if not find_pool_thread.is_alive():
                    find_pool_thread = threading.Thread(target=web_server_obj.find_pool)
                    find_pool_thread.start()
                if not get_plan_path_thread.is_alive():
                    get_plan_path_thread = threading.Thread(target=web_server_obj.get_plan_path)
                    get_plan_path_thread.start()
                if not send_bank_distance_thread.is_alive():
                    send_bank_distance_thread = threading.Thread(target=web_server_obj.send_bank_distance)
                    send_bank_distance_thread.start()
                if not check_online_ship_thread.is_alive():
                    check_online_ship_thread = threading.Thread(target=web_server_obj.check_online_ship)
                    check_online_ship_thread.start()
                if not check_reconnrct_thread.is_alive():
                    check_reconnrct_thread = threading.Thread(target=web_server_obj.check_reconnrct)
                    check_reconnrct_thread.start()
                time.sleep(1)
        except Exception as e:
            print({'error': e})
            time.sleep(2)
