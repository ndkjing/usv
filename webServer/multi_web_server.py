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
        os.path.dirname(
            os.path.abspath(__file__)),
        'externalConnect'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'moveControl'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'statics'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))

import config
from moveControl.pathPlanning import a_star
# from externalConnect import baidu_map
from webServer import server_baidu_map as baidu_map
from utils.log import LogHandler
from webServer import ship_state_utils
from webServer.web_server_data import ServerData
from webServer import server_data_define
from webServer import server_config
# import get_eviz_url


class WebServer:
    def __init__(self):
        self.data_define_obj_dict = {}
        for ship_code in server_config.ship_code_list:
            self.data_define_obj_dict.update({ship_code: server_data_define.DataDefine(ship_code=ship_code)})

        self.baidu_map_obj_dict = {}
        # 日志对象
        self.logger = LogHandler('web_server', level=20)
        self.server_log = LogHandler('server_data')
        self.map_log = LogHandler('map_log')
        # mqtt服务器数据收发对象
        self.server_data_obj_dict = {}
        for ship_code in server_config.ship_code_list:
            self.server_data_obj_dict.update({ship_code: ServerData(self.server_log,
                                                                    topics=self.data_define_obj_dict.get(
                                                                        ship_code).topics,
                                                                    ship_code=ship_code)})
        # 记录目标点击地点
        self.current_target_gaode_lng_lats_dict = {}
        # 记录路径规划地点
        self.plan_path = None
        self.current_map_type = baidu_map.MapType.gaode

    def send(
            self,
            method,
            data,
            ship_code,
            topic='test',
            qos=0,
            http_type='POST',
            url=''):
        """
        :param ship_code:
        :param url:
        :param http_type:
        :param qos:
        :param data:
        :param topic:
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt',
                          'com'], 'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj_dict.get(ship_code).send_server_http_data(
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
            elif http_type == 'POST' and r'upData' in url:
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                else:
                    return True
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                save_data = content_data["data"]["mapList"]
                return save_data
        elif method == 'mqtt':
            self.server_data_obj_dict.get(ship_code).send_server_mqtt_data(
                data=data, topic=topic, qos=qos)

    def get_pool_and_save(self, send_data, ship_code, save_map_path, save_pool_lng_lats):
        """
        @param send_data:
        @param ship_code:
        @param save_map_path:
        @return:
        """
        try:
            pool_id = self.send(
                method='http',
                data=send_data,
                ship_code=ship_code,
                url=server_config.http_save,
                http_type='POST')
            self.logger.info({'新的湖泊 poolid': pool_id})
            assert isinstance(pool_id, str)
        except Exception as e1:
            self.logger.error({'config.http_save:': e1})
            return False
        with open(save_map_path, 'r') as f:
            local_map_data = json.load(f)
        with open(save_map_path, 'w') as f:
            # 以前存储键值
            # local_map_data["mapList"].append({"id": pool_id,
            #                                   "longitudeLatitude": self.baidu_map_obj.pool_center_lng_lat,
            #                                   "mapData": self.baidu_map_obj.pool_lng_lat,
            #                                   "pool_cnt": pool_cnts.tolist()})
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
        return pool_id

    # 状态检查函数，检查自身状态发送对应消息
    def find_pool(self):
        while True:
            # 循环等待一定时间
            time.sleep(0.1)
            for ship_code in server_config.ship_code_list:
                save_map_path = os.path.join(server_config.save_map_dir, 'map_%s.json' % ship_code)
                # # 若是用户没有点击点
                # if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_lng_lat is None or \
                #         self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_zoom is None:
                #     continue
                # 查找与更新湖泊id
                save_img_dir = os.path.join(server_config.root_path, 'statics', 'imgs')
                if not os.path.exists(save_img_dir):
                    os.mkdir(save_img_dir)
                # 超过1000张图片时候删除图片
                save_img_name_list = os.listdir(save_img_dir)
                if len(save_img_name_list) > 1000:
                    for i in save_img_name_list:
                        print(os.path.join(save_img_dir, i))
                        os.remove(os.path.join(save_img_dir, i))
                # 两种方式点击湖泊 新增和修改
                if self.server_data_obj_dict.get(
                        ship_code).mqtt_send_get_obj.pool_click_lng_lat and self.server_data_obj_dict.get(
                    ship_code).mqtt_send_get_obj.pool_click_zoom:
                    click_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_lng_lat
                    click_zoom = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.pool_click_zoom
                elif self.server_data_obj_dict.get(
                        ship_code).mqtt_send_get_obj.update_pool_click_lng_lat and self.server_data_obj_dict.get(
                    ship_code).mqtt_send_get_obj.update_pool_click_zoom:
                    click_lng_lat = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_lng_lat
                    click_zoom = self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_zoom
                else:
                    continue
                if self.current_map_type == baidu_map.MapType.gaode:
                    title = 'gaode'
                elif self.current_map_type == baidu_map.MapType.tecent:
                    title = 'tecent'
                else:
                    title = 'baidu'
                save_img_path = os.path.join(
                    save_img_dir, '%s_%f_%f_%i_%i.png' %
                                  (title, click_lng_lat[0],
                                   click_lng_lat[1],
                                   click_zoom,
                                   1))
                # 创建于查找湖泊
                if self.data_define_obj_dict.get(ship_code).pool_code or not os.path.exists(save_img_path):
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
                    pool_cnts, (pool_cx, pool_cy) = self.baidu_map_obj_dict.get(ship_code).get_pool_pix()
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
                    self.baidu_map_obj_dict.get(ship_code).get_pool_name()
                    if self.baidu_map_obj_dict.get(ship_code).pool_name is not None:
                        server_config.pool_name = self.baidu_map_obj_dict.get(ship_code).pool_name
                    else:
                        server_config.pool_name = self.baidu_map_obj_dict.get(ship_code).address
                    self.logger.info({'server_config.pool_name': server_config.pool_name})
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
                        "name": server_config.pool_name,
                        "pixData": json.dumps(save_pool_cnts)}

                    # 本地保存经纬度信息，放大1000000倍 用来只保存整数
                    save_pool_lng_lats = [[int(i[0] * 1000000), int(i[1] * 1000000)]
                                          for i in self.baidu_map_obj_dict.get(ship_code).pool_lng_lats]
                    if not os.path.exists(save_map_path):
                        if isinstance(self.baidu_map_obj_dict.get(ship_code).pool_cnts, np.ndarray):
                            save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts.tolist()
                        else:
                            save_pool_cnts = self.baidu_map_obj_dict.get(ship_code).pool_cnts

                        # pood_id = self.get_pool_and_save(send_data=send_data,
                        #                        ship_code=ship_code,
                        #                        save_map_path=save_map_path,
                        #                        save_pool_lng_lats=save_pool_lng_lats)
                        # if pool_id:
                        #     # 发送请求获取湖泊ID
                        #     self.logger.info({'pood_id': pood_id})
                        # else:
                        #     self.logger.error({'请求湖泊id出错误': pood_id})
                        try:
                            pool_id = self.send(
                                method='http',
                                data=send_data,
                                ship_code=ship_code,
                                url=server_config.http_save,
                                http_type='POST')
                        except Exception as e:
                            self.logger.error({'error': e})
                            continue
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
                        sum_circle = self.baidu_map_obj_dict.get(ship_code).cal_map_circle(save_pool_lng_lats)
                        self.logger.info({'周长为': sum_circle})
                    else:
                        with open(save_map_path, 'r') as f:
                            local_map_data = json.load(f)
                            pool_id = baidu_map.is_in_contours(
                                (self.baidu_map_obj_dict.get(ship_code).lng_lat[0] * 1000000,
                                 self.baidu_map_obj_dict.get(ship_code).lng_lat[1] * 1000000),
                                local_map_data)
                            print('pool_id', pool_id)
                        if pool_id is not None:
                            self.logger.info({'在本地找到湖泊 poolid': pool_id})
                            # 判断是否是用的更新湖泊
                            if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_pool_click_lng_lat and \
                                    self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_map_id:
                                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.update_map_id == pool_id:
                                    self.logger.info({'更新湖泊 poolid': pool_id})
                                    send_data.update({"id": pool_id})
                                    update_flag = self.send(
                                        method='http',
                                        data=send_data,
                                        ship_code=ship_code,
                                        url=server_config.http_update_map,
                                        http_type='POST')
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
                                            http_type='POST')
                                    except Exception as e1:
                                        self.logger.error({'config.http_save:': e1})
                                    self.logger.info({'新的湖泊 poolid': pool_id})
                                    with open(save_map_path, 'w') as f:
                                        # 以前存储键值
                                        # local_map_data["mapList"].append({"id": pool_id,
                                        #                                   "longitudeLatitude": self.baidu_map_obj.pool_center_lng_lat,
                                        #                                   "mapData": self.baidu_map_obj.pool_lng_lat,
                                        #                                   "pool_cnt": pool_cnts.tolist()})
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
                        # 不存在获取新的id
                        else:
                            try:
                                pool_id = self.send(
                                    method='http',
                                    data=send_data,
                                    ship_code=ship_code,
                                    url=server_config.http_save,
                                    http_type='POST')
                            except Exception as e1:
                                self.logger.error({'server_config.http_save:': e1})
                            self.logger.info({'新的湖泊 poolid': pool_id})
                            with open(save_map_path, 'w') as f:
                                # 以前存储键值
                                # local_map_data["mapList"].append({"id": pool_id,
                                #                                   "longitudeLatitude": self.baidu_map_obj.pool_center_lng_lat,
                                #                                   "mapData": self.baidu_map_obj.pool_lng_lat,
                                #                                   "pool_cnt": pool_cnts.tolist()})
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
                            self.logger.error(
                                {
                                    'base_setting_data is None': self.server_data_obj_dict.get(
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
                if len_target_lng_lat >= 0:
                    # 单点航行模式
                    if len_target_lng_lat == 1:
                        target_lng_lats = copy.deepcopy(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat)
                        # 判断是否是上次的点
                        if self.current_target_gaode_lng_lats_dict.get(ship_code) == target_lng_lats:
                            pass
                        else:
                            self.logger.info({'target_lng_lats': target_lng_lats})
                            self.current_target_gaode_lng_lats_dict.update({ship_code: copy.deepcopy(target_lng_lats)})
                            try:
                                self.path_planning(target_lng_lats=target_lng_lats, ship_code=ship_code)
                            except Exception as e:
                                self.logger.error({'error': e})
                    # 多点巡航模式
                    elif len_target_lng_lat > 1:
                        target_lng_lats = copy.deepcopy(
                            self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.target_lng_lat)
                        # print('###########################target_lng_lats', target_lng_lats)
                        # 是否返航
                        # if int(self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.back_home) == 1:
                        #     if config.home_debug:
                        #         target_lng_lats.append(config.ship_gaode_lng_lat)
                        #     else:
                        #         # 添加当前经纬度作为返航点
                        #         if self.baidu_map_obj_dict.get(ship_code) is not None and self.server_data_obj_dict.get(
                        #                 ship_code).mqtt_send_get_obj.home_lng_lat is not None:
                        #             target_lng_lats.append(
                        #                 self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.home_lng_lat)

                        if self.current_target_gaode_lng_lats_dict.get(ship_code) == target_lng_lats:
                            pass
                        else:
                            self.logger.info({'target_lng_lats': target_lng_lats})
                            self.current_target_gaode_lng_lats_dict.update({ship_code: copy.deepcopy(target_lng_lats)})
                            self.path_planning(target_lng_lats=target_lng_lats, ship_code=ship_code)

                # 客户端获取基础设置数据
                if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data_info in [1, 4]:
                    if self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.server_base_setting_data is None:
                        self.logger.error(
                            {'base_setting_data is None': self.server_data_obj_dict.get(
                                ship_code).mqtt_send_get_obj.mqtt_send_get_obj.server_base_setting_data})
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
                            meter_gap=int(
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.base_setting_data.get(
                                    'row')),
                            col_meter_gap=int(
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.base_setting_data.get(
                                    'col')),
                            safe_meter_distance=int(
                                self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.base_setting_data.get(
                                    'secure_distance')),
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

    # 发送离岸距离
    def check_online_ship(self):
        ship_status_dict = {}  # 船状态字典
        while True:
            time.sleep(0.1)
            for ship_code in server_config.ship_code_list:
                if ship_status_dict.get(ship_code) is None:
                    b_online = ship_state_utils.get_status(url=server_config.http_get_ship_status, devicd_id=ship_code)
                    if b_online is None:
                        continue
                    else:
                        ship_status_dict.update({ship_code: b_online})
            # 判断是否需要更新在线消息
            for ship_code in server_config.ship_code_list:
                if time.time() - self.server_data_obj_dict.get(ship_code).mqtt_send_get_obj.receice_time[0] < 5:
                    if ship_status_dict.get(ship_code) == 0:
                        is_success = ship_state_utils.send_status(url=server_config.http_set_ship_status,
                                                                  data={"deviceId": ship_code, "state": "1"})
                        if is_success:
                            ship_status_dict.update({ship_code: 1})
                        else:
                            time.sleep(2)
                        # print('ship_status_dict', ship_status_dict)
                else:
                    if ship_status_dict.get(ship_code) == 1:
                        is_success = ship_state_utils.send_status(url=server_config.http_set_ship_status,
                                                                  data={"deviceId": ship_code, "state": "0"})

                        if is_success:
                            ship_status_dict.update({ship_code: 0})
                        else:
                            time.sleep(2)
                        print('ship_status_dict', ship_status_dict)


if __name__ == '__main__':
    while True:
        try:
            web_server_obj = WebServer()
            find_pool_thread = threading.Thread(target=web_server_obj.find_pool)
            get_plan_path_thread = threading.Thread(target=web_server_obj.get_plan_path)
            send_bank_distance_thread = threading.Thread(target=web_server_obj.send_bank_distance)
            check_online_ship_thread = threading.Thread(target=web_server_obj.check_online_ship)
            find_pool_thread.start()
            get_plan_path_thread.start()
            send_bank_distance_thread.start()
            check_online_ship_thread.start()
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
                time.sleep(1)
        except Exception as e:
            print({'error': e})
            time.sleep(1)