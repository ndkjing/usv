"""
网络数据收发
"""
# from messageBus.data_define import DataDefine
# import config
# from messageBus import data_define
# from webServer import server_config
# from webServer import distribution_map
from utils import log
import server_config
import copy
import os
import paho.mqtt.client as mqtt
import time
import json
import requests
import distribution_map
import upload_file

class ServerData:
    def __init__(self, logger,
                 topics,
                 ship_code):
        self.logger = logger
        self.topics = topics
        self.http_send_get_obj = HttpSendGet()
        self.mqtt_send_get_obj = MqttSendGet(self.logger, ship_code=ship_code)
        # 启动后自动订阅话题
        for topic_, qos_ in self.topics:
            self.mqtt_send_get_obj.subscribe_topic(topic=topic_, qos=qos_)

    # 重新连接mqtt话题后订阅
    def resubscribe(self):
        for topic_, qos_ in self.topics:
            self.mqtt_send_get_obj.subscribe_topic(topic=topic_, qos=qos_)

    # 发送数据到服务器http
    def send_server_http_data(self, request_type, data, url, token=None):
        # 请求头设置
        payloadHeader = {
            'Content-Type': 'application/json',
        }
        if token:
            payloadHeader.update({'token': token})
        assert request_type in ['POST', 'GET']
        if request_type == 'POST':
            dumpJsonData = json.dumps(data)
            return_data = requests.post(
                url=url, data=dumpJsonData, headers=payloadHeader)
        else:
            return_data = requests.get(url=url)
        return return_data

    # 发送数据到服务器mqtt
    def send_server_mqtt_data(self, topic='test', data="", qos=1):
        self.mqtt_send_get_obj.publish_topic(topic=topic, data=data, qos=qos)


class HttpSendGet:
    """
    处理ｊｓｏｎ数据收发
    """

    def __init__(self, base_url='127.0.0.1'):
        self.base_url = base_url

    def send_data(self, uri, data):
        """
        :param uri 发送接口uri
        :param data  需要发送数据
        """
        send_url = self.base_url + uri
        response = requests.post(send_url, data=data)

    def get_data(self, uri):
        """
        :param uri 发送接口uri
        """
        get_url = self.base_url + uri
        response = requests.get(uri)


class MqttSendGet:
    """
    处理mqtt数据收发
    """

    def __init__(
            self,
            logger,
            ship_code,
            mqtt_host=server_config.mqtt_host,
            mqtt_port=server_config.mqtt_port,
            client_id=server_config.ship_code
    ):
        self.logger = logger  # 日志
        self.mqtt_host = mqtt_host  # mqtt ip地址
        self.mqtt_port = mqtt_port  # mqtt  ip端口
        if ship_code is not None:
            client_id = ship_code
            self.mqtt_user = 'dk_linux_x' + ship_code
        else:
            client_id = client_id + 'dk_windwos'
            self.mqtt_user = 'dk_windwos'
        self.mqtt_passwd = 'public'
        self.ship_code = ship_code
        self.logger.info({'client_id': client_id})
        self.mqtt_client = mqtt.Client(client_id=client_id)
        self.mqtt_client.username_pw_set(self.mqtt_user, password=self.mqtt_passwd)
        self.mqtt_client.on_connect = self.on_connect_callback  # mqtt连接回调
        self.mqtt_client.on_publish = self.on_publish_callback  # mqtt发布话题消息回调
        self.mqtt_client.on_disconnect = self.on_disconnect_callback  # mqtt断开回调
        # self.mqtt_client.on_subscribe = self.on_message_come
        self.mqtt_client.on_message = self.on_message_callback  # mqtt订阅话题消息回调
        self.mqtt_connect()  # 连接mqtt服务器
        # 湖泊初始点击点信息
        self.pool_click_lng_lat = None
        self.pool_click_zoom = None
        # 更新湖泊初始点击点信息
        self.update_pool_click_lng_lat = None
        self.update_pool_click_zoom = None
        self.update_map_id = None
        # 接收到点击的经纬度目标地点和点击是地图层次，二维矩阵
        self.target_lng_lat = []
        self.zoom = []
        self.meter_pix = {}
        self.mode = []
        self.pool_id = None
        # 记录经纬度是不是已经到达或者放弃到达（在去的过程中手动操作） 0准备过去(自动) -1放弃（手动）  1 已经到达的点  2:该点是陆地
        self.target_lng_lat_status = []
        # 当前航线  -1是还没选择
        self.current_lng_lat_index = -1
        self.confirm_index = -1
        # 路径规划话题中的消息
        self.path_planning_points = []
        self.path_planning_points_status = []
        # 船当前经纬度 给服务器路径规划使用
        self.current_lng_lat = None
        # 船返航点经纬度 给服务器路径规划使用
        self.home_lng_lat = None
        # 自动求取经纬度设置 行间距 列间距 离岸边安全距离
        self.row_gap = None
        self.col_gap = None
        self.safe_gap = 10
        # 环绕湖运行距离岸边间距
        self.round_pool_gap = None
        # 行驶轨迹确认ID 与是否确认
        self.path_id = None
        self.path_id_confirm = None
        # 前后左右移动控制键　0 为前进　90 度向左　　180 向后　　270向右　　360为停止
        self.control_move_direction = str(360)
        # 测量控制位　0为不采样　1为采样
        self.b_sampling = 0
        # 抽水控制位  0为不抽水　1为抽水
        self.b_draw = 0
        # 启动还是停止寻点模式
        self.b_start = 0
        # 请求设置类型
        self.base_setting_data = None
        self.base_setting_data_info = -1
        # 请求设置类型
        self.server_base_setting_data = None
        self.server_base_default_setting_data = None
        self.server_base_setting_data_info = -1
        # 点击湖泊
        self.b_pool_click = 0
        # 重置选择湖泊
        self.reset_pool_click = 0
        # 检查要发给前端绘图话题数据
        self.need_send_distribution = None
        self.height_width = 100  # 宽高比 设置宽为100  计算高度与宽比值
        # 更新船当前到岸边距离 当收到新的经纬度后设置该值为True
        self.update_safe_distance = False
        self.back_home = 0
        self.fix_point = 0
        self.receice_time = [0, self.ship_code]  # 记录下最近接受到数据的时间和船号
        self.b_send_online = 0  # 判断是否需要发送在线数据
        self.start_time = time.time()
        self.is_need_reconnect = False  # 判断是否需要重连
        self.is_reconnect_connected = False  # 判断重连是否连上
        self.token = None  # 存储自身上传数据token
        self.alarm_picture_data = {}
        self.action_type = 0
        self.action_id = ""
        self.delete_pool_mapId = ""
        self.delete_pool_deviceId = ""

    # 断开MQTt回调
    def on_disconnect_callback(self, client, userdata, rc):
        self.logger.info('disconnected with result code:  ' + str(rc), )
        self.is_need_reconnect = True

    # 连接MQTT服务器
    def mqtt_connect(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        # 开启接收循环，直到程序终止
        self.mqtt_client.loop_start()

    # 建立连接时候回调
    def on_connect_callback(self, client, userdata, flags, rc):
        self.logger.info('Connected with result code:  ' + str(rc), )
        if self.is_need_reconnect:  # 如果本次是重连就设置重连标志位让后续可以再次订阅话题
            self.is_reconnect_connected = True

    # 发布消息回调
    def on_publish_callback(self, client, userdata, mid):
        pass
        # print('publish',mid)

    # 消息处理函数回调
    def on_message_callback(self, client, userdata, msg):
        topic = msg.topic
        # self.receice_time = [time.time(), self.ship_code]  # 记录下最近接受到数据的时间和船号
        # 处理初始点击确定湖数据
        if topic == 'pool_click_%s' % self.ship_code:
            pool_click_data = json.loads(msg.payload)
            if pool_click_data.get('lng_lat') is None:
                self.logger.error('pool_click  用户点击经纬度数据没有经纬度字段')
                return
            if pool_click_data.get('zoom') is None:
                self.logger.error('pool_click 用户点击经纬度数据没有zoom字段')
                return
            lng_lat = pool_click_data.get('lng_lat')
            self.pool_click_lng_lat = lng_lat
            zoom = int(round(float(pool_click_data.get('zoom')), 0))
            self.pool_click_zoom = zoom
            self.b_pool_click = 1
            self.logger.info({'topic': topic,
                              'pool_click_data': pool_click_data
                              })
        # 处理重新选择湖数据
        if topic == 'update_pool_click_%s' % self.ship_code:
            update_pool_click_data = json.loads(msg.payload)
            if update_pool_click_data.get('lng_lat') is None:
                self.logger.error('update_pool_click_data  用户点击经纬度数据没有经纬度字段')
                return
            if update_pool_click_data.get('zoom') is None:
                self.logger.error('update_pool_click_data 用户点击经纬度数据没有zoom字段')
                return
            lng_lat = update_pool_click_data.get('lng_lat')
            self.update_pool_click_lng_lat = lng_lat
            zoom = int(round(float(update_pool_click_data.get('zoom')), 0))
            self.update_pool_click_zoom = zoom
            # 清空选择湖泊
            self.pool_click_lng_lat = None
            self.pool_click_zoom = None
            if update_pool_click_data.get('mapId'):
                self.update_map_id = update_pool_click_data.get('mapId')
            self.b_pool_click = 1
            self.logger.info({'topic': topic,
                              'self.update_pool_click_lng_lat': self.update_pool_click_lng_lat,
                              'self.update_pool_click_zoom': self.update_pool_click_zoom,
                              'self.update_map_id': self.update_map_id
                              })

        # 用户点击经纬度和图层 保存到指定路径
        elif topic == 'user_lng_lat_%s' % self.ship_code:
            user_lng_lat_data = json.loads(msg.payload)
            if user_lng_lat_data.get('lng_lat') is None:
                self.logger.info('user_lng_lat_用户点击经纬度数据没有经纬度字段来源于其他启动方式')
                return
            # 添加新的点
            lng_lat = user_lng_lat_data.get('lng_lat')
            self.target_lng_lat = lng_lat
            self.target_lng_lat_status = [0] * len(lng_lat)
            self.logger.info({'topic': topic,
                              'target_lng_lat': self.target_lng_lat,
                              })
        # 用户设置自动求取检测点经纬度
        elif topic == 'auto_lng_lat_%s' % self.ship_code:
            auto_lng_lat_data = json.loads(msg.payload)
            if auto_lng_lat_data.get('config') is None:
                self.logger.error('auto_lng_lat_用户设置自动求取检测点经纬度没有config字段')
                return
            if auto_lng_lat_data.get('config').get('row_gap') is None:
                self.logger.error('auto_lng_lat_用户设置自动求取检测点经纬度config字段没有row_gap')
                return
            self.row_gap = auto_lng_lat_data.get('config').get('row_gap')
            self.col_gap = auto_lng_lat_data.get('config').get('col_gap')
            if auto_lng_lat_data.get('config').get('safe_gap') is not None:
                self.safe_gap = auto_lng_lat_data.get('config').get('safe_gap')
            self.round_pool_gap = auto_lng_lat_data.get('config').get('round_pool_gap')
            self.logger.info({'topic': topic,
                              'row_gap': self.row_gap,
                              'col_gap': self.col_gap,
                              'safe_gap': self.safe_gap,
                              'round_pool_gap': self.round_pool_gap})
        # 返回路径规划点
        elif topic == 'path_planning_%s' % self.ship_code:
            path_planning_data = json.loads(msg.payload)
            if path_planning_data.get('path_points') is None:
                self.logger.error('path_planning_用户确认轨迹 没有path_points字段')
                return
            self.path_planning_points = path_planning_data.get('path_points')
            self.path_planning_points_status = [0] * len(self.path_planning_points)
            self.logger.info({'topic': topic,
                              'path_points': path_planning_data.get('path_points'),
                              })
        # 用户确认轨迹
        elif topic == 'path_planning_confirm_%s' % self.ship_code:
            path_planning_confirm_data = json.loads(msg.payload)
            if not path_planning_confirm_data.get('path_id'):
                self.logger.error('path_planning_confirm_用户确认轨迹 没有path_id字段')
                return
            if not path_planning_confirm_data.get('confirm'):
                self.logger.error('path_planning_confirm_用户确认轨迹 没有confirm字段')
                return
            self.path_id = path_planning_confirm_data.get('path_id')
            self.path_id_confirm = path_planning_confirm_data.get('confirm')

            self.logger.info({'topic': topic,
                              'path_id': path_planning_confirm_data.get('path_id'),
                              'path_id_confirm': path_planning_confirm_data.get('confirm'),
                              })
        # 启动设备
        elif topic == 'start_%s' % self.ship_code:
            start_data = json.loads(msg.payload)
            if not start_data.get('search_pattern'):
                self.logger.error('start_设置启动消息没有search_pattern字段')
                return
            self.b_start = int(start_data.get('search_pattern'))
            self.logger.info({'topic': topic, 'b_start': start_data.get('search_pattern')})
        # 湖泊id
        elif topic == 'pool_info_%s' % self.ship_code:
            pool_info_data = json.loads(msg.payload)
            if not pool_info_data.get('mapId'):
                self.logger.error('pool_info_data设置启动消息没有mapId字段')
                return
            self.pool_id = str(pool_info_data.get('mapId'))
            self.logger.info({'topic': topic, 'mapId': pool_info_data.get('mapId')})

        # 服务器从状态数据中获取 当前经纬度
        elif topic == 'status_data_%s' % self.ship_code:
            status_data = json.loads(msg.payload)
            self.receice_time = [time.time(), self.ship_code]  # 记录下最近接受到数据的时间和船号
            # print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),'状态消息',self.receice_time)
            if status_data.get("current_lng_lat") is None:
                # self.logger.error('"status_data"设置启动消息没有"current_lng_lat"字段')
                return
            else:
                self.current_lng_lat = status_data.get('current_lng_lat')
                self.update_safe_distance = True  # 接受到经纬度后开始计算离岸边距离
            if status_data.get("home_lng_lat") is None:
                pass
            else:
                self.home_lng_lat = status_data.get('home_lng_lat')

        # 获取前端发送的用于请求空间数据接口
        elif topic == 'distribution_map_%s' % self.ship_code:
            distribution_map_data = json.loads(msg.payload)
            print('distribution_map_data')
            if distribution_map_data.get("deviceId"):
                deviceId = distribution_map_data.get("deviceId")
                mapId = distribution_map_data.get("mapId")
                planId = distribution_map_data.get("planId")
                data_type = distribution_map_data.get("data_type")
                # 请求数据
                distribution_info = distribution_map.save_geo_json_map(deviceId=deviceId,
                                                                       mapId=mapId,
                                                                       planId=planId,
                                                                       data_type=data_type,
                                                                       token=self.token)
                self.height_width = distribution_info[1]
                if distribution_info[0] == 2:
                    # 发送到mqtt话题
                    self.need_send_distribution = 2
                else:
                    # 绘制地图
                    distribution_map.MyOrdinaryKriging(planId,
                                                       data_type,
                                                       )
                    # 发送给服务器
                    file_target = "%s_%s.png" % (planId, data_type)
                    try:
                        # 发送数据到服务器
                        print('发送数据到服务器')
                        # url_data = "http://service.newship.xxlun.com:12345/union/user/uploadimage?deviceId=%s&planId=%s" % (
                        #     deviceId, planId)
                        url_data = "https://peri.xxlun.com/union/user/uploadimage?deviceId=%s&planId=%s" % (
                            deviceId, planId)
                        upload_file.post_data(url=url_data, file=file_target, token=self.token)
                    except Exception as e:
                        print('e', e)
                        self.need_send_distribution = 2

        # 服务器基础配置
        elif topic == 'server_base_setting_%s' % self.ship_code:
            server_base_setting_path = os.path.join(server_config.setting_dir, 'setting_%s.json' % self.ship_code)
            if not os.path.exists(server_base_setting_path):
                server_config.write_setting(True, True, self.ship_code)
            self.logger.info({'server_base_setting_ ': json.loads(msg.payload)})
            if len(msg.payload) < 5:
                return
            server_base_setting_data = json.loads(msg.payload)
            if server_base_setting_data.get("info_type") is None:
                self.logger.error('"server_base_setting_data"设置启动消息没有"info_type"字段')
                return
            else:
                info_type = int(server_base_setting_data.get('info_type'))
                self.server_base_setting_data_info = info_type
                if info_type == 1:
                    with open(server_base_setting_path, 'r') as f:
                        self.server_base_setting_data = json.load(f)
                elif info_type == 2:
                    with open(server_base_setting_path, 'r') as f:
                        self.server_base_setting_data = json.load(f)
                    with open(server_base_setting_path, 'w') as f:
                        self.server_base_setting_data.update(server_base_setting_data)
                        json.dump(self.server_base_setting_data, f)
                    server_config.update_base_setting(server_base_setting_path)
            # 恢复默认配置
            # elif info_type == 4:
            #     with open(server_base_setting_path, 'w') as f:
            #         with open(server_config.server_base_default_setting_path, 'r') as df:
            #             self.server_base_default_setting_data = json.load(df)
            #             self.server_base_setting_data = copy.deepcopy(self.server_base_default_setting_data)
            #             json.dump(self.server_base_setting_data, f)
            #     server_config.update_base_setting()

        # 在线船只话题
        elif topic == 'online_ship':
            online_ship_data = json.loads(msg.payload)
            if online_ship_data.get("info_type") is None:
                self.logger.error('"online_ship_data"消息没有"info_type"字段')
                return
            else:
                if int(online_ship_data.get("info_type")) == 1:
                    self.b_send_online = 1
            # print('online_ship_data',online_ship_data,self.b_send_online)

        elif topic == 'token_%s' % self.ship_code:
            token_data = json.loads(msg.payload)
            if token_data.get('type') and token_data.get('type') == 1:
                self.publish_topic('token_%s' % self.ship_code, data={'type': 2, 'token': self.token})
            self.logger.info({'topic': topic,
                              'token_data': token_data,
                              })

        elif topic == 'alarm_picture_%s' % self.ship_code:
            alarm_picture_data = json.loads(msg.payload)
            if alarm_picture_data.get('type') and alarm_picture_data.get('type') == 2:
                self.alarm_picture_data = alarm_picture_data
                self.logger.info({'topic': topic,
                                  'alarm_picture_data': alarm_picture_data,
                                  })

        elif topic == 'action_%s' % self.ship_code:
            action_data = json.loads(msg.payload)
            if action_data.get('action_type') and action_data.get('action_id'):
                self.action_type = action_data.get('action_type')
                self.action_id = action_data.get('action_id')
            self.logger.info({'topic': topic,
                              'action_data': action_data,
                              })

        # 船坞相关设置
        elif topic == 'delete_pool_%s' % self.ship_code:
            delete_pool_data = json.loads(msg.payload)  # a_delete_status
            self.logger.info({'收到删除湖泊数据': delete_pool_data})
            if delete_pool_data.get("a_delete_status") is None and delete_pool_data.get(
                    "mapId") is not None and delete_pool_data.get("deviceId") is not None:
                self.delete_pool_mapId = delete_pool_data.get("mapId")
                self.delete_pool_deviceId = delete_pool_data.get("deviceId")
                self.logger.info({'确定删除湖泊数据': delete_pool_data})

    # 发布消息
    def publish_topic(self, topic, data, qos=0):
        """
        向指定话题发布消息
        :param topic 发布话题名称
        :param data 　发布消息
        :param qos　　发布质量
        """
        if isinstance(data, list):
            data = str(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        elif isinstance(data, dict):
            data = json.dumps(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        elif isinstance(data, int) or isinstance(data, float):
            data = str(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        else:
            self.mqtt_client.publish(topic, payload=data, qos=qos)

    # 订阅消息
    def subscribe_topic(self, topic='qqq', qos=0):
        """
        :param topic 订阅的话题
        :param qos　　发布质量
        """
        self.logger.info({'topic': topic, 'qos': qos})
        self.mqtt_client.subscribe(topic, qos)


if __name__ == '__main__':
    logger = log.LogHandler('server_data_test')
    mqtt_obj = MqttSendGet(logger, ship_code=None)
