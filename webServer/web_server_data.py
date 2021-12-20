"""
网络数据收发
"""
from messageBus.data_define import DataDefine
import config
from messageBus import data_define
from webServer import server_config
from utils import log
import copy
import os
import paho.mqtt.client as mqtt
import time
import json
import requests


class ServerData:
    def __init__(self, logger,
                 topics,
                 ship_code):
        self.logger = logger
        self.topics = topics
        self.http_send_get_obj = HttpSendGet()
        self.mqtt_send_get_obj = MqttSendGet(self.logger, ship_code=ship_code)
        # 启动后自动订阅话题
        for topic, qos in self.topics:
            self.mqtt_send_get_obj.subscribe_topic(topic=topic, qos=qos)

    # 发送数据到服务器http
    def send_server_http_data(self, request_type, data, url):
        # 请求头设置
        payloadHeader = {
            'Content-Type': 'application/json',
        }
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
            mqtt_host=config.mqtt_host,
            mqtt_port=config.mqtt_port,
            client_id=config.ship_code
    ):
        self.logger = logger
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
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
        self.mqtt_client.on_connect = self.on_connect_callback
        self.mqtt_client.on_publish = self.on_publish_callback
        # self.mqtt_client.on_subscribe = self.on_message_come
        self.mqtt_client.on_message = self.on_message_callback
        self.mqtt_connect()
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
        self.server_base_setting_data = None
        self.server_base_default_setting_data = None
        self.server_base_setting_data_info = -1
        # 点击湖泊
        self.b_pool_click = 0
        # 重置选择湖泊
        self.reset_pool_click = 0

        # 更新船当前到岸边距离 当收到新的经纬度后设置该值为True
        self.update_safe_distance = False
        self.back_home = 0
        self.fix_point = 0


    # 连接MQTT服务器
    def mqtt_connect(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        # 开启接收循环，直到程序终止
        self.mqtt_client.loop_start()

    # 建立连接时候回调
    def on_connect_callback(self, client, userdata, flags, rc):
        self.logger.info('Connected with result code:  ' + str(rc), )

    # 发布消息回调
    def on_publish_callback(self, client, userdata, mid):
        pass

    # 消息处理函数回调
    def on_message_callback(self, client, userdata, msg):
        topic = msg.topic
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
            # 清空更新湖泊
            self.update_pool_click_lng_lat = None
            self.update_pool_click_zoom = None

            self.b_pool_click = 1
            self.logger.info({'topic': topic,
                              'lng_lat': pool_click_data.get('lng_lat'),
                              'zoom': pool_click_data.get('zoom')
                              })
        # 处理初始点击确定湖数据
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
            self.pool_click_lng_lat=None
            self.pool_click_zoom=None
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
                self.logger.error('user_lng_lat_用户点击经纬度数据没有经纬度字段')
                return
            # if user_lng_lat_data.get('zoom') is None:
            #     self.logger.error('user_lng_lat_用户点击经纬度数据没有zoom字段')
            #     return
            # if user_lng_lat_data.get('meter_pix') is None:
            #     self.logger.error('user_lng_lat_用户点击经纬度数据没有meter_pix字段')
            if user_lng_lat_data.get('config') is None:
                self.logger.error('user_lng_lat_用户点击经纬度数据没有config字段')
                return
            # 添加新的点
            lng_lat = user_lng_lat_data.get('lng_lat')
            self.target_lng_lat = lng_lat
            self.target_lng_lat_status = [0] * len(lng_lat)
            # zoom = int(round(float(user_lng_lat_data.get('zoom')), 0))
            # self.zoom.append(zoom)
            # self.meter_pix.update({zoom: float(user_lng_lat_data.get('meter_pix'))})
            if user_lng_lat_data.get('config').get('back_home') is not None:
                self.back_home = user_lng_lat_data.get('config').get('back_home')
            self.fix_point = user_lng_lat_data.get('config').get('fixpoint')
            self.logger.info({'topic': topic,
                              'target_lng_lat': self.target_lng_lat,
                              'back_home': self.back_home,
                              'fix_point': self.fix_point,
                              })

        # 用户设置自动求取检测点经纬度
        elif topic == 'auto_lng_lat_%s' % (self.ship_code):
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
        elif topic == 'path_planning_%s' % (self.ship_code):
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
        elif topic == 'path_planning_confirm_%s' % (self.ship_code):
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
        elif topic == 'start_%s' % (self.ship_code):
            start_data = json.loads(msg.payload)
            if not start_data.get('search_pattern'):
                self.logger.error('start_设置启动消息没有search_pattern字段')
                return
            self.b_start = int(start_data.get('search_pattern'))
            self.logger.info({'topic': topic, 'b_start': start_data.get('search_pattern')})

        # 湖泊id
        elif topic == 'pool_info_%s' % (self.ship_code):
            pool_info_data = json.loads(msg.payload)
            if not pool_info_data.get('mapId'):
                self.logger.error('pool_info_data设置启动消息没有mapId字段')
                return
            self.pool_id = str(pool_info_data.get('mapId'))
            self.logger.info({'topic': topic, 'mapId': pool_info_data.get('mapId')})

        # 服务器从状态数据中获取 当前经纬度
        elif topic == 'status_data_%s' % (self.ship_code):
            status_data = json.loads(msg.payload)
            if status_data.get("current_lng_lat") is None:
                # self.logger.error('"status_data"设置启动消息没有"current_lng_lat"字段')
                return
            else:
                self.current_lng_lat = status_data.get('current_lng_lat')
                self.update_safe_distance = True
            if status_data.get("home_lng_lat") is None:
                pass
            else:
                self.home_lng_lat = status_data.get('home_lng_lat')
        # 服务器基础配置
        elif topic == 'server_base_setting_%s' % self.ship_code:
            server_base_setting_path = os.path.join(server_config.setting_dir, 'setting_%s.json' % self.ship_code)
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
    # obj = ServerData()
    logger = log.LogHandler('server_data_test')
    mqtt_obj = MqttSendGet(logger)
    data_define_obj = DataDefine()
    # 启动后自动订阅话题
    for topic, qos in data_define_obj.topics:
        logger.info(topic + '    ' + str(qos))
        mqtt_obj.subscribe_topic(topic=topic, qos=qos)
    # http发送检测数据给服务器
    while True:
        mqtt_obj.publish_topic(
            topic='status_data_%s' %
                  (config.ship_code),
            data=data_define.init_ststus_data,
            qos=1)
        mqtt_obj.publish_topic(
            topic='detect_data_%s' %
                  (config.ship_code),
            data=data_define.init_detect_data,
            qos=1)
        time.sleep(1)