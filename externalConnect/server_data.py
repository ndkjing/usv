"""
网络数据收发
"""
from messageBus.data_define import DataDefine
import config
from messageBus import data_define
from utils import log
import copy
import paho.mqtt.client as mqtt
import time
import json
import requests


class ServerData:
    def __init__(self, logger,
                 topics, ship_code=None):
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
        if (config.current_platform == "l_pi"):
            client_id = client_id + 'dk_linux2'
            self.mqtt_user = 'dk_linux2'
        elif (config.current_platform == "l_j"):
            client_id = client_id + 'dk_linux_j'
            self.mqtt_user = 'dk_linux_j'
        elif (config.current_platform == "l_j"):
            client_id = client_id + 'dk_linux_x'
            self.mqtt_user = 'dk_linux_x'
        elif ship_code is not None:
            client_id = ship_code
            self.mqtt_user = 'dk_linux_x' + ship_code
        else:
            client_id = client_id + 'dk_windwos'
            self.mqtt_user = 'dk_windwos'
        self.mqtt_passwd = 'public'
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
        self.sampling_points = []
        self.path_planning_points = []
        self.sampling_points_status = []

        # 船当前经纬度 给服务器路径规划使用
        self.current_lng_lat = None
        # 船返航点经纬度 给服务器路径规划使用
        self.home_lng_lat = []

        # 自动求取经纬度设置 使用行间距和记录当前路径点是使用行间距
        self.row_gap = None
        self.use_col_gap = False
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
        # 启动还是停止
        self.b_start = 0
        # 基础设置数据
        self.base_setting_data = None
        # 基础数据设置类型
        self.base_setting_data_info = None
        self.base_setting_default_data = None
        # 高级设置
        self.height_setting_data = None
        # 类型
        self.height_setting_data_info = None
        self.height_setting_default_data = None
        # 刷新后请求数据
        self.refresh_info_type = 0

        # 重置湖泊
        self.reset_pool_click = 0
        # 检查超过指定时间没有收到服务器数据就开启  断网返航
        self.last_command_time = time.time()
        self.b_network_backhome = 0

    # 连接MQTT服务器
    def mqtt_connect(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        # 开启接收循环，直到程序终止
        self.mqtt_client.loop_start()

    # 建立连接时候回调
    def on_connect_callback(self, client, userdata, flags, rc):
        self.logger.info('Connected with result code:  ' + str(rc))

    # 发布消息回调
    def on_publish_callback(self, client, userdata, mid):
        pass
        # print('publish',mid)

    # 消息处理函数回调
    def on_message_callback(self, client, userdata, msg):
        try:
            # 回调更新控制数据
            # 判断topic
            topic = msg.topic
            if config.network_backhome:
                if time.time()-self.last_command_time>min(config.network_backhome,300):
                    self.b_network_backhome=1
            self.last_command_time = time.time()
            # 处理控制数据
            if topic == 'control_data_%s' % (config.ship_code):
                control_data = json.loads(msg.payload)
                if control_data.get('move_direction') is None:
                    self.logger.error('control_data_处理控制数据没有move_direction')
                    return
                # 判断当前是否是寻点模式如果是则
                if self.row_gap:
                    if not self.use_col_gap:
                        self.use_col_gap = True
                    # 已经为True表示已经传递过一次规划路径又传递了一次，此时取消寻点标记
                    else:
                        self.use_col_gap = False
                        self.row_gap = 0
                self.control_move_direction = int(control_data.get('move_direction'))
                if self.control_move_direction ==-1:
                    self.sampling_points = []
                    self.path_planning_points = []
                    self.sampling_points_status = []
                self.logger.info({'topic': topic,
                                  'control_move_direction': control_data.get('move_direction'),
                                  })

            # 处理开关信息
            if topic == 'switch_%s' % (config.ship_code):
                switch_data = json.loads(msg.payload)
                if switch_data.get('b_draw') is None:
                    self.logger.error('switch_data_处理控制数据没有b_draw b_sampling')
                if switch_data.get('b_sampling') is not None:
                    self.b_sampling = int(switch_data.get('b_sampling'))
                if switch_data.get('b_draw') is not None:
                    self.b_draw = int(switch_data.get('b_draw'))
                self.logger.info({'topic': topic,
                                  'b_sampling': switch_data.get('b_sampling'),
                                  'b_draw': switch_data.get('b_draw')
                                  })

            # 处理初始点击确定湖数据
            elif topic == 'pool_click_%s' % (config.ship_code):
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
                self.logger.info({'topic': topic,
                                  'lng_lat': pool_click_data.get('lng_lat'),
                                  'zoom': pool_click_data.get('zoom')
                                  })

            # 用户点击经纬度和图层 保存到指定路径
            elif topic == 'user_lng_lat_%s' % (config.ship_code):
                user_lng_lat_data = json.loads(msg.payload)
                if user_lng_lat_data.get('lng_lat') is None:
                    self.logger.error('user_lng_lat_用户点击经纬度数据没有经纬度字段')
                    return
                if user_lng_lat_data.get('zoom') is None:
                    self.logger.error('user_lng_lat_用户点击经纬度数据没有zoom字段')
                    return
                if user_lng_lat_data.get('meter_pix') is None:
                    self.logger.error('user_lng_lat_用户点击经纬度数据没有meter_pix字段')
                if user_lng_lat_data.get('config') is None:
                    self.logger.error('user_lng_lat_用户点击经纬度数据没有config字段')

                # 添加新的点
                lng_lat = user_lng_lat_data.get('lng_lat')
                self.target_lng_lat = lng_lat
                self.target_lng_lat_status = [0] * len(lng_lat)
                zoom = int(round(float(user_lng_lat_data.get('zoom')), 0))
                self.zoom.append(zoom)
                self.meter_pix.update({zoom: float(user_lng_lat_data.get('meter_pix'))})
                if user_lng_lat_data.get('config').get('back_home') is not None:
                    self.back_home = user_lng_lat_data.get('config').get('back_home')

                self.fix_point = user_lng_lat_data.get('config').get('fixpoint')

                self.logger.info({'topic': topic,
                                  'target_lng_lat': self.target_lng_lat,
                                  'zoom': zoom,
                                  'meter_pix': user_lng_lat_data.get('meter_pix'),
                                  'back_home': self.back_home,
                                  'fix_point': self.fix_point,
                                  })

            # 用户设置自动求取检测点经纬度
            elif topic == 'auto_lng_lat_%s' % (config.ship_code):
                auto_lng_lat_data = json.loads(msg.payload)
                if auto_lng_lat_data.get('config') is None:
                    self.logger.error('auto_lng_lat_用户设置自动求取检测点经纬度没有config字段')
                    return
                if auto_lng_lat_data.get('config').get('row_gap') is None:
                    self.logger.error('auto_lng_lat_用户设置自动求取检测点经纬度config字段没有row_gap')
                    return
                self.row_gap = 1
                self.logger.info({'topic': topic,
                                  'row_gap': self.row_gap})

            # 返回路径规划点
            elif topic == 'path_planning_%s' % (config.ship_code):
                path_planning_data = json.loads(msg.payload)
                if path_planning_data.get('path_points') is None:
                    self.logger.error('path_planning_用户确认轨迹 没有path_points字段')
                    return
                # 判断当前是否是寻点模式如果是则
                if self.row_gap:
                    if not self.use_col_gap:
                        self.use_col_gap = True
                    # 已经为True表示已经传递过一次规划路径又传递了一次，此时取消寻点标记
                    else:
                        self.use_col_gap = False
                        self.row_gap = 0
                # 从路径规划话题中提取
                self.sampling_points = path_planning_data.get('sampling_points')
                # 存储目标点到达状态
                self.sampling_points_status = [0] * len(self.sampling_points)
                self.path_planning_points = path_planning_data.get('path_points')
                self.logger.info({'topic': topic,
                                  'sampling_points':path_planning_data.get('sampling_points'),
                                  'path_points': path_planning_data.get('path_points'),
                                  })

            # 用户确认轨迹
            elif topic == 'path_planning_confirm_%s' % (config.ship_code):
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
            elif topic == 'start_%s' % (config.ship_code):
                start_data = json.loads(msg.payload)
                if not start_data.get('search_pattern'):
                    self.logger.error('start_设置启动消息没有search_pattern字段')
                    return
                self.b_start = int(start_data.get('search_pattern'))
                self.logger.info({'topic': topic, 'b_start': start_data.get('search_pattern')})

            # 湖泊id
            elif topic == 'pool_info_%s' % (config.ship_code):
                pool_info_data = json.loads(msg.payload)
                if not pool_info_data.get('mapId'):
                    self.logger.error('pool_info_data设置启动消息没有mapId字段')
                    return
                self.pool_id = str(pool_info_data.get('mapId'))
                self.logger.info({'topic': topic, 'mapId': pool_info_data.get('mapId')})

            # 服务器从状态数据中获取 当前经纬度
            elif topic == 'status_data_%s' % (config.ship_code):
                status_data = json.loads(msg.payload)
                if not status_data.get("current_lng_lat"):
                    # self.logger.error('"status_data"设置启动消息没有"current_lng_lat"字段')
                    return
                self.current_lng_lat = status_data.get('current_lng_lat')
                # self.logger.info({'topic': topic,
                #                   'current_lng_lat': status_data.get('current_lng_lat')})

            # 基础配置
            elif topic == 'base_setting_%s' % (config.ship_code):
                self.logger.info({'base_setting ': json.loads(msg.payload)})
                if len(msg.payload) < 5:
                    return
                base_setting_data = json.loads(msg.payload)
                if base_setting_data.get("info_type") is None:
                    self.logger.error('"base_setting_data"设置启动消息没有"info_type"字段')
                    return
                else:
                    info_type = int(base_setting_data.get('info_type'))
                    self.base_setting_data_info = info_type
                    if info_type == 1:
                        with open(config.base_setting_path, 'r') as f:
                            self.base_setting_data = json.load(f)
                    elif info_type == 2:
                        with open(config.base_setting_path, 'r') as f:
                            self.base_setting_data = json.load(f)
                        with open(config.base_setting_path, 'w') as f:
                            self.base_setting_data.update(base_setting_data)
                            json.dump(self.base_setting_data, f)
                        config.update_base_setting()
                    # 恢复默认配置
                    elif info_type == 4:
                        with open(config.base_setting_path, 'w') as f:
                            with open(config.base_setting_default_path, 'r') as df:
                                self.base_setting_default_data = json.load(df)
                                self.base_setting_data = copy.deepcopy(self.base_setting_default_data)
                                json.dump(self.base_setting_data, f)
                        config.update_base_setting()

            # 高级配置
            elif topic == 'height_setting_%s' % (config.ship_code):
                self.logger.info({'height_setting_data': json.loads(msg.payload)})
                height_setting_data = json.loads(msg.payload)
                if height_setting_data.get("info_type") is None:
                    self.logger.error('"height_setting_data"设置启动消息没有"info_type"字段')
                    return
                else:
                    info_type = int(height_setting_data.get('info_type'))
                    self.height_setting_data_info = info_type
                    if info_type == 1:
                        with open(config.height_setting_path, 'r') as f:
                            self.height_setting_data = json.load(f)
                    elif info_type == 2:
                        with open(config.height_setting_path, 'r') as f:
                            self.height_setting_data = json.load(f)
                        with open(config.height_setting_path, 'w') as f:
                            self.height_setting_data.update(height_setting_data)
                            json.dump(self.height_setting_data, f)
                        config.update_height_setting()
                    # 恢复默认配置
                    elif info_type == 4:
                        with open(config.height_setting_path, 'w') as f:
                            with open(config.height_setting_default_path, 'r') as df:
                                self.height_setting_default_data = json.load(df)
                                self.height_setting_data = copy.deepcopy(self.height_setting_default_data)
                                json.dump(self.height_setting_data, f)
                        config.update_height_setting()

            # 刷新后请求数据消息
            elif topic == 'refresh_%s' % (config.ship_code):
                self.logger.info({'refresh_setting ': json.loads(msg.payload)})
                refresh_data = json.loads(msg.payload)
                if refresh_data.get("info_type") is None:
                    self.logger.error('"refresh_"设置启动消息没有"info_type"字段')
                    return
                else:
                    info_type = int(refresh_data.get('info_type'))
                    self.refresh_info_type = info_type

            # 处理重置
            elif topic == 'reset_pool_%s' % (config.ship_code):
                reset_pool_data = json.loads(msg.payload)
                if reset_pool_data.get('reset_pool') is None:
                    self.logger.error('reset_pool_处理控制数据没有reset_pool')
                    return
                self.reset_pool_click = int(reset_pool_data.get('reset_pool'))
                self.logger.info({'topic': topic,
                                  'reset_pool': reset_pool_data.get('reset_pool'),
                                  })
        except Exception as e:
            self.logger.error({'error': e})

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
        time.sleep(config.pi2mqtt_interval)
