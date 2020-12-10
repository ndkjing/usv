"""
网络数据收发
"""
from dataGetSend.data_define import DataDefine
import config
from dataGetSend import data_define
from utils import log

import paho.mqtt.client as mqtt
import time
import json
import requests


class ServerData:

    def __init__(self,logger,
                 topics):
        self.logger = logger
        self.topics = topics
        self.http_send_get_obj = HttpSendGet()
        self.mqtt_send_get_obj = MqttSendGet(self.logger)
        # 启动后自动订阅话题
        for topic,qos in self.topics:
            self.mqtt_send_get_obj.subscribe_topic(topic=topic,qos=qos)

    # 发送数据到服务器http
    def send_server_http_data(self,request_type,data,url):
        # 请求头设置
        payloadHeader = {
            # 'Host': 'sellercentral.amazon.com',
            'Content-Type': 'application/json',
        }
        assert request_type in ['POST','GET']
        if request_type=='POST':
            # print(type(data))
            dumpJsonData = json.dumps(data)
            # print(f"dumpJsonData = {dumpJsonData}")
            # print('url',url)
            return_data = requests.post(url=url,data=dumpJsonData,headers=payloadHeader)
        else :
            return_data = requests.get(url=url)
        return return_data

    # 发送数据到服务器mqtt
    def send_server_mqtt_data(self,topic='test', data="", qos=1):
        self.mqtt_send_get_obj.publish_topic(topic=topic, data=data, qos=qos)



class HttpSendGet:
    """
    处理ｊｓｏｎ数据收发
    """
    def __init__(self,base_url='127.0.0.1'):
        self.base_url = base_url

    def send_data(self,uri,data):
        """
        :param uri 发送接口uri
        :param data  需要发送数据
        """
        send_url = self.base_url+uri
        response = requests.post(send_url,data=data)


    def get_data(self,uri):
        """
        :param uri 发送接口uri
        """
        get_url = self.base_url+uri
        response = requests.get(uri)


class MqttSendGet:
    """
    处理mqtt数据收发
    """
    def __init__(self,logger,mqtt_host='116.62.44.118',mqtt_port=1883,client_id='jing'):
        self.logger = logger
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client= mqtt.Client(client_id=client_id)
        self.mqtt_client.on_connect = self.on_connect_callback
        self.mqtt_client.on_publish = self.on_publish_callback
        # self.mqtt_client.on_subscribe = self.on_message_come
        self.mqtt_client.on_message = self.on_message_callback
        self.mqtt_connect()

        # 前后左右移动控制键　0 为前进　90 度向左　　180 向后　　270向右　　
        self.move_direction = 360
        # 测量控制位　0为不采样　1为采样
        self.b_sampling = 0
        # 抽水控制位  0为不抽水　1为抽水
        self.b_draw = 0
        # 接收到的经纬度
        self.lng_lat = None
        self.zoom = None


    # 连接MQTT服务器
    def mqtt_connect(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        # 开启接收循环，直到程序终止
        self.mqtt_client.loop_start()

    # 建立连接时候回调
    def on_connect_callback(self,client, userdata, flags, rc):
        print("Connected with result code " + str(rc))

    # 发布消息回调
    def on_publish_callback(self,client, userdata, mid):
        pass
        # print('publish',mid)

    # 消息处理函数回调
    def on_message_callback(self, client, userdata, msg):
        # print(msg.topic + " " + ":" + str(msg.payload),type(msg.payload))
        ## 回调更新控制数据
        # 判断topic
        topic = msg.topic
        if topic =='control_data_%s'%(data_define.ship_code):
            # 处理控制数据
            control_data = json.loads(msg.payload)
            assert isinstance(control_data,dict),'please send dict data'
            if control_data.get('move_direction') is not None:
                self.move_direction = control_data['move_direction']
                self.logger('move_direction', self.move_direction)
            if control_data.get('b_sampling') is not None:
                if int(control_data['b_sampling'])==1:
                    self.b_sampling = 1
            if control_data.get('b_draw') is not None:
                if int(control_data['b_draw'])==1:
                    self.b_draw = 1

        elif topic=='user_lng_lat_%s'%(data_define.ship_code):
            # 用户点击经纬度和图层 保存到指定路径
            user_lng_lat_data = json.loads(msg.payload)
            # TODO 暂时不保存
            # with open(config.usr_lng_lat_path,'w') as f:
            #     json.dump(user_lng_lat_data,f)
            if user_lng_lat_data.get('b_draw') is not None:
                self.lng_lat = user_lng_lat_data.get('lng_lat')
                self.zoom = user_lng_lat_data.get('zoom')
                self.logger.info({'lng_lat':user_lng_lat_data.get('lng_lat'),'zoom':user_lng_lat_data.get('zoom')})

        elif topic=='path_confirm_%s'%(data_define.ship_code):
            # 判断是否确然当前路径
            path_confirm_data=json.loads(msg.payload)
            with open(config.usr_lng_lat_path,'rw') as f:
                user_lng_lat_data = json.load(f)
                b_confirm = path_confirm_data['confirm']
                if b_confirm:
                    user_lng_lat_data.update({'confirm':True})
                else:
                    user_lng_lat_data.update({'confirm': False})

    # 发布消息
    def publish_topic(self,topic, data, qos=0):
        """
        向指定话题发布消息
        :param topic 发布话题名称
        :param data 　发布消息
        :param qos　　发布质量
        """
        if isinstance(data,list):
            data = str(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        elif isinstance(data,dict):
            data = json.dumps(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        elif isinstance(data,int) or isinstance(data,float):
            data = str(data)
            self.mqtt_client.publish(topic, payload=data, qos=qos)
        else :
            self.mqtt_client.publish(topic, payload=data, qos=qos)

    # 订阅消息
    def subscribe_topic(self,topic='qqq' ,qos=0):
        """
        :param topic 订阅的话题
        :param qos　　发布质量
        """
        self.mqtt_client.subscribe(topic, qos)


if __name__ == '__main__':
    # obj = ServerData()
    logger = log.LogHandler('server_data_test')
    mqtt_obj = MqttSendGet(logger)
    data_define_obj = DataDefine()
    # 启动后自动订阅话题
    for topic, qos in data_define_obj.topics:
        mqtt_obj.subscribe_topic(topic=topic, qos=qos)
    # http发送检测数据给服务器
    while True:
        mqtt_obj.publish_topic(topic='status_data_%s' % (data_define.ship_code), data=data_define.status_data(), qos=1)
        mqtt_obj.publish_topic(topic='detect_data_%s' % (data_define.ship_code), data=data_define.detect_data(), qos=1)
        time.sleep(config.pi2mqtt_interval)

