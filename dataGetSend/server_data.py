"""
网络数据收发
"""
from dataGetSend.data_define import DataDefine
from utils.log import LogHandler

import paho.mqtt.client as mqtt
import time
import json
import requests


class ServerData:
    def __init__(self,topic='control_data'):
        self.data_define_obj = DataDefine()
        data_dict = {}
        data_dict.update({'statistics_data': self.data_define_obj.statistics_data()})
        data_dict.update({'status_data': self.data_define_obj.status_data()})
        data_dict.update({'meteorological_data': self.data_define_obj.meteorological_data()})
        data_dict.update({'water_quality_data': self.data_define_obj.water_quality_data()})
        self.log = LogHandler('server_data')
        # self.log.info(data_dict)
        self.http_send_get_obj = HttpSendGet()
        self.mqtt_send_get_obj = MqttSendGet()
        # 启动后自动订阅话题
        self.mqtt_send_get_obj.subscribe_topic(topic=topic,qos=1)



    # 发送数据到服务器http
    def send_server_http_data(self,data):
        pass

    # 发送数据到服务器mqtt
    def send_server_mqtt_data(self,topic='test', data="", qos=1):
        self.mqtt_send_get_obj.publish_topic(topic=topic, data=data, qos=qos)

    # 从服务器mqtt接收数据
    def get_server_mqtt_data(self):
        pass
    # 从服务器http接收数据
    def get_server_http_data(self):
        pass


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
    def __init__(self,mqtt_host='116.62.44.118',mqtt_port=1883,client_id='jing'):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client= mqtt.Client(client_id=client_id)
        self.mqtt_client.on_connect = self.on_connect_callback
        self.mqtt_client.on_publish = self.on_publish_callback
        # self.mqtt_client.on_subscribe = self.on_message_come
        self.mqtt_client.on_message = self.on_message_callback
        self.mqtt_connect()

        # 前后左右移动控制键　0 为前进　90 度向左　　180 向后　　270向右　　
        self.move_direction = -1
        # 测量控制位　false为不采样　true为采样
        self.b_sampling = False
        # 抽水控制位
        self.b_draw = False

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
        print('publish',mid)

    # 消息处理函数回调
    def on_message_callback(self, client, userdata, msg):
        # print(msg.topic + " " + ":" + str(msg.payload),type(msg.payload))
        # 回调更新控制数据
        control_data = json.loads(msg.payload)
        assert isinstance(control_data,dict),'please send dict data'
        if control_data.get('move_direction') is not None:
            # print('self.move_direction',self.move_direction)
            self.move_direction = control_data['move_direction']
        if control_data.get('b_sampling') is not None:
            self.b_sampling = control_data['b_sampling']
        if control_data.get('b_draw') is not None:
            self.b_draw = control_data['b_draw']


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
    obj = MqttSendGet()
    obj.subscribe_topic(topic='qqq')
    while True:
        obj.publish_topic(topic='qqq', data={'12312':'Hello,EMQ!'}, qos=2)
        time.sleep(2)
    # obj.mqtt_client.loop_forever()
    # while True:
    #     print("%s"%(str(time.time())))
    #     obj.on_publish("jing1", "%s"%(str(time.time())), 1)
    #     time.sleep(2)
