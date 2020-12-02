"""
网络数据收发
"""
from dataGetSend.data_define import DataDefine
from utils.log import LogHandler

import paho.mqtt.client as mqtt
import time


class ServerData:
    def __init__(self):
        obj = DataDefine()
        data_dict = {}
        data_dict.update({'statistics_data': obj.statistics_data()})
        data_dict.update({'status_data': obj.status_data()})
        data_dict.update({'meteorological_data': obj.meteorological_data()})
        data_dict.update({'water_quality_data': obj.water_quality_data()})
        self.log = LogHandler('server_data')
        self.log.info(data_dict)


    # 发送数据到服务器http
    def send_server_http_data(self):
        pass

    # 发送数据到服务器mqtt
    def send_server_mqtt_data(self):
        pass

    # 从服务器mqtt接收数据
    def get_server_mqtt_data(self):
        pass
    # 从服务器http接收数据
    def get_server_http_data(self):
        pass
    #

class HttpSendGet:
    """
    处理ｊｓｏｎ数据收发
    """
    def __init__(self):
        pass


class MqttSendGet:
    """
    处理mqtt数据收发
    """
    def __init__(self,mqtt_host='ws://116.62.44.118',mqtt_port=8083):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client = mqtt.Client()
        self.on_mqtt_connect()

    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        # self.mqtt_client.loop_start()

    # publish 消息
    def on_publish(self,topic, payload, qos):
        self.mqtt_client.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, userdata, msg):
        print(msg.topic + " " + ":" + str(msg.payload))

    # subscribe 消息
    def on_subscribe(self):
        self.mqtt_client.subscribe("jing1", 1)
        self.mqtt_client.on_message = self.on_message_come  # 消息到来处理函数


if __name__ == '__main__':
    # obj = ServerData()
    obj = MqttSendGet()

    obj.on_subscribe()
    obj.mqtt_client.loop_forever()
    while True:
        print("%s"%(str(time.time())))
        obj.on_publish("jing1", "%s"%(str(time.time())), 1)
        time.sleep(2)
