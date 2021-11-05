from unittest import TestCase
import unittest
from utils import log
from externalConnect import server_data
from messageBus import data_define
import config
import time


class TestMqttSendGet(TestCase):
    def __init__(self,*args, **kwargs):
        super(TestMqttSendGet, self).__init__(*args, **kwargs)
        self.src_point = [114.4314, 30.523558]
        # obj = ServerData()
        self.logger_ = log.LogHandler('server_data_test')
        self.mqtt_obj = server_data.MqttSendGet(self.logger_)
        self.data_define_obj = server_data.DataDefine()

    def test_publish_topic(self):
        # 启动后自动订阅话题
        for topic, qos in self.data_define_obj.topics:
            self.logger_.info(topic + '    ' + str(qos))
            self.mqtt_obj.subscribe_topic(topic=topic, qos=qos)
        # http发送检测数据给服务器
        while True:
            self.mqtt_obj.publish_topic(
                topic='status_data_%s' %
                      (config.ship_code),
                data=data_define.init_ststus_data,
                qos=0)
            self.mqtt_obj.publish_topic(
                topic='detect_data_%s' %
                      (config.ship_code),
                data=data_define.init_detect_data,
                qos=0)
            time.sleep(1)


if __name__ == '__main__':
    unittest.main()