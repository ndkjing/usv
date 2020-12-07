"""
管理数据收发
"""
from dataGetSend.data_define import DataDefine
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import ComData
from utils.log import LogHandler

import time

class DataManager:
    def __init__(self):
        self.data_define_obj = DataDefine()
        self.data_dict = {}
        self.base_data()
        # 日志对象

        self.log = LogHandler('log')
        self.server_log = LogHandler('server_data')
        self.com_log = LogHandler('com_data')
        # 服务器数据收发对象
        self.server_data_obj = ServerData()
        # 串口数据收发对象
        self.com_data_obj = ComData()


    def base_data(self):
        """
        获取基础数据
        """
        self.data_dict.update({'statistics_data': self.data_define_obj.statistics_data()})
        self.data_dict.update({'status_data': self.data_define_obj.status_data()})
        self.data_dict.update({'meteorological_data': self.data_define_obj.meteorological_data()})
        self.data_dict.update({'water_quality_data': self.data_define_obj.water_quality_data()})


    def update_data(self):
        """
        更新数据
        """


    def send(self,method,data,topic='test',qos=0):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http','mqtt','com'],'method error not in http mqtt com'
        if method == 'http':
            self.server_data_obj.send_server_http_data(data=data)
        elif method=='mqtt':
            self.server_data_obj.send_server_mqtt_data(data=data,topic=topic,qos=qos)
        elif method =='com':
            self.com_data_obj.send_data(data=data)

    def get(self,method):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
        if method == 'http':
            data = self.server_data_obj.get_server_http_data()
        elif method == 'mqtt':
            data = self.server_data_obj.get_server_mqtt_data()
        elif method == 'com':
            data = self.com_data_obj.get_data()


if __name__ == '__main__':
    obj = DataManager()

    while True:
        move_direction = obj.server_data_obj.mqtt_send_get_obj.move_direction
        obj.send(method='com',data=move_direction)
        obj.log.info('move_direction: %f'%(float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位

