"""
管理数据收发
"""
from dataGetSend.data_define import DataDefine
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import ComData
from utils.log import LogHandler
from config import map_data_path
import time
import json

class DataManager:
    def __init__(self,ship_code=None):
        self.data_define_obj = DataDefine(ship_code)
        self.data_dict = {}
        self.base_data()
        # 日志对象
        self.log = LogHandler('log')
        self.server_log = LogHandler('server_data')
        self.com_log = LogHandler('com_data')
        # 服务器数据收发对象
        self.server_data_obj = ServerData(self.server_log)
        # 串口数据收发对象
        self.com_data_obj = ComData(self.com_log)

        # 保存地图数据路径


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


    def send(self,method,data,topic='test',qos=0,http_type='POST',url=''):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http','mqtt','com'],'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type,data,url)
            self.log.info({'status_code':return_data.status_code})
            # 如果是POST返回的数据，添加数据到地图数据保存文件中
            if http_type == 'POST':
                content_data = json.loads(return_data.content)
                self.server_log.info({'content_data success':content_data["success"]})
                if not content_data["success"]:
                    self.server_log.error('POST请求失败')
                # POST 返回湖泊ID
                pool_id = content_data['data']['id']
                return pool_id
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.server_log.error('GET请求失败')
                save_data = content_data["data"]["mapList"]
                return save_data

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

