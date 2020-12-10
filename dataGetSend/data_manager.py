"""
管理数据收发
"""
from dataGetSend.data_define import DataDefine
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData
from utils.log import LogHandler
import  config
import time
import json

class DataManager:
    def __init__(self,ship_code=None):
        self.data_define_obj = DataDefine(ship_code)
        # 日志对象
        self.logger = LogHandler('data_manager_log')
        self.server_log = LogHandler('server_data')
        self.com_log = LogHandler('com_data')
        # mqtt服务器数据收发对象
        self.server_data_obj = ServerData(self.server_log,topics=self.data_define_obj.topics)
        # 串口数据收发对象
        self.com_data_obj = SerialData('com8', 9600, timeout=1/config.com2pi_interval,logger=self.com_log)
        # 移动方向
        self.ship_move_direction=360

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        while True:
            com_data_read = self.com_data_obj.read_Line()
            ## 解析串口发送过来的数据
            # 经纬度
            lng_lat = com_data_read
            self.data_define_obj.status['current_lng_lat'] = lng_lat
            # 左右侧的超声波检测距离
            l_distance,r_distance = com_data_read
            ## 水质数据
            # 水温
            water_temperature = com_data_read
            # TDO
            water_td = com_data_read
            self.data_define_obj.water['TD'] = water_td


    # 读取函数会阻塞 必须使用线程
    def send_com_data(self):
        while True:
            self.com_data_obj.send_data('A%sZ'%(self.ship_move_direction))
            # 打印传输给单片机的控制数据
            self.logger.info('move_direction: '+'A%sZ'%(self.ship_move_direction))
            time.sleep(1/config.pi2com_interval)


    def send(self,method,data,topic='test',qos=0,http_type='POST',url=''):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http','mqtt','com'],'method error not in http mqtt com'
        if method == 'http':
            return_data = self.server_data_obj.send_server_http_data(http_type,data,url)
            self.logger.info({'请求 url':url})
            self.logger.info({'status_code':return_data.status_code})
            # 如果是POST返回的数据，添加数据到地图数据保存文件中
            if http_type == 'POST' and r'map/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.info({'map/save content_data success':content_data["success"]})
                if not content_data["success"]:
                    self.logger.error('POST请求发送地图数据失败')
                # POST 返回湖泊ID
                pool_id = content_data['data']['id']
                return pool_id
            # http发送检测数据给服务器
            elif http_type == 'POST' and r'data/save' in url:
                content_data = json.loads(return_data.content)
                self.logger.info({'data/save content_data success': content_data["success"]})
                if not content_data["success"]:
                    self.logger.error('POST发送检测请求失败')
            elif http_type == 'GET' and r'device/binding' in url:
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                save_data = content_data["data"]
                return save_data
            else:
                # 如果是GET请求，返回所有数据的列表
                content_data = json.loads(return_data.content)
                if not content_data["success"]:
                    self.logger.error('GET请求失败')
                save_data = content_data["data"]["mapList"]
                return save_data
        elif method=='mqtt':
            self.server_data_obj.send_server_mqtt_data(data=data,topic=topic,qos=qos)


    def get(self,method):
        """
        :param method 获取数据方式　http mqtt com
        """
        assert method in ['http', 'mqtt', 'com'], 'method error not in http mqtt com'
        if method == 'http':
            data = self.server_data_obj.get_server_http_data()
        elif method == 'mqtt':
            data = self.server_data_obj.get_server_mqtt_data()


if __name__ == '__main__':
    obj = DataManager()

    while True:
        move_direction = obj.server_data_obj.mqtt_send_get_obj.move_direction
        obj.send(method='com',data=move_direction)
        obj.logger.info('move_direction: %f'%(float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位

