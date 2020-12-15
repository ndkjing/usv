"""
管理数据收发
"""
import time
import json

from dataGetSend.data_define import DataDefine
from dataGetSend import data_define
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData
from utils.log import LogHandler
from baiduMap import baidu_map
import  config


class DataManager:
    def __init__(self,ship_code=None):
        self.data_define_obj = DataDefine()
        # 日志对象
        self.logger = LogHandler('data_manager_log')
        self.server_log = LogHandler('server_data')
        self.com_log = LogHandler('com_data')
        # mqtt服务器数据收发对象
        self.server_data_obj = ServerData(self.server_log,topics=self.data_define_obj.topics)
        # 串口数据收发对象
        self.com_data_obj = SerialData('com8', 115200, timeout=1/config.com2pi_interval,logger=self.com_log)
        # 船最终控制移动方向
        self.ship_move_direction=str(360)
        # 船当前朝向
        self.ship_current_direction = -1

    # 读取函数会阻塞 必须使用线程
    def get_com_data(self):
        while True:
            com_data_read = self.com_data_obj.readline()
            ## 解析串口发送过来的数据
            if com_data_read is None:
                continue
            com_data_list = com_data_read.split(',')
            # 角度，TDS，温度，经度，纬度，距离1，距离2
            # 当前朝向
            self.ship_current_direction = com_data_list[0].split('AAA')[1]
            # TDO
            self.data_define_obj.water['TD'] = com_data_list[1]
            ## 水质数据
            # 水温
            self.data_define_obj.water['wt'] = com_data_list[2]
            # 经纬度
            if float(com_data_list[3]) < 0 or float(com_data_list[4]) < 0:
                self.data_define_obj.status['current_lng_lat'] = None
            elif float(com_data_list[3]) > 180 or float(com_data_list[4]) >80:
                self.data_define_obj.status['current_lng_lat'] = None
            else:
                self.data_define_obj.status['current_lng_lat'] = [float(com_data_list[3]),float(com_data_list[4])]
            # 左右侧的超声波检测距离
            l_distance,r_distance = com_data_list[5],com_data_list[6]
            self.logger.info({'ship_current_direction':self.ship_current_direction,
                              'TD':self.data_define_obj.water['TD'],
                              'water_temperature':self.data_define_obj.water['wt'],
                             'current_lng_lat': self.data_define_obj.status['current_lng_lat'],
                            'r_distance':r_distance,
                            'l_distance':l_distance,})

    # 发送函数会阻塞 必须使用线程
    def send_com_data(self):
        # 0 自动  1手动
        manul_or_auto = 0
        while True:
            # 判断当前是手动控制还是自动控制
            if len(self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status)==0:
                manul_or_auto = 1
            elif self.data_define_obj.status['current_lng_lat']==None:
                manul_or_auto = 1
            else:
                for i in self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status:
                    if i==0:
                        manul_or_auto=0
                        break
                    manul_or_auto=1
            # 手动模式使用用户给定角度
            if manul_or_auto==1:
                self.com_data_obj.send_data('A%sZ'%(self.server_data_obj.mqtt_send_get_obj.control_move_direction))
                self.logger.info('control_move_direction: '+str(self.server_data_obj.mqtt_send_get_obj.control_move_direction))
            # 自动模式计算角度
            elif manul_or_auto==0:
                # 计算目标角度
                target_degree = baidu_map.get_degree(self.data_define_obj.status['current_lng_lat'][0],
                                    self.data_define_obj.status['current_lng_lat'][1],
                                    self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status[-1][0],
                                    self.server_data_obj.mqtt_send_get_obj.target_lng_lat_status[-1][1])
                # 偏差角度
                delta_degree = target_degree-self.ship_current_direction
                auto_move_direction = 360
                # 判断移动方向
                if delta_degree>=0:
                    if delta_degree < 45 or delta_degree >= 315:
                        auto_move_direction = 0
                    elif delta_degree >= 45 and delta_degree < 135:
                        auto_move_direction = 90
                    elif delta_degree >= 135 and delta_degree<225:
                        auto_move_direction = 180
                    elif delta_degree >= 225 and delta_degree < 315:
                        auto_move_direction = 270
                else:
                    if abs(delta_degree) < 45 or abs(delta_degree) >= 315:
                        auto_move_direction = 0
                    elif abs(delta_degree) >= 45 and abs(delta_degree) < 135:
                        auto_move_direction = 270
                    elif abs(delta_degree) >= 135 and abs(delta_degree)<225:
                        auto_move_direction = 180
                    elif abs(delta_degree) >= 225 and abs(delta_degree) < 315:
                        auto_move_direction = 90

                self.com_data_obj.send_data('A%sZ' % (str(auto_move_direction)))
                self.logger.info('auto_move_direction: ' + str(auto_move_direction))

            time.sleep(1/config.pi2com_interval)

    # 读取函数会阻塞 必须使用线程
    # 发送mqtt状态数据和检测数据
    def send_mqtt_data(self):
        while True:
            status_data = self.data_define_obj.status
            status_data.update({'mapId':self.data_define_obj.pool_code})
            detect_data = self.data_define_obj.detect
            detect_data.update({'mapId': self.data_define_obj.pool_code})
            # 替换键
            for k_all,v_all in data_define.name_mappings.items():
                for old_key,new_key in v_all.items():
                    # pop_value = detect_data[k_all].pop(old_key)
                    if detect_data[k_all].get(old_key) ==None:
                        continue
                    detect_data[k_all].update({new_key:detect_data[k_all].pop(old_key)})

            # 向mqtt发送数据
            if self.data_define_obj.pool_code=='':
                self.send(method='mqtt',topic='status_data_%s'%(self.data_define_obj.ship_code), data=status_data, qos=1)
                self.logger.info({"发送状态数据": status_data})
            else:
                self.send(method='mqtt',topic='status_data_%s'%(self.data_define_obj.ship_code), data=status_data, qos=1)
                self.send(method='mqtt',topic='detect_data_%s'%(self.data_define_obj.ship_code), data=detect_data, qos=1)
                self.logger.info({"发送检测数据": detect_data})
                self.logger.info({"发送状态数据": status_data})
                # http发送检测数据给服务器
                self.send(method='http', data=detect_data,
                                                    url='http://192.168.8.13:8009/admin/xxl/data/save',
                                                    http_type='POST')
            time.sleep(1/config.pi2mqtt_interval)


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



if __name__ == '__main__':
    obj = DataManager()

    while True:
        move_direction = obj.server_data_obj.mqtt_send_get_obj.move_direction
        obj.send(method='com',data=move_direction)
        obj.logger.info('move_direction: %f'%(float(move_direction)))
        time.sleep(2)
        # 等待一段时间后归位

