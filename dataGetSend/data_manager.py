"""
管理数据收发
"""
from dataGetSend.data_define import DataDefine
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import ComData
from utils.log import LogHandler


class DataManager:
    def __init__(self):
        self.data_define_obj = DataDefine()
        self.data_dict = {}
        self.log = LogHandler('server_data')
        self.server_data_obj = ServerData()
        self.com_data_obj = ComData()

    def update_data(self):
        self.data_dict.update({'statistics_data': self.data_define_obj.statistics_data()})
        self.data_dict.update({'status_data': self.data_define_obj.status_data()})
        self.data_dict.update({'meteorological_data': self.data_define_obj.meteorological_data()})
        self.data_dict.update({'water_quality_data': self.data_define_obj.water_quality_data()})

    def send(self):
        pass

    def get(self):
        pass







