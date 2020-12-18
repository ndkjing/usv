"""
定义数据类型
"""
# from utils import data_generate

from uuid import uuid4
import random
import copy
import time

import config

# 本地名称与发送名称映射
name_mappings = {
    # 'pool_id':'mapId',
    # 'ship_code':'deviceId',
    'water':
        {
        'pH':'ph',
        'DO':'doDo',
        'COD':'cod',
        'EC':'ec',
        'TD':'td',
        'NH3_NH4':'nh3Nh4',
        'TN':'tn',
        'TP':'tp'},

    'weather':
        {
        'wind_speed': 'windSpeed',
       'wind_direction': 'windDirection',
       'rainfall': 'rainfall',
       'illuminance': 'illuminance',
       'temperature': 'temperature',
       'humidity': 'humidity',
       'pm25': 'pm25',
       'pm10': 'pm10',
        }
}



# 生成船号
def get_ship_code():
    return str(uuid4())

# 剩余电量
def get_dump_energy():
    return init_dump_energy - round((time.time() - init_time)/60, 1)

# 当前经纬度
def get_current_lng_lat(init_lng_lat):
    if random.random()>0.5:
        init_lng_lat = [round(init_lng_lat[0] - round(random.random(), 1)/1000,6),
                        round(init_lng_lat[1],6)]
    else:
        init_lng_lat = [round(init_lng_lat[0],6),
                        round(init_lng_lat[1] - round(random.random(),1)/1000,6)]

    return init_lng_lat


# 船号

# if ship_code == None:
#     ship_code = get_ship_code()

# 湖号
pool_code = None
# 电量
init_dump_energy=100
# 初始时间
init_time = time.time()
# 经纬度
init_lng_lat = [114.39458,30.547412]



init_ststus_data={"dump_energy": 0.000,
                   "current_lng_lat": [100.155214,36.993676],
                   "liquid_level": 0.000,
                   "b_leakage": False,
                   "direction": 90.0,
                   "speed": 0.000,
                   "attitude_angle": [0,0,90.0],
                   "b_online": True,
                   "b_homing": False,
                   "charge_energy": 0.000,
                   "sampling_depth": 0.000,
                   "ship_code": config.ship_code,
                   "pool_code":pool_code,
                   "data_flow": 0.000,
                   "sampling_count": 0,
                   "capicity": 0.00
                   }

init_detect_data={
    "water":{
        "wt":0.000,
        "pH": 0.000,
       "DO": 0.000,
       "COD": 0.000,
       "EC": 0.000,
       "TD": 0.000,
       "NH3_NH4": 0.000,
       "TN": 0.000,
       "TP": 0.000,
       },
    "weather":{"wind_speed": 0.000,
                       "wind_direction": "",
                       "rainfall": 0.000,
                       "illuminance": 0.000,
                       "temperature": 0.000,
                       "humidity": 0.000,
                       "pm25": 0.000,
                       "pm10": 0.000,
                       }
}
# 风向定义['东北','正北','西北','正西','西南','正南','东南','正东']
wind_direction=['315','0','45','90','135','180','225','270']


# 返回状态数据
def status_data(data_define_obj=None):
    """
    返回检测数据
    :param data_define_obj: 该对象不为空表示用数据内部部位None的数据替换随机数据
    :return: 检测数据字典
    """
    if data_define_obj == None:
        return_dict = copy.deepcopy(init_ststus_data)
        return_dict.update({'dump_energy': get_dump_energy()})
        return_dict.update({'current_lng_lat': get_current_lng_lat(init_lng_lat)})
        return_dict.update({'sampling_depth': round(random.random(), 2)})
        return_dict.update({"deviceId": config.ship_code})
    else:
        return_dict = copy.deepcopy(init_ststus_data)
        for k, v in data_define_obj.status:
            if v is not None:
                return_dict.update({k, v})
    return return_dict


# 返回检测数据
def detect_data(data_define_obj=None):
    """
    返回检测数据
    :param data_define_obj: 该对象不为空表示用数据内部部位None的数据替换随机数据
    :return: 检测数据字典
    """
    if data_define_obj == None:
        init_detect_data["weather"].update({"wind_speed": round(round(random.random(), 2) * 10, 1)})
        init_detect_data["weather"].update(
            {"wind_direction": wind_direction[random.randint(0, len(wind_direction) - 1)]})
        init_detect_data["weather"].update({"rainfall": round(round(random.random(), 2) * 10, 1)})
        init_detect_data["weather"].update({"illuminance": round(round(random.random(), 2) * 10, 2)})
        init_detect_data["weather"].update({"temperature": random.randint(0, 40)})
        init_detect_data["weather"].update({"humidity": random.randint(40, 90)})
        init_detect_data["weather"].update({"pm25": random.randint(0, 20)})
        init_detect_data["weather"].update({"pm10": random.randint(20, 40)})

        init_detect_data["water"].update({"pH": random.randint(50, 90) / 10.0})
        init_detect_data["water"].update({"DO": random.randint(20, 100) / 10.0})
        init_detect_data["water"].update({"COD": random.randint(50, 400) / 10.0})
        init_detect_data["water"].update({"TD": random.randint(1, 10) / 10.0})
        init_detect_data["water"].update({"NH3_NH4": random.randint(2, 100) / 100.0})
        init_detect_data["water"].update({"TN": random.randint(10, 200) / 10.0})
        init_detect_data["water"].update({"TP": random.randint(0, 2) / 10.0})
        init_detect_data["water"].update({"EC": random.randint(480, 600) / 10.0})

        init_detect_data.update({"deviceId": config.ship_code})
    else:
        return_dict = copy.deepcopy(init_detect_data)
        for k, v in data_define_obj.water:
            if v is not None:
                return_dict['water'].update({k, v})
        for k, v in data_define_obj.weather:
            if v is not None:
                return_dict['weather'].update({k, v})

    return init_detect_data

class DataDefine:
    def __init__(self):
        """
        数据定义对象
        """
        self.ship_code = config.ship_code

        self.topics = (('control_data_%s' % (self.ship_code), 0),
                         ('path_confirm_%s' % (self.ship_code), 0),
                         ('user_lng_lat_%s' % (self.ship_code), 0))
        self.pool_code = ''
        self.water = self.water_data()
        self.weather = self.weather_data()

        self.status = {"dump_energy": None,
                       "current_lng_lat": None,
                       "liquid_level": None,
                       "b_leakage": None,
                       "direction": None,
                       "speed": None,
                       "attitude_angle": None,
                       "b_online": True,
                       "b_homing": None,
                       "charge_energy": None,
                       "sampling_depth": None,
                       "ship_code": self.ship_code,
                       "pool_code":None,
                       "data_flow": None,
                       "sampling_count": None,
                       "capicity": None
                       }

        self.control = self.control_data()
        self.detect = self.detect_data()

    # 水质数据
    def water_data(self):
        """
        :param value_from 数据来源
        解释　　　　　字典键名称　　数据类型　　
        酸碱度       pH　　　　　 浮点数　　
        溶解氧　　　　DO          浮点数　　
        化学需氧量   COD　　　　  浮点数　　
        电导率      EC　　　   　浮点数　　
        浊度       TD　　　　   浮点数　　
        氨氮       NH3_NH4　　　浮点数　　
        总氮       TN　　　　   浮点数　　
        总磷       TP　　　   　浮点数　　
        """
        return_dict = {'wt':None,
                        "pH": None,
                       "DO": None,
                       "COD": None,
                       "EC": None,
                       "TD": None,
                       "NH3_NH4": None,
                       "TN": None,
                       "TP": None,
                       }
        return return_dict

    # 气象数据
    def weather_data(self):
        """
        解释　　　　　字典键名称　　数据类型　　
        风速    wind_speed    浮点数
        风向    wind_direction　　字符串：东南西北，东北，东南，西北，西南等
        降雨量  rainfall　　　　　浮点数
        光照度   illuminance　　　浮点数
        温度    temperature　　　　浮点数
        湿度    humidity　　　　浮点数
        PM2.5   pm25　浮点数
        PM10    pm10　浮点数
        """

        return_dict = {"wind_speed": None,
                       "wind_direction": None,
                       "rainfall": None,
                       "illuminance": None,
                       "temperature": None,
                       "humidity":None,
                       "pm25": None,
                       "pm10": None,
                       }
        return return_dict

    # 控制数据
    def control_data(self):
        """
        地图湖泊中间一个点经纬度坐标
        解释　　　　　字典键名称　   　数据类型　
        前进方向    move_direcion  　float  0：正前方：90 左 180：后 270：右  360:停止
        执行采样    b_sampling      整数枚举  0：不检测  1：检测
        执行抽水    b_draw          整数枚举  0：不抽水  1：抽水
        """
        return_dict = {'move_direction':360,
                     'b_sampling':0,
                     'b_draw':0,
        }
        return return_dict

    # 状态数据
    def status_data(self):
        """
        解释　　　　　字典键名称　　数据类型　　
        剩余电量      dump_energy   浮点数（0.0--1.0 百分比率）
        当前经纬度   current_lng_lat  列表［浮点数，浮点数］（［经度，纬度］）
        液位        liquid_level    浮点数（采样深度液位）
        漏水　　　　　b_leakage      布尔值（船舱内部是否漏水）
        船头方向　　　direction      浮点数（０.0－３６０　单位度）
        速度　　　　　speed　　　　　　浮点数　（ｍ/s）
        姿态        attitude_angle   列表[r,p,y]  (ｒ,p,y 为横滚，俯仰，偏航角度　取值均为浮点数０.0－３６０)
        在线／离线  　b_online      布尔值（船是否在线）
        归位状态：　　b_homing　　    布尔值    （是否返回充电桩）　　　
        充电状态：　　charge_energy　　浮点数（0.0--1.0  百分比率　在充电状态下的充电电量）　　　
        采样深度：　　sampling_depth　　浮点数(单位:ｍ)
        船号：　　　　ship_code      字符串（船出厂编号）
        湖泊编号     pool_code      字符串（湖泊编号）
        4G卡流量：　　data_flow       浮点数（单位：ＭＢ）
        采样量：　　　sampling_count　　整数（采样点的个数）
        船舱容量状态：　　capicity　　　浮点数（0.0--1.0  百分比率　垃圾收集船内部容量）
        """
        return_dict = {"dump_energy": None,
                       "current_lng_lat": None,
                       "liquid_level": None,
                       "b_leakage": None,
                       "direction": None,
                       "speed": None,
                       "attitude_angle": None,
                       "b_online": True,
                       "b_homing": None,
                       "charge_energy": None,
                       "sampling_depth": None,
                       "ship_code": self.ship_code,
                       "pool_code":None,
                       "data_flow": None,
                       "sampling_count": None,
                       "capicity": None
                       }
        return return_dict


    def statistics_data(self):
        """
        解释　　　　　字典键名称　　    数据类型　　
        工作时长   work_time  　　  浮点数(单位：秒)
        工作距离　　work_distance   浮点数（单位：米）
        采样点经纬度　sampling_lng_lat  列表［浮点数，浮点数］（数据说明［经度，纬度］）
        """
        return_dict = {"work_time": None,
                       "work_distance": None,
                       "sampling_lng_lat": None,
                       }
        return return_dict

    # 返回检测数据
    def detect_data(self,data_define_obj=None):
        """
        返回检测数据
        :param data_define_obj: 该对象不为空表示用数据内部部位None的数据替换随机数据
        :return: 检测数据字典
        """
        return_detect_data={}
        return_detect_data['weather'] = self.weather
        return_detect_data['water'] = self.water
        return_detect_data['deviceId'] = config.ship_code

        return return_detect_data
        # if data_define_obj == None:
        #     init_detect_data["weather"].update({"wind_speed": round(round(random.random(), 2) * 10, 1)})
        #     init_detect_data["weather"].update(
        #         {"wind_direction": wind_direction[random.randint(0, len(wind_direction) - 1)]})
        #     init_detect_data["weather"].update({"rainfall": round(round(random.random(), 2) * 10, 1)})
        #     init_detect_data["weather"].update({"illuminance": round(round(random.random(), 2) * 10, 2)})
        #     init_detect_data["weather"].update({"temperature": random.randint(0, 40)})
        #     init_detect_data["weather"].update({"humidity": random.randint(40, 90)})
        #     init_detect_data["weather"].update({"pm25": random.randint(0, 20)})
        #     init_detect_data["weather"].update({"pm10": random.randint(20, 40)})
        #
        #     init_detect_data["water"].update({"pH": random.randint(50, 90) / 10.0})
        #     init_detect_data["water"].update({"DO": random.randint(20, 100) / 10.0})
        #     init_detect_data["water"].update({"COD": random.randint(50, 400) / 10.0})
        #     init_detect_data["water"].update({"TD": random.randint(1, 10) / 10.0})
        #     init_detect_data["water"].update({"NH3_NH4": random.randint(2, 100) / 100.0})
        #     init_detect_data["water"].update({"TN": random.randint(10, 200) / 10.0})
        #     init_detect_data["water"].update({"TP": random.randint(0, 2) / 10.0})
        #     init_detect_data["water"].update({"EC": random.randint(480, 600) / 10.0})
        #
        #     init_detect_data.update({"deviceId": ship_code})
        # else:
        #     return_dict = copy.deepcopy(init_detect_data)
        #     for k, v in data_define_obj.water:
        #         if v is not None:
        #             return_dict['water'].update({k, v})
        #     for k, v in data_define_obj.weather:
        #         if v is not None:
        #             return_dict['weather'].update({k, v})
        #
        # return init_detect_data

if __name__ == '__main__':
    # 简单测试获取数据
    # obj = DataDefine()
    # data_dict = {}
    # data_dict.update({'statistics_data': obj.statistics_data()})
    # data_dict.update({'status_data':obj.status_data()})
    # data_dict.update({'weather':obj.weather})
    # data_dict.update({'water':obj.water})
    print(detect_data())

