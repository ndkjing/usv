"""
生成随机数据
"""
from uuid import uuid4
import random
import copy
import time

"""
        解释　　　　　字典键名称　　数据类型　　 范围（默认保留一位小数）
        酸碱度       pH　　　　　 浮点数　　  1-14
        溶解氧　　　　DO          浮点数　　  2-10
        化学需氧量   COD　　　　  浮点数　　  5-40
        电导率      EC　　　   　浮点数　　  （48-60）*10e-4
        浊度       TD　　　　   浮点数　　   0.1-1
        氨氮       NH3_NH4　　　浮点数　　  0.02-1
        总氮       TN　　　　   浮点数　　  1-20
        总磷       TP　　　   　浮点数　　  0-0.2
        return_dict = {"pH": 0.000,
                       "DO": 0.000,
                       "COD": 0.000,
                       "EC": 0.000,
                       "TD": 0.000,
                       "NH3_NH4": 0.000,
                       "TN": 0.000,
                       "TP": 0.000,
                       }
"""

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
        
        return_dict = {"wind_speed": 0.000,
               "wind_direction": "",
               "rainfall": 0.000,
               "illuminance": 0.000,
               "temperature": 0.000,
               "humidity": 0.000,
               "pm25": 0.000,
               "pm10": 0.000,
               }
"""
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
        
        return_dict = {"dump_energy": 0.000,
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
                       "ship_code": self.ship_code,
                       "pool_code":self.pool_code,
                       "data_flow": 0.000,
                       "sampling_count": 0,
                       "capicity": 0.00
                       }
        """


# 生成船号
def get_ship_code():
    return str(uuid4())

# 剩余电量
def get_dump_energy():
    return init_dump_energy - round((time.time() - init_time)/60, 1)

# 当前经纬度
def get_current_lng_lat(init_lng_lat):
    if random.random()>0.5:
        init_lng_lat = [init_lng_lat[0] - round(random.random(), 1)/1000,init_lng_lat[1]]
    else:
        init_lng_lat = [init_lng_lat[0],init_lng_lat[1] - round(random.random(),1)/1000]

    return init_lng_lat


# 船号
ship_code = '3c50f4c3-a9c1-4872-9f18-883af014380b'
if ship_code == None:
    ship_code = get_ship_code()

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
                   "ship_code": ship_code,
                   "pool_code":pool_code,
                   "data_flow": 0.000,
                   "sampling_count": 0,
                   "capicity": 0.00
                   }

init_detect_data={
    "water_quality_data":{"pH": 0.000,
                       "DO": 0.000,
                       "COD": 0.000,
                       "EC": 0.000,
                       "TD": 0.000,
                       "NH3_NH4": 0.000,
                       "TN": 0.000,
                       "TP": 0.000,
                       },
    "meteorological_data":{"wind_speed": 0.000,
                       "wind_direction": "",
                       "rainfall": 0.000,
                       "illuminance": 0.000,
                       "temperature": 0.000,
                       "humidity": 0.000,
                       "pm25": 0.000,
                       "pm10": 0.000,
                       }
}
#
# 风向定义['东北','正北','西北','正西','西南','正南','东南','正东']
wind_direction=['东北','正北','西北','正西','西南','正南','东南','正东']


# 返回状态数据
def status_data():
    return_dict = copy.deepcopy(init_ststus_data)
    return_dict.update({'dump_energy':get_dump_energy()})
    return_dict.update({'current_lng_lat':get_current_lng_lat(init_lng_lat)})
    return_dict.update({'sampling_depth':round(random.random(),2)})

    return_dict.update({"deviceId": ship_code})
    return return_dict

def detect_data():
    init_detect_data["meteorological_data"].update({"wind_speed":round(random.random(),2)*10})
    init_detect_data["meteorological_data"].update({"wind_direction":wind_direction[random.randint(0,len(wind_direction)-1)]})
    init_detect_data["meteorological_data"].update({"rainfall":round(random.random(),2)*10})
    init_detect_data["meteorological_data"].update({"illuminance":round(random.random(),2)*10})
    init_detect_data["meteorological_data"].update({"temperature":random.randint(0,40)})
    init_detect_data["meteorological_data"].update({"humidity":random.randint(40,90)})
    init_detect_data["meteorological_data"].update({"pm25":random.randint(0,20)})
    init_detect_data["meteorological_data"].update({"pm10":random.randint(20,40)})

    init_detect_data["water_quality_data"].update({"pH":random.randint(50,90)/10.0})
    init_detect_data["water_quality_data"].update({"DO":random.randint(20,100)/10.0})
    init_detect_data["water_quality_data"].update({"COD":random.randint(50,400)/10.0})
    init_detect_data["water_quality_data"].update({"TD":random.randint(1,10)/10.0})
    init_detect_data["water_quality_data"].update({"NH3_NH4":random.randint(2,100)/100.0})
    init_detect_data["water_quality_data"].update({"TN":random.randint(10,200)/10.0})
    init_detect_data["water_quality_data"].update({"TP":random.randint(0,2)/10.0})
    init_detect_data["water_quality_data"].update({"EC":random.randint(480,600)/10.0})

    init_detect_data.update({"deviceId":ship_code})


    return init_detect_data

if __name__ == '__main__':
    print(ship_code)
    while 1:
        print(status_data())
        print(detect_data())
        time.sleep(2)
    # print(random.randint(0,1))

