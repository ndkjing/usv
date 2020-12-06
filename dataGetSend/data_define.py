"""
定义数据类型
"""


class DataDefine:

    def __init__(self):
        ## 水质
        self.pH = 0.00
        self.DO = 0.00
        self.COD = 0.00
        self.EC = 0.00
        self.TD = 0.00
        self.NH3_NH4 = 0.00
        self.TN = 0.00
        self.TP = 0.00

        ## 气象
        self.wind_speed = 0.000
        self.wind_direction = ""
        self.rainfall = 0.000
        self.illuminance = 0.000
        self.temperature = 0.000
        self.humidity = 0.000
        self.pm25 = 0.000
        self.pm10 = 0.000

        ## 控制
        # 前后左右移动控制键　0 为前进　90 度向左　　180 向后　　270向右　　
        self.move_direction = -1
        # 测量控制位　false为不采样　true为采样
        self.b_sampling = False
        # 抽水控制位
        self.b_draw = False

        ## 状态数据
        self.dump_energy = 0.000
        self.current_lng_lat = [100.155214, 36.993676]
        self.liquid_level = 0.000
        self.b_leakage = False
        self.direction = 90.0
        self.speed = 0.000
        self.attitude_angle = [0, 0, 90.0]
        self.b_online = True
        self.b_homing = False
        self.charge_energy = 0.000
        self.sampling_depth = 0.000
        self.ship_code = "S9999"
        self.data_flow = 0.000
        self.sampling_count = 0
        self.capicity = 0.00

        ## 统计数据
        self.work_time=.000
        self.work_distance=0.000
        self.sampling_lng_lat=[[100.155214, 36.993676], [100.837558, 37.117129]]



    # 水质数据
    def water_quality_data(self):
        """
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
        return_dict = {"pH": 0.000,
                       "DO": 0.000,
                       "COD": 0.000,
                       "EC": 0.000,
                       "TD": 0.000,
                       "NH3_NH4": 0.000,
                       "TN": 0.000,
                       "TP": 0.000,
                       }
        return return_dict


    # 气象数据
    def meteorological_data(self):
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

        return_dict = {"wind_speed": 0.000,
                       "wind_direction": "",
                       "rainfall": 0.000,
                       "illuminance": 0.000,
                       "temperature": 0.000,
                       "humidity": 0.000,
                       "pm25": 0.000,
                       "pm10": 0.000,
                       }
        return return_dict


    # 控制数据
    def control_data(self):
        """
        地图湖泊中间一个点经纬度坐标
        解释　　　　　字典键名称　　数据类型　　
        """


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
        4G卡流量：　　data_flow       浮点数（单位：ＭＢ）
        采样量：　　　sampling_count　　整数（采样点的个数）
        船舱容量状态：　　capicity　　　浮点数（0.0--1.0  百分比率　垃圾收集船内部容量）
        """
        return_dict = {"dump_energy": 0.000,
                       "current_lng_lat": [100.155214, 36.993676],
                       "liquid_level": 0.000,
                       "b_leakage": False,
                       "direction": 90.0,
                       "speed": 0.000,
                       "attitude_angle": [0, 0, 90.0],
                       "b_online": True,
                       "b_homing": False,
                       "charge_energy": 0.000,
                       "sampling_depth": 0.000,
                       "ship_code": "S9999",
                       "data_flow": 0.000,
                       "sampling_count": 0,
                       "capicity": 0.00
                       }

        return return_dict


    def statistics_data(self):
        """
        解释　　　　　字典键名称　　    数据类型　　
        工作时长   work_time  　　  浮点数(单位：秒)
        工作距离　　work_distance   浮点数（单位：米）
        采样点经纬度　sampling_lng_lat  列表［浮点数，浮点数］（数据说明［经度，纬度］）
        """
        return_dict = {"work_time": 0.000,
                       "work_distance": 0.000,
                       "sampling_lng_lat": [[100.155214, 36.993676], [100.837558, 37.117129]],
                       }
        return return_dict

    def get_all_data(self):
        """
        获取所有数据
        """
        data_dict = {}
        data_dict.update({'statistics_data': self.statistics_data()})  # http
        data_dict.update({'status_data': self.status_data()})  # mqtt
        data_dict.update({'control_data': self.control_data()})  # mqtt
        data_dict.update({'meteorological_data': self.meteorological_data()})  # http
        data_dict.update({'water_quality_data': self.water_quality_data()})  # http
        return data_dict


if __name__ == '__main__':
    # 简单测试获取数据
    obj = DataDefine()
    data_dict = obj.get_all_data()
    print(data_dict)
