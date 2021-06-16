import numpy as np
import enum
from utils import crawl_water_data
import config

min_max_wt = (0, 35)
wt = [
    29.8,
    32.5,
    30.4,
    29.8,
    30.8,
    30.5,
    27.4,
    26.6,
    31.8,
    29.6,
    30.7,
    29.5,
    30.0,
    30.4,
    31.1,
    30.5,
    26.8
]
min_max_EC = (140, 468)
EC = [
    465.7,
    183.9,
    141.7,
    176.4,
    205.1,
    156.1,
    298.8,
    292.3,
    212.4,
    194.1,
    273.6,
    147.4,
    172.9,
    514.3,
    468.0,
    403.3,
    273.1
]

min_max_pH = (6, 9)
pH = [7.23,
      8.25,
      8.18,
      8.29,
      8.30,
      8.58,
      7.76,
      6.20,
      7.18,
      7.53,
      8.01,
      7.40,
      7.39,
      6.73,
      7.28,
      6.99,
      7.69]

min_max_DO = (2, 7.5)
DO = [
    3.28,
    12.39,
    8.14,
    7.49,
    8.20,
    7.77,
    7.49,
    6.57,
    7.35,
    7.88,
    9.02,
    6.40,
    7.93,
    2.16,
    3.04,
    2.91,
    6.85
]
min_max_TD = (2.5, 50)
TD = [
    44.2,
    34.0,
    9.4,
    2.8,
    13.0,
    7.3,
    9.1,
    29.6,
    16.1,
    26.7,
    23.4,
    27.2,
    12.1,
    32.4,
    180.0,
    30.9,
    23.4
]
min_max_NH3_NH4 = (0.15, 2.0)
NH3_NH4 = [
    0.170,
    0.025,
    0.025,
    0.025,
    0.025,
    0.025,
    0.056,
    0.025,
    0.233,
    0.212,
    0.063,
    0.130,
    0.144,
    0.660,
    0.025,
    0.226,
    0.130
]

water_data_dict = {}
water_data_dict.update(
    {config.WaterType.wt: {'min_data': min_max_wt[0], 'max_data': min_max_wt[1], 'data': wt, 'keep_valid_decimals': 1}})
water_data_dict.update(
    {config.WaterType.pH: {'min_data': min_max_pH[0], 'max_data': min_max_pH[1], 'data': pH, 'keep_valid_decimals': 2}})
water_data_dict.update(
    {config.WaterType.EC: {'min_data': min_max_EC[0], 'max_data': min_max_EC[1], 'data': EC, 'keep_valid_decimals': 1}})
water_data_dict.update(
    {config.WaterType.DO: {'min_data': min_max_DO[0], 'max_data': min_max_DO[1], 'data': DO, 'keep_valid_decimals': 2}})
water_data_dict.update(
    {config.WaterType.TD: {'min_data': min_max_TD[0], 'max_data': min_max_TD[1], 'data': TD, 'keep_valid_decimals': 1}})
water_data_dict.update(
    {config.WaterType.NH3_NH4: {'min_data': min_max_NH3_NH4[0], 'max_data': min_max_NH3_NH4[1], 'data': NH3_NH4,
                                'keep_valid_decimals': 3}})
try:
    water_crawl_obj = crawl_water_data.CrawlWaterData()
    data_dict = water_crawl_obj.get_data_dict()
    print('data_dict', data_dict)
    if isinstance(data_dict, dict):
        water_data_dict.update({config.WaterType.wt: {'min_data': min(data_dict[config.WaterType.wt]),
                                                      'max_data': max(data_dict[config.WaterType.wt]),
                                                      'data': data_dict[config.WaterType.wt],
                                                      'keep_valid_decimals': 1}})
        water_data_dict.update({config.WaterType.pH: {'min_data': min(data_dict[config.WaterType.pH]),
                                                      'max_data': max(data_dict[config.WaterType.pH]),
                                                      'data': data_dict[config.WaterType.pH],
                                                      'keep_valid_decimals': 2}})
        water_data_dict.update({config.WaterType.EC: {'min_data': min(data_dict[config.WaterType.EC]),
                                                      'max_data': max(data_dict[config.WaterType.EC]),
                                                      'data': data_dict[config.WaterType.EC],
                                                      'keep_valid_decimals': 1}})
        water_data_dict.update({config.WaterType.DO: {'min_data': min(data_dict[config.WaterType.DO]),
                                                      'max_data': max(data_dict[config.WaterType.DO]),
                                                      'data': data_dict[config.WaterType.DO],
                                                      'keep_valid_decimals': 2}})
        water_data_dict.update({config.WaterType.TD: {'min_data': min(data_dict[config.WaterType.TD]),
                                                      'max_data': max(data_dict[config.WaterType.TD]),
                                                      'data': data_dict[config.WaterType.TD],
                                                      'keep_valid_decimals': 1}})
        water_data_dict.update(
            {config.WaterType.NH3_NH4: {'min_data': min(data_dict[config.WaterType.NH3_NH4]),
                                        'max_data': max(data_dict[config.WaterType.NH3_NH4]),
                                        'data': data_dict[config.WaterType.NH3_NH4],
                                        'keep_valid_decimals': 3}})
except Exception as e:
    print({'error': e})

def get_water_data(water_type, count=1, keep_valid_decimals=None):
    """
    返回该类型数据的最近统计数据的高斯分布数据
    :param water_type:水质数据类型
    :param count: 数量
    :param keep_valid_decimals 保留有效位数，如果不手动输入则按照默认值
    :return: 返回样式[30.648319731147538, 42.35755219891315]
    """
    # 求均值
    arr_mean = np.nanmean(water_data_dict[water_type]['data'])
    # 求方差
    arr_var = np.nanvar(water_data_dict[water_type]['data'])
    return_list = []
    for i in range(count):
        # d = np.random.normal(arr_mean, arr_var, 1)[0]  # 依据指定的均值和协方差生成数据
        # d = round(d,water_data_dict[water_type]['keep_valid_decimals'])
        d = -1000
        while d < water_data_dict[water_type]['min_data'] or d > water_data_dict[water_type]['max_data']:
            d = np.random.normal(arr_mean, arr_var, 1)[0]  # 依据指定的均值和协方差生成数据
            if keep_valid_decimals:
                d = round(d, keep_valid_decimals)
            else:
                d = round(d, water_data_dict[water_type]['keep_valid_decimals'])
        return_list.append(d)
    return return_list


def valid_water_data(water_type, data, keep_valid_decimals=None):
    """
    验证数据是否是合法值
    :param water_type: 水质数据类型
    :param data: 数据，浮点数
    :param keep_valid_decimals: 保留有效小数位数
    :return: 如果数据符合要求则返回原始数据，如果不符合要求范围则生成该数据
    """
    if water_data_dict[water_type]['min_data'] < data < water_data_dict[water_type]['max_data']:
        return data
    else:
        # 求均值
        arr_mean = np.nanmean(water_data_dict[water_type]['data'])
        # 求方差
        arr_var = np.nanvar(water_data_dict[water_type]['data'])
        d = -1000
        while d < water_data_dict[water_type]['min_data'] or d > water_data_dict[water_type]['max_data']:
            d = np.random.normal(arr_mean, arr_var, 1)[0]  # 依据指定的均值和协方差生成数据
            if keep_valid_decimals:
                d = round(d, keep_valid_decimals)
            else:
                d = round(d, water_data_dict[water_type]['keep_valid_decimals'])
        return d


if __name__ == '__main__':
    for i in config.WaterType:
        print(i)
        data = get_water_data(water_type=i, count=5)
        print(data)
        print(valid_water_data(i, -10))
