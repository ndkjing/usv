"""
实现vfh算法
"""
import time

import config


def vfh_func(obstacle_list, ceil_max_=None):
    """
    返回是否有可以通过区域 正常返回相对船头角度，返回-1表示没有可以通过区域
    :param obstacle_list:
    :return:
    """
    index_i = 0
    value_list = []
    cell_size = len(obstacle_list)
    point_angle_index = cell_size // 2
    if ceil_max_:
        ceil_max = ceil_max_
    else:
        ceil_max = config.ceil_max  # 可以通过扇区阈值
    view_cell = config.view_cell  # 量化角度单元格
    field_of_view = config.field_of_view
    while index_i < cell_size:
        kr = index_i
        index_j = index_i
        while index_j < cell_size and obstacle_list[index_j] == 0:
            kl = index_j
            if kl - kr >= ceil_max - 1:  # 判断是否是宽波谷
                v = round((kl + kr) / 2)
                value_list.append(v)
                break
            index_j = index_j + 1
        index_i += 1
    print('value_list', value_list)
    # 没有可以通过通道
    if len(value_list) == 0:
        return -1
    else:
        how = []
        for value_i in value_list:
            howtemp = abs(value_i - point_angle_index)
            how.append(howtemp)
        ft = how.index(min(how))
        kb = value_list[int(ft)]
        print('kb', kb, 'value_list', value_list)
        angle = int(kb * view_cell - field_of_view / 2)
        # 该角度为相对船头角度不是相对于北方角度  左正右负
        angle = -1 * angle
        # if angle < 0:
        #     angle += 360
        return angle


if __name__ == '__main__':
    import random

    # obstacle_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    obstacle_list = [0 if random.random() > 0.4 else 1 for i in range(config.field_of_view // config.view_cell)]
    print(len(obstacle_list), obstacle_list)
    print(vfh_func(obstacle_list, 2))
