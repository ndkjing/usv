import numpy as np
import random

import config


# 判断该角度范围内是否有障碍物
# if -config.steer_max_angle <= angle_point_temp < -config.steer_max_angle * 3 / 5:
#     point_angle_index = 0
# elif -config.steer_max_angle * 3 / 5 <= angle_point_temp < -config.steer_max_angle * 1 / 5:
#     point_angle_index = 1
# elif -config.steer_max_angle * 1 / 5 <= angle_point_temp < config.steer_max_angle * 1 / 5:
#     point_angle_index = 2
# elif config.steer_max_angle * 1 / 5 <= angle_point_temp < config.steer_max_angle * 3 / 5:
#     point_angle_index = 3
# elif config.steer_max_angle * 3 / 5 <= angle_point_temp < config.steer_max_angle:
#     point_angle_index = 4
# elif angle_point_temp < -config.steer_max_angle:
#     point_angle_index = -1
# else:
#     point_angle_index = 5
def func():
    smax = 2
    angle_ceil_size = 5  # 一个扇区角度值
    # 单侧检测范围 双侧为[-45,45]
    detect_angle = 45
    while True:
        index_i = 0
        obstacle_list = [1 if random.random() > 0.8 else 0 for i in range(int(detect_angle*2/5))]
        angle_point = random.randint(-detect_angle, detect_angle)
        if angle_point > 180:
            angle_point_temp = angle_point - 360
        else:
            angle_point_temp = angle_point
        point_angle_index = int((angle_point_temp + detect_angle) / angle_ceil_size)
        value_list = []
        while index_i < len(obstacle_list):
            kr = index_i
            index_j = index_i
            while index_j < len(obstacle_list) and obstacle_list[index_j] == 0:
                kl = index_j
                if (kl - kr >= config.ceil_max):  # 判断是否是宽波谷
                    print(obstacle_list, round(kl - config.ceil_max // 2))
                    v = round((kl + kr) / 2)
                    value_list.append(v)
                    break
                index_j = index_j + 1
            index_i += 1
        # 没有可以通过通道
        if len(value_list) == 0:
            print('no')
        else:
            how = []
            for value_i in (value_list):
                howtemp = abs(value_i - point_angle_index)
                how.append(howtemp)
            ft = how.index(min(how))
            kb = value_list[int(ft)]
            angle = kb * config.view_cell - (config.field_of_view) / 2
            if angle < 0:
                angle += 360
            str1 = ' '
            str2 = ' '
            for i in obstacle_list:
                if i == 0:
                    str1 += '  *  '
                else:
                    str1 += '  #  '
            print('str1', str1)
            for index_l, value_l in enumerate(obstacle_list):
                if index_l == int(kb):
                    str2 += '  ^  '
                if index_l == point_angle_index:
                    str2 += '  &  '
                else:
                    str2 += '     '
            print('str2',str2)

def func2():
    print('w,a,s,d 为前后左右，q为停止\n'
          'r,t 左右抽水泵\n'
          'y,u,i,o,p 摄像头舵机 y回中 u右  i左  o上   p下\n'
          'f  测距\n'
          'g  获取gps数据\n'
          'h  获取罗盘数据\n'
          'j  声光报警器  j1 开 j0关\n '
          'k  左舷灯 k1 开 k0关\n '
          'l  右舷灯  l1 开 l0关\n '
          'v  大灯控制  v1 开 v0关\n '
          'z 退出\n'
          'x  接受遥控器输入\n'
          'c  声呐数据\n'
          'b  毫米波数据\n'
          'n  距离字典\n'
          'A0 A1  关闭和开启水泵'
          'B0 B1  关闭和开启舷灯'
          'C0 C1  关闭和开启前面大灯'
          'D0 D1  关闭和开启声光报警器'
          'E0 E1 E2 E3 E4  状态灯'
          )
def func3():
    import binascii
    a = b'\x01\x0212'
    a = b'\x01\x02\x12'
    b = binascii.b2a_hex(a)
    c = str(b)
    print(a,b,c)

def func4():
    print(int(3/4))
    for i in range(1,1):
        print(i)

def func5():
    d_m = []
    for i in range(5):
        d_list = []
        for j in range(3):
            d_list.append(random.random())
        d_m.append(d_list)
    print(len(d_m),len(d_m[0]))
    min_index_list = []
    a_d_m = np.asarray(d_m)
    for k in range(len(d_m[0])):
        temp_a = a_d_m[:,k]
        temp_list = temp_a.tolist()
        index = temp_list.index(min(temp_list))
        min_index_list.append(index)
    print(a_d_m)
    print(min_index_list)

if __name__ == '__main__':
    func5()