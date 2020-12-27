import threading
import random
import time
"""

class A:
    def __init__(self):
        self.a=1
        self.b=[1,2,3]
        self.c = {1:'1',2:'2','3':3}
        self.d = {1:'1',2:'2','3':3,4:{1:1,2:2,3:3}}
    def print(self):
        print(self.a)
        print(self.b)
        print(self.c)

class B:
    def __init__(self):
        self.a_obj = A()

    def set(self):
        while True:
            self.a_obj.a = random.randint(1,6)
            self.a_obj.b[1] = random.randint(1,6)
            self.a_obj.c.update({'3':random.randint(1,6)})
            self.a_obj.c.update({2:str(random.randint(1,6))})
            self.a_obj.d[4].update({1:str(random.randint(1,6))})
            time.sleep(2)

    def get(self):
        while True:
            print('a',self.a_obj.a)
            print('b',self.a_obj.b)
            print('c',self.a_obj.c)
            print('d',self.a_obj.d)
            time.sleep(3)

if __name__ == '__main__':
    b_obj = B()
    t_get = threading.Thread(target=b_obj.get)
    t_set = threading.Thread(target=b_obj.set)

    t_get.start()
    t_set.start()

    t_set.join()
    t_get.join()

"""

# import numpy as np
# a = [[1,2],[3.32,4.421]]
# str_a = str(a)
#
# print(str_a,type(str_a))
# array_a = np.array(str_a)
# # print(type(array_a),array_a,array_a.shape,array_a[0])
# new_array =[]
# for i in array_a:
#     new_array.append([i[0]*100,i[1]*100])
#
# print(new_array)
# import re
# s = 'AAA231.31'
#
# print(s.split('AAA'))
# print('dsa,ds,asdsa,wq,3123'.count(','))

# send_data=''
# while True:
#     i = input('direction:')
#     if len(i)==1:
#         print(type(i))
#         print('A%sZ'%(i))
        # time.sleep(0.1)
# from audios import audios_manager
# audios_manager.play_audio('setup.mp3')

import numpy as np

# m = np.full(shape=(10,10),fill_value=np.inf)
# print(m.shape,m)

#
# import math
#
# def distance(p0, p1, digits=2):
#     a = map(lambda x: (x[0] - x[1]) ** 2, zip(p0, p1))
#     return round(math.sqrt(sum(a)), digits)
#
#
# print(distance([0,0],[3,4]))
#
#
# l=[1,2,3,4]
# print(l[::-1])
#
# from audios import audios_manager
#
# audios_manager.play_audio('setup.mp3')
# a = [[[1,2]],[[1,2],[2,3],[1,3]]]
# s = [[0],[0,0,0]]
# print(len(a[0]))
# from tsp_solver.greedy import solve_tsp
# a = np.array([
#     [0,2,3],
#     [1,0,2],
#     [3,1,0],
# ])
#
# print(solve_tsp(a,endpoints=(0,2)))
#
# b = [1,2,3,5]
# b.insert(0,10)
# print(b)
import math


# 测试通过
# def DDD2DMS(number):
#     D = number // 1
#     temp = number % 1
#     M = (temp * 60) // 1
#     temp = (temp * 60) % 1
#     S = (temp * 60)
#     return D + (M / 100) + (S / 10000)


# 求连点的经纬度 返回0-360 顺时针为正
# def angleFromCoordinate(long1, lat1, long2, lat2):
#     lat1 = math.radians(DDD2DMS(lat1))
#     lat2 = math.radians(DDD2DMS(lat2))
#     long1 = math.radians(DDD2DMS(long1))
#     long2 = math.radians(DDD2DMS(long2))
#     y = math.sin(long2 - long1) * math.cos(lat2)
#     x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
#     deltaLon = long2 - long1
#     theta = math.atan2(y, x)
#     theta = math.degrees(theta)
#     theta = (theta + 360) % 360
#     return theta
#
#
# # 一直两点经纬度求两点的距离单位，返回单位厘米
# def distanceFromCoordinate(lon1, lat1, lon2, lat2):  # 经度1，纬度1，经度2，纬度2 （十进制度数）
#     """
#     Calculate the great circle distance between two points
#     on the earth (specified in decimal degrees)
#     """
#     # 将十进制度数转化为弧度
#     lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
#
#     # haversine公式
#     dlon = lon2 - lon1
#     dlat = lat2 - lat1
#     a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
#     c = 2 * math.asin(math.sqrt(a))
#     r = 6371  # 地球平均半径，单位为公里
#     return c * r * 1000
#
# # 已知一点的经纬度和移动方向与距离，求终点的经纬度
# def one_point_diatance_to_end(lng,lat,brng,d):
#     R = 6378.1  # Radius of the Earth
#     brng = math.radians(brng) # Bearing is 90 degrees converted to radians.
#     d = d/100000 # Distance in km
#
#     # lat2  52.20444 - the lat result I'm hoping for
#     # lon2  0.36056 - the long result I'm hoping for.
#
#     lat1 = math.radians(lat)  # Current lat point converted to radians
#     lon1 = math.radians(lng)  # Current long point converted to radians
#
#     lat2 = math.asin(math.sin(lat1) * math.cos(d / R) +
#                      math.cos(lat1) * math.sin(d / R) * math.cos(brng))
#
#     lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / R) * math.cos(lat1),
#                              math.cos(d / R) - math.sin(lat1) * math.sin(lat2))
#
#     lat2 = math.degrees(lat2)
#     lon2 = math.degrees(lon2)
#
#     print(lon2)
#     print(lat2)

#
#
# temp = angleFromCoordinate(114.316966,30.576768, 114.397346,30.58709)
# print(temp)
# temp = distanceFromCoordinate(114.316966,30.576768, 114.397346,30.58709)
# print(temp)
# one_point_diatance_to_end(114.316966,30.576768,78.7,777974)

# zoom = [7,15.321,14.3497808125]
# for i in zoom:
#         print(math.pow(2,18-i))

# x=[math.sqrt(3),math.sqrt(3),-math.sqrt(3),-math.sqrt(3)]
# y=[1,-1,1,-1]
# for i,j in zip(x,y):
#     print('i,j',i,j)
#     theta = round(math.degrees(math.atan2(i,j)),1)
#     theta = theta if theta>0 else 360+theta
#     print('math.atan2(i,j)',theta)
# a = [1,3.321]
# b = tuple(a)
# print(b,type(b),b[0])


# a='3213'
# b=a
# c=2

# import execjs
#
# print(execjs.eval("'red yellow blue'.split(' ')"))
#
# ctx = execjs.compile(
#     """
#     function add(x,y){
#         return x+y;
#     }
#     """
# )
# print(ctx.call("add",2,3))
# a = np.array([
#     [1,2,3],
#     [4,5,6],
#     [7,8,9],
# ])
# a[0,:]=-1
# a[:,0]=-1
# a[2,:]=-1
# a[:,2]=-1
# print(a)

# a = [1,23,4,321]
# print(a.sort())
# print(a)
# print(a.index(4))
# a = round(15.65,0)
# print(a,type(a))

# a = np.array([[1,2],[3,4]])
# print(type(a))
# print(isinstance(a,np.ndarray))
# print(type(a.tolist()))
# print(type(a))

import math
#points内的点处在同一条直线上吗？
#points内至少有3个点。
def on_one_line(points):
    delta_x = points[1][0] - points[0][0]
    delta_y = points[1][1] - points[0][1]
    distance_square = delta_x **2 + delta_y **2
    sin_times_cos = delta_x * delta_y/ distance_square
    for j in range(2, len(points)):
        dx = points[j][0] - points[0][0]
        dy = points[j][1] - points[0][1]
        if math.fabs(dx * dy / (dx * dx + dy * dy) - sin_times_cos) > 10 ** -9:
            return False
    return True

points=[[1,2],[2,4],[4,8]]
if on_one_line(points):
    print("True")
else:
    print("False")

print(math.sin(math.radians(30)),math.sin(math.radians(150)))
print(math.cos(math.radians(60)),math.cos(math.radians(-60)))

#
# 此时经度弧度模式=此时经度*PI/180;
#         此时纬度弧度模式=此时纬度*PI/180;
#         目标经度弧度模式=目标经度*PI/180;
#         目标纬度弧度模式=目标纬度*PI/180;
#         经度差=目标经度弧度模式-此时经度弧度模式;
#         纬度差=目标纬度弧度模式-此时纬度弧度模式;
#         东西距离=1000*2*sin(经度差/2)*cos(此时纬度弧度模式)*地球半径;//经纬度差与东西南北距离差同正负
#         南北距离=1000*2*sin(纬度差/2)*地球半径;
#         总距离=sqrt(pow(东西距离,2)+pow(南北距离,2));




