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
# print(solve_tsp(a,endpoints=(0,0)))

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
# points内的点处在同一条直线上吗？
# points内至少有3个点。
# def on_one_line(points):
#     delta_x = points[1][0] - points[0][0]
#     delta_y = points[1][1] - points[0][1]
#     distance_square = delta_x **2 + delta_y **2
#     sin_times_cos = delta_x * delta_y/ distance_square
#     for j in range(2, len(points)):
#         dx = points[j][0] - points[0][0]
#         dy = points[j][1] - points[0][1]
#         if math.fabs(dx * dy / (dx * dx + dy * dy) - sin_times_cos) > 10 ** -9:
#             return False
#     return True
#
# points=[[1,2],[2,4],[4,8]]
# print(points[:2])
# print(points.pop(0))
# print(points)
# if on_one_line(points):
#     print("True")
# else:
#     print("False")
#
# print(math.sin(math.radians(30)),math.sin(math.radians(150)))
# print(math.cos(math.radians(60)),math.cos(math.radians(-60)))

# 此时经度弧度模式=此时经度*PI/180;
#         此时纬度弧度模式=此时纬度*PI/180;
#         目标经度弧度模式=目标经度*PI/180;
#         目标纬度弧度模式=目标纬度*PI/180;
#         经度差=目标经度弧度模式-此时经度弧度模式;
#         纬度差=目标纬度弧度模式-此时纬度弧度模式;
#         东西距离=1000*2*sin(经度差/2)*cos(此时纬度弧度模式)*地球半径;//经纬度差与东西南北距离差同正负
#         南北距离=1000*2*sin(纬度差/2)*地球半径;
#         总距离=sqrt(pow(东西距离,2)+pow(南北距离,2));


# lng_lats = [[114.431193,30.525967],
#             [114.432802,30.525247],
#             [114.433382,30.523344],
#             [114.433489,30.521866],
#             [114.432802,30.520629],
#             [114.433961,30.519594],
#             [114.431686,30.518837],
#             [114.430399,30.519446],
#             [114.429712,30.520998],
#             [114.429411,30.524434]]
#
# h,w = 600,800
#
# import queue
# q = queue.Queue()
#
# q.put([1,2])
# q.put([2,3])
# print(type(q.get()))
# print(q.get())
# print(q.get())
# a= {'1':1,2:2}
# import json
# b = json.dumps(a)
# c = json.dumps(b)
# print(type(a))
# print(type(b),b)
# print(c)

# import asyncio
# import websockets
#
# # 向服务器端认证，用户名密码通过才能退出循环
# async def auth_system(websocket):
#     while True:
#         # cred_text = input("please enter your username and password: ")
#         await websocket.send('321321')
#         response_str = await websocket.recv()
#         if "congratulation" in response_str:
#             return True
# import json
# # 向服务器端发送认证后的消息
# async def send_msg(websocket):
#     while True:
#         # _text = input("please enter your context: ")
#         # if _text == "exit":
#         #     print(f'you have enter "exit", goodbye')
#         #     await websocket.close(reason="user exit")
#         #     return False
#         # await websocket.send(_text)
#         a = {'123':[1]*3000}
#         # await websocket.send(json.dumps(a))
#         recv_text = await websocket.recv()
#         json_data = json.loads(recv_text)
#         print(f"{recv_text}",type(recv_text))
#         if json_data['deviceId']=='3c50f4c3-a9c1-4872-9f18-883af014380c':
#             retutn_daya = {
#                 # 设备号
#                 "deviceId": "asd2312",
#             # 湖泊编号（正确找到湖才返回）
#             "mapId": "1347114972149161986",
#             # 用户点击经纬度一维数组 先经度 后纬度 保留6位小数
#             "lng_lat": [114.123269, 30.321129],
#             }
#
#             await websocket.send(json.dumps(retutn_daya))
#             # 客户端主逻辑
#
# async def main_logic():
#     async with websockets.connect('ws://101.37.119.148/') as websocket:
#         # await auth_system(websocket)
#         await send_msg(websocket)
#
# asyncio.get_event_loop().run_until_complete(main_logic())

# import random
#
# print(random.randint(0,1))
# import math
#
#
# print(math.sin(math.radians(60)))
# print(math.sin(math.radians(150)))
# print(math.sin(math.radians(240)))
# print(math.sin(math.radians(330)))
#
# print(math.cos(math.radians(60)))
# print(math.cos(math.radians(150)))
# print(math.cos(math.radians(240)))
# print(math.cos(math.radians(330)))
# print(math.sin(math.radians()))
# print(math.sin(math.radians()))
# print(math.sin(math.radians()))
# print(math.sin(math.radians()))
# print(math.sin(math.radians()))
# while True:
#     print(time.time()%60==0)
#     time.sleep(1)

# import platform
#
# def TestPlatform( ):
#     print ("----------Operation System--------------------------")
#     #  获取Python版本
#     print(platform.python_version())
#
#     #   获取操作系统可执行程序的结构，，(’32bit’, ‘WindowsPE’)
#     print(platform.architecture())
#
#     #   计算机的网络名称，’acer-PC’
#     print(platform.node())
#
#     #获取操作系统名称及版本号，’Windows-7-6.1.7601-SP1′
#     print(platform.platform()  )
#
#     #计算机处理器信息，’Intel64 Family 6 Model 42 Stepping 7, GenuineIntel’
#     print(platform.processor())
#
#     # 获取操作系统中Python的构建日期
#     print(platform.python_build())
#
#     #  获取系统中python解释器的信息
#     print(platform.python_compiler())
#
#     if platform.python_branch()=="":
#         print(platform.python_implementation())
#         print(platform.python_revision())
#     print(platform.release())
#     print(platform.system())
#
#     #print platform.system_alias()
#     #  获取操作系统的版本
#     print(platform.version())
#
#     #  包含上面所有的信息汇总
#     print(platform.uname())

#
# import platform
# sysstr = platform.system()
# if (sysstr == "Windows"):
#     print("Call Windows tasks")
# elif (sysstr == "Linux"):   # 树莓派上也是Linux
#     print("Call Linux tasks")
# else:
#     print("other System tasks")
# import json
# print([[114.431465, 30.524369]])
# print(json.dumps([[114.431465, 30.524369]]))
#
# print((100-300)//abs((100-300)))
# print((300-100)//abs((300-100)))
# print((100-300)//(100-300)*100+100)
# print((300-100)//(300-100)*100+100)


# print({'1':100,'2':200}=={'1':100,'2':200})
# print({'1':100,'2':200}=={'1':100,'2':300})
#
# try:
#     while True:
#         # w,a,s,d 为前后左右，q为后退 按键后需要按回车才能生效
#         key_input = input('please input:')
#         # 前 后 左 右 停止  1为右侧电机是反桨叶  3位左侧电机是正桨叶
#         if key_input=='w':
#             print('key_input',key_input)
#         elif key_input=='a':
#             print('key_input', key_input)
#         elif key_input=='s':
#             print('key_input',key_input)
#         elif key_input=='d':
#             print('key_input',key_input)
#         elif key_input=='q':
#             print('key_input',key_input)
#
#         #arm
#         elif key_input == 'z':
#             print('key_input',key_input)
#         # disarm
#         elif key_input == 'x':
#             print('key_input',key_input)
#
#         # manual模式
#         elif key_input == 'm':
#             print('key_input',key_input)
#         # guide模式
#         elif key_input == 'g':
#             print('key_input',key_input)
#         # b 回家
#         elif key_input == 'b':
#             print('key_input',key_input)
#
#         # 角度控制
#         elif key_input.startswith('r'):
#             try:
#                 theta = int(key_input[1:])
#                 print(theta)
#             except Exception as e:
#                 print({'error': e})
#
#         # 运动方向速度控制
#         elif key_input.startswith('n'):
#             speed_x = None
#             speed_y = None
#             try:
#                 str_x,str_y = key_input[1:].split(',')
#                 speed_x = int(str_x)
#                 speed_y = int(str_y)
#                 print(speed_x,speed_y)
#             except Exception as e:
#                 print({'error': e})
#
#         # 到达目标点控制
#         elif key_input.startswith('t'):
#             point_x=None
#             point_y=None
#             try:
#                 str_x, str_y = key_input[1:].split(',')
#                 point_x = int(str_x)
#                 point_y = int(str_y)
#                 print(point_x,point_y)
#             except Exception as e:
#                 print({'error':e})
#
# except:
#     pass
# import math
# print(math.degrees(math.atan2(2,2)))
# print(math.degrees(math.atan2(2,-2)))
# print(math.degrees(math.atan2(-2,2)))
# print(math.degrees( math.atan2(-2,-2)))
# print(math.sqrt(math.pow(-2,2)))
# print(float('123.321'))
#
# print(int(time.time()) % 2 == 0)
#
# print((383.213146851/372.0/2))
# import os
# # if (os.path.exists('test.txt.py')):
# print(os.remove('test.txt.py'))


# 多边形周长
# # shape of polygon: [N, 2]
# def Perimeter(polygon: np.array):
#     N, d = polygon.shape
#     if N < 3 or d != 2:
#         raise ValueError
#
#     permeter = 0.
#     for i in range(N):
#         permeter += np.linalg.norm(polygon[i-1] - polygon[i])
#     return permeter
#
#
# # 面积
# def Area(polygon: np.array):
#     N, d = polygon.shape
#     if N < 3 or d != 2:
#         raise ValueError
#
#     area = 0.
#     vector_1 = polygon[1] - polygon[0]
#     for i in range(2, N):
#         vector_2 = polygon[i] - polygon[0]
#         area += np.abs(np.cross(vector_1, vector_2))
#         vector_1 = vector_2
#     return area / 2
#
# # |r| < 1
# # r > 0, 内缩
# # r < 0, 外扩
# def calc_shrink_width(polygon: np.array, r):
#     area = Area(polygon)
#     perimeter = Perimeter(polygon)
#     L = area * (1 - r ** 2) / perimeter
#     return L if r > 0 else -L
#
#
# def shrink_polygon(polygon: np.array, r):
#     N, d = polygon.shape
#     if N < 3 or d != 2:
#         raise ValueError
#
#     shrinked_polygon = []
#     L = calc_shrink_width(polygon, r)
#     for i in range(N):
#         Pi = polygon[i]
#         v1 = polygon[i-1] - Pi
#         v2 = polygon[(i+1)%N] - Pi
#
#         normalize_v1 = v1 / np.linalg.norm(v1)
#         normalize_v2 = v2 / np.linalg.norm(v2)
#
#         sin_theta = np.abs(np.cross(normalize_v1, normalize_v2))
#
#         Qi = Pi + L / sin_theta * (normalize_v1 + normalize_v2)
#         shrinked_polygon.append(Qi)
#     return np.asarray(shrinked_polygon)
#
#
# if __name__ == "__main__":
# poly = np.array([[0, 0], [0, 1], [0.5, 2], [1, 1], [1, 0]])
# perimeter = Perimeter(poly)
# area = Area(poly)
#
# shrink_poly = shrink_polygon(poly, 0.5)
# expansion_poly = shrink_polygon(shrink_poly, -0.5)
# print(perimeter, area, shrink_poly, expansion_poly)

# a = [1,2,3]
# print(a.pop(2),a)
#
# a = {1:2,3:4,5:{1:2,2:3}}
# print(a.get(1),a.get(5).get(2),a.get(6))
# path_info=[1.23,32.321321]
# print({"path_info": '当前目标点:%d 目标点总数: %d' %(int(path_info[0]),int(path_info[1]))})

# # print(type([[1,23],[321,321]]),isinstance(np.asarray([1,2]),list),isinstance(np.asarray([1,2]),np.ndarray))
# a = {1:2,3:2}
# print(a,a.update({5:5}),a)
# import json
# while 1:
#     if time.time() % 2 <0.1:
#         print(time.time(),int(time.time())%2==0)
#         time.sleep(0.1)
# theta=30
# distance=10
# print(math.sin(math.radians(theta)) * distance)
# map_data = np.zeros((2,3))
# print(map_data.shape)
# a = [1,1,1,2]
# print(a.count(0))

a = [3,1,2,1,5]
print(a.index(max(a)))
del a[0:0]
print(a)