from utils import log

import config
import cv2
import numpy as np
from flask import Flask, request
from flask import render_template
from flask_cors import CORS
import os
import json
import time
import threading
import sys

print('path: ',os.path.dirname(os.path.abspath(__file__)))
# 获取资源路径
def resource_path(relative_path):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

app = Flask(__name__, template_folder=resource_path('templates'),static_folder=resource_path('statics'))
CORS(app, resources=r'/*')
#测试使用
b_test=False

# 接收第一次点击经纬都找湖
@app.route('/')
def index():
    return render_template('map3.html')

logger = log.LogHandler('main')

# 湖轮廓像素位置
@app.route('/pool_cnts', methods=['GET', 'POST'])
def pool_cnts():
    print(request)
    print("request.url", request.url)
    print("request.data", request.data)
    if b_test:
        return json.dumps({'data':'391,599 745,539 872,379 896,254 745,150 999,63 499,0 217,51  66,181 0,470'})
    else:
    # 失败返回提示信息
        if ship_obj.pix_cnts is None:
            return '初始经纬度像素点未生成'
        #{'data':'391, 599 745, 539 872, 379 896, 254 745, 150 999, 63 499, 0 217, 51  66, 181 0, 470'}
        else:
            str_pix_points = ''
            for index, value in enumerate(ship_obj.pix_cnts):
                if index == len(ship_obj.pix_cnts) - 1:
                    str_pix_points += str(value[0]) + ',' + str(value[1])
                else:
                    str_pix_points += str(value[0]) + ',' + str(value[1]) + ' '
            return_json = json.dumps({'data': str_pix_points})
            print('pool_cnts',return_json)
            return return_json

# 获取在线船列表
@app.route('/online_ship', methods=['GET', 'POST'])
def online_ship():
    print(request)
    print('request.data', request.data)
    if b_test:
        return_data = {
            # 船号
            "ids": [1, 2, 8,10,18],
            # 船像素信息数组
            "pix_postion": [[783, 1999], [132, 606], [52, 906], [0, 1569]],
            # 船是否配置行驶点 1为已经配置  0位还未配置
            "config_path": [1, 1, 0, 1],
            # 船剩余电量0-100整数
            "dump_energy": [90, 37, 80, 60],
            # 船速度 单位：m/s  浮点数
            "speed": [3.5, 2.0, 1.0, 5.0]
        }
        return json.dumps(return_data)

    else:
        return_data = {
            # 船号
            "ids": ship_obj.online_ship_list,
            # 船像素信息数组
            "pix_postion": [ship_obj.ship_pix_position_dict.get(i) for i in ship_obj.online_ship_list],
            # 船是否配置行驶点 1为已经配置  0位还未配置
            "config_path": [1 if i in ship_obj.config_ship_lng_lats_dict else 0 for i in ship_obj.online_ship_list],
            # 船剩余电量0-100整数
            "dump_energy": [ship_obj.ship_dump_energy_dict.get(i) for i in ship_obj.online_ship_list],
            # 船速度 单位：m/s  浮点数
            "speed": [ship_obj.ship_speed_dict.get(i) for i in ship_obj.online_ship_list],
            "direction":[ship_obj.ship_direction_dict.get(i) for i in ship_obj.online_ship_list]
        }
        print('online_ship data',return_data)
        return json.dumps(return_data)

# 发送一条船配置路径
@app.route('/ship_path', methods=['GET', 'POST'])
def ship_path():
    print(request)
    print('request.data', request.data)
    data = json.loads(request.data)
    if ship_obj.pix_cnts is None:
        return '还没有湖，别点'
    ids_list = []
    for i in data['id'].split(' '):
        try:
            id = int(i)
            ids_list.append(id)
        except Exception as e :
            logger.error({'error: ':e})
    # 没有合法id
    if len(ids_list)==0 or len(data['data'])<=0:
        return
    for id in ids_list:
        if data['data'][0][0].endswith('px'):
            click_pix_points =  [[int(i[0][:-2]),int(i[1][:-2])] for i in data['data']]
        else:
            click_pix_points =  [[int(i[0]),int(i[1])] for i in data['data']]
        click_lng_lats = []
        for point in click_pix_points:
            in_cnt = cv2.pointPolygonTest(np.array(ship_obj.pix_cnts), (point[0], point[1]), False)
            if in_cnt >= 0:
                click_lng_lat = ship_obj.pix_to_lng_lat(point)
                click_lng_lats.append(click_lng_lat)
        ship_obj.config_ship_lng_lats_dict.update({id:click_lng_lats})
    # logger.debug({'config_ship_lng_lats_dict':ship_obj.config_ship_lng_lats_dict})
    return 'ship_path'

# 发送所有配置路径到船并启动
@app.route('/send_path', methods=['GET', 'POST'])
def send_path():
    print(request)
    ship_obj.b_send_path = True
    # ship_obj.b_send_control = True
    for i in ship_obj.online_ship_list:
        ship_obj.ship_control_dict.update({int(i):1})
    return 'send_path'

# 控制船启动
@app.route('/ship_start', methods=['GET', 'POST'])
def ship_start():
    print(request)
    print('request.data', request.data)
    ship_obj.b_send_control=True
    data = json.loads(request.data)
    for i in data['id']:
        ship_obj.ship_control_dict.update({int(i):1})
    return 'ship_start'

# 控制船停止
@app.route('/ship_stop', methods=['GET', 'POST'])
def ship_stop():
    print(request)
    print('request.data', request.data)
    ship_obj.b_send_control = True
    data = json.loads(request.data)
    for i in data['id']:
        ship_obj.ship_control_dict.update({int(i):0})
    return 'ship_stop'

class Ship:
    def __init__(self):
        self.logger = log.LogHandler('mian')
        self.com_logger = log.LogHandler('com_logger')

        # 湖泊像素轮廓点
        self.pix_cnts = None

        # 当前接收到的船号，
        self.online_ship_list = []


        # 手动控制状态
        self.ship_control_dict={}

        # 像素位置与经纬度
        self.ship_pix_position_dict = {}
        self.ship_lng_lat_position_dict = {}

        # 用户点击像素点
        self.click_pix_points_dict = {}
        # 船配置航点
        self.config_ship_lng_lats_dict = {}
        # 船剩余电量
        self.ship_dump_energy_dict={}
        # 船速度
        self.ship_speed_dict = {}
        # 船朝向
        self.ship_direction_dict={}

        # 是否发送所有路径到船
        self.b_send_path = False
        self.b_send_control=False
        # 采集点经纬度
        self.lng_lats_list = []

        # 记录当前存在的串口
        self.serial_obj = None

    # 必须放在主线程中
    @staticmethod
    def run_flask(debug=True):
        # app.run(host='192.168.199.171', port=5500, debug=True)
        app.run(host='0.0.0.0', port=8899, debug=debug)

    # 经纬度转像素
    def lng_lat_to_pix(self,lng_lat):
        """
        :param lng_lat: 经纬度
        :return:
        """
        int_lng_lat = [int(lng_lat[0] * 1000000), int(lng_lat[1] * 1000000)]
        int_lng_lats_offset = [int_lng_lat[0] - self.left_up_x, int_lng_lat[1] - self.left_up_y]
        int_lng_lats_pix = [int(int_lng_lats_offset[0] / self.scale_w), int(int_lng_lats_offset[1] / self.scale_h)]
        return int_lng_lats_pix

    # 像素转经纬度
    def pix_to_lng_lat(self, pix):
        """
        :param pix:像素位置 先w 后h
        :return: 经纬度
        """
        lng = round((self.left_up_x + pix[0] * self.scale_w) / 1000000.0, 6)
        lat = round((self.left_up_y + pix[1] * self.scale_h) / 1000000.0, 6)
        return [lng,lat]

    def init_cnts_lng_lat_to_pix(self, b_show=False):
        lng_lats_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lng_lats.txt')
        while not os.path.exists(lng_lats_path):
            time.sleep(1)
        try:
            with open(lng_lats_path, 'r') as f:
                temp_list = f.readlines()
                for i in temp_list:
                    i = i.strip()
                    self.lng_lats_list.append(
                        [float(i.split(',')[0]), float(i.split(',')[1])])
        except Exception as e:
            self.logger.error({'lng_lats.txt 格式错误':e})

        int_lng_lats_list = [[int(i[0] * 1000000), int(i[1] * 1000000)]
                        for i in self.lng_lats_list]
        (left_up_x, left_up_y, w, h) = cv2.boundingRect(np.array(int_lng_lats_list))
        self.left_up_x = left_up_x
        self.left_up_y = left_up_y
        self.logger.info({'(x, y, w, h) ': (left_up_x, left_up_y, w, h)})

        ## 像素到单位缩放
        # 等比拉伸
        if w>=h:
            self.scale_w = float(w) / config.pix_w
            self.scale_h = float(w) / config.pix_w
        else:
            self.scale_w = float(h) / config.pix_w
            self.scale_h = float(h) / config.pix_w
        # 强制拉伸到同样长宽
        # self.scale_w = float(w) / config.pix_w
        # self.scale_h = float(h) / config.pix_h

        # 经纬度转像素
        self.pix_cnts = [self.lng_lat_to_pix(i) for i in self.lng_lats_list]
        self.logger.info({'self.pix_cnts': self.pix_cnts})

        if b_show:
            img = np.zeros((config.pix_h, config.pix_w, 3), dtype=np.uint8)
            cv2.circle(img, (int(config.pix_w / 2),
                             int(config.pix_h / 2)), 5, (255, 0, 255), -1)
            cv2.drawContours(
                img,
                np.array(
                    [self.pix_cnts]),
                contourIdx=-1,
                color=(255, 0, 0))

            print(img.shape)
            # 鼠标回调函数
            # x, y 都是相对于窗口内的图像的位置
            def draw_circle(event, x, y, flags, param):
                # 判断事件是否为 Left Button Double Clicck
                if event == cv2.EVENT_LBUTTONDBLCLK or event == cv2.EVENT_LBUTTONDOWN:
                    in_cnt = cv2.pointPolygonTest(
                        np.array([self.pix_cnts]), (x, y), False)
                    # 大于0说明属于该轮廓
                    if in_cnt >= 0:
                        print('像素', x, y)
                        lng = round((left_up_x + x * self.scale_w) / 1000000.0, 6)
                        lat = round((left_up_y + y * self.scale_h) / 1000000.0, 6)
                        print('经纬度', lng, lat)
                        cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
                if event == cv2.EVENT_RBUTTONDOWN:
                    in_cnt = cv2.pointPolygonTest(
                        np.array([self.pix_cnts ]), (x, y), False)
                    # 大于0说明属于该轮廓
                    if in_cnt >= 0:
                        print('像素', x, y)
                        lng = round((left_up_x + x * self.scale_w) / 1000000.0, 6)
                        lat = round((left_up_y + y * self.scale_h) / 1000000.0, 6)
                        print('经纬度', lng, lat)
                        cv2.circle(img, (x, y), 5, (255, 0, 0), -1)

            cv2.namedWindow('img')
            # 设置鼠标事件回调
            cv2.setMouseCallback('img', draw_circle)
            while (True):
                cv2.imshow('img', img)
                if cv2.waitKey(1) == ord('q'):
                    break
            # cv2.waitKey(0)
            cv2.destroyAllWindows()

    # 发送串口数据
    def send_com_data(self):
        while True:

            if self.serial_obj is None:
                time.sleep(1)
                continue
            # 发送配置点





if __name__ == '__main__':
    ship_obj = Ship()
    init_cnts_lng_lat_to_pix = threading.Thread(target=ship_obj.init_cnts_lng_lat_to_pix,args=(False,))
    get_com_thread = threading.Thread(target=ship_obj.get_com_data)
    send_com_thread = threading.Thread(target=ship_obj.send_com_data)
    #
    # init_cnts_lng_lat_to_pix.setDaemon(True)
    # get_com_thread.setDaemon(True)
    # send_com_thread.setDaemon(True)
    #
    init_cnts_lng_lat_to_pix.start()
    get_com_thread.start()
    send_com_thread.start()

    # init_cnts_lng_lat_to_pix.join()
    # get_com_thread.join()
    # send_com_thread.join()
    # run_flask()
    ship_obj.run_flask(debug=False)
