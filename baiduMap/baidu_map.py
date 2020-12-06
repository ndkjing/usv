import requests
import json
import random
import cv2
import numpy as np
import math
import os
import sys
"""
ak='wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
"""


def color_block_finder(img, lowerb, upperb,
                       min_w=0, max_w=None, min_h=0, max_h=None):
    '''
    色块识别 返回矩形信息
    '''
    # 转换色彩空间 HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 根据颜色阈值转换为二值化图像
    img_bin = cv2.inRange(img_hsv, lowerb, upperb)

    # 寻找轮廓（只寻找最外侧的色块）
    contours, hier = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 声明画布 拷贝自img
    show_img = np.copy(img)
    # 外接矩形区域集合
    rects = []

    if max_w is None:
        # 如果最大宽度没有设定，就设定为图像的宽度
        max_w = img.shape[1]
    if max_h is None:
        # 如果最大高度没有设定，就设定为图像的高度
        max_h = img.shape[0]

    # 遍历所有的边缘轮廓集合
    # for cidx, cnt in enumerate(contours):
    #     # 获取联通域的外界矩形
    #     (x, y, w, h) = cv2.boundingRect(cnt)
    #
    #     if w >= min_w and w <= max_w and h >= min_h and h <= max_h:
    #         # 将矩形的信息(tuple)添加到rects中
    #         rects.append((x, y, w, h))
    # 绘制轮廓
    # show_img = cv2.drawContours(show_img, contours, -1, (0, 255, 0), 2)
    point = np.array((512,512))
    in_cnt=-1
    # 找到中心点所在的轮廓
    return_cnt=None
    for index,cnt in enumerate(contours):
        in_cnt = cv2.pointPolygonTest(cnt,(512,512),False)
        # print('in cnt',in_cnt)

        if in_cnt>0:
            # print('len(cnt)',len(cnt))
            return_cnt = cnt
            show_img = cv2.drawContours(show_img, cnt, -1, (0, 0, 255), 3)
    return show_img,return_cnt

    # return rects


def draw_color_block_rect(img, rects, color=(0, 0, 255)):
    '''
    绘制色块的矩形区域
    '''
    # 声明画布(canvas) 拷贝自img
    canvas = np.copy(img)
    # 遍历矩形区域
    for rect in rects:
        (x, y, w, h) = rect
        # 在画布上绘制矩形区域（红框）
        cv2.rectangle(canvas, pt1=(x, y), pt2=(x + w, y + h), color=color, thickness=3)

    return canvas


class BaiduMap(object):
    def __init__(self,lng_lat,ak='wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY',height=1024,width=1024,zoom=9):
        # 经纬度
        self.lng_lat=lng_lat
        # 访问秘钥
        self.ak = ak
        # 图像高度和宽度
        self.height = height
        self.width = width
        # 缩放比例
        self.zoom = zoom
        self.addr = str(round(100*random.random(),3))
        # ＨＳＶ阈值　［［低　ＨＳＶ］,　［高　ＨＳＶ］］
        self.threshold_hsv = [(84,72,245),(118,97,255)]
        # 百度不同缩放比例下比例尺对应的实际距离　　缩放尺寸　长度（米）　像素距离
        self.scale_map = {
            # {zoom: 19, length: 20, Pixels: 48},
            19:[20,48],
            18:[50, 61],
            17:[100, 61],
            16:[200, 61],
            15:[500, 78],
            14:[1000, 78],
            13:[2000, 78],
            12:[5000, 99],
            11:[10000, 99],
            10:[20000, 99],
            9:[25000, 59],
            8:[50000, 59],
            7:[100000, 59],
            6:[200000, 59],
            5:[500000, 76],
            4:[1000000,76],
            }
        self.save_img_path = './imgs/%f_%f_%i.png'%(self.lng_lat[0],self.lng_lat[1],self.zoom)
        if not os.path.exists(self.save_img_path):
            self.draw_image()

    # 获取地址的url
    def get_url(self,addr):
        self.addr = addr
        if len(addr) < 1:
            return None

        return 'http://api.map.baidu.com/geocoding/v3/?address={inputAddress}&output=json&ak={myAk}'.format(
                    inputAddress=addr, myAk=self.ak)

    # 通过地址url获取经纬度
    def get_position(self,addr):
        '''返回经纬度信息'''
        res = requests.get(self.get_url(addr))
        json_data = json.loads(res.text)
        # print(json_data)
        if json_data['status'] == 0:
            lat = json_data['result']['location']['lat']  # 纬度
            lng = json_data['result']['location']['lng']  # 经度
        else:
            print("Error output!")
            return json_data['status']
        return lat, lng

    # 获取经纬度url
    def get_image_url(self):
        '''
            调用地图API获取待查询地址专属url
            最高查询次数30w/天，最大并发量160/秒
            http://api.map.baidu.com/staticimage/v2?ak=E4805d16520de693a3fe707cdc962045&mcode=666666&center=116.403874,39.914888&width=300&height=200&zoom=11
            '''
        return 'http://api.map.baidu.com/staticimage/v2?ak={myAk}&center={position}&width={width}&height={height}&zoom={zoom}'.format(
            myAk=self.ak,position='%f,%f'%(self.lng_lat[0],self.lng_lat[1]), width=self.width,height=self.height,zoom=self.zoom)

    # 按照经纬度url获取静态图
    def draw_image(self,):
        png_url = self.get_image_url()
        print('png_url',png_url)
        response = requests.get(png_url)
        # 获取的文本实际上是图片的二进制文本
        img = response.content
        # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
        with open(self.save_img_path,'wb' ) as f:
            f.write(img)

    # 静态图蓝色护坡区域抠图
    def get_pool_pix(self,b_show=True):
        self.row_img = cv2.imread(self.save_img_path)
        # 图片路径
        # 颜色阈值下界(HSV) lower boudnary
        lowerb = self.threshold_hsv[0]
        # 颜色阈值上界(HSV) upper boundary
        upperb = self.threshold_hsv[1]

        # 读入素材图片 BGR
        img = cv2.imread(self.save_img_path, cv2.IMREAD_COLOR)
        # 检查图片是否读取成功
        if img is None:
            print("Error: 请检查图片文件路径")
            exit(1)

        # 识别色块 获取矩形区域数组
        show_img,return_cnt = color_block_finder(img, lowerb, upperb)
        if return_cnt is None:
            print('无法在点击处找到湖')
            exit()

        # 绘制色块的矩形区域
        # canvas = draw_color_block_rect(img, rects)
        # 在HighGUI窗口 展示最终结果
        if b_show:
            cv2.namedWindow('result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            # cv2.imshow('result', canvas)
            cv2.imshow('result', show_img)

            # 等待任意按键按下
            # cv2.waitKey(0)
            # 关闭其他窗口
            # cv2.destroyAllWindows()
        return return_cnt

    # 区域像素点转换为经纬度坐标点
    def pix_to_gps(self,cnt):
        """
        :param cnt:
        :return:
        """
        # 返回经纬度坐标集合
        return_gps = []
        # 初始点（中心点）经纬度坐标
        # 初始点（中心点）像素坐标
        center = (self.width/2,self.height/2)

        for point in cnt:
            delta_pix_x = point[0]-center[0]
            delta_pix_y = point[1]-center[1]
            # pix_2_meter = math.pow(2,18-self.zoom)
            pix_2_meter = float(self.scale_map[self.zoom][0])/self.scale_map[self.zoom][1]
            delta_meter_x = delta_pix_x*(pix_2_meter)
            delta_meter_y = delta_pix_y*(pix_2_meter)
            # 方法一：直接计算
            # 方法二：当做圆球计算
            method=2
            if method==1:
                L = 6381372 * math.pi * 2
                W = L
                H = L / 2
                mill = 2.3
                delta_lat = ((H / 2 - delta_meter_y) * 2 * mill) / (1.25 * H)
                delta_lat = ((math.atan(math.exp(delta_lat)) - 0.25 * math.pi) * 180) / (0.4 * math.pi)
                delta_lon = (delta_meter_x - W / 2) * 360 / W

                center_lat = ((H / 2 - 0) * 2 * mill) / (1.25 * H)
                center_lat = ((math.atan(math.exp(center_lat)) - 0.25 * math.pi) * 180) / (0.4 * math.pi)
                center_lon = (0 - W / 2) * 360 / W

                gpx_x = -(center_lon-delta_lon)
                gpx_y = -(center_lat-delta_lat)

                # TODO 最终需要确认经纬度保留小数点后几位
                point_gps = [self.lng_lat[0]+gpx_x,self.lng_lat[1]+gpx_y]
                return_gps.append({"lat":point_gps[1],"lng":point_gps[0]})

            elif method==2:
                """
                    地理中常用的数学计算，把地球简化成了一个标准球形，如果想要推广到任意星球可以改成类的写法，然后修改半径即可
                """
                earth_radius = (6370.8560)*1000  # 地球平均半径，单位km，最简单的模型往往把地球当做完美的球形，这个值就是常说的RE  平均半径　6370.856　　赤道半径6378.1370
                math_2pi = math.pi * 2
                pis_per_degree = math_2pi / 360  # 角度一度所对应的弧度数，360对应2*pi
                # 计算维度上圆面半径
                real_radius = earth_radius * math.cos(self.lng_lat[1] * pis_per_degree)
                # 经度偏差
                delta_lng = (delta_meter_x/real_radius)/pis_per_degree
                # 纬度偏差
                delta_lat = -(delta_meter_y/earth_radius)/pis_per_degree
                print(delta_lng,delta_lat)
                # TODO 最终需要确认经纬度保留小数点后几位
                point_gps = [self.lng_lat[0] + delta_lng, self.lng_lat[1] + delta_lat]
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})

        print('len ',len(return_gps))
        with open('map.json','w') as f:
            json.dump(return_gps,f)
        return return_gps

    def scan_pool(self,contour,pix_gap=20,b_show=True):
        """
        传入湖泊像素轮廓返回活泼扫描点
        :param contour 轮廓点
        :param pix_gap 指定扫描间隔，单位像素
        """
        (x, y, w, h) = cv2.boundingRect(contour)
        print('(x, y, w, h)',(x, y, w, h))
        # 循环生成点同时判断点是否在湖泊范围在则添加到列表中
        scan_points = []
        # 起始点
        start_x,start_y = x,y+pix_gap
        # 当前点
        current_x,current_y = start_x,start_y
        # 判断x轴是递增的加还是减 True 为加
        b_add_or_sub = True

        while current_y<(y+h):
            while current_x<=(x+w) and current_x>=x:
                point = (current_x,current_y )
                in_cnt = cv2.pointPolygonTest(contour, point, False)
                # print('in cnt',in_cnt)
                if in_cnt > 0:
                    scan_points.append(list(point))
                if b_add_or_sub:
                    current_x+=pix_gap
                else:
                    current_x -= pix_gap
            current_y+=pix_gap
            if b_add_or_sub:
                current_x -= pix_gap
                b_add_or_sub=False
            else:
                current_x += pix_gap
                b_add_or_sub = True
        show_img = np.copy(self.row_img)
        if b_show:
            for point in scan_points:
                cv2.circle(show_img,tuple(point), 3, (0,0,255), -1)
            cv2.polylines(show_img,[np.array(scan_points,dtype=np.int32)],False,(255,0,0),2)
            cv2.imshow('scan',show_img)
            cv2.waitKey(0)
        return scan_points





if __name__ == '__main__':
    obj = BaiduMap([99.937205,45.161601],zoom=4)
    # obj.select_roi()
    # obj.analyse_hsv()
    # obj.hsv_image_threshold()
    pool_cnt = obj.get_pool_pix(b_show=True)
    pool_cnt = np.squeeze(pool_cnt)

    scan_cnt = obj.scan_pool(pool_cnt,pix_gap=40,b_show=True)

    all_cnt = []
    all_cnt.extend(list(pool_cnt))
    all_cnt.extend(scan_cnt)
    gps = obj.pix_to_gps(all_cnt)
    # print(gps)
    # 请求指定位置图片
    # obj.draw_image()

