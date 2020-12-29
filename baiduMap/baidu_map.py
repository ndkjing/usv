import requests
import json
import random
import cv2
import numpy as np
import math
import os
from math import radians, cos, sin, degrees, atan2
import copy
import enum
import config
from utils import lng_lat_calculate
import sys

"""
百度地图
ak='wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
"""
method_0 = cv2.CHAIN_APPROX_NONE
method_1 = cv2.CHAIN_APPROX_SIMPLE


def color_block_finder(img, lowerb, upperb,
                       min_w=0, max_w=None, min_h=0, max_h=None, map_type=None,
                       scale=1):
    '''
    色块识别 返回矩形信息，若没有找到返回矩形框为None
    '''
    # 转换色彩空间 HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 根据颜色阈值转换为二值化图像
    img_bin = cv2.inRange(img_hsv, lowerb, upperb)

    # 寻找轮廓（只寻找最外侧的色块）
    contours, hier = cv2.findContours(
        img_bin, cv2.RETR_EXTERNAL, method=method_0)
    # 声明画布 拷贝自img
    show_img = np.copy(img)
    # 外接矩形区域集合
    rects = []
    # print('len(contours)', len(contours))
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
    contours_cx = -1
    contours_cy = -1
    # 找到中心点所在的轮廓
    return_cnt = None
    for index, cnt in enumerate(contours):
        # 判断是否在轮廓内部
        if map_type == MapType.baidu:
            center = 512 * scale
            in_cnt = cv2.pointPolygonTest(cnt, (center, center), True)
        else:
            center = 512 * scale
            in_cnt = cv2.pointPolygonTest(cnt, (center, center), True)
        # -5 保留一定误差范围
        if in_cnt > -5:
            # 通过面积排除一些特别小的干扰
            (x, y, w, h) = cv2.boundingRect(cnt)
            if (w*h)<1000:
                continue
            # 计算轮廓的中心点
            M = cv2.moments(contours[index])  # 计算第一条轮廓的矩
            # print(M)
            # 这两行是计算中心点坐标
            contours_cx = int(M['m10'] / M['m00'])
            contours_cy = int(M['m01'] / M['m00'])
            show_img = cv2.drawContours(show_img, cnt, -1, (0, 0, 255), 3)
            return show_img, cnt, (contours_cx, contours_cy)
    return None, None, (contours_cx, contours_cy)


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
        cv2.rectangle(
            canvas, pt1=(
                x, y), pt2=(
                x + w, y + h), color=color, thickness=3)

    return canvas


# 判断地图上一点是否属于曾经出现在湖泊上的点
def is_in_contours(point, local_map_data):
    # 没有返回None
    if len(local_map_data) == 0:
        return None
    else:
        # 判断是否在轮廓内部
        # for index, cnt in enumerate(local_map_data['mapList']):
        for index, cnt in enumerate(local_map_data['mapList']):
            # 直接使用像素位置判断
            in_cnt = cv2.pointPolygonTest(
                np.array(cnt['pool_lng_lats']), (point[0], point[1]), False)
            # 使用经纬度判断
            # new_cnt = []
            # for i in cnt['mapData']:
            #     new_cnt.append([int(i[0]*1000000),int(i[1]*1000000)])
            # in_cnt = cv2.pointPolygonTest(np.array(new_cnt), (point[0][0],point[0][1]), False)
            # 大于0说明属于该轮廓
            if in_cnt >= 0:
                return cnt['id']
        # 循环结束返回None
        return None


def get_degree(lonA, latA, lonB, latB):
    """
    两点经纬度计算角度　以第一点为中心　第一点（经度，纬度），第二点（经度，纬度）
    Args:
        point p1(latA, lonA)
        point p2(latB, lonB)
    Returns:
        bearing between the two GPS points,
        default: the basis of heading direction is north
    """
    radLatA = radians(latA)
    radLonA = radians(lonA)
    radLatB = radians(latB)
    radLonB = radians(lonB)
    dLon = radLonB - radLonA
    y = sin(dLon) * cos(radLatB)
    x = cos(radLatA) * sin(radLatB) - sin(radLatA) * cos(radLatB) * cos(dLon)
    brng = degrees(atan2(y, x))
    brng = (brng + 360) % 360
    return_brg = 360 - brng
    if int(return_brg) == 360:
        return 0
    else:
        return return_brg

class MapType(enum.Enum):
    baidu = 1
    gaode = 2


class BaiduMap(object):
    def __init__(self, lng_lat,
                 zoom=None,
                 logger=None,
                 height=1024,
                 width=1024,
                 scale=1,
                 map_type=MapType.gaode):
        if logger is None:
            import logging
            logging.basicConfig(
                format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                level=logging.DEBUG)
            self.logger = logging
        else:
            self.logger = logger
        self.map_type = map_type
        # 访问秘钥
        self.baidu_key = 'wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
        self.gaode_key = '8177df6428097c5e23d3280ffdc5a13a'
        # 湖泊像素轮廓
        self.pool_cnts = []
        # 湖泊经纬度轮廓
        self.pool_lng_lats = []
        # 湖泊像素中心
        self.pool_center_cnt = []
        # 湖泊经纬度中心
        self.pool_center_lng_lat = []
        # 监测点像素
        self.scan_point_cnts = []
        # 监测点经纬度
        self.scan_point_lng_lats = []
        # 规划路径像素
        self.path_planning_cnts = []
        # 规划路径经纬度
        self.path_planning_lng_lats = []
        # 不在湖泊区域像素
        self.outpool_cnts_set = None
        # 不在湖泊区域经纬度
        self.outpool_lng_lats_set = []

        self.ship_gps=None
        self.ship_gaode_lng_lat=None
        self.ship_pix=None

        self.init_ship_gps=None
        self.init_ship_gaode_lng_lat = None
        self.init_ship_pix = None

        self.scale = scale

        # 请求地图位置经纬度
        self.lng_lat = lng_lat
        self.lng_lat_pix = (512 * self.scale,512 * self.scale)
        # 图像高度和宽度
        self.height = height
        self.width = width
        # 缩放比例
        self.zoom = zoom

        self.pix_to_meter = 0.12869689044 * math.pow(2, 19 - self.zoom)
        self.addr = str(round(100 * random.random(), 3))
        # ＨＳＶ阈值　［［低　ＨＳＶ］,　［高　ＨＳＶ］］
        self.threshold_hsv = [(84, 72, 245), (118, 97, 255)]
        # 百度不同缩放比例下比例尺对应的实际距离　　缩放尺寸　长度（米）　像素距离
        self.scale_map = {
            # {zoom: 19, length: 20, Pixels: 48},
            19: [20, 48],
            18: [50, 61],
            17: [100, 61],
            16: [200, 61],
            15: [500, 78],
            14: [1000, 78],
            13: [2000, 78],
            12: [5000, 99],
            11: [10000, 99],
            10: [20000, 99],
            9: [25000, 59],
            8: [50000, 59],
            7: [100000, 59],
            6: [200000, 59],
            5: [500000, 76],
            4: [1000000, 76],
        }
        save_img_dir = os.path.join(config.root_path, 'baiduMap','imgs')
        if not os.path.exists(save_img_dir):
            os.mkdir(save_img_dir)
        if self.map_type == MapType.baidu:
            self.save_img_path = os.path.join(
                save_img_dir, 'baidu_%f_%i_%i.png' %
                (self.lng_lat[0], self.lng_lat[1], self.zoom))
        elif self.map_type == MapType.gaode:
            self.save_img_path = os.path.join(
                save_img_dir, 'gaode_%f_%f_%i_%i.png' %
                (self.lng_lat[0], self.lng_lat[1], self.zoom, self.scale))
        if not os.path.exists(self.save_img_path):
            self.draw_image()

    # 获取地址的url
    def get_url(self, addr):
        self.addr = addr
        if len(addr) < 1:
            return None
        return 'http://api.map.baidu.com/geocoding/v3/?address={inputAddress}&output=json&ak={myAk}'.format(
            inputAddress=addr, myAk=self.baidu_key)

    # 通过地址url获取经纬度
    def get_position(self, addr):
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
        if self.map_type == MapType.baidu:
            return 'http://api.map.baidu.com/staticimage/v2?ak={myAk}&center={position}&width={width}&height={height}&zoom={zoom}'.format(
                myAk=self.baidu_key, position='%f,%f' % (self.lng_lat[0], self.lng_lat[1]), width=self.width,
                height=self.height,
                zoom=self.zoom)
        elif self.map_type == MapType.gaode:
            return 'https://restapi.amap.com/v3/staticmap?location={position}&zoom={zoom}&size={h}*{w}&scale={scale}&key={key}'.format(
                position='%f,%f' %
                (self.lng_lat[0], self.lng_lat[1]), zoom=(
                    self.zoom), h=self.height, w=self.width, scale=self.scale, key=self.gaode_key)

    # 按照经纬度url获取静态图
    def draw_image(self, ):
        png_url = self.get_image_url()
        self.logger.info({'png_url': png_url})
        response = requests.get(png_url)
        # 获取的文本实际上是图片的二进制文本
        img = response.content
        # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
        with open(self.save_img_path, 'wb') as f:
            f.write(img)

    # 静态图蓝色护坡区域抠图
    def get_pool_pix(self, b_show=False):
        """
        查找点击位置湖泊锁在的轮廓
        :param b_show: True显示轮廓图像
        :return:
        """
        self.logger.info({'save_img_path': self.save_img_path})
        if not os.path.exists(self.save_img_path):
            self.logger.error('no image')
        self.row_img = cv2.imread(self.save_img_path)
        # add edge
        h, w = self.row_img.shape[:2]
        self.row_img[0, :, :] = [0, 0, 0]
        self.row_img[h - 1, :, :] = [0, 0, 0]
        self.row_img[:, 0, :] = [0, 0, 0]
        self.row_img[:, w - 1, :] = [0, 0, 0]
        # cv2.imshow('test',self.row_img)
        # cv2.waitKey(0)
        # 图片路径
        # 颜色阈值下界(HSV) lower boudnary
        lowerb = self.threshold_hsv[0]
        # 颜色阈值上界(HSV) upper boundary
        upperb = self.threshold_hsv[1]

        # 读入素材图片 BGR
        # 检查图片是否读取成功
        if self.row_img is None:
            self.logger.error("Error: 无法找到保存的地图图片,请检查图片文件路径")
            return None, (-1, -1)

        # 识别色块 获取矩形区域数组
        self.show_img, pool_cnts, (contours_cx, contours_cy) = color_block_finder(
            self.row_img, lowerb, upperb, map_type=self.map_type, scale=self.scale)
        self.center_cnt = (contours_cx, contours_cy)
        if pool_cnts is None:
            self.logger.info('无法在点击处找到湖')
            return pool_cnts, (-2, -2)

        # 绘制色块的矩形区域
        cv2.circle(
            self.show_img, (contours_cx, contours_cy), 5, [
                255, 255, 0], -1)
        if b_show:
            cv2.namedWindow(
                'result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.imshow('result', self.show_img)
            # 等待任意按键按下
            cv2.waitKey(0)
            # 关闭其他窗口
            # cv2.destroyAllWindows()
        pool_cnts = np.squeeze(pool_cnts)
        self.pool_cnts = pool_cnts
        return self.pool_cnts, self.center_cnt

    # gps模块转换为高德经纬度
    def gps_to_gaode_lng_lat(self, lng_lat):
        url = 'https://restapi.amap.com/v3/assistant/coordinate/convert?locations={lng_lat}&coordsys=gps&key={key}'.format(
            lng_lat="%f,%f" % (lng_lat[0], lng_lat[1]), key=self.gaode_key)
        response = requests.get(url=url)
        response = json.loads(response.content)
        self.logger.info({'response':response})
        gaode_lng_lat = [float(i) for i in response['locations'].split(',')]
        return gaode_lng_lat

    # 高德经纬度转换转换为像素位置
    def gaode_lng_lat_to_pix(self, gaode_lng_lat):
        # 计算两点间距离和角度
        theta = lng_lat_calculate.angleFromCoordinate(
            self.lng_lat[0], self.lng_lat[1], gaode_lng_lat[0], gaode_lng_lat[1])
        distance = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0], self.lng_lat[1], gaode_lng_lat[0], gaode_lng_lat[1])
        delta_x_distance = math.sin(math.radians(theta)) * distance
        delta_y_distance = math.cos(math.radians(theta)) * distance

        delta_x_pix = -delta_x_distance / (self.pix_to_meter)
        delta_y_pix = -delta_y_distance / (self.pix_to_meter)
        pix = [int(self.height * self.scale / 2 + delta_x_pix),
               int(self.width * self.scale / 2 + delta_y_pix)]
        return pix

    # 区域像素点转换为经纬度坐标点
    def pix_to_gps(self, cnts):
        """
        :param cnt:
        :return:
        """
        self.logger.debug({'pix_to_gps len(cnts)':len(cnts)})
        # 返回经纬度坐标集合
        return_gps = []
        # 给后端的返回
        return_gps_list = []
        # 初始点（中心点）经纬度坐标
        # 初始点（中心点）像素坐标
        center = (self.width / 2, self.height / 2)
        draw_gps = ''
        for point in cnts:
            delta_pix_x = point[0] - center[0]
            delta_pix_y = point[1] - center[1]
            # baidu
            pix_2_meter = math.pow(2, 18 - self.zoom)
            # gaode
            pix_2_meter = 0.12859689044*math.pow(2, 19 - self.zoom)
            delta_meter_x = delta_pix_x * (pix_2_meter)
            delta_meter_y = delta_pix_y * (pix_2_meter)
            distance = math.sqrt(
                math.pow(
                    delta_meter_x,
                    2) +
                math.pow(
                    delta_meter_y,
                    2))
            # 方法一：直接计算
            # 方法二：当做圆球计算
            method = 0

            if method == 1:
                L = 6381372 * math.pi * 2
                W = L
                H = L / 2
                mill = 2.3
                delta_lat = ((H / 2 - delta_meter_y) * 2 * mill) / (1.25 * H)
                delta_lat = ((math.atan(math.exp(delta_lat)) -
                              0.25 * math.pi) * 180) / (0.4 * math.pi)
                delta_lon = (delta_meter_x - W / 2) * 360 / W

                center_lat = ((H / 2 - 0) * 2 * mill) / (1.25 * H)
                center_lat = ((math.atan(math.exp(center_lat)) -
                               0.25 * math.pi) * 180) / (0.4 * math.pi)
                center_lon = (0 - W / 2) * 360 / W

                gpx_x = -(center_lon - delta_lon)
                gpx_y = -(center_lat - delta_lat)

                # TODO 最终需要确认经纬度保留小数点后几位
                point_gps = [self.lng_lat[0] + gpx_x, self.lng_lat[1] + gpx_y]
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})

            elif method == 2:
                """
                    地理中常用的数学计算，把地球简化成了一个标准球形，如果想要推广到任意星球可以改成类的写法，然后修改半径即可
                """
                earth_radius = (
                    6370.8560) * 1000  # 地球平均半径，单位km，最简单的模型往往把地球当做完美的球形，这个值就是常说的RE  平均半径　6370.856　　赤道半径6378.1370
                math_2pi = math.pi * 2
                pis_per_degree = math_2pi / 360  # 角度一度所对应的弧度数，360对应2*pi
                # 计算维度上圆面半径
                real_radius = earth_radius * \
                    math.cos(self.lng_lat[1] * pis_per_degree)
                # 经度偏差
                delta_lng = (delta_meter_x / real_radius) / pis_per_degree
                # 纬度偏差
                delta_lat = -(delta_meter_y / earth_radius) / pis_per_degree
                # print(delta_lng,delta_lat)
                # TODO 最终需要确认经纬度保留小数点后几位
                point_gps = [
                    self.lng_lat[0] + delta_lng,
                    self.lng_lat[1] + delta_lat]
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})
                return_gps_list.append(point_gps)
            else:
                theta = round(
                    math.degrees(
                        math.atan2(
                            delta_meter_x, -delta_meter_y)), 1)
                theta = theta if theta > 0 else 360 + theta
                point_gps = lng_lat_calculate.one_point_diatance_to_end(
                    self.lng_lat[0], self.lng_lat[1], theta, distance)
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})
                draw_gps=draw_gps+'%f,%f;'%(point_gps[0],point_gps[1])
                return_gps_list.append(point_gps)
        with open('map.json', 'w') as f:
            json.dump({'gps':draw_gps}, f)
        return return_gps, return_gps_list

    def scan_pool(self, contour, pix_gap=20, safe_distance=20, b_show=False):
        """
        传入湖泊像素轮廓返回活泼扫描点
        :param contour 轮廓点
        :param pix_gap 指定扫描间隔，单位像素
        :param safe_distance
        """
        # 求坐标点最大外围矩阵
        (x, y, w, h) = cv2.boundingRect(contour)
        self.logger.debug({'(x, y, w, h)': (x, y, w, h)})
        # 循环生成点同时判断点是否在湖泊范围在则添加到列表中
        scan_points = []
        # 起始点
        start_x, start_y = x, y + pix_gap
        # 当前点
        current_x, current_y = start_x, start_y
        # 判断x轴是递增的加还是减 True 为加
        b_add_or_sub = True
        while current_y < (y + h):
            while current_x <= (x + w) and current_x >= x:
                point = (current_x, current_y)
                in_cnt = cv2.pointPolygonTest(contour, point, True)
                if in_cnt > safe_distance:
                    scan_points.append(list(point))
                if b_add_or_sub:
                    current_x += pix_gap
                else:
                    current_x -= pix_gap
            current_y += pix_gap
            if b_add_or_sub:
                current_x -= pix_gap
                b_add_or_sub = False
            else:
                current_x += pix_gap
                b_add_or_sub = True
        for point in scan_points:
            cv2.circle(self.show_img, tuple(point), 5, (0, 255, 255), -1)
        if b_show:
            # cv2.polylines(self.show_img,[np.array(scan_points,dtype=np.int32)],False,(255,0,0),2)
            cv2.imshow('scan', self.show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        self.scan_point_cnts = scan_points
        return self.scan_point_cnts



if __name__ == '__main__':
    # obj = BaiduMap([114.432092, 30.522893], zoom=16,map_type=MapType.gaode)
    # obj = BaiduMap([114.431529, 30.524413], zoom=15, scale=1, map_type=MapType.gaode)
    # obj = BaiduMap([114.438009, 30.540082], zoom=14, scale=1, map_type=MapType.gaode)
    # obj = BaiduMap([114.373904, 30.540625], zoom=14, scale=1, map_type=MapType.gaode)
    obj = BaiduMap([114.431529, 30.524413], zoom=15,
                   scale=1, map_type=MapType.gaode)
    # obj = BaiduMap([114.393142, 30.558963], zoom=15,map_type=MapType.baidu)
    # obj = BaiduMap([114.718257,30.648004],zoom=14)
    # obj = BaiduMap([114.566767,30.541689],zoom=14)
    # obj = BaiduMap([114.565976,30.541317],zoom=15.113213)
    # obj = BaiduMap([114.393142,30.558981],zoom=14)
    gaode_lng_lat = obj.gps_to_gaode_lng_lat([114.431529, 30.524413])
    pix = obj.gaode_lng_lat_to_pix([114.431529, 30.525413])
    print('pix',pix)
    print(type(gaode_lng_lat), gaode_lng_lat)
    pool_cnts, (pool_cx, pool_cy) = obj.get_pool_pix(b_show=False)
    print('pool_cnts', len(pool_cnts))
    scan_cnts = obj.scan_pool(pool_cnts, pix_gap=30, b_show=True)
    obj.pix_to_gps(obj.pool_cnts)
    if pool_cnts is None:
        pass
    else:
        all_cnt = []
        all_cnt.extend(list(pool_cnts))
        all_cnt.extend(scan_cnts)
        gps = obj.pix_to_gps(all_cnt)
        # print(gps)
        # 请求指定位置图片
        # obj.draw_image()
        # 求坐标点最大外围矩阵
        (x, y, w, h) = cv2.boundingRect(pool_cnts)
        print('(x, y, w, h)', (x, y, w, h))
