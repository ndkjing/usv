
import requests
import json
import random
import cv2
import numpy as np
import math
import os
from math import radians,cos,sin,degrees,atan2
import copy

from utils import lng_lat_calculate
method_0 = cv2.CHAIN_APPROX_NONE
method_1 = cv2.CHAIN_APPROX_SIMPLE

def color_block_finder(img, lowerb, upperb,
                       min_w=0, max_w=None, min_h=0, max_h=None,):
    '''
    色块识别 返回矩形信息，若没有找到返回矩形框为None
    '''
    # 转换色彩空间 HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 根据颜色阈值转换为二值化图像
    img_bin = cv2.inRange(img_hsv, lowerb, upperb)

    # 寻找轮廓（只寻找最外侧的色块）
    contours, hier = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, method=method_0)
    # 声明画布 拷贝自img
    show_img = np.copy(img)
    # 外接矩形区域集合
    rects = []
    print('len(contours)',len(contours))
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
    in_cnt=-1
    contours_cx = -1
    contours_cy = -1
    # 找到中心点所在的轮廓
    return_cnt=None
    for index,cnt in enumerate(contours):
        # 判断是否在轮廓内部
        in_cnt = cv2.pointPolygonTest(cnt,(1024,1024),True)
        # print('in cnt',in_cnt)
        if in_cnt>2:
            print('index,cnt',index,cnt)
            # print('len(cnt)',len(cnt))
            # 计算轮廓的中心点
            M = cv2.moments(contours[index])  # 计算第一条轮廓的矩
            # print(M)
            # 这两行是计算中心点坐标
            contours_cx = int(M['m10'] / M['m00'])
            contours_cy = int(M['m01'] / M['m00'])
            show_img = cv2.drawContours(show_img, cnt, -1, (0, 0, 255), 3)
            return show_img,cnt,(contours_cx,contours_cy)
    return None,None,(contours_cx,contours_cy)


class GaodeMap(object):
    def __init__(self,lng_lat,ak='8177df6428097c5e23d3280ffdc5a13a',zoom=None,logger=None,height=1024,width=1024):
        if logger == None:
            import logging
            logging.basicConfig(format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                                level=logging.DEBUG)
            self.logger = logging
        else:
            self.logger = logger

        # 在湖泊中生产的轮廓经纬度和中心经纬度
        self.pool_cnts = []
        self.pool_lng_lats = []
        self.pool_center_cnt = []
        self.pool_center_lng_lat = []
        self.scan_point_cnts = []
        self.scan_point_lng_lats = []
        self.path_planning_cnts=[]
        self.path_planning_lng_lats=[]
        self.outpool_cnts_set=None
        self.outpool_lng_lats_set=[]

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
        if not os.path.exists('./imgs'):
            os.mkdir('./imgs')
        self.save_img_path = './imgs/%f_%f_%i.png'%(self.lng_lat[0],self.lng_lat[1],self.zoom)
        if not os.path.exists(self.save_img_path):
            self.draw_image()

    # 获取地址的url
    def get_url(self,addr):
        self.addr = addr
        if len(addr) < 1:
            return None
        return 'https://restapi.amap.com/v3/staticmap?markers=mid,0xFF0000,A:116.37359, 39.92437;116.47359, 39.92437 &key = 您的key'.format(
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
             https://restapi.amap.com/v3/staticmap?location=116.48482,39.94858&zoom=10&size=400*400&key=您的key
        '''
        return 'https://restapi.amap.com/v3/staticmap?location={position}&zoom={zoom}&size=1024*1024&scale=2&key={key}'.format(
            position='%f,%f' % (self.lng_lat[0], self.lng_lat[1]), zoom=self.zoom-1,key='8177df6428097c5e23d3280ffdc5a13a')
    # 按照经纬度url获取静态图
    def draw_image(self,):
        png_url1 = self.get_image_url()
        print('png_url1',png_url1)

        response = requests.get(png_url1)
        # 获取的文本实际上是图片的二进制文本
        img = response.content
        # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
        with open(self.save_img_path,'wb' ) as f:
            f.write(img)

    # 静态图蓝色护坡区域抠图
    def get_pool_pix(self,b_show=False):
        """
        查找点击位置湖泊锁在的轮廓
        :param b_show: True显示轮廓图像
        :return:
        """
        self.logger.info({'save_img_path':self.save_img_path})
        if not os.path.exists(self.save_img_path):
            self.logger.error('no image')
        self.row_img = cv2.imread(self.save_img_path)
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
        self.show_img, pool_cnts,(contours_cx,contours_cy) = color_block_finder(self.row_img, lowerb, upperb)
        self.center_cnt =(contours_cx,contours_cy)
        if pool_cnts is None:
            self.logger.info('无法在点击处找到湖')
            return pool_cnts,(-2,-2)

        # 绘制色块的矩形区域
        cv2.circle(self.show_img, (contours_cx, contours_cy), 5, [255, 255, 0], -1)
        if b_show:
            cv2.namedWindow('result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.imshow('result', self.show_img)
            # 等待任意按键按下
            cv2.waitKey(0)
            # 关闭其他窗口
            # cv2.destroyAllWindows()
        pool_cnts = np.squeeze(pool_cnts)
        self.pool_cnts = pool_cnts
        return self.pool_cnts,self.center_cnt


if __name__ == '__main__':
    obj = GaodeMap([114.431899,30.524176],zoom=16)
    obj.get_pool_pix(b_show=True)