import enum
import json
import logging
import math
import os
import random
import cv2
import numpy as np
import requests
import copy
import server_config
from utils import lng_lat_calculate

logging.getLogger("urllib3").setLevel(logging.WARNING)

# 方法一：找所有点
method_0 = cv2.CHAIN_APPROX_NONE
# 方法二：找最简单包围的点，多点共线就省略点
method_1 = cv2.CHAIN_APPROX_SIMPLE
pool_x, pool_y, pool_w, pool_h = 0, 0, 0, 0


def color_block_finder(img,
                       lowerb,
                       upperb,
                       map_type=None,
                       scale=1):
    """
    色块识别 返回矩形信息，若没有找到返回矩形框为None
    """
    # 转换色彩空间 HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 根据颜色阈值转换为二值化图像
    img_bin = cv2.inRange(img_hsv, lowerb, upperb)
    h = img.shape[0]
    # 寻找轮廓（只寻找最外侧的色块）版本不同返回不同
    if server_config.sysstr == "Linux":
        try:
            _, contours, hier = cv2.findContours(
                img_bin, cv2.RETR_EXTERNAL, method=method_0)
        except ValueError:
            contours, hier = cv2.findContours(
                img_bin, cv2.RETR_EXTERNAL, method=method_0)
    else:
        contours, hier = cv2.findContours(
            img_bin, cv2.RETR_EXTERNAL, method=method_0)
    # 声明画布 拷贝自img
    show_img = np.copy(img)
    # if max_w is None:
    #     # 如果最大宽度没有设定，就设定为图像的宽度
    #     max_w = img.shape[1]
    # if max_h is None:
    #     # 如果最大高度没有设定，就设定为图像的高度
    #     max_h = img.shape[0]
    contours_cx = -1
    contours_cy = -1
    # 找到中心点所在的轮廓
    for index, cnt in enumerate(contours):
        # 判断是否在轮廓内部
        if map_type == MapType.baidu:
            # center = 512 * scale
            center = h // 2
            in_cnt = cv2.pointPolygonTest(cnt, (center, center), True)
        else:
            # center = 512 * scale
            center = h // 2
            in_cnt = cv2.pointPolygonTest(cnt, (center, center), True)
        # -5 保留一定误差范围
        if in_cnt > -5:
            # 通过面积排除一些特别小的干扰
            (x_, y_, w_, h_) = cv2.boundingRect(cnt)
            if (w_ * h_) < 100:
                continue
            # 计算轮廓的中心点
            m = cv2.moments(contours[index])  # 计算第一条轮廓的矩
            # print(M)
            # 这两行是计算中心点坐标
            contours_cx = int(m['m10'] / m['m00'])
            contours_cy = int(m['m01'] / m['m00'])
            show_img = cv2.drawContours(show_img, cnt, -1, (0, 0, 255), 3)
            return show_img, cnt, (contours_cx, contours_cy)
    return None, None, (contours_cx, contours_cy)


def draw_color_block_rect(img, rects, color=(0, 0, 255)):
    """
    绘制色块的矩形区域
    """
    # 声明画布(canvas) 拷贝自img
    canvas = np.copy(img)
    # 遍历矩形区域
    for rect in rects:
        (x_draw, y_draw, w_draw, h_draw) = rect
        # 在画布上绘制矩形区域（红框）
        cv2.rectangle(
            canvas, pt1=(
                x_draw, y_draw), pt2=(
                x_draw + w_draw, y_draw + h_draw), color=color, thickness=3)
    return canvas


# 判断地图上一点是否属于曾经出现在湖泊上的点
def is_in_contours(point_, local_map_data):
    # 没有返回None
    if len(local_map_data) == 0:
        return None
    else:
        # 判断是否在轮廓内部
        for index, cnt in enumerate(local_map_data['mapList']):
            # 直接使用像素位置判断
            in_cnt = cv2.pointPolygonTest(
                np.array(cnt['pool_lng_lats']), (point_[0], point_[1]), True)
            # 使用经纬度判断 大于0说明属于该轮廓
            print('in_cnt', in_cnt)
            if in_cnt >= 0:
                print(r'cnt id ', cnt['id'])
                print('周长', cal_map_circle(cnt))
                return cnt['id']
        # 循环结束返回None
        return None


# 计算地图轮廓周长 返回单位米
def cal_map_circle(cnt):
    """
    @return:
    """
    sum_circle = 0
    f_point = None
    s_point = None
    for index, point in enumerate(cnt['pool_lng_lats']):
        if index == 0:
            f_point = [point[0] / 1000000, point[1] / 1000000]
            continue
        else:
            s_point = [point[0] / 1000000, point[1] / 1000000]
        if index == len(cnt['pool_lng_lats']) - 1:
            return sum_circle
        if f_point is not None and s_point is not None:
            sum_circle += lng_lat_calculate.distanceFromCoordinate(f_point[0],
                                                                   f_point[1],
                                                                   s_point[0],
                                                                   s_point[1])
            f_point = copy.deepcopy(s_point)


# 枚举地图类型
class MapType(enum.Enum):
    baidu = 1
    gaode = 2
    tecent = 3


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
        zoom = min(17, zoom)
        self.map_type = map_type
        # 访问秘钥
        self.baidu_key = server_config.baidu_key
        self.gaode_key = server_config.gaode_key
        self.tecent_key = server_config.tencent_key
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

        self.ship_gps = None
        self.ship_gaode_lng_lat = None
        self.ship_pix = None

        self.init_ship_gps = None
        self.init_ship_gaode_lng_lat = None
        self.init_ship_pix = None

        self.scale = scale
        # 障碍物地图
        self.obstacle_map = None
        # 请求地图位置经纬度
        self.lng_lat = lng_lat
        self.lng_lat_pix = (512 * self.scale, 512 * self.scale)
        # 图像高度和宽度
        self.height = height
        self.width = width
        # 缩放比例
        self.zoom = zoom

        # 湖泊地址
        self.pool_name = None
        # 湖泊名称
        self.address = None
        # gaode
        if self.map_type == MapType.gaode:
            self.pix_2_meter = 0.12859689044 * math.pow(2, 19 - self.zoom)
        elif self.map_type == MapType.tecent:
            self.pix_2_meter = 0.515071433939516 * math.pow(2, 18 - self.zoom)
        elif self.map_type == MapType.baidu:
            # baidu
            self.pix_2_meter = 1 * math.pow(2, 18 - self.zoom)

        self.addr = str(round(100 * random.random(), 3))
        # ＨＳＶ阈值　［［低　ＨＳＶ］,　［高　ＨＳＶ］］
        self.gaode_threshold_hsv = [(84, 72, 245), (118, 97, 255)]
        self.tecent_threshold_hsv = [(84, 72, 245), (118, 130, 255)]
        # save_img_dir = os.path.join(config.root_path, 'baiduMap','imgs')
        save_img_dir = os.path.join(server_config.root_path, 'statics', 'imgs')
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
        elif self.map_type == MapType.tecent:
            self.save_img_path = os.path.join(
                save_img_dir, 'tecent_%f_%f_%i_%i.png' %
                              (self.lng_lat[0], self.lng_lat[1], self.zoom, self.scale))
        # 不存在图片则保存图片
        if not os.path.exists(self.save_img_path):
            try:
                self.draw_image()
            # 下载出错则删除可能出错的图片
            except Exception as e:
                self.logger.error({'error': e})
                if os.path.exists(self.save_img_path):
                    os.remove(self.save_img_path)
        self.show_img = None
        self.center_cnt = None
        self.province = ''  # 省
        self.city = ''  # 市
        self.district = ''  # 区
        self.township = ''  # 街道

    def build_obstacle_map(self, b_show_obstacle_map=False):
        """
        构建obstacle地图
        :return:
        """
        h, w = self.row_img.shape[:2]
        # scale = cell_size / self.pix_2_meter
        # map_width = int(w / scale)
        # map_height = int(h / scale)
        # map_height, map_width = min(map_height,cell_hw),min(map_width,cell_hw)
        # print('map_height, map_width', map_height, map_width)
        # print('湖泊宽长',w*self.pix_2_meter, h*self.pix_2_meter)
        if self.pool_cnts is not None:
            self.obstacle_map = np.zeros((h, w))
            for i in range(w):
                for j in range(h):
                    # 对应原图上坐标点
                    pix_point = (i, j)
                    in_cnt = cv2.pointPolygonTest(self.pool_cnts, pix_point, True)
                    if in_cnt > 0:
                        self.obstacle_map[j, i] = 255
            if b_show_obstacle_map:
                cv2.imshow('obstacle_map', self.obstacle_map)
                cv2.waitKey(0)

    def update_obstacle_map(self, gaode_point, b_show=False):
        pix_target = self.gaode_lng_lat_to_pix(gaode_point)
        print('pix_target', pix_target)
        self.obstacle_map[pix_target[1], pix_target[0]] = 0
        if b_show:
            cv2.imshow('obstacle_map', self.obstacle_map)
            cv2.waitKey(0)

    # 获取地址的url
    def get_url(self, addr):
        self.addr = addr
        if len(addr) < 1:
            return None
        return 'http://api.map.baidu.com/geocoding/v3/?address={inputAddress}&output=json&ak={myAk}'.format(
            inputAddress=addr, myAk=self.baidu_key)

    # 通过地址url获取经纬度
    def get_position(self, addr):
        """
        返回经纬度信息
        """
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
    def get_image_url(self, lng_lat=None):
        """
            调用地图API获取待查询地址专属url
        """
        if lng_lat:
            cal_lng_lat = lng_lat
        else:
            cal_lng_lat = self.lng_lat
        if self.map_type == MapType.baidu:
            return 'http://api.map.baidu.com/staticimage/v2?ak={myAk}&center={position}&width={width}&height={height}&zoom={zoom}'.format(
                myAk=self.baidu_key, position='%f,%f' % (cal_lng_lat[0], cal_lng_lat[1]), width=self.width,
                height=self.height,
                zoom=self.zoom)
        elif self.map_type == MapType.gaode:
            return 'https://restapi.amap.com/v3/staticmap?location={position}&zoom={zoom}&size={h}*{w}&scale={scale}&key={key}'.format(
                position='%f,%f' %
                         (cal_lng_lat[0], cal_lng_lat[1]), zoom=(
                    self.zoom), h=self.height, w=self.width, scale=self.scale, key=self.gaode_key)

        elif self.map_type == MapType.tecent:
            return 'https://apis.map.qq.com/ws/staticmap/v2/?center={position}&zoom={zoom}&size={h}*{w}&scale={scale}&maptype=roadmap&key={key}'.format(
                position='%f,%f' %
                         (cal_lng_lat[1], cal_lng_lat[0]), zoom=(
                    self.zoom), h=self.height, w=self.width, scale=self.scale, key=self.tecent_key)

    # 获取地址url
    def get_address_url(self):
        """
            调用地图API获取经纬度查询地址url
        """
        #  https://restapi.amap.com/v3/geocode/regeo?output=json&location=114.524096, 30.506853&key=8177df6428097c5e23d3280ffdc5a13a&radius=1000&extensions=all
        return 'https://restapi.amap.com/v3/geocode/regeo?output=json&location={position}&key={key}&radius=1000&extensions=all&poitype=湖泊'.format(
            position='%f,%f' % (self.lng_lat[0], self.lng_lat[1]), key=self.gaode_key)

    def get_pool_name(self):
        pool_name = None  # 湖泊名称
        address = None  # 地址
        province = ''  # 省
        city = ''  # 市
        district = ''  # 区
        township = ''  # 街道
        address_url = self.get_address_url()
        response = requests.get(address_url)
        address_response_data = json.loads(response.content)
        if int(address_response_data.get('status')) == 1:
            if len(address_response_data.get('regeocode').get('pois')) > 0:
                pool_name = address_response_data.get('regeocode').get('pois')[0].get('name')
            address = address_response_data.get('regeocode').get('formatted_address')
            if address_response_data.get('regeocode'):
                if address_response_data.get('regeocode').get('addressComponent'):
                    if address_response_data.get('regeocode').get('addressComponent').get('province'):
                        province = address_response_data.get('regeocode').get('addressComponent').get('province')
                    if address_response_data.get('regeocode').get('addressComponent').get('city'):
                        city = address_response_data.get('regeocode').get('addressComponent').get('city')
                    if address_response_data.get('regeocode').get('addressComponent').get('district'):
                        district = address_response_data.get('regeocode').get('addressComponent').get('district')
                    if address_response_data.get('regeocode').get('addressComponent').get('township'):
                        township = address_response_data.get('regeocode').get('addressComponent').get('township')
        self.pool_name = pool_name
        self.address = address
        self.province = province
        self.city = city
        self.district = district
        self.township = township

    @staticmethod
    def get_area_code(lng_lat):
        address_url = 'https://restapi.amap.com/v3/geocode/regeo?output=json&location={position}&key={key}&radius=1000&extensions=all&poitype=湖泊'.format(
            position='%f,%f' % (lng_lat[0], lng_lat[1]), key=server_config.gaode_key)
        response = requests.get(address_url)
        address_response_data = json.loads(response.content)
        if int(address_response_data.get('status')) == 1:
            adcode = address_response_data.get('regeocode').get('addressComponent').get('adcode')
            return adcode

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
        查找点击位置湖泊所在的轮廓
        :param b_show: True显示轮廓图像
        :return:
        """
        self.logger.info({'save_img_path': self.save_img_path})
        # 图片路径
        if not os.path.exists(self.save_img_path):
            self.logger.error('no image')
            return None, (1003, 1003)
        self.row_img = cv2.imread(self.save_img_path)
        # 检查图片是否读取成功
        if self.row_img is None:
            self.logger.error("Error: 无法找到保存的地图图片,请检查图片文件路径")
            return None, (1004, 1004)
        # 为了点击位置层次过大没有找到全部湖轮廓或报错，手动给图片边缘添加一个边框
        h, w = self.row_img.shape[:2]
        self.row_img[0, :, :] = [0, 0, 0]
        self.row_img[h - 1, :, :] = [0, 0, 0]
        self.row_img[:, 0, :] = [0, 0, 0]
        self.row_img[:, w - 1, :] = [0, 0, 0]

        # 颜色阈值下界(HSV)不同地图类型 lower boudnary # 颜色阈值上界(HSV) upper boundary
        if self.map_type == MapType.tecent:
            lowerb = self.tecent_threshold_hsv[0]
            upperb = self.tecent_threshold_hsv[1]
        else:
            lowerb = self.gaode_threshold_hsv[0]
            upperb = self.gaode_threshold_hsv[1]

        # 识别色块 获取矩形区域数组
        self.show_img, pool_cnts, (contours_cx, contours_cy) = color_block_finder(
            self.row_img, lowerb, upperb, map_type=self.map_type, scale=self.scale)
        self.center_cnt = (contours_cx, contours_cy)
        if pool_cnts is None:
            self.logger.info('无法在点击处找到湖')
            return None, (1001, 1001)
        pool_cnts = np.squeeze(pool_cnts)
        self.pool_cnts = pool_cnts
        (pool_x, pool_y, pool_w, pool_h) = cv2.boundingRect(self.pool_cnts)
        print((pool_x, pool_y, pool_w, pool_h))
        print('湖泊宽长', pool_w * self.pix_2_meter, pool_h * self.pix_2_meter)
        # 绘制色块的矩形区域
        cv2.circle(
            self.show_img, (self.center_cnt[0], self.center_cnt[1]), 5, [
                0, 255, 0], -1)
        if b_show:
            cv2.namedWindow(
                'result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.imshow('result', self.show_img)
            # 等待任意按键按下
            cv2.waitKey(0)
            # 关闭其他窗口
            cv2.destroyAllWindows()
        return self.pool_cnts, self.center_cnt

    # 静态图蓝色护坡区域抠图
    # def get_pool_pix_func(self, b_show=False):
    #     """
    #     查找点击位置湖泊所在的轮廓
    #     :param b_show: True显示轮廓图像
    #     :return:
    #     """
    #     self.logger.info({'save_img_path': self.save_img_path})
    #     # 图片路径
    #     if not os.path.exists(self.save_img_path):
    #         self.logger.error('no image')
    #         return None, (1003, 1003)
    #     self.row_img = cv2.imread(self.save_img_path)
    #     # 检查图片是否读取成功
    #     if self.row_img is None:
    #         self.logger.error("Error: 无法找到保存的地图图片,请检查图片文件路径")
    #         return None, (1004, 1004)
    #     # 为了点击位置层次过大没有找到全部湖轮廓或报错，手动给图片边缘添加一个边框
    #     h, w = self.row_img.shape[:2]
    #     self.row_img[0, :, :] = [0, 0, 0]
    #     self.row_img[h - 1, :, :] = [0, 0, 0]
    #     self.row_img[:, 0, :] = [0, 0, 0]
    #     self.row_img[:, w - 1, :] = [0, 0, 0]
    #
    #     # 颜色阈值下界(HSV)不同地图类型 lower boudnary # 颜色阈值上界(HSV) upper boundary
    #     if self.map_type == MapType.tecent:
    #         lowerb = self.tecent_threshold_hsv[0]
    #         upperb = self.tecent_threshold_hsv[1]
    #     else:
    #         lowerb = self.gaode_threshold_hsv[0]
    #         upperb = self.gaode_threshold_hsv[1]
    #     # 识别色块 获取矩形区域数组
    #     self.show_img, pool_cnts, (contours_cx, contours_cy) = color_block_finder(
    #         self.row_img, lowerb, upperb, map_type=self.map_type, scale=self.scale)
    #     self.center_cnt = (contours_cx, contours_cy)
    #     if pool_cnts is None:
    #         self.logger.info('无法在点击处找到湖')
    #         return None, (1001, 1001)
    #     pool_cnts = np.squeeze(pool_cnts)
    #     self.pool_cnts = pool_cnts
    #     (pool_x, pool_y, pool_w, pool_h) = cv2.boundingRect(self.pool_cnts)
    #     print((pool_x, pool_y, pool_w, pool_h))
    #     print('湖泊宽长', pool_w * self.pix_2_meter, pool_h * self.pix_2_meter)
    #     # 绘制色块的矩形区域
    #     cv2.circle(
    #         self.show_img, (self.center_cnt[0], self.center_cnt[1]), 5, [
    #             0, 255, 0], -1)
    #     if b_show:
    #         cv2.namedWindow(
    #             'result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    #         cv2.imshow('result', self.show_img)
    #         cv2.imwrite(self.save_img_path.split('.png')[0]+'line.png',self.show_img)
    #         # 等待任意按键按下
    #         cv2.waitKey(0)
    #         # 关闭其他窗口
    #         cv2.destroyAllWindows()
    #     return self.pool_cnts, self.center_cnt

    # def get_pool_pix(self):
    #     for concate_i in range(1, 5):
    #         # src_point = [114.431400, 30.523558]
    #         # 1请求图片
    #         row = 1 + 2 * concate_i
    #         col = 1 + 2 * concate_i
    #         image_paths = [""] * row ** 2
    #         # image_paths[1 + (row ** 2) // 2] = obj.save_img_path  # 中间图片地址
    #         # print(obj.get_pool_name())
    #         # print(obj.get_area_code(src_point))
    #         # 2获取湖泊轮廓
    #         pool_cnts, (pool_cx, pool_cy) = self.get_pool_pix_func(b_show=False)
    #
    #         # 3判断湖泊轮廓是否包含直线边线，20个像素在边沿且是连续直线   则认为有 走到->4  没有则结束
    #         if isinstance(pool_cnts, np.ndarray):
    #             save_pool_cnts = pool_cnts.tolist()
    #         else:
    #             save_pool_cnts = pool_cnts
    #         print('长度', len(save_pool_cnts))
    #         b_need_concate = False  # 是否需要拼接
    #         side_count = 0
    #         for index, item in enumerate(save_pool_cnts):
    #             if index == len(save_pool_cnts) - 1:
    #                 break
    #             if item[0] == self.width - 2 or item[0] == 1 or item[1] == self.height - 2 or item[1] == 1:
    #                 side_count += 1
    #             else:
    #                 side_count = 0
    #             if side_count >= 50:
    #                 b_need_concate = True
    #                 side_count = 0
    #                 break
    #         if b_need_concate:
    #             for j in range(col):
    #                 # 当列变化时需要修改最上面一点经纬度
    #                 row_distance = (row // 2 - j) * self.pix_2_meter * self.width
    #                 col_distance = col // 2 * self.pix_2_meter * self.width
    #                 theta = math.degrees(math.atan2(row_distance, col_distance))
    #                 theta = (theta + 360) % 360
    #                 top_lng_lat = lng_lat_calculate.one_point_diatance_to_end(self.lng_lat[0],
    #                                                                           self.lng_lat[1],
    #                                                                           theta,
    #                                                                           (
    #                                                                                   row_distance ** 2 + col_distance ** 2) ** 0.5)
    #                 for i in range(row):
    #                     delta_distance_lat = self.pix_2_meter * self.height * i  # 纬度偏差距离
    #                     # delta_distance_lng = self.map_obj.pix_2_meter * self.image_size * (j + 1)  # 经度偏差距离
    #                     # delta_distance = ((delta_distance_lat ** 2) + (delta_distance_lng ** 2)) ** 0.5
    #                     # 计算下一点经纬度
    #                     lng_lat = lng_lat_calculate.one_point_diatance_to_end(top_lng_lat[0], top_lng_lat[1],
    #                                                                           180, delta_distance_lat)
    #                     save_img_path = self.save_img_path.split('.png')[0] + '_' + str(concate_i) + '_' + str(
    #                         i + j * row) + '.png'
    #                     image_paths[i + j * col] = save_img_path
    #                     if not os.path.exists(save_img_path):
    #                         png_url = self.get_image_url(lng_lat)
    #                         response = requests.get(png_url)
    #                         # 获取的文本实际上是图片的二进制文本
    #                         img = response.content
    #                         # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
    #                         with open(save_img_path, 'wb') as f:
    #                             f.write(img)
    #                         print('保存图片', save_img_path)
    #             concate_image_path = self.save_img_path.split('.png')[0] + '_concate.png'  # 合并图片路径
    #             print('拼接图片路径:', concate_image_path)
    #             # 计算合并图片尺寸
    #             if not os.path.exists(concate_image_path):
    #                 vis = np.zeros((row * 1024, col * 1024, 3), dtype="uint8")
    #                 for j in range(col):
    #                     for i in range(row):
    #                         save_img_path = image_paths[i + j * col]
    #                         image = cv2.imread(save_img_path)
    #                         vis[1024 * i:1024 * (i + 1), 1024 * j:1024 * (j + 1)] = image
    #                 cv2.imwrite(concate_image_path, vis)
    #             # 5 再次获取湖泊轮廓 转到3
    #             self.save_img_path = concate_image_path
    #         else:
    #             return pool_cnts, (pool_cx, pool_cy)
    #         if concate_i == 4:
    #             return pool_cnts, (pool_cx, pool_cy)

    @staticmethod
    def gps_to_gaode_lng_lat(lng_lat):
        """
        gps模块经纬度转换为高德经纬度
        :param lng_lat: 真实gps 列表，[经度，纬度]
        :return:高德经纬度 列表，[经度，纬度]
        """
        url = 'https://restapi.amap.com/v3/assistant/coordinate/convert?locations={lng_lat}&coordsys=gps&key={key}'.format(
            lng_lat="%f,%f" % (lng_lat[0], lng_lat[1]), key=server_config.gaode_key)
        response = requests.get(url=url)
        response = json.loads(response.content)
        gaode_lng_lat = [float(i) for i in response['locations'].split(',')]
        return gaode_lng_lat

    def gaode_lng_lat_to_pix(self, gaode_lng_lat):
        """
        高德经纬度转换转换为像素位置
        :param gaode_lng_lat: 高德经纬度 列表，[经度，纬度]
        :return: 经纬度在当前地图对象图像上的像素位置
        """
        # 计算两点间距离和角度
        theta = lng_lat_calculate.angleFromCoordinate(
            self.lng_lat[0], self.lng_lat[1], gaode_lng_lat[0], gaode_lng_lat[1])
        distance = lng_lat_calculate.distanceFromCoordinate(
            self.lng_lat[0], self.lng_lat[1], gaode_lng_lat[0], gaode_lng_lat[1])
        # theta = 360 - theta
        # print('theta', theta)
        theta1 = lng_lat_calculate.angleFromCoordinate(
            gaode_lng_lat[0], gaode_lng_lat[1], self.lng_lat[0], self.lng_lat[1])
        # print('theta1', theta1)
        if 0 <= theta < 90:
            delta_x_distance = math.sin(math.radians(theta)) * distance
            delta_y_distance = math.cos(math.radians(theta)) * distance
            delta_x_pix = -delta_x_distance / (self.pix_2_meter)
            delta_y_pix = -delta_y_distance / (self.pix_2_meter)
            pix = [int(self.width * self.scale / 2 + delta_x_pix),
                   int(self.height * self.scale / 2 + delta_y_pix)]
        elif 90 <= theta < 180:
            t2_theta = 180 - theta
            delta_x_distance = math.sin(math.radians(t2_theta)) * distance
            delta_y_distance = math.cos(math.radians(t2_theta)) * distance
            delta_x_pix = -delta_x_distance / self.pix_2_meter
            delta_y_pix = delta_y_distance / self.pix_2_meter
            pix = [int(self.width * self.scale / 2 + delta_x_pix),
                   int(self.height * self.scale / 2 + delta_y_pix)]
        elif 180 <= theta < 270:
            t3_theta = 270 - theta
            delta_x_distance = math.cos(math.radians(t3_theta)) * distance
            delta_y_distance = math.sin(math.radians(t3_theta)) * distance
            delta_x_pix = delta_x_distance / self.pix_2_meter
            delta_y_pix = delta_y_distance / self.pix_2_meter
            pix = [int(self.width * self.scale / 2 + delta_x_pix),
                   int(self.height * self.scale / 2 + delta_y_pix)]
        else:
            t4_theta = 360 - theta
            # print('t4_theta', t4_theta)
            delta_x_distance = math.sin(math.radians(t4_theta)) * distance
            delta_y_distance = math.cos(math.radians(t4_theta)) * distance
            delta_x_pix = delta_x_distance / (self.pix_2_meter)
            delta_y_pix = -delta_y_distance / (self.pix_2_meter)
            pix = [int(self.width * self.scale / 2 + delta_x_pix),
                   int(self.height * self.scale / 2 + delta_y_pix)]
        return pix

    # 区域像素点转换为经纬度坐标点
    def pix_to_gps(self, cnts):
        """
        :param cnts:二维矩阵
        :return:
        """
        self.logger.debug({'pix_to_gps len(cnts)': len(cnts)})
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

            delta_meter_x = delta_pix_x * self.pix_2_meter
            delta_meter_y = delta_pix_y * self.pix_2_meter
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
                point_gps = [
                    self.lng_lat[0] + delta_lng,
                    self.lng_lat[1] + delta_lat]
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})
                return_gps_list.append(point_gps)
            else:
                theta = round(
                    math.degrees(
                        math.atan2(
                            delta_meter_x, -delta_meter_y)), 3)
                theta = theta if theta > 0 else 360 + theta
                theta = 360 - theta
                point_gps = lng_lat_calculate.one_point_diatance_to_end(
                    self.lng_lat[0], self.lng_lat[1], theta, distance)
                return_gps.append({"lat": point_gps[1], "lng": point_gps[0]})
                draw_gps = draw_gps + '%f,%f;' % (point_gps[0], point_gps[1])
                return_gps_list.append(point_gps)
        # with open('map.json', 'w') as f:
        #     json.dump({'gps':draw_gps}, f)
        return return_gps, return_gps_list

    def scan_pool(self,
                  meter_gap=20,
                  col_meter_gap=None,
                  safe_meter_distance=None,
                  b_show=False):
        """
        传入湖泊像素轮廓和采样间隔返回采样扫描点
        :param b_show:
        :param safe_meter_distance:
        :param col_meter_gap:
        :param meter_gap:
        """
        # 求坐标点最大外围矩阵
        if safe_meter_distance is None:
            safe_meter_distance = 10
        if col_meter_gap is None:
            col_meter_gap = meter_gap
        (x, y, w, h) = cv2.boundingRect(self.pool_cnts)
        self.logger.debug({'(x, y, w, h)': (x, y, w, h)})
        # 循环生成点同时判断点是否在湖泊范围在则添加到列表中
        scan_points = []
        pix_gap = int(meter_gap / self.pix_2_meter)
        pix_safe_distance = int(safe_meter_distance / self.pix_2_meter)
        # 起始点
        start_x, start_y = x, y + pix_gap
        # 当前点
        current_x, current_y = start_x, start_y
        # 判断x轴是递增的加还是减 True 为加
        b_add_or_sub = True
        while current_y < (y + h):
            while current_x <= (x + w) and current_x >= x:
                point = (current_x, current_y)
                in_cnt = cv2.pointPolygonTest(self.pool_cnts, point, True)
                if in_cnt > pix_safe_distance:
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
        if b_show:
            for point in scan_points:
                cv2.circle(self.show_img, tuple(point), 5, (0, 255, 255), -1)
            cv2.imshow('scan', self.show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        self.scan_point_cnts = scan_points
        return self.scan_point_cnts

    def surround_pool(self, safe_distance=20, pool_center_distance=None, b_show=False):
        """
        环绕湖泊
        :param safe_distance: 离岸边距离
        :param pool_center_distance: 离湖中心距离
        :param b_show: 是否显示
        :return: 环湖高德地图经纬度点
        """
        from utils import line_calculate
        shrink_pool_cnts = line_calculate.shrink_polygon(np.asarray(self.pool_cnts), r=0.7)[0]
        save_shrink_pool_cnts = []
        for point in shrink_pool_cnts:
            point_test = tuple((point[0], point[1]))
            in_cnt = cv2.pointPolygonTest(np.asarray(self.pool_cnts), point_test, method_0)
            if in_cnt > 0:
                save_shrink_pool_cnts.append(list(point_test))
        if b_show:
            for p in save_shrink_pool_cnts:
                cv2.circle(self.show_img, tuple(p), 5, (0, 255, 255), -1)
            # shrink_img = cv2.drawContours(self.show_img, np.asarray([save_shrink_pool_cnts]), -1, (0, 0, 255), 3)
            cv2.imshow('shrink_img', self.show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    @staticmethod
    def generate_geojson(point_gps_list, deep_list=None):
        """
        传入经纬度，生成深度信息，结合在一起转化为geojson格式
        :param point_gps_list 经纬度列表
        ：param 深度信息列表 或者其他需要展示的数据列表
        :return: geojson data
        """
        if not deep_list:
            deep_list = [random.randrange(10, 50) / 10.0 for i in point_gps_list]
        return_json_data = {"type": "FeatureCollection", "features": []}
        for deep, lng_lat in zip(deep_list, point_gps_list):
            feature = {
                "type": "Feature",
                "properties": {
                    "count": deep
                },
                # "std": 5,
                "geometry": {
                    "type": "Point",
                    "coordinates": [
                        lng_lat[0],
                        lng_lat[1]
                    ]
                }
            }
            return_json_data.get("features").append(feature)
        with open('geojeson_data.json', 'w') as f:
            json.dump(return_json_data, f)
        return return_json_data

    @staticmethod
    def cal_bank_distance(pool_cnts, current_pix, pix_2_meter):
        """
        @param pool_cnts: 湖泊像素轮廓
        @param current_pix: 当前像素位置
        @param pix_2_meter: 像素对应米
        @return: 距离岸边实际距离单位米
        """
        in_cnt = cv2.pointPolygonTest(
            pool_cnts, (current_pix[0], current_pix[1]), True)
        bank_distance = in_cnt * pix_2_meter
        return bank_distance


if __name__ == '__main__':
    pass
    # src_point = [114.4314, 30.523558]  # 喻家湖
    src_point = [114.53107,30.256]  # 喻家湖
    obj = BaiduMap(src_point, zoom=15,
                   scale=1, map_type=MapType.gaode)
    # 最大拼接次数 4
    for concate_i in range(1, 9):
        # src_point = [114.431400, 30.523558]
        # 1请求图片
        row = 1 + 2 * concate_i
        col = 1 + 2 * concate_i
        image_paths = [""] * row ** 2
        # image_paths[1 + (row ** 2) // 2] = obj.save_img_path  # 中间图片地址
        # print(obj.get_pool_name())
        # print(obj.get_area_code(src_point))
        # 2获取湖泊轮廓
        pool_cnts, (pool_cx, pool_cy) = obj.get_pool_pix_func(b_show=True)
        # 3判断湖泊轮廓是否包含直线边线，20个像素在边沿且是连续直线   则认为有 走到->4  没有则结束
        if isinstance(pool_cnts, np.ndarray):
            save_pool_cnts = pool_cnts.tolist()
        else:
            save_pool_cnts = pool_cnts
        print('长度', len(save_pool_cnts))
        b_need_concate = False  # 是否需要拼接
        side_count = 0
        for index, item in enumerate(save_pool_cnts):
            if index == len(save_pool_cnts) - 1:
                break
            if item[0] == obj.width - 2 or item[0] == 1 or item[1] == obj.height - 2 or item[1] == 1:
                side_count += 1
            else:
                side_count = 0
            if side_count >= 50:
                b_need_concate = True
                side_count = 0
                break
        if b_need_concate:
            for j in range(col):
                # 当列变化时需要修改最上面一点经纬度
                row_distance = (row // 2 - j) * obj.pix_2_meter * obj.width
                col_distance = col // 2 * obj.pix_2_meter * obj.width
                theta = math.degrees(math.atan2(row_distance, col_distance))
                theta = (theta + 360) % 360
                top_lng_lat = lng_lat_calculate.one_point_diatance_to_end(obj.lng_lat[0],
                                                                          obj.lng_lat[1],
                                                                          theta,
                                                                          (
                                                                                      row_distance ** 2 + col_distance ** 2) ** 0.5)
                for i in range(row):
                    delta_distance_lat = obj.pix_2_meter * obj.height * i  # 纬度偏差距离
                    # delta_distance_lng = self.map_obj.pix_2_meter * self.image_size * (j + 1)  # 经度偏差距离
                    # delta_distance = ((delta_distance_lat ** 2) + (delta_distance_lng ** 2)) ** 0.5
                    # 计算下一点经纬度
                    lng_lat = lng_lat_calculate.one_point_diatance_to_end(top_lng_lat[0], top_lng_lat[1],
                                                                          180, delta_distance_lat)
                    save_img_path = obj.save_img_path.split('.png')[0] + '_' + str(concate_i) + '_' + str(
                        i + j * row) + '.png'
                    image_paths[i + j * col] = save_img_path
                    if not os.path.exists(save_img_path):
                        png_url = obj.get_image_url(lng_lat)
                        response = requests.get(png_url)
                        # 获取的文本实际上是图片的二进制文本
                        img = response.content
                        # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
                        with open(save_img_path, 'wb') as f:
                            f.write(img)
                        print('保存图片', save_img_path)
            concate_image_path = obj.save_img_path.split('.png')[0] + '_concate.png'  # 合并图片路径
            print('拼接图片路径:', concate_image_path)
            # 计算合并图片尺寸
            if not os.path.exists(concate_image_path):
                vis = np.zeros((row * 1024, col * 1024, 3), dtype="uint8")
                for j in range(col):
                    for i in range(row):
                        save_img_path = image_paths[i + j * col]
                        image = cv2.imread(save_img_path)
                        vis[1024 * i:1024 * (i + 1), 1024 * j:1024 * (j + 1)] = image
                cv2.imwrite(concate_image_path, vis)
            # 5 再次获取湖泊轮廓 转到3
            obj.save_img_path = concate_image_path
        else:
            break
    # scan_cnts = obj.scan_pool(meter_gap=50, safe_meter_distance=10, b_show=False)
    # return_gps, return_gps_list = obj.pix_to_gps(scan_cnts)
    # return_gps1, return_gps_list1 = obj.pix_to_gps([obj.center_cnt])
    # print(return_gps, return_gps_list)
    # print(return_gps1, return_gps_list1)
    # # obj.build_obstacle_map(False)
    # point = [src_point[0] + 0.001, src_point[1] + 0.002]
    # obj.scan_pool(meter_gap=50)
    # scan_point_gps1, scan_point_gps_list1 = obj.pix_to_gps(obj.scan_point_cnts)
    # print(len(scan_point_gps_list1), scan_point_gps_list1)
    # obj.generate_geojson(scan_point_gps_list1)
    # obj.update_obstacle_map(point,True)
    # obj.surround_pool(b_show=True)
    # obj = BaiduMap([114.393142, 30.558963], zoom=15,map_type=MapType.baidu)
    # obj = BaiduMap([114.718257,30.648004],zoom=14)
    # obj = BaiduMap([114.566767,30.541689],zoom=14)
    # obj = BaiduMap([114.565976,30.541317],zoom=15.113213)
    # obj = BaiduMap([114.393142,30.558981],zoom=14)
    # pix_src = obj.gaode_lng_lat_to_pix(src_point)
    # gaode_point2 = [114.429812, 30.526649]
    # gaode_point3 = [114.428895, 30.520323]
    # gaode_point4 = [114.433235, 30.520342]
    # gaode_point1 = [114.432303, 30.530362]
    # gaode_point = gaode_point1
    # pix_target = obj.gaode_lng_lat_to_pix(gaode_point)
    # print('pix_src', pix_src, 'pix_target', pix_target)
    # b_show = 0
    # cv2.circle(
    #     obj.show_img, (pix_src[0], pix_src[1]), 5, [
    #         255, 0, 0], -1)
    # cv2.circle(
    #     obj.show_img, (pix_target[0], pix_target[1]), 5, [
    #         0, 0, 255], -1)
    # if b_show:
    #     cv2.namedWindow(
    #         'result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
    #     cv2.imshow('result', obj.show_img)
    #     # 等待任意按键按下
    #     cv2.waitKey(0)
    #     # 关闭其他窗口
    #     cv2.destroyAllWindows()
    # return_gps, return_gps_list = obj.pix_to_gps([pix_target])
    # print(return_gps, return_gps_list)
    # print(lng_lat_calculate.distanceFromCoordinate(114.439899, 30.526094, return_gps_list[0][0], return_gps_list[0][1]))
    # # obj.pix_to_gps(obj.pool_cnts)
    # if pool_cnts is None:
    #     pass
    # else:
    #     all_cnt = []
    #     all_cnt.extend(list(pool_cnts))
    #     all_cnt.extend(scan_cnts)
    #     gps = obj.pix_to_gps(all_cnt)
    # print(gps)
    # 请求指定位置图片
    # obj.draw_image()
    # 求坐标点最大外围矩阵
    # (x, y, w, h) = cv2.boundingRect(pool_cnts)
    # print('(x, y, w, h)', (x, y, w, h))
