import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import cv2
import json
import tqdm
from externalConnect import baidu_map
from utils import lng_lat_calculate

EXTEND_AREA = 10.0  # [m] grid map extention length

show_animation = True

class SonarMap:
    """
    根据声呐检测深度构建地图
    第一检测点经纬度 作为原点中心
    后续经纬度计算  相对于第一个点距离和坐标
    改变地图大小
    """
    def __init__(self):
        start_point = [114.431400, 30.523558]
        self.point_list = []
        self.deep_list = []
        self.cell_size = 1
        self.std = 5
        loop_time = 100
        self.int_lng_lats_pix = []
        for i in range(loop_time):
            point = [start_point[0] + (np.random.rand() - 0.5) * 2 / 500,
                     start_point[1] + (np.random.rand() - 0.5) * 2 / 500]
            deep = round(np.random.rand() * 30, 1)
            self.point_list.append(point)
            self.deep_list.append(deep)

    def update_map(self):
        map_point_list = [[int(i[0] * 1000000), int(i[1] * 1000000)]
                          for i in self.point_list]
        (left_up_x, left_up_y, w, h) = cv2.boundingRect(np.array(map_point_list))
        distane_x = lng_lat_calculate.distanceFromCoordinate(left_up_x/1000000,left_up_y/1000000,(left_up_x+w)/1000000,left_up_y/1000000)
        distane_y = lng_lat_calculate.distanceFromCoordinate(left_up_x/1000000,left_up_y/1000000,left_up_x/1000000,(left_up_y+h)/1000000)
        print('distane_x, w', distane_x, w)
        print('distane_y, h', distane_y, h)
        print(distane_y/distane_x, h/w)
        self.left_up_x = left_up_x
        self.left_up_y = left_up_y
        self.w = int(distane_x/self.cell_size)
        self.h = int(distane_y/self.cell_size)
        self.sonar_map = np.zeros((self.h+1, self.w+1),dtype=np.uint8)
        # self.sonar_map = cv2.cvtColor(self.sonar_map, cv2.COLOR_GRAY2BGR)
        print('self.sonar_map.shape',self.sonar_map.shape)
        self.int_lng_lats_pix = [self.lng_lat_to_pix(i) for i in self.point_list]
        print(len(self.int_lng_lats_pix), self.int_lng_lats_pix)
        for ix in tqdm.tqdm(range(self.w)):
            for iy in range(self.h):
                p_list = []
                mindis = float("inf")
                for index, (iox, ioy) in enumerate(self.int_lng_lats_pix):
                    d = math.hypot(iox - ix, ioy - iy)
                    # if d > self.std**2:
                    if d > 20:
                        p_list.append(0)
                    else:
                        pdf = 2*(1.0 - norm.cdf(d, 0.0, self.std))
                        p_list.append(pdf)
                t = np.asarray(p_list).dot(np.asarray(self.deep_list))
                deep=t
                # 防止全为0
                if sum(p_list) < 0.01:
                    deep = 0
                elif sum(p_list) > 1:
                    deep = t/sum(p_list)
                if deep>max(self.deep_list):
                    print('deep  max(self.deep_list)',deep,max(self.deep_list))
                self.sonar_map[iy][ix] = int((deep/max(self.deep_list))*255)
                # if max(p_list)>0.5:
                #     print('self.deep_list', self.deep_list)
                #     print('p_list,t,deep,pix_val', p_list, t, deep,int((deep/max(self.deep_list))*255))
        print('self.sonar_map', self.sonar_map)
        # for i in self.int_lng_lats_pix:
        #     self.sonar_map[i[1], i[0]] = 255

    def lng_lat_to_pix(self, lng_lat):
        """
        经纬度转像素
        :param lng_lat: 经纬度
        :return:
        """
        distane_x = lng_lat_calculate.distanceFromCoordinate(self.left_up_x / 1000000, self.left_up_y / 1000000,
                                                             lng_lat[0], self.left_up_y / 1000000)
        distane_y = lng_lat_calculate.distanceFromCoordinate(self.left_up_x / 1000000, self.left_up_y / 1000000,
                                                             self.left_up_x / 1000000, lng_lat[1])
        # print('distane_x,distane_y',distane_x,distane_y)
        int_lng_lat = [int(lng_lat[0] * 1000000), int(lng_lat[1] * 1000000)]
        int_lng_lats_offset = [int_lng_lat[0] - self.left_up_x, int_lng_lat[1] - self.left_up_y]
        int_lng_lats_pix = [int(distane_x/self.cell_size),
                            int(distane_y/self.cell_size)]
        # print('int_lng_lats_pix', int_lng_lats_pix)
        return int_lng_lats_pix

    def show_map(self):
        self.update_map()
        cv2.imshow('sonar', self.sonar_map)
        cv2.waitKey(0)

    @staticmethod
    def generate_geojson(point_gps_list, deep_list=None, b_save_data=True):
        """
        传入经纬度，生成深度信息，结合在一起转化为geojson格式
        :param point_gps_list 经纬度列表
        ：param 深度信息列表 或者其他需要展示的数据列表
        :return: geojson data
        """
        if not deep_list:
            deep_list = [random.randrange(10, 50)/10.0 for i in point_gps_list]
        return_json_data = {"type": "FeatureCollection","features":[]}
        for deep,lng_lat in zip(deep_list,point_gps_list):
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
        if b_save_data:
            with open('geojeson_data.json','w') as f:
                json.dump(return_json_data,f)
        return return_json_data

if __name__ == '__main__':
    sonar_obj = SonarMap()
    import random
    # sonar_obj.show_map()
    print(random.randrange(1,5))