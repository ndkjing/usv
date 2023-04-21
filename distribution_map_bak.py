# 导入数据处理库
import numpy as np
import pandas as pd
import plotnine
from plotnine import *
# 导入插值库
from pykrige.ok import OrdinaryKriging
from pykrige.kriging_tools import write_asc_grid
import pykrige.kriging_tools as kt
# 导入绘图库
import matplotlib.pyplot as plt
# from mpl_toolkits.basemap import Basemap
import json
import geopandas as gpd
import requests
import os
import cv2

draw_data = {}  # 用于绘制数据
height_width = 100


# 1 请求轮廓并保存为geojson
def save_geo_json_map(deviceId="XXLJC4LCGSCAHSD0DA000",
                      mapId="1433665418824626178",
                      startTime="2019-03-01",
                      endTime="2022-03-01",
                      data_type="wt"):
    global height_width
    print('data_type', data_type)
    url1 = "https://ship.xxlun.com/union/admin/xxl/data/getData"
    url1 = url1 + "?deviceId=%s&mapId=%s&startTime=%s&endTime=%s" % (deviceId, mapId, startTime, endTime)
    print('url1', url1)
    response1 = requests.get(url=url1, timeout=8)
    print('response1', response1)
    content_data1 = json.loads(response1.content)
    know_lon = []
    know_lat = []
    know_z_pH = []
    know_z_doDO = []
    know_z_td = []
    know_z_ec = []
    know_z_wt = []
    # 判断是否有数据
    if content_data1.get("success") and content_data1.get("data") and content_data1.get("data").get("data") and len(
            content_data1.get("data").get("data").get("water")) > 0:
        for i in content_data1.get("data").get("data").get("water"):
            pH = round(float(i.get('ph')),1)
            doDO = round(float(i.get('doDo')),1)
            td = round(float(i.get('td')),1)
            ec = round(float(i.get('ec')),1)
            wt = round(float(i.get('wt')),1)
            gjwd = json.loads(i.get('gjwd'))
            know_lon.append(gjwd[0])
            know_lat.append(gjwd[1])
            know_z_pH.append(pH)
            know_z_doDO.append(doDO)
            know_z_td.append(td)
            know_z_ec.append(ec)
            know_z_wt.append(wt)
        draw_data['know_lon'] = know_lon
        draw_data['know_lat'] = know_lat
        if data_type == "ph":
            draw_data['know_z'] = know_z_pH
        elif data_type == "doDo":
            draw_data['know_z'] = know_z_doDO
        elif data_type == "td":
            draw_data['know_z'] = know_z_td
        elif data_type == "ec":
            draw_data['know_z'] = know_z_ec
        else:
            draw_data['know_z'] = know_z_wt
        print('draw_dataknow_z',draw_data['know_z'])
    else:
        # 告诉用户没有数据
        return [2, height_width]
    # 2  请求数据并绘制绘制地图图片保存到本地
    url2 = "https://ship.xxlun.com/union//admin/xxl/map/list/0/1"
    url2 = url2 + "?mapId=%s" % mapId
    response2 = requests.get(url=url2, timeout=8)
    content_data2 = json.loads(response2.content)
    if content_data2.get("success") and content_data2.get("data") and \
            content_data2.get("data").get("mapList") and \
            content_data2.get("data").get("mapList").get("records") and \
            len(content_data2.get("data").get("mapList").get("records")) > 0:
        pool_lng_lats = json.loads(content_data2.get("data").get("mapList").get("records")[0].get("mapData"))
        pool_int_cnts = []  # 整形经纬度用于计算轮廓
        for i1 in pool_lng_lats:
            pool_int_cnts.append([int(i1[0] * 1000000), int(i1[1] * 1000000)])

        # 计算宽高比 近似用经纬度之比就可以
        (x, y, w, h) = cv2.boundingRect(np.asarray(pool_int_cnts))
        height_width = int((h / w) * 100)
        print('height_width', height_width)
        # from utils import lng_lat_calculate
        # x_dis = lng_lat_calculate.distanceFromCoordinate(x/1000000.0,y/1000000.0,(x+w)/1000000.0,y/1000000.0)
        # y_dis = lng_lat_calculate.distanceFromCoordinate(x/1000000.0,y/1000000.0,x/1000000.0,(y+h)/1000000.0)
        geojson_dict = {
            "type": "Polygon",
            "coordinates": [
                [
                    [114.429812, 30.526649],
                    [114.428895, 30.520323],
                    [114.433235, 30.520342],
                    [114.432303, 30.530362]
                ]
            ]
        }
        coordinates = []
        pool_lng_lats.append(pool_lng_lats[0])
        coordinates.append(pool_lng_lats)
        geojson_dict["coordinates"] = coordinates
        with open("map_geojson.json", 'w') as f:
            json.dump(geojson_dict, f)
        return [1, height_width]
    else:
        # 找不到地图
        return [2, height_width]


# 3  发送数据到服务器
url3 = "/admin/xxl/data/getData"

# js = gpd.read_file(r"江苏省.json")  # 读取geojson 文件
# js = gpd.read_file(r"test_geojson.json")  # 读取geojson 文件
# pm = pd.read_excel("pmdata.xlsx")
# nj = ["南京", "苏州", "南通", "连云港", "徐州", "扬州", "无锡", "常州", "镇江", "泰州", "淮安", "盐城", "宿迁"]
# nj_data = pm[pm["city"].isin(nj)]
# nj_data = nj_data.dropna()
import math
# 更换求距离的函数
from math import radians, cos, sin, asin, sqrt


# degree to km (Haversine method)
def haversine(lon1, lat1, lon2, lat2):
    # R = 3959.87433 # this is in miles.  For Earth radius in kilometers use 6372.8 km
    R = 6372.8
    dLon = radians(lon2 - lon1)
    dLat = radians(lat2 - lat1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    a = sin(dLat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dLon / 2) ** 2
    c = 2 * asin(sqrt(a))
    d = R * c
    return d


def IDW(x, y, z, xi, yi):
    lstxyzi = []
    for p in range(len(xi)):
        lstdist = []
        for s in range(len(x)):
            d = (haversine(x[s], y[s], xi[p], yi[p]))
            lstdist.append(d)
        sumsup = list((1 / np.power(lstdist, 2)))
        suminf = np.sum(sumsup)
        sumsup = np.sum(np.array(sumsup) * np.array(z))
        u = sumsup / suminf
        xyzi = [xi[p], yi[p], u]
        lstxyzi.append(xyzi)
    return (lstxyzi)


"""
测试用经纬度
114.431341,30.527065    1
114.431384,30.52542    2
114.430483,30.522962   3
114.430741,30.521021   4
114.432929,30.519431   5
"""

# know_lon = [i for i in nj_data["lon"]]
# know_lat = [i for i in nj_data["lat"]]
# know_z =   [i for i in nj_data["PM2.5"]]

# know_lon = [114.431341, 114.431384, 114.430483, 114.430741, 114.432929]
# know_lat = [30.527065, 30.52542, 30.522962, 30.521021, 30.519431]
# know_z = [1, 3, 2, 1.5, 5.4]

# know_lon = [114.523019, 114.52237,114.525438]
# know_lat = [30.505932, 30.505974, 30.50541]
# know_z = [1, 3, 2]
# know_z = know_z_pH


# 可视化绘图  plotnine  散点图
import plotnine
from plotnine import *


# 绘制数据散点图
def save_scatter():
    plotnine.options.figure_size = (5, 4.5)
    idw_scatter = (ggplot() +
                   geom_map(js, fill='none', color='gray', size=0.4) +
                   geom_point(pm, aes(x='lon', y='lat', fill='PM2.5'), size=5) +
                   scale_fill_cmap(cmap_name='Spectral_r', name='PM2.5',
                                   breaks=[30, 40, 60, 80]
                                   ) +
                   scale_x_continuous(breaks=[117, 118, 119, 120, 121, 122]) +
                   labs(title="Map Charts in Python Exercise 02: Map IDM point",
                        ) +
                   # 添加文本信息
                   annotate('text', x=116.5, y=35.3, label="processed map charts with plotnine", ha="left",
                            size=10) +
                   annotate('text', x=120, y=30.6, label="Visualization by DataCharm", ha="left", size=9) +
                   theme(
                       text=element_text(family="Roboto Condensed"),
                       # 修改背景
                       panel_background=element_blank(),
                       axis_ticks_major_x=element_blank(),
                       axis_ticks_major_y=element_blank(),
                       axis_text=element_text(size=12),
                       axis_title=element_text(size=14, weight="bold"),
                       panel_grid_major_x=element_line(color="gray", size=.5),
                       panel_grid_major_y=element_line(color="gray", size=.5),
                   ))
    idw_scatter.save(r"idw_mao_point_plotnine.png",
                     width=5, height=4, dpi=900, kwargs={"bbox_inches": 'tight'})


def idw_render():
    pm_idw = IDW(know_lon, know_lat, know_z, grid_lon_list, grid_lat_list)
    IDW_grid_df = pd.DataFrame(pm_idw, columns=["lon", "lat", "idw_value"])
    plotnine.options.figure_size = (5, 4.5)
    idw_scatter_inter = (ggplot() +
                         geom_tile(IDW_grid_df, aes(x='lon', y='lat', fill='idw_value'), size=0.1) +
                         geom_map(js, fill='none', color='gray', size=0.4) +
                         geom_point(pm, aes(x='lon', y='lat', fill='PM2.5'), size=4, stroke=.3, show_legend=False) +
                         scale_fill_cmap(cmap_name='Spectral_r', name='idw_value',
                                         breaks=[30, 40, 60, 80]
                                         ) +
                         scale_x_continuous(breaks=[117, 118, 119, 120, 121, 122]) +
                         labs(title="Map Charts in Python Exercise 02: Map IDM point",
                              ) +
                         # 添加文本信息
                         annotate('text', x=116.5, y=35.3, label="processed map charts with plotnine", ha="left",
                                  size=10) +
                         annotate('text', x=120, y=30.6, label="Visualization by DataCharm", ha="left", size=9) +
                         theme(
                             text=element_text(family="Roboto Condensed"),
                             # 修改背景
                             panel_background=element_blank(),
                             axis_ticks_major_x=element_blank(),
                             axis_ticks_major_y=element_blank(),
                             axis_text=element_text(size=12),
                             plot_title=element_text(size=15, weight="bold"),
                             axis_title=element_text(size=14),
                             panel_grid_major_x=element_line(color="gray", size=.5),
                             panel_grid_major_y=element_line(color="gray", size=.5),
                         ))
    idw_scatter_inter.save(r"idw_map_point_inter_point_plotnine.png",
                           width=5, height=4, dpi=900, kwargs={"bbox_inches": 'tight'})


def idw_clip():
    pm_idw = IDW(know_lon, know_lat, know_z, grid_lon_list, grid_lat_list)
    IDW_grid_df = pd.DataFrame(pm_idw, columns=["lon", "lat", "idw_value"])
    idw_grid_geo = gpd.GeoDataFrame(IDW_grid_df, geometry=gpd.points_from_xy(IDW_grid_df["lon"], df_grid["lat"]),
                                    crs="EPSG:4326")
    idw_grid_clip = gpd.clip(idw_grid_geo, js)
    plotnine.options.figure_size = (5, 4.5)
    idw_scatter_inter_clip = (ggplot() +
                              geom_tile(idw_grid_clip, aes(x='lon', y='lat', fill='idw_value'), size=0.1) +
                              geom_map(js, fill='none', color='gray', size=0.4) +
                              # geom_point(pm,aes(x='经度',y='纬度',fill='PM2.5'),size=4,stroke=.3,show_legend=False) +
                              # scale_fill_cmap(cmap_name='Spectral_r', name='idw_value',
                              #                 breaks=[30, 40, 60, 80]
                              #                 ) +
                              # scale_x_continuous(breaks=[117, 118, 119, 120, 121, 122]) +
                              # labs(title="Map Charts in Python Exercise 02: Map IDM Point Clip",
                              #      ) +
                              # 添加文本信息
                              # annotate('text', x=116.5, y=35.3, label="processed map charts with plotnine", ha="left",
                              #          size=10) +
                              # annotate('text', x=120, y=30.6, label="Visualization by DataCharm", ha="left", size=9) +
                              theme(
                                  text=element_text(family="Roboto Condensed"),
                                  # 修改图例
                                  legend_key_width=10,
                                  # 修改背景
                                  panel_background=element_blank(),
                                  axis_ticks_major_x=element_blank(),
                                  axis_ticks_major_y=element_blank(),
                                  axis_text=element_text(size=12),
                                  plot_title=element_text(size=15, weight="bold"),
                                  axis_title=element_text(size=14),
                                  panel_grid_major_x=element_line(color="gray", size=.5),
                                  panel_grid_major_y=element_line(color="gray", size=.5),
                              ))
    idw_scatter_inter_clip.save(r"idw_map_Nopoint_inter_clip_plotnine.png",
                                width=5, height=4, dpi=900, kwargs={"bbox_inches": 'tight'})


# OrdinaryKriging 方法
def MyOrdinaryKriging(deviceId, data_type):
    global height_width
    print('data_type', data_type)
    js = gpd.read_file(r"map_geojson.json")  # 读取geojson 文件
    js_box = js.geometry.total_bounds  # 获取包围框
    # 还是插入400*400的网格点  暂时使用100*100 网格大会比较清晰但是会导致内存不够和计算时间延长 根据电脑配置设置
    grid_lon = np.linspace(js_box[0], js_box[2], 100)
    grid_lat = np.linspace(js_box[1], js_box[3], 100)

    # np.meshgrid():生成网格点坐标矩阵,二维坐标下，形成的一个一个的网格点
    xgrid, ygrid = np.meshgrid(grid_lon, grid_lat)

    # 将插值网格数据整理
    df_grid = pd.DataFrame(dict(long=xgrid.flatten(), lat=ygrid.flatten()))

    # 这里将数组转成列表
    grid_lon_list = df_grid["long"].tolist()
    grid_lat_list = df_grid["lat"].tolist()
    know_lon = draw_data.get('know_lon')
    know_lat = draw_data.get('know_lat')
    know_z = draw_data.get('know_z')
    OK = OrdinaryKriging(know_lon, know_lat, know_z, variogram_model='gaussian', nlags=6)
    z1, ss1 = OK.execute('grid', grid_lon, grid_lat)
    # 将插值网格数据整理
    df_grid = pd.DataFrame(dict(long=xgrid.flatten(), lat=ygrid.flatten()))
    df_grid.head()
    Krig_result = z1.data.flatten()
    df_grid[data_type] = Krig_result
    df_grid_geo = gpd.GeoDataFrame(df_grid, geometry=gpd.points_from_xy(df_grid["long"], df_grid["lat"]),
                                   crs="EPSG:4326")
    js_Krig_gaussian_clip = gpd.clip(df_grid_geo, js)
    plotnine.options.figure_size = (5, 4.5)
    Krig_inter_no_grid = (ggplot() +
                          geom_tile(js_Krig_gaussian_clip, aes(x='long', y='lat', fill=data_type), size=0.1) +
                          geom_map(js, fill='none', color='gray', size=0.3) +  # 绘制轮廓
                          # scale_fill_cmap(cmap_name='Spectral_r', name='Values',    # 设置对比颜色
                          #                 breaks=[30, 40, 60, 80]
                          #                 ) +
                          # scale_x_continuous(breaks=[117, 118, 119, 120, 121, 122]) +  # 指定x轴范围
                          # labs(title="Map Charts in Python Exercise 02: Map point kriging interpolation",
                          #      ) +
                          # 添加文本信息
                          # annotate('text', x=116.5, y=35.3, label="processed map charts with plotnine", ha="left",
                          #          size=10) +
                          # annotate('text', x=120, y=30.6, label="Visualization by DataCharm", ha="left", size=9) +
                          theme(
                              text=element_text(family="Roboto Condensed"),
                              # 修改背景
                              panel_background=element_blank(),
                              axis_ticks_major_x=element_blank(),
                              axis_ticks_major_y=element_blank(),
                              axis_text=element_text(size=12),
                              axis_title=element_text(size=14),
                              panel_grid_major_x=element_line(color="gray", size=.5),
                              panel_grid_major_y=element_line(color="gray", size=.5),
                          ))
    save_img_name = r"%s.png" % deviceId
    if os.path.exists(save_img_name):
        os.remove(save_img_name)
    width=5
    height_width = int(width*height_width/100.0)
    if height_width > 4*width:
        height_width = 4*width
    Krig_inter_no_grid.save(save_img_name,
                            width=width, height=height_width, dpi=900, kwargs={"bbox_inches": 'tight'})


if __name__ == '__main__':
    deviceId = "XXLJC4LCGSCAHSD0DA000"
    # 请求数据
    save_geo_json_map(deviceId=deviceId)
    # 绘制地图
    MyOrdinaryKriging(deviceId, data_type='wt')
    # 发送给服务器
    from webServer import upload_file

    ip_local = '192.168.8.26:8009'
    ip_xxl = 'ship.xxlun.com'
    url_data = "https://%s/union/admin/uploadFile" % ip_xxl
    # file = "weixin.jpg"
    file = "%s.png" % deviceId
    # file = "F:\downloads\SampleVideo_1280x720_5mb.mp4"
    save_name = upload_file.post_data(url=url_data, file=file)
    # 发送数据到服务器

    # 发送到mqtt话题
