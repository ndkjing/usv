"""
入口函数
"""
import threading
import os
import json
import numpy as np
import time
from dataGetSend import data_define
import config
from utils import log
from utils import data_generate
from baiduMap.baidu_map import BaiduMap
from baiduMap import baidu_map
from dataGetSend.data_manager import DataManager
from audios import audios_manager
import sys
import os

sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'baiduMap'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'dataGetSend'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))


def main(mod='auto',b_play_audio=False):
    if b_play_audio:
        audios_manager.play_audio('setup.mp3')
    logger = log.LogHandler('main_log')

    # 数据处理对象
    data_manager_obj = DataManager()

    # 查询改船是否注册 若未注册直接退出
    try:
        binding_data = data_manager_obj.send(
            method='http', data="", url=config.http_binding, http_type='GET')
        if int(binding_data['flag']) != 1:
            if b_play_audio:
                audios_manager.play_audio('register.mp3')
            logger.error({'binding status': binding_data['flag']})
            # TODO 未注册暂时跳过
            # exit(-1)
        logger.info({'binding status': binding_data['flag']})
    except Exception as e:
        logger.error({'binding_data error': e})

    # 发送获取所有数据
    # save_map_data = data_manager_obj.send(method='http',data="", url='http://192.168.8.13:8009/admin/xxl/map/list',
    #                       http_type='GET')

    # 启动串口数据收发和mqtt数据收发
    get_com_data_thread = threading.Thread(
        target=data_manager_obj.get_com_data)
    send_mqtt_data_thread = threading.Thread(
        target=data_manager_obj.send_mqtt_data)
    send_com_data_thread = threading.Thread(
        target=data_manager_obj.send_com_data)

    # get_com_data_thread.setDaemon(True)
    # send_mqtt_data_thread.setDaemon(True)
    # send_com_data_thread.setDaemon(True)

    get_com_data_thread.start()
    send_mqtt_data_thread.start()
    send_com_data_thread.start()

    get_com_data_thread.join()
    send_mqtt_data_thread.join()
    send_com_data_thread.join()
    while True:
        try:
            # 创建地图对象
            assert mod in ['manual', 'auto']
            if mod == 'auto':
                # 获取话题user_lng_lat数据
                # with open(config.usr_lng_lat_path,'r') as f:
                #     user_lng_lat = json.load(f)
                while len(
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.target_lng_lat) <= 0:
                    time.sleep(2)
                    logger.info('没有点击湖，等待用户执行操作')
                user_lng_lat = data_manager_obj.server_data_obj.mqtt_send_get_obj.target_lng_lat[-1]
                zoom = data_manager_obj.server_data_obj.mqtt_send_get_obj.target_lng_lat.zoom[-1]
                baidu_map_obj = BaiduMap(user_lng_lat, zoom)
            else:
                user_lng_lat = [116.99868, 40.511224]
                zoom = 12
                # baidu_map_obj = BaiduMap([114.393142, 31.558981], zoom=14)
                # baidu_map_obj = BaiduMap([114.710639,30.656827], zoom=13)
                baidu_map_obj = BaiduMap(user_lng_lat, zoom)
                # baidu_map_obj = BaiduMap([117.574294,31.539694], zoom=11)

            pool_cnt, (pool_cx, pool_cy) = baidu_map_obj.get_pool_pix(
                b_show=False)
            if pool_cnt is None:
                # 若返回为None表示没找到湖 定义错误代码 99表示没有湖 0表示正常
                logger.error({'没有找到湖': user_lng_lat})
                data_manager_obj.send(
                    method='mqtt', topic='pool_info', data={
                        'error_code': 99}, qos=1)
                continue
            pool_cnt = np.squeeze(pool_cnt)

            # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用，
            _, pool_gps_points = baidu_map_obj.pix_to_gps(pool_cnt)
            _, pool_gps_center = baidu_map_obj.pix_to_gps([[pool_cx, pool_cy]])
            logger.info({'pool_gps_center': pool_gps_center})

            # 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
            send_data = {"longitudeLatitude": str(pool_gps_center),
                         "mapData": str(pool_gps_points),
                         "deviceId": config.ship_code,
                         "pixData": str(pool_cnt)}
            if not os.path.exists(config.local_map_data_path):
                with open(config.local_map_data_path, 'w') as f:
                    # 发送请求获取湖泊ID
                    pool_id = data_manager_obj.send(
                        method='http', data=send_data, url=config.http_save, http_type='POST')
                    save_data = {"mapList": [{"id": pool_id,
                                              "longitudeLatitude": pool_gps_center,
                                              "mapData": pool_gps_points,
                                              "pool_cnt": pool_cnt.tolist()}]}
                    logger.info({'pool_id': pool_id})
                    json.dump(save_data, f)
            else:
                with open(config.local_map_data_path, 'r') as f:
                    local_map_data = json.load(f)
                    # 判断是否在曾经出现的湖泊中
                    pool_id = baidu_map.is_in_contours(
                        pool_gps_center, local_map_data)
                if pool_id is not None:
                    logger.info({'在本地找到湖泊 poolid': pool_id})
                # 不存在获取新的id
                else:
                    pool_id = data_manager_obj.send(
                        method='http', data=send_data, url=config.http_save, http_type='POST')
                    logger.info({'新的湖泊 poolid': pool_id})
                    with open(config.local_map_data_path, 'w') as f:
                        local_map_data["mapList"].append({"id": pool_id,
                                                          "longitudeLatitude": str(pool_gps_center),
                                                          "mapData": str(pool_gps_points),
                                                          "pool_cnt": pool_cnt.tolist()})
                        json.dump(local_map_data, f)
            data_manager_obj.data_define_obj.pool_code = pool_id
        except Exception as e:
            logger.error({'error': e})
            continue


if __name__ == '__main__':
    main()
