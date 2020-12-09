"""
main
"""
from dataGetSend.data_manager import DataManager
from baiduMap import baidu_map
from baiduMap.baidu_map import BaiduMap
from utils import data_generate
from utils import log
import config
from dataGetSend import data_define

import time
import numpy as np
import json
import os

def main(mod='manual'):
    log_obj = log.LogHandler('main_log')

    # 数据处理对象
    data_manager_obj = DataManager()

    # 发送获取所有数据
    save_map_data = data_manager_obj.send(method='http',data="", url='http://192.168.8.13:8009/admin/xxl/map/list',
                          http_type='GET')

    ## 创建地图对象
    assert mod in ['manual','auto']
    if mod=='auto':
        # 获取话题user_lng_lat数据
        with open(config.usr_lng_lat_path,'r') as f:
            user_lng_lat = json.load(f)
        baidu_map_obj = BaiduMap(user_lng_lat['lng_lat'], user_lng_lat['zoom'])
    else:
        baidu_map_obj = BaiduMap([114.393142, 30.558981], zoom=14)

    pool_cnt,(pool_cx,pool_cy) = baidu_map_obj.get_pool_pix(b_show=False)
    if pool_cnt is None:
        # 若返回为None表示没找到湖 定义错误代码 99表示没有湖 0表示正常
        log_obj.info({'没有找到湖':user_lng_lat['lng_lat']})
        data_manager_obj.send(method='mqtt', topic='pool_info', data={'error_code':99}, qos=1)

    pool_cnt = np.squeeze(pool_cnt)
    # 获取湖泊轮廓与中心点经纬度位置 _位置为提供前端直接绘图使用，
    _,pool_gps_points = baidu_map_obj.pix_to_gps(pool_cnt)
    _,pool_gps_center = baidu_map_obj.pix_to_gps([[pool_cx,pool_cy]])
    log_obj.info({'pool_gps_center':pool_gps_center})

    ## 判断当前湖泊是否曾经出现，出现过则获取的ID 没出现过发送请求获取新ID
    # if mod=='manual':
    #     try:
    #         os.remove(config.local_map_data_path)
    #     except :
    #         pass
    if not os.path.exists(config.local_map_data_path):
        with open(config.local_map_data_path, 'w') as f:
            # 发送请求获取湖泊ID
            send_data = {"longitudeLatitude": str(pool_gps_center),
                         "mapData": str(pool_gps_points)}
            pool_id = data_manager_obj.send(method='http', data=send_data,
                                            url='http://192.168.8.13:8009/admin/xxl/map/save',
                                            http_type='POST')
            save_data = {"mapList":[{"id":pool_id,
                                    "longitudeLatitude":str(pool_gps_center),
                                    "mapData":str(pool_gps_points),
                                    "pool_cnt":pool_cnt.tolist()}]}
            log_obj.info({'pool_id':pool_id})
            json.dump(save_data,f)
    else:
        with open(config.local_map_data_path, 'r') as f:
            local_map_data = json.load(f)
            # 判断是否在曾经出现的湖泊中
            pool_id = baidu_map.is_in_contours((pool_cx,pool_cy),local_map_data)
            log_obj.info({'在本地找到湖泊 poolid':pool_id})
        # 不存在获取新的id
        if pool_id == None:
            send_data = {"longitudeLatitude": str(pool_gps_center),
                         "mapData": str(pool_gps_points)}
            pool_id = data_manager_obj.send(method='http', data=send_data,
                                            url='http://192.168.8.13:8009/admin/xxl/map/save',
                                            http_type='POST')
            with open(config.local_map_data_path, 'w') as f:
                local_map_data["mapList"].append({"id":pool_id,
                                    "longitudeLatitude":str(pool_gps_center),
                                    "mapData":str(pool_gps_points),
                                    "pool_cnt":pool_cnt.tolist()})
                json.dump(local_map_data, f)


    while True:
        # 向服务器发送HTTP请求ID

        # 向串口发送数据
        # obj.send(method='com', data=move_direction)

        # 向mqtt服务器发送数据
        send_mqtt_data="test123"

        # 生成构造数据    'pool_id':'mapId',
        #     'ship_code':'deviceId',
        status_data = data_generate.status_data()
        status_data.update({'mapId':pool_id})
        detect_data = data_generate.detect_data()
        detect_data.update({'mapId': pool_id})

        for k_all,v_all in data_define.name_mappings.items():
            for old_key,new_key in v_all.items():
                # pop_value = detect_data[k_all].pop(old_key)
                detect_data[k_all].update({new_key:detect_data[k_all].pop(old_key)})
        log_obj.info({"status_data": status_data})
        log_obj.info({"detect_data": detect_data})
        data_manager_obj.send(method='mqtt',topic='status_data', data=status_data, qos=1)
        data_manager_obj.send(method='mqtt',topic='detect_data', data=detect_data, qos=1)

        # 接收控制数据
        move_direction = data_manager_obj.server_data_obj.mqtt_send_get_obj.move_direction
        data_manager_obj.log.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)


if __name__ == '__main__':
    main()
