import random
import json
import config

def generate_geojson(point_gps_list, deep_list=None, b_save_data=True):
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
    if b_save_data:
        with open(config.save_sonar_path, 'w') as f:
            json.dump(return_json_data, f)
    return return_json_data