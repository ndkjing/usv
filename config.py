# 保存地图数据路径
import os
root_path = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(root_path,'mapsData')
if not os.path.exists(maps_dir):
    os.mkdir(maps_dir)
# 保存所有地图湖泊信息位置
map_data_path = os.path.join(maps_dir,'map.json')
local_map_data_path = os.path.join(maps_dir,'local_map.json')

# 保存当前用户点击位置相关信息
usr_lng_lat_path = os.path.join(maps_dir,'usr_lng_lat_path.json')

