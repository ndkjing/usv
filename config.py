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

# 单片机发送给树莓派频率
com2pi_interval=1
# 给单片机发送频率
pi2com_interval=10

# 给服务器发送频率
pi2mqtt_interval = 0.2

# 接收服务器方向控制间隔
mqtt_control_interval = 1

# 船编号
ship_code = '3c50f4c3-a9c1-4872-9f18-883af014380c'

# 串口位置 和波特率
port = '/dev/ttyUSB0'
baud = 115200

