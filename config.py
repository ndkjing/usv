# 保存地图数据路径
import os
root_path = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(root_path, 'mapsData')
if not os.path.exists(maps_dir):
    os.mkdir(maps_dir)
# 保存所有地图湖泊信息位置
map_data_path = os.path.join(maps_dir, 'map.json')
local_map_data_path = os.path.join(maps_dir, 'local_map.json')

# 保存当前用户点击位置相关信息
usr_lng_lat_path = os.path.join(maps_dir, 'usr_lng_lat_path.json')

# 单片机发送给树莓派频率
com2pi_interval = 1
# 给单片机发送频率
pi2com_interval = 2

# 给服务器发送频率
pi2mqtt_interval = 0.2

# 接收服务器方向控制间隔
mqtt_control_interval = 1

# 检查船状态间隔 单位秒
# 检查经纬度
check_status_interval = 1
# 检查网络连接状态间隔
check_network_interval=10

# 上传给单片机心跳时间间隔 单位秒
com_heart_time = 1*60

# 船编号
ship_code = '3c50f4c3-a9c1-4872-9f18-883af014380c'

## 串口位置和波特率
# 单片机
port = '/dev/stc'
baud = 115200
# imu
imu_port = '/dev/imu'
imu_baud = 115200

# http 接口
# 查询船是否注册  wuhanligong.xxlun.com/union
http_binding = 'http://wuhanligong.xxlun.com/union/admin/xxl/device/binding/%s'%(ship_code)
# 注册新的湖泊ID
# http_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/map/save'
http_save = 'http://192.168.8.13:8009/union/admin/xxl/map/save'
# 发送检测数据
http_data_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/data/save'

mqtt_host='47.97.183.24'
# 手动模式和自动模式
# mod in ['manual', 'auto']
mod='auto'
# 是否播放声音
b_play_audio=False

# 在家调试模式
home_debug = True
init_gps = [114.431623, 30.523246]

# 到达点距离范围判断，单位米
arrive_distance = 4
