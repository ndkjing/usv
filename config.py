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

import platform
sysstr = platform.system()
if (sysstr == "Windows"):
    print("Call Windows tasks")
    current_platform='w'
elif (sysstr == "Linux"):   # 树莓派上也是Linux
    print("Call Linux tasks")
    # 公司Linux电脑名称
    if platform.node()=='jing':
        current_platform = 'l_j'
    elif platform.node()=='xxl':
        current_platform = 'l_x'
    else:
        current_platform = 'l'
else:
    print("other System tasks")
    current_platform = 'o'

# 高德秘钥
gaode_key = '8177df6428097c5e23d3280ffdc5a13a'
# 腾讯地图key
tencent_key = 'PSABZ-URMWP-3ATDK-VBRCR-FBBMF-YHFCE'
# 百度地图key
baidu_key = 'wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'

# 检测像素间隔
pix_interval=4

# 罗盘等待时间间隔
compass_timeout = 0.2

# 单片机发送给树莓派等待时间
com2pi_interval = 1

# 给单片机发送等待时间
pi2com_interval = 0.05

# 给服务器发送时间间隔
pi2mqtt_interval = 1.5

# 接收服务器方向控制间隔
mqtt_control_interval = 1

# 检查船状态间隔 单位秒
check_status_interval = 1.5

# 检查网络连接状态间隔
check_network_interval=10

# 上传给单片机心跳时间间隔 单位秒
com_heart_time = 1*60

# 船编号
ship_code = '3c50f4c3-a9c1-4872-9f18-883af014380c'

## 串口位置和波特率
# 单片机
port = '/dev/ttyAMA0'
baud = 115200
# imu
imu_port = '/dev/imu'
imu_baud = 115200
# 飞控
if current_platform=='l':
    pix_port='/dev/ttyACM0'
else:
    pix_port = 'com22'
pix_baud=115200
b_use_pix=False

# GPS
gps_port = '/dev/gps'
gps_baud = 115200

# 罗盘
compass_port = '/dev/compass0'
compass_baud = 9600

compass_port1 = '/dev/compass1'
compass_baud1 = 9600

# http 接口
# 查询船是否注册  wuhanligong.xxlun.com/union
http_binding = 'http://wuhanligong.xxlun.com/union/admin/xxl/device/binding/%s'%(ship_code)
# 注册新的湖泊ID
http_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/map/save'
# http_save = 'http://192.168.8.13:8009/union/admin/xxl/map/save'
# 发送检测数据
http_data_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/data/save'
# http_data_save = 'http://192.168.8.13:8009/union/admin/xxl/data/save'

mqtt_host='47.97.183.24'
mqtt_port = 1884
# 手动模式和自动模式
# mod in ['manual', 'auto']
mod='auto'
# 是否播放声音
b_play_audio=False

# 在家调试模式
home_debug = False
init_gaode_gps = [114.348713,30.464501]
ship_gaode_lng_lat=[114.434561,30.519726]
# ship_gaode_lng_lat=[114.5242, 30.506895]

# 路径搜索像素安全距离
path_search_safe_distance = 10

# 到达点距离范围判断，单位米
arrive_distance = 2.5

# 查找数量
find_points_num=7

max_pwm = 2000
min_pwm = 1000
# pid间隔
pid_interval=0.1

# 保存返航点地址路径
home_location_path = os.path.join(root_path, 'home_location.json')

# 使用路径规划和避免湖泊轮廓
if home_debug:
    b_use_path_planning=True
else:
    b_use_path_planning=False
# 检查路径规划
b_check_path_planning = False

# 是否使用启动按钮
b_use_start=False

########### 树莓派相关设置
# 使用树莓派控制电机
b_use_pi=True

# 正反桨页设置  0 正桨叶   1 反桨叶
left_motor_cw = 0
right_motor_cw = 0
# 电机前进分量
motor_forward = 350
# 电机转弯分量
motor_steer = 450
# 大于多少米全速前进
full_speed_meter=5.0
kp = 1
ki = 0.1
kd = 0.3

# 左侧电机信号输出控制口
left_pwm_pin = 23
# 右侧电机信号输出控制口
right_pwm_pin = 24

# 是否使用遥控器
b_use_remote_control=False
#1 通道 水平
channel_1_pin = 12
#3 通道 垂直
channel_3_pin = 16

# 是否使用超声波
b_use_ultrasonic=False
ultrasonic_baud = 9600
left_rx = 4
left_tx = 17
right_rx = 27
right_tx = 22

# 抽水时间 单位秒
draw_time = 20
# 开机前等待时间
start_sleep_time=6
# 电机初始化时间
motor_init_time = 4

