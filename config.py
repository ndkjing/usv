# 保存地图数据路径
import os
import json
import platform

root_path = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(root_path, 'mapsData')
if not os.path.exists(maps_dir):
    os.mkdir(maps_dir)

# 保存所有地图湖泊信息位置
map_data_path = os.path.join(maps_dir, 'map.json')
local_map_data_path = os.path.join(maps_dir, 'local_map.json')

# 保存当前用户点击位置相关信息
usr_lng_lat_path = os.path.join(maps_dir, 'usr_lng_lat_path.json')
base_setting_path = os.path.join(root_path, 'configs', 'base_setting.json')
base_setting_default_path = os.path.join(root_path, 'configs', 'base_setting_default.json')
height_setting_path = os.path.join(root_path, 'configs', 'height_setting.json')
height_setting_default_path = os.path.join(root_path, 'configs', 'height_setting_default.json')

sysstr = platform.system()
if (sysstr == "Windows"):
    print("Call Windows tasks")
    current_platform = 'w'
    if platform.node()=='DESKTOP-MSUAAG9':
        current_platform = 'w_j'
elif (sysstr == "Linux"):  # 树莓派上也是Linux
    print("Call Linux tasks")
    # 公司Linux电脑名称
    if platform.node() == 'jing':
        current_platform = 'l_j'
    elif platform.node() == 'xxl':
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

# 速度等级 1到5级 速度从低到高，仅能控制手动模式下速度   1 级表示1600 5 2000
speed_grade = 2
arrive_distance = 2.5
# 多点和寻点模式下查找连接点数量
keep_point = 0
# 路径搜索保留离湖泊边缘安全路径
path_search_safe_distance = 5
# 寻点模式行间隔
row_gap = 50
# 寻点模式列间隔
col_gap = 50

def update_base_setting():
    global speed_grade
    global arrive_distance
    global find_points_num
    global path_search_safe_distance
    global row_gap
    global col_gap
    if os.path.exists(base_setting_path):
        try:
            with open(base_setting_path, 'r') as f:
                base_setting_data = json.load(f)
            # 读取配置
            if base_setting_data.get('speed_grade'):
                try:
                    s_speed_grade = int(base_setting_data.get('speed_grade'))
                    if s_speed_grade >= 5:
                        s_speed_grade = 5
                    elif s_speed_grade <= 1:
                        s_speed_grade = 1
                    speed_grade = s_speed_grade
                except Exception as e:
                    print({'error': e})
            if base_setting_data.get('arrive_range'):
                try:
                    s_arrive_distance = float(base_setting_data.get('arrive_range'))
                    if s_arrive_distance < 2:
                        s_arrive_distance = 2.0
                    elif s_arrive_distance > 10:
                        s_arrive_distance = 10.0
                    arrive_distance = s_arrive_distance
                except Exception as e:
                    print({'error': e})

            if base_setting_data.get('keep_point'):
                try:
                    s_keep_point = int(base_setting_data.get('keep_point'))
                    if s_keep_point <= 0:
                        s_keep_point = 0
                    elif s_keep_point >= 1:
                        s_keep_point = 1
                    keep_point = s_keep_point
                except Exception as e:
                    print({'error': e})

            if base_setting_data.get('secure_distance'):
                try:
                    s_path_search_safe_distance = int(base_setting_data.get('secure_distance'))
                    if s_path_search_safe_distance > 10:
                        s_path_search_safe_distance = 10
                    elif s_path_search_safe_distance < 2:
                        s_path_search_safe_distance = 2
                    path_search_safe_distance = s_path_search_safe_distance
                except Exception as e:
                    print({'error': e})

            if base_setting_data.get('row'):
                try:
                    s_row_gap = int(base_setting_data.get('row'))
                    if s_row_gap < 0:
                        s_row_gap = 10
                    row_gap = s_row_gap
                except Exception as e:
                    print({'error': e})

            if base_setting_data.get('col'):
                try:
                    s_col_gap = int(base_setting_data.get('col'))
                    if s_col_gap < 0:
                        s_col_gap = 10
                    col_gap = s_col_gap
                except Exception as e:
                    print({'error': e})

        except Exception as e:
            print({'error': e})


update_base_setting()

# 罗盘等待时间间隔
compass_timeout = 0.2

# 单片机发送给树莓派等待时间
com2pi_interval = 1

# 给单片机发送等待时间
pi2com_interval = 0.05

# 给服务器发送时间间隔
pi2mqtt_interval = 1

# 上传给单片机心跳时间间隔 单位秒
com_heart_time = 1 * 60

# 船编号
ship_code = '3c50f4c3-a9c1-4872-9f18-883af014380a'

# 串口位置和波特率
# 单片机
port = '/dev/ttyAMA0'
baud = 115200
# imu
imu_port = '/dev/imu'
imu_baud = 115200
# 飞控
if current_platform == 'l':
    pix_port = '/dev/ttyACM0'
else:
    pix_port = 'com22'
pix_baud = 115200
b_use_pix = False

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
http_binding = 'http://wuhanligong.xxlun.com/union/admin/xxl/device/binding/%s' % (ship_code)
# 注册新的湖泊ID
http_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/map/save'
# http_save = 'http://192.168.8.13:8009/union/admin/xxl/map/save'
# 发送检测数据
http_data_save = 'http://wuhanligong.xxlun.com/union/admin/xxl/data/save'
# http_data_save = 'http://192.168.8.13:8009/union/admin/xxl/data/save'

mqtt_host = '47.97.183.24'
mqtt_port = 1884
# 手动模式和自动模式
# mod in ['manual', 'auto']
mod = 'auto'
# 是否播放声音
b_play_audio = False

# 在家调试模式
home_debug = False
init_gaode_gps = [114.348713, 30.464501]
ship_gaode_lng_lat = [114.432893, 30.527554]
# ship_gaode_lng_lat=[114.5242, 30.506895]

# 路径搜索像素安全距离
# path_search_safe_distance = 10
# 到达点距离范围判断，单位米
# arrive_distance = 2.5
# # 检测像素间隔
# pix_interval=4

# 电机前进分量
motor_forward = 350
# 电机转弯分量
motor_steer = 450
# pid三参数
kp = 1.0
ki = 0.1
kd = 0.3
# 大于多少米全速
full_speed_meter = 5.0
# 发送状态数据时间间隔
check_status_interval = 1.0
# 最大pwm值
max_pwm = 2000
# 最小pwm值
min_pwm = 1000
# 停止中位pwm
stop_pwm = 1500
# 左侧电机正反桨  0 正桨叶   1 反桨叶
left_motor_cw = 0
# 右侧电机正反桨  0 正桨叶   1 反桨叶
right_motor_cw = 0
# 抽水时间单位秒
draw_time = 20
# pid间隔
pid_interval = 0.1
# 开机前等待时间
start_sleep_time = 6
# 电机初始化时间
motor_init_time = 4
# 检查网络连接状态间隔
check_network_interval = 10
# 断网返航 0关闭  1开启 大于1的数值表示断网超过该值就返航，默认100秒
network_backhome = 0
# 剩余电量返航 0关闭  1开启 大于1的数值表示剩余电量低于该值就返航，默认30
energy_backhome = 0
# 最多查找连接点数量
find_points_num=5

def update_height_setting():
    global motor_forward
    global motor_steer
    global kp
    global ki
    global kd
    global full_speed_meter
    global check_status_interval
    global check_network_interval
    global max_pwm
    global min_pwm
    global stop_pwm
    global left_motor_cw
    global right_motor_cw
    global draw_time
    global pid_interval
    global start_sleep_time
    global motor_init_time
    global network_backhome
    global energy_backhome
    global find_points_num

    if os.path.exists(height_setting_path):
        try:
            with open(height_setting_path, 'r') as f:
                height_setting_data = json.load(f)
            # 读取配置
            if height_setting_data.get('motor_forward'):
                try:
                    s_motor_forward = int(height_setting_data.get('motor_forward'))
                    if s_motor_forward <= 0:
                        s_motor_forward = 200
                    elif s_motor_forward >= 500:
                        s_motor_forward = 500
                    motor_forward = s_motor_forward
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('motor_steer'):
                try:
                    s_motor_steer = int(height_setting_data.get('motor_steer'))
                    if s_motor_steer <= 0:
                        s_motor_steer = 200
                    elif s_motor_steer >= 1000:
                        s_motor_steer = 1000
                    motor_steer = s_motor_steer
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('kp'):
                try:
                    s_kp = float(height_setting_data.get('kp'))
                    kp = s_kp
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('ki'):
                try:
                    s_ki = float(height_setting_data.get('ki'))
                    ki = s_ki
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('kd'):
                try:
                    s_kd = float(height_setting_data.get('kd'))
                    kd = s_kd
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('full_speed_meter'):
                try:
                    s_full_speed_meter = float(height_setting_data.get('full_speed_meter'))
                    if s_full_speed_meter < 3:
                        s_full_speed_meter = 3
                    full_speed_meter = s_full_speed_meter
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('check_status_interval'):
                try:
                    s_check_status_interval = float(height_setting_data.get('check_status_interval'))
                    if s_check_status_interval < 1:
                        s_check_status_interval = 1
                    elif s_check_status_interval > 10:
                        s_check_status_interval = 10
                    check_status_interval = s_check_status_interval
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('check_network_interval'):
                try:
                    s_check_network_interval = float(height_setting_data.get('check_network_interval'))
                    if s_check_network_interval < 5:
                        s_check_network_interval = 5
                    elif s_check_network_interval > 100:
                        s_check_network_interval = 100
                    check_network_interval = s_check_network_interval
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('max_pwm'):
                try:
                    s_max_pwm = int(height_setting_data.get('max_pwm'))
                    if s_max_pwm >= 2200:
                        s_max_pwm = 2200
                    max_pwm = s_max_pwm
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('min_pwm'):
                try:
                    s_min_pwm = int(height_setting_data.get('min_pwm'))
                    if s_min_pwm <= 800:
                        s_min_pwm = 800
                    min_pwm = s_min_pwm
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('stop_pwm'):
                try:
                    s_stop_pwm = int(height_setting_data.get('stop_pwm'))
                    if s_stop_pwm <= min_pwm or s_stop_pwm > max_pwm:
                        s_stop_pwm = int((min_pwm + max_pwm) / 2)
                    stop_pwm = s_stop_pwm
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('left_motor_cw'):
                try:

                    left_motor_cw = int(height_setting_data.get('left_motor_cw'))
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('right_motor_cw'):
                try:
                    right_motor_cw = int(height_setting_data.get('right_motor_cw'))
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('draw_time'):
                try:
                    s_draw_time = int(height_setting_data.get('draw_time'))
                    if s_draw_time < 0:
                        s_draw_time = 10
                    draw_time = s_draw_time
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('pid_interval'):
                try:
                    s_pid_interval = float(height_setting_data.get('pid_interval'))
                    if s_pid_interval < 0:
                        s_pid_interval = 0.1
                    pid_interval = s_pid_interval
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('start_sleep_time'):
                try:
                    s_start_sleep_time = int(height_setting_data.get('start_sleep_time'))
                    if s_start_sleep_time < 0:
                        s_start_sleep_time = 3
                    start_sleep_time = s_start_sleep_time
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('motor_init_time'):
                try:
                    s_motor_init_time = int(height_setting_data.get('motor_init_time'))
                    if s_motor_init_time < 0:
                        s_motor_init_time = 3
                    motor_init_time = s_motor_init_time
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('network_backhome'):
                try:
                    s_network_backhome = int(height_setting_data.get('network_backhome'))
                    if s_network_backhome <= 0:
                        s_network_backhome = 0
                    network_backhome = s_network_backhome
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('energy_backhome'):
                try:
                    s_energy_backhome = int(height_setting_data.get('energy_backhome'))
                    if s_energy_backhome <= 0:
                        s_energy_backhome = 0
                    elif s_energy_backhome >= 100:
                        s_energy_backhome = 80

                    energy_backhome = s_energy_backhome
                except Exception as e:
                    print({'error': e})

            if height_setting_data.get('find_points_num'):
                try:
                    s_find_points_num = int(height_setting_data.get('find_points_num'))
                    if s_find_points_num <= 0:
                        s_find_points_num = 5
                    elif s_find_points_num >= 20:
                        s_find_points_num = 20
                    find_points_num = s_find_points_num
                except Exception as e:
                    print({'error': e})
        except Exception as e:
            print({'error': e})

update_height_setting()

# 保存配置到文件中
def write_setting(b_base=False, b_height=False, b_base_default=False, b_height_default=False):
    if b_base:
        with open(base_setting_path, 'w') as bf:
            json.dump({'speed_grade': speed_grade,
                       'arrive_range': arrive_distance,
                       'keep_point': find_points_num,
                       'secure_distance': path_search_safe_distance,
                       'row': row_gap,
                       'col': col_gap,
                       },
                      bf)
    if b_base_default:
        with open(base_setting_default_path, 'w') as bdf:
            json.dump({'speed_grade': speed_grade,
                       'arrive_range': arrive_distance,
                       'keep_point': find_points_num,
                       'secure_distance': path_search_safe_distance,
                       'row': row_gap,
                       'col': col_gap,
                       },
                      bdf)
    if b_height:
        with open(height_setting_path, 'w') as hf:
            json.dump({'motor_forward': motor_forward,
                       'motor_steer': motor_steer,
                       'kp': kp,
                       'ki': ki,
                       'kd': kd,
                       'full_speed_meter': full_speed_meter,
                       'check_status_interval': check_status_interval,
                       'max_pwm': max_pwm,
                       'min_pwm': min_pwm,
                       'left_motor_cw': left_motor_cw,
                       'right_motor_cw': right_motor_cw,
                       'draw_time': draw_time,
                       'pid_interval': pid_interval,
                       'start_sleep_time':start_sleep_time,
                       'motor_init_time':motor_init_time,
                       'check_network_interval':check_network_interval,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num':find_points_num
                       },
                      hf)
    if b_height_default:
        with open(height_setting_default_path, 'w') as hdf:
            json.dump({'motor_forward': motor_forward,
                       'motor_steer': motor_steer,
                       'kp': kp,
                       'ki': ki,
                       'kd': kd,
                       'full_speed_meter': full_speed_meter,
                       'check_status_interval': check_status_interval,
                       'max_pwm': max_pwm,
                       'min_pwm': min_pwm,
                       'left_motor_cw': left_motor_cw,
                       'right_motor_cw': right_motor_cw,
                       'draw_time': draw_time,
                       'pid_interval': pid_interval,
                       'start_sleep_time':start_sleep_time,
                       'motor_init_time':motor_init_time,
                       'check_network_interval':check_network_interval,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num': find_points_num
                       },
                      hdf)

# 保存返航点地址路径
home_location_path = os.path.join(root_path, 'home_location.json')

# 使用路径规划和避免湖泊轮廓
if home_debug:
    b_use_path_planning = True
else:
    b_use_path_planning = False
# 检查路径规划
b_check_path_planning = False

# 是否使用启动按钮
b_use_start = False

########### 树莓派GPIO端口相关设置
# 使用树莓派控制电机
b_use_pi = True
# 左侧电机信号输出控制口
left_pwm_pin = 23
# 右侧电机信号输出控制口
right_pwm_pin = 24
# 是否使用遥控器
b_use_remote_control = False
# 1 通道 水平
channel_1_pin = 12
# 3 通道 垂直
channel_3_pin = 16

# 是否使用超声波
b_use_ultrasonic = False
ultrasonic_baud = 9600
left_rx = 4
left_tx = 17
right_rx = 27
right_tx = 22

if home_debug:
    write_setting(True,True,True,True)
