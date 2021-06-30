# 保存地图数据路径
import enum
import json
import os
import platform
root_path = os.path.dirname(os.path.abspath(__file__))
import ship_code_config
maps_dir = os.path.join(root_path, 'statics', 'mapsData')
if not os.path.exists(maps_dir):
    os.mkdir(os.path.join(root_path, 'statics'))
    os.mkdir(os.path.join(root_path, 'statics', 'mapsData'))

# 保存所有地图湖泊信息位置
# map_data_path = os.path.join(maps_dir, 'map.json')
local_map_data_path = os.path.join(maps_dir, 'local_map.json')

# 保存当前用户点击位置相关信息
usr_lng_lat_path = os.path.join(maps_dir, 'usr_lng_lat_path.json')
# 保存行驶路径和时间数据
run_distance_time_path = os.path.join(root_path, 'statics', 'run_distance_time_path.json')
base_setting_path = os.path.join(root_path, 'statics', 'configs', 'base_setting.json')
base_setting_default_path = os.path.join(root_path, 'statics', 'configs', 'base_setting_default.json')
height_setting_path = os.path.join(root_path, 'statics', 'configs', 'height_setting.json')
height_setting_default_path = os.path.join(root_path, 'statics', 'configs', 'height_setting_default.json')
# 保存湖号和路径数据
save_plan_path = os.path.join(root_path, 'statics', 'configs', 'save_plan_path.json')
# 保存声呐信息路径
save_sonar_path = os.path.join(root_path, 'statics', 'geojeson_data.json')


class CurrentPlatform(enum.Enum):
    windows = 1
    linux = 2
    pi = 3
    others = 4


sysstr = platform.system()
if sysstr == "Windows":
    print("Call Windows tasks")
    current_platform = CurrentPlatform.windows
elif sysstr == "Linux":  # 树莓派上也是Linux
    print("Call Linux tasks")
    # 公司Linux电脑名称
    if platform.node() == 'raspberrypi':
        current_platform = CurrentPlatform.pi
    else:
        current_platform = CurrentPlatform.linux
else:
    print("other System tasks")
    current_platform = CurrentPlatform.others

# 百度地图key
baidu_key = 'wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
# 高德秘钥
gaode_key = '8177df6428097c5e23d3280ffdc5a13a'
# 腾讯地图key
tencent_key = 'PSABZ-URMWP-3ATDK-VBRCR-FBBMF-YHFCE'

# 速度等级 1到5级 速度从低到高，仅能控制手动模式下速度   1 级表示1600 5 2000
speed_grade = 3
arrive_distance = 2.5
# 多点和寻点模式下查找连接点数量
keep_point = 0
# 路径搜索保留离湖泊边缘安全路径
path_search_safe_distance = 15
# 寻点模式行间隔
row_gap = 50
# 寻点模式列间隔
col_gap = 50
# 湖泊名称
pool_name = "梁子湖"
# 视频链接
video_url = "http://123.32132.321321.213"


def update_base_setting():
    global speed_grade
    global arrive_distance
    global find_points_num
    global path_search_safe_distance
    global row_gap
    global col_gap
    global pool_name
    global video_url
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
            if base_setting_data.get('pool_name'):
                try:
                    s_pool_name = base_setting_data.get('pool_name')
                    pool_name = s_pool_name
                except Exception as e:
                    print({'error': e})
            if base_setting_data.get('video_url'):
                try:
                    s_video_url = base_setting_data.get('video_url')
                    video_url = s_video_url
                except Exception as e:
                    print({'error': e})
        except Exception as e:
            print({'error': e})


# 罗盘等待时间间隔
compass_timeout = 0.1
# 单片机发送给树莓派等待时间
stc2pi_timeout = 1
# 给单片机发送等待时间
pi2com_timeout = 0.05
# 给服务器发送时间间隔
pi2mqtt_interval = 1
# 上传给单片机心跳时间间隔 单位秒
# com_heart_time = 1 * 60
# 船编号
ship_code = ship_code_config.ship_code

# 串口位置和波特率
# 单片机
# stc_port = '/dev/ttyAMA0'
stc_port = '/dev/ttyUSB0'
stc_baud = 115200
# imu
imu_port = '/dev/imu'
imu_baud = 115200

if current_platform == CurrentPlatform.pi:
    pix_port = '/dev/ttyACM0'
else:
    pix_port = 'tcp:127.0.0.1:5760'
pix_baud = 115200
b_use_pix = False

# GPS
gps_port = '/dev/gps'
gps_baud = 115200
gps_frequency = 1
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

ship_gaode_lng_lat = [114.524096, 30.506853]
# ship_gaode_lng_lat = [117.202177,39.901856]

# 电机前进分量
motor_forward = 200
# 电机转弯分量
motor_steer = 200
# pid三参数
kp = 2.0
ki = 0.3
kd = 1.0
# 大于多少米全速
full_speed_meter = 6.0
# 发送状态数据时间间隔
check_status_interval = 1.0
# 最大pwm值
max_pwm = 1800
# 最小pwm值
min_pwm = 1200
# 停止中位pwm
stop_pwm = 1500
# 左侧电机正反桨  0 正桨叶   1 反桨叶
left_motor_cw = 1
# 右侧电机正反桨  0 正桨叶   1 反桨叶
right_motor_cw = 0
# 抽水时间单位秒
draw_time = 30
# pid间隔
pid_interval = 0.2
# 开机前等待时间
start_sleep_time = 6
# 电机初始化时间
motor_init_time = 1
# 检查网络连接状态间隔
check_network_interval = 10
# 断网返航 0关闭  1开启 大于1的数值表示断网超过该值就返航，默认100秒
network_backhome = 1
# 剩余电量返航 0关闭  1开启 大于1的数值表示剩余电量低于该值就返航，默认30
energy_backhome = 1
# 最多查找连接点数量
find_points_num = 5
# TSP优化路径 0 不使用  1使用
b_tsp = 0
# 断网检查
b_check_network = 1
# 是否播放声音
b_play_audio = 0
# 不是在树莓派上都是用调试模式
if current_platform == CurrentPlatform.pi:
    home_debug = 0
else:
    home_debug = 1
# 添加避障方式设置0 不避障 1 停止  2 绕行 3 手动模式下避障
obstacle_avoid_type = 0
# 路径规划方式  0 不平滑路径 1 平滑路径
path_plan_type = 1
# 路径跟踪方式  1 pid    2 pure pursuit  3 宫凯调试的pid
path_track_type = 1
# 校准罗盘  0 不校准 1 开始校准 2 结束校准
calibration_compass = 0
# 地图规划最小单位，米
cell_size = int(arrive_distance)
# 是否使用平滑路径  1 平滑路径 0 不平滑
b_smooth_path = path_plan_type
# 平滑路径最小单位 m
smooth_path_ceil_size = 5
# 前视觉距离
forward_see_distance = 9
# 舵机最大扫描角度单侧 左边为正右边为负
steer_max_angle = 30
# 最小转向距离
min_steer_distance = 6

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
    global b_check_network
    global b_play_audio
    global home_debug
    global obstacle_avoid_type
    global path_plan_type
    global path_track_type
    global calibration_compass

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
            if height_setting_data.get('b_check_network'):
                try:
                    s_b_check_network = int(height_setting_data.get('b_check_network'))
                    if s_b_check_network in [0, 1]:
                        pass
                    else:
                        s_b_check_network = 0
                    b_check_network = s_b_check_network
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('b_play_audio'):
                try:
                    s_b_play_audio = int(height_setting_data.get('b_play_audio'))
                    if s_b_play_audio in [0, 1, 2, 3, 4, 5, 6, 7, 8]:
                        pass
                    else:
                        s_b_play_audio = 0
                    b_play_audio = s_b_play_audio
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('home_debug'):
                try:
                    s_home_debug = int(height_setting_data.get('home_debug'))
                    if s_home_debug in [0, 1]:
                        pass
                    else:
                        s_home_debug = 0
                    home_debug = s_home_debug
                    # 如果在树莓派上不能使用调试模式
                    if current_platform == CurrentPlatform.pi:
                        home_debug = 0
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('obstacle_avoid_type'):
                try:
                    s_obstacle_avoid_type = int(height_setting_data.get('obstacle_avoid_type'))
                    if s_obstacle_avoid_type in [0, 1, 2, 3, 4]:
                        pass
                    else:
                        s_obstacle_avoid_type = 0
                    obstacle_avoid_type = s_obstacle_avoid_type
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('path_plan_type'):
                try:
                    s_path_plan_type = int(height_setting_data.get('path_plan_type'))
                    if s_path_plan_type in [0, 1]:
                        pass
                    else:
                        s_path_plan_type = 0
                    path_plan_type = s_path_plan_type
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('path_track_type'):
                try:
                    s_path_track_type = int(height_setting_data.get('path_track_type'))
                    if s_path_track_type in [0, 1, 2, 3]:
                        pass
                    else:
                        s_path_track_type = 0
                    path_track_type = s_path_track_type
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('calibration_compass'):
                try:
                    s_calibration_compass = int(height_setting_data.get('calibration_compass'))
                    if s_calibration_compass in [0, 1, 2]:
                        pass
                    else:
                        s_calibration_compass = 0
                    calibration_compass = s_calibration_compass
                except Exception as e:
                    print({'error': e})

        except Exception as e:
            print({'error': e})


def update_setting():
    update_base_setting()
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
                       'pool_name': pool_name,
                       'video_url': video_url
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
                       'pool_name': pool_name,
                       'video_url': video_url
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
                       'start_sleep_time': start_sleep_time,
                       'motor_init_time': motor_init_time,
                       'check_network_interval': check_network_interval,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num': find_points_num,
                       'b_tsp': b_tsp,
                       'b_check_network': b_check_network,
                       'b_play_audio': b_play_audio,
                       'home_debug': home_debug,
                       'obstacle_avoid_type': obstacle_avoid_type,
                       'path_plan_type': path_plan_type,
                       'path_track_type': path_track_type,
                       'calibration_compass': calibration_compass
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
                       'start_sleep_time': start_sleep_time,
                       'motor_init_time': motor_init_time,
                       'check_network_interval': check_network_interval,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num': find_points_num,
                       'b_tsp': b_tsp,
                       'b_check_network': b_check_network,
                       'b_play_audio': b_play_audio,
                       'home_debug': home_debug,
                       'obstacle_avoid_type': obstacle_avoid_type,
                       'path_plan_type': path_plan_type,
                       'path_track_type': path_track_type,
                       'calibration_compass': calibration_compass
                       },
                      hdf)


# 保存返航点地址路径
home_location_path = os.path.join(root_path, 'home_location.json')
# 是否使用启动按钮
b_use_start = False

########### 树莓派GPIO端口相关设置 均使用BCM编码端口
# 是否使用超声波
b_use_ultrasonic = 0
# 使用树莓派控制电机
b_use_pi = True
# 水下摄像头云台水平和俯仰
pin_pan = 2
pin_tilt = 3
# 左侧电机信号输出控制口
left_pwm_pin = 20
# 右侧电机信号输出控制口
right_pwm_pin = 21
# 软串口罗盘
b_pin_compass = 1
pin_compass_baud = 9600
pin_compass_tx = 27
pin_compass_rx = 22
# 软串口gps
b_pin_gps = 1
pin_gps_baud = 9600
pin_gps_tx = 10
pin_gps_rx = 9
# 是否使用遥控器
b_use_remote_control = True
# usv a 遥控器  水平是1通道   垂直是2通道
# 水平
channel_1_pin = 5
# 垂直
channel_3_pin = 6
# 开启遥控器输入pin口
channel_remote_pin = 11
# 激光雷达
b_laser = 0
laser_tx = 13
laser_rx = 19
laser_baud = 115200
laser_hz = 40
# 激光雷达舵机输出
steer_engine_pin = 26
# 毫米波雷达 millimeter wave radar
b_millimeter_wave = 1
angle_ceil_size = 5
detect_angle = 45
field_of_view = 90
view_cell = 5
ceil_max = 3 #  可以通过扇区阈值
millimeter_wave_tx = 13
millimeter_wave_rx = 19
millimeter_wave_baud = 115200
millimeter_wave_hz = 40
# 单片机串口
b_pin_stc = 0
stc_tx = 14
stc_rx = 15
stc_baud = 115200
# 舷灯 左舷灯 右舷灯
side_left_gpio_pin = 18
side_right_gpio_pin = 23
# 前大灯
headlight_gpio_pin = 24
# 左抽水泵  右抽水泵
draw_left_gpio_pin = 25
draw_right_gpio_pin = 8
# 声光报警器
audio_light_alarm_gpio_pin = 7
# 漏水传感器
leak_gpio_pin = 12
# 声呐
b_sonar = 0
# RX
sonar_rx = 16
# TX
sonar_tx = 20
sonar_baud = 9600
# 声呐舵机
sonar_steer = 21
test_all=0

# 使用角度  1 使用罗盘1角度   3 使用经纬度移动计算角度
if home_debug:
    use_shape_theta_type = 3
else:
    use_shape_theta_type = 1

b_draw = 1
# 测试在家调试也发送数据
debug_send_detect_data = 1
class WaterType(enum.Enum):
    wt = 0
    EC = 1
    pH = 2
    DO = 3
    TD = 4
    NH3_NH4 = 5
if __name__ == '__main__':
    write_setting(True, True, True, True)
