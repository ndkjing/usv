# 保存地图数据路径
import enum
import json
import os
import platform
import ship_code_config

root_path = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(root_path, 'statics', 'mapsData')
if not os.path.exists(maps_dir):
    if not os.path.exists(os.path.join(root_path, 'statics')):
        os.mkdir(os.path.join(root_path, 'statics'))
    os.mkdir(os.path.join(root_path, 'statics', 'mapsData'))

# 保存所有地图湖泊信息位置
local_map_data_path = os.path.join(maps_dir, 'local_map.json')
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
# 保存抓取的水质数据
save_water_data_path = os.path.join(root_path, 'statics', 'water_data.json')
# 保存返航点地址路径
home_location_path = os.path.join(root_path, 'home_location.json')
# 记录罗盘数据
save_compass_data_dir = os.path.join(root_path, 'statics')


class CurrentPlatform(enum.Enum):
    windows = 1
    linux = 2
    pi = 3
    others = 4


# 船类型
class ShipType(enum.Enum):
    single_draw = 1
    multi_draw = 2
    water_detect = 3
    dock = 4
    adcp = 5


current_ship_type = ShipType.water_detect

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
# 不是在树莓派上都是用调试模式
if current_platform == CurrentPlatform.pi:
    home_debug = 0
else:
    home_debug = 1
# 百度地图key
baidu_key = 'wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
# 高德秘钥
gaode_key = '8177df6428097c5e23d3280ffdc5a13a'
# 腾讯地图key
tencent_key = 'PSABZ-URMWP-3ATDK-VBRCR-FBBMF-YHFCE'

draw_time = 30

# 速度等级 1到5级 速度从低到高，仅能控制手动模式下速度   1 级表示1600 5 2000
speed_grade = 3
arrive_distance = 2.5


# 多点和寻点模式下查找连接点数量
# keep_point = 0


def update_base_setting():
    global speed_grade
    global arrive_distance
    global find_points_num
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
        except Exception as e:
            print({'error': e})


# 单片机发送给树莓派等待时间
stc2pi_timeout = 1
# 船编号
ship_code = ship_code_config.ship_code
# 串口位置和波特率
b_use_com_stc = 0  # 是否使用单片机硬件转接串口
stc_port = '/dev/ttyUSB0'  # '/dev/ttyAMA0'
stc_baud = 115200
b_com_stc = os.path.exists(stc_port) and b_use_com_stc  # 判断是否存在以及是否使用
# http 接口
# 查询船是否注册  wuhanligong.xxlun.com/union
http_binding = 'https://ship.xxlun.com/union/admin/xxl/device/binding/%s' % ship_code
# 注册新的湖泊ID
http_save = 'https://ship.xxlun.com/union/admin/xxl/map/save'
# http_save = 'http://192.168.199.186:8009/union/admin/xxl/map/save'
# 更新湖泊轮廓
http_update_map = 'https://ship.xxlun.com/union/admin/xxl/map/upData'
# http_update_map = 'http://192.168.199.186:8009/union/admin/xxl/map/upData'
# 发送检测数据
http_data_save = 'https://ship.xxlun.com/union/admin/xxl/data/save'
# http_data_save = 'http://192.168.199.186:8009/union/admin/xxl/data/save'
# 发送抽水瓶号数据
http_draw_save = 'https://ship.xxlun.com/union/admin/xxl/data/sampling/save'
# http_draw_save = 'http://192.168.199.186:8009/union/admin/xxl/data/sampling/save'
# 获取存储的任务数据
http_get_task = 'https://ship.xxlun.com/union/admin/xxl/task/getOne'
http_update_task = 'http://ship.xxlun.com/union/admin/xxl/task/upDataTask'
http_delete_task = 'http://ship.xxlun.com/union/admin/xxl/task/delTask'
# 上传日志接口
http_log = 'https://ship.xxlun.com/union/admin/xxl/log/save'
# http_log = 'http://192.168.199.186:8009/union/admin/xxl/log/save'
# 里程接口
http_mileage_get = 'https://ship.xxlun.com/union/admin/xxl/mileage/getOne'
http_mileage_save = 'https://ship.xxlun.com/union/admin/xxl/mileage/save'
http_mileage_update = 'https://ship.xxlun.com/union/admin/xxl/mileage/upData'
# http_mileage_get = 'http://192.168.199.186:8009/union/admin/xxl/mileage/getOne'
# http_mileage_save = 'http://192.168.199.186:8009/union/admin/xxl/mileage/save'
# http_mileage_update = 'http://192.168.199.186:8009/union/admin/xxl/mileage/upData'
# 发送手动记录路劲数据
http_record_path = "https://ship.xxlun.com/union/admin/xxl/device/saveRoute"
# http_record_path = "http://192.168.8.26:8009/union/admin/xxl/device/saveRoute"
# 获取手动记录轨迹
http_record_get = "https://ship.xxlun.com/union/admin/xxl/device/getRoute/1/1"
# http_record_get = "http://192.168.8.26:8009/union/admin/xxl/device/getRoute/1/1"
# mqtt服务器ip地址和端口号
mqtt_host = '47.97.183.24'
mqtt_port = 1884
# 调试的时候使用初始经纬度
ship_gaode_lng_lat = [114.524096, 30.506853]  # 九峰水库
# ship_gaode_lng_lat = [114.431419,30.524192]     # 喻家湖
# pid三参数
kp = 0.6
ki = 0.2
kd = 2.0
# 最大pwm值
max_pwm = 1800
# 最小pwm值
min_pwm = 1200
# 停止中位pwm
stop_pwm = 1500
# 左侧电机正反桨  0 正桨叶   1 反桨叶
left_motor_cw = 0
# 右侧电机正反桨  0 正桨叶   1 反桨叶
right_motor_cw = 1
# 断网返航 0关闭  1开启 大于1的数值表示断网超过该值就返航，默认600秒
network_backhome = 1
# 剩余电量返航 0关闭  1开启 大于1的数值表示剩余电量低于该值就返航，默认30
energy_backhome = 1
# 最多查找连接点数量
find_points_num = 5
# TSP优化路径 0 不使用  1使用
b_tsp = 0
# 添加避障方式设置0 不避障 1 避障停止  2 自动避障绕行  3 自动避障绕行和手动模式下避障停止
obstacle_avoid_type = 0
control_obstacle_distance = 2.5  # 手动模式避障距离 单位m
# 路径规划方式  0 不平滑路径 1 平滑路径
path_plan_type = 1
# 校准罗盘  0 不校准 1 开始校准 2 结束校准
calibration_compass = 0
# 地图规划最小单位，米
cell_size = int(arrive_distance)
# 平滑路径最小单位 m
smooth_path_ceil_size = 4
# 前视觉距离
forward_see_distance = 9
# 舵机最大扫描角度单侧 左边为正右边为负
steer_max_angle = 30
# 最小转向距离
min_steer_distance = 20  # 自动模式下避障距离 单位m
# 测试在家调试也发送数据
debug_send_detect_data = 0
# 转向速度
angular_velocity = 90

motor_init_time = 1


def update_height_setting():
    global kp
    global ki
    global kd
    global max_pwm
    global min_pwm
    global stop_pwm
    global left_motor_cw
    global right_motor_cw
    global network_backhome
    global energy_backhome
    global find_points_num
    global home_debug
    global obstacle_avoid_type
    global path_plan_type
    global calibration_compass
    if os.path.exists(height_setting_path):
        try:
            with open(height_setting_path, 'r') as f:
                height_setting_data = json.load(f)
            # 读取配置
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
            if height_setting_data.get('max_pwm'):
                try:
                    s_max_pwm = int(height_setting_data.get('max_pwm'))
                    if s_max_pwm >= 2000:
                        s_max_pwm = 2000
                    max_pwm = s_max_pwm
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('min_pwm'):
                try:
                    s_min_pwm = int(height_setting_data.get('min_pwm'))
                    if s_min_pwm <= 1000:
                        s_min_pwm = 1000
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
            if height_setting_data.get('left_motor_cw') is not None:
                try:
                    print('height_setting_data.getleft_motor_cw', height_setting_data.get('left_motor_cw'))
                    left_motor_cw = int(height_setting_data.get('left_motor_cw'))
                except Exception as e:
                    print({'error': e})
            if height_setting_data.get('left_motor_cw') is not None:
                try:
                    right_motor_cw = int(height_setting_data.get('right_motor_cw'))
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
                       },
                      bf)
    if b_base_default:
        with open(base_setting_default_path, 'w') as bdf:
            json.dump({'speed_grade': speed_grade,
                       'arrive_range': arrive_distance,
                       'keep_point': find_points_num,
                       },
                      bdf)
    if b_height:
        with open(height_setting_path, 'w') as hf:
            json.dump({'kp': kp,
                       'ki': ki,
                       'kd': kd,
                       'max_pwm': max_pwm,
                       'min_pwm': min_pwm,
                       'left_motor_cw': left_motor_cw,
                       'right_motor_cw': right_motor_cw,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num': find_points_num,
                       'b_tsp': b_tsp,
                       'home_debug': home_debug,
                       'obstacle_avoid_type': obstacle_avoid_type,
                       'path_plan_type': path_plan_type,
                       'calibration_compass': calibration_compass
                       },
                      hf)
    if b_height_default:
        with open(height_setting_default_path, 'w') as hdf:
            json.dump({'kp': kp,
                       'ki': ki,
                       'kd': kd,
                       'max_pwm': max_pwm,
                       'min_pwm': min_pwm,
                       'left_motor_cw': left_motor_cw,
                       'right_motor_cw': right_motor_cw,
                       'stop_pwm': stop_pwm,
                       'network_backhome': network_backhome,
                       'energy_backhome': energy_backhome,
                       'find_points_num': find_points_num,
                       'b_tsp': b_tsp,
                       'home_debug': home_debug,
                       'obstacle_avoid_type': obstacle_avoid_type,
                       'path_plan_type': path_plan_type,
                       'calibration_compass': calibration_compass
                       },
                      hdf)


# 树莓派GPIO端口相关设置 均使用BCM编码端口
# 水下摄像头云台水平和俯仰
# 激光雷达
b_laser = 0
laser_tx = 13
laser_rx = 19
laser_baud = 115200
laser_hz = 40
# 电机信号输出控制口
left_pwm_pin = 6  # 左侧
right_pwm_pin = 5  # 右侧电机
# 软串口罗盘
b_pin_compass = 1
pin_compass_baud = 9600
pin_compass_tx = 27
pin_compass_rx = 22
# 软串口gps
b_pin_gps = 1
pin_gps_baud = 9600
pin_gps_tx = 23
pin_gps_rx = 24
# usv 串口遥控器
b_lora_remote_control = 1
lora_tx = 25
lora_rx = 8
lora_baud = 9600
b_lora_com = 0  # lora是否使用TTL转串口模块
# 单片机串口
b_pin_stc = 1
stc_tx = 3
stc_rx = 4
remote_control_stc_baud = 115200
# 是否通用2.4g遥控器
b_use_remote_control = 0
channel_1_pin = 5  # 水平是1通道
channel_3_pin = 6  # 垂直是2通道
channel_remote_pin = 11  # 开启遥控器输入pin口
# 毫米波雷达 millimeter wave radar
b_millimeter_wave = 1
field_of_view = 120  # 视场角
view_cell = 5   # 量化角度单元格
ceil_max = 3  # 可以通过扇区阈值
millimeter_wave_tx = 16
millimeter_wave_rx = 20
millimeter_wave_baud = 115200
millimeter_wave_hz = 40
# 声呐
b_sonar = 0
sonar_rx = 16  # RX
sonar_tx = 20  # TX
sonar_baud = 9600
sonar_steer = 21  # 声呐舵机

# 抽水
b_draw = 1  # 是否有抽水泵
b_control_deep = 1  # 是否可调深度
draw_steer = 13  # 舵机接口

# 排水
b_drain = 1  # 是否有排水泵

min_deep_steer_pwm = 800  # 最下面
max_deep_steer_pwm = 2400  # 最上面


class WaterType(enum.Enum):
    wt = 0
    EC = 1
    pH = 2
    DO = 3
    TD = 4
    NH3_NH4 = 5


draw_deep = 0.5  # 抽水深度
draw_capacity = 1000  # 需要抽水容量
max_draw_capacity = 5000  # 单个瓶子最大抽水容量
draw_speed = 2000  # 抽水速度 毫升/分钟
number_of_bottles = 4  # 总共包含抽水瓶数
max_draw_time = int(60 * max_draw_capacity / draw_speed)

# 障碍物点
obstacle_points = [[114.523433, 30.506193],
                   [114.523519, 30.506526],
                   [114.523943, 30.506466],
                   [114.524066, 30.506517],
                   [114.524254, 30.506378],
                   [114.523696, 30.506378],
                   [114.5239, 30.506369],
                   [114.524055, 30.506378],
                   [114.524334, 30.506304]
                   ]

if __name__ == '__main__':
    write_setting(True, True, True, True)
