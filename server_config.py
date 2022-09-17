import enum
import json
import os
import platform

"""
服务器相关数据设置
"""
ship_code_list = [
    # 'XXLJC4LCGSCAHSD0DA000',
    # 'XXLJC4LCGSCSD1DA001',
    # 'XXLJC4LCGSCSD1DA002',
    'XXLJC4LCGSCSD1DA003',
    'XXLJC4LCGSCSD1DA004',
    'XXLJC4LCGSCSD1DA005',
    # 'XXLJC4LCGSCSD1DA006',
    # 'XXLJC4LCGSCSD1DA007',
    # 'XXLJC4LCGSCSD1DA008',
    # 'XXLJC4LCGSCSD1DA009',
    # 'XXLJC4LCGSCSD1DA010',
    # 'XXLJC4LCGSCSD1DA011',
    # 'XXLJC4LCGSCSD1DA012'
]
root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
b_use_path_planning = 1
# 检测像素间隔
pix_interval = 4
# 构建地图单元格大小单位米
cell_size = 1
save_map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mapsData')
setting_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'settingsData')
if not os.path.exists(save_map_dir):
    os.mkdir(save_map_dir)
if not os.path.exists(setting_dir):
    os.mkdir(setting_dir)
save_token_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'save_token.json')
# 路径搜索保留离湖泊边缘安全路径  单位米
path_search_safe_distance = 15
# 湖泊名称
pool_name = "梁子湖"
local_http = False
if local_http:
    http_domin = '192.168.8.26:8008'
else:
    http_domin = 'peri.xxlun.com'
# 注册新的湖泊ID
http_save = 'http://%s/union/map/save' % http_domin
# 更新湖泊轮廓
http_update_map = 'http://%s/union/map/update' % http_domin
# 获取船状态
http_get_ship_status = 'http://%s/union/admin/xxl/data/state' % http_domin
# 更新船状态
http_set_ship_status = 'http://%s/union/device/update' % http_domin
# 获取token
http_get_token = "http://%s/union/device/login" % http_domin


def update_base_setting(ship_code_):
    global path_search_safe_distance
    global pool_name
    server_base_setting_path = os.path.join(setting_dir, 'setting_%s.json' % ship_code_)
    if os.path.exists(server_base_setting_path):
        try:
            with open(server_base_setting_path, 'r') as f:
                base_setting_data = json.load(f)
            # 读取配置
            if base_setting_data.get('secure_distance'):
                try:
                    s_path_search_safe_distance = int(base_setting_data.get('secure_distance'))
                    if s_path_search_safe_distance > 100:
                        s_path_search_safe_distance = 100
                    elif s_path_search_safe_distance < 2:
                        s_path_search_safe_distance = 2
                    path_search_safe_distance = s_path_search_safe_distance
                except Exception as e:
                    print({'error': e})
            if base_setting_data.get('pool_name'):
                try:
                    s_pool_name = base_setting_data.get('pool_name')
                    pool_name = s_pool_name
                except Exception as e:
                    print({'error': e})
        except Exception as e:
            print({'error': e})


# 保存配置到文件中
def write_setting(b_base=False, b_base_default=False):
    for ship_code in ship_code_list:
        server_base_setting_path = os.path.join(setting_dir, 'setting_%s.json' % ship_code)
        server_base_default_setting_path = os.path.join(setting_dir, 'setting_default_%s.json' % ship_code)
        print('server_base_setting_path', server_base_setting_path)
        if b_base:
            with open(server_base_setting_path, 'w') as bf:
                json.dump({'secure_distance': path_search_safe_distance,
                           'pool_name': pool_name,
                           },
                          bf)
        if b_base_default:
            with open(server_base_default_setting_path, 'w') as bdf:
                json.dump({'secure_distance': path_search_safe_distance,
                           'pool_name': pool_name,
                           },
                          bdf)


def write_ship_code_setting(ship_code_):
    server_base_setting_path = os.path.join(setting_dir, 'setting_%s.json' % ship_code_)
    print('server_base_setting_path', server_base_setting_path)
    with open(server_base_setting_path, 'w') as bf:
        json.dump({'secure_distance': path_search_safe_distance,
                   'pool_name': pool_name,
                   },
                  bf)


# 百度地图key
baidu_key = 'wIt2mDCMGWRIi2pioR8GZnfrhSKQHzLY'
# 高德秘钥
gaode_key = '8177df6428097c5e23d3280ffdc5a13a'
# 腾讯地图key
tencent_key = 'PSABZ-URMWP-3ATDK-VBRCR-FBBMF-YHFCE'


class CurrentPlatform(enum.Enum):
    windows = 1
    linux = 2
    pi = 3
    others = 4


sysstr = platform.system()
if sysstr == "Windows":
    current_platform = CurrentPlatform.windows
elif sysstr == "Linux":  # 树莓派上也是Linux
    # 公司Linux电脑名称
    if platform.node() == 'raspberrypi':
        current_platform = CurrentPlatform.pi
    else:
        current_platform = CurrentPlatform.linux
else:
    print("other System tasks")
    current_platform = CurrentPlatform.others

root_path = os.path.dirname(os.path.abspath(__file__))
mqtt_host = '47.97.183.24'
mqtt_port = 1884
ship_code = 'XXLJC4LCGSCSD1DA004'
b_tsp = 0
ship_gaode_lng_lat = [114.524096, 30.506853]  # 九峰水库
home_debug = 0
find_points_num = 5

tcp_server_ip = "0.0.0.0"
tcp_server_port = 5566

if __name__ == '__main__':
    write_setting(True, True)
