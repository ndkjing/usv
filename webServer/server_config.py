import os
import json
"""
服务器相关数据设置
"""
ship_code_list = [
    'XXLJC4LCGSCAHSD0DA000',
    'XXLJC4LCGSCSD1DA001',
    'XXLJC4LCGSCSD1DA002',
    'XXLJC4LCGSCSD1DA003',
    'XXLJC4LCGSCSD1DA004',
    'XXLJC4LCGSCSD1DA005',
    'XXLJC4LCGSCSD1DA006',
    'XXLJC4LCGSCSD1DA007',
    # 'XXLJC4LCGSCSD1DA008',
    # 'XXLJC4LCGSCSD1DA009',
    # 'XXLJC4LCGSCSD1DA010',
    # 'XXLJC4LCGSCSD1DA011',
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
# 注册新的湖泊ID
http_save = 'https://ship.xxlun.com/union/admin/xxl/map/save'
# http_save = 'http://192.168.199.186:8009/union/admin/xxl/map/save'
# 更新湖泊轮廓
http_update_map = 'https://ship.xxlun.com/union/admin/xxl/map/upData'
# http_update_map = 'http://192.168.199.186:8009/union/admin/xxl/map/upData'
# 获取船状态
http_get_ship_status = 'https://ship.xxlun.com/union/admin/xxl/data/state'
# http_get_ship_status = 'http://192.168.8.26:8009/union/admin/xxl/data/state'
# 更新船状态
http_set_ship_status = 'https://ship.xxlun.com/union/admin/xxl/data/upstate'
# http_set_ship_status = 'http://192.168.8.26:8009/union/admin/xxl/data/upstate'


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
        print('server_base_setting_path',server_base_setting_path)
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
    print('server_base_setting_path',server_base_setting_path)
    with open(server_base_setting_path, 'w') as bf:
        json.dump({'secure_distance': path_search_safe_distance,
                   'pool_name': pool_name,
                   },
                  bf)

if __name__ == '__main__':
    write_setting(True, True)