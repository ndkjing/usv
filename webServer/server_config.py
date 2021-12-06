import os
import json

ship_code_list = [
    'XXLJC4LCGSCAHSD0DA000',
    'XXLJC4LCGSCSD1DA001',
    'XXLJC4LCGSCSD1DA002',
    'XXLJC4LCGSCSD1DA003',
    'XXLJC4LCGSCSD1DA004',
]
root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pool_name = "梁子湖"
b_use_path_planning = 1
# 检测像素间隔
pix_interval = 4
# 构建地图单元格大小单位米
cell_size = 1
ship_code_video_dict = {
    'XXLJC4LCGSCAHSD0DA000': 'C99929838',
    'XXLJC4LCGSCSD1DA001': 'C99929854',
    'XXLJC4LCGSCSD1DA002': 'C99929528',
    'XXLJC4LCGSCSD1DA003': 'C99929528',
    'XXLJC4LCGSCSD1DA004': 'C99929838',
}

save_map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mapsData')
print('save_map_dir', save_map_dir)
if not os.path.exists(save_map_dir):
    os.mkdir(save_map_dir)
save_token_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'save_token.json')
server_base_setting_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'server_base_setting.json')
server_base_default_setting_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'server_base_setting.json')
# 路径搜索保留离湖泊边缘安全路径  单位米
path_search_safe_distance = 15
# 湖泊名称
pool_name = "梁子湖"


def update_base_setting():
    global path_search_safe_distance
    global pool_name
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
    if b_base:
        with open(server_base_setting_path, 'w') as bf:
            json.dump({'secure_distance': path_search_safe_distance,
                       'pool_name': pool_name,
                       },
                      bf)
    if b_base_default:
        with open(server_base_setting_path, 'w') as bdf:
            json.dump({'secure_distance': path_search_safe_distance,
                       'pool_name': pool_name,
                       },
                      bdf)


if __name__ == '__main__':
    write_setting(True, True)