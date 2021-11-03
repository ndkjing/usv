import os

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
