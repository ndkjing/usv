import os
ship_code_list = [
    '3c50f4c3-a9c1-4872-9f18-883af014380a',
    '3c50f4c3-a9c1-4872-9f18-883af014380b',
    '3c50f4c3-a9c1-4872-9f18-883af014380c',
]
import config
# if config.current_platform=='w_j':
#     ship_code_list = ['3c50f4c3-a9c1-4872-9f18-883af014380c']
root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pool_name = "梁子湖"
b_use_path_planning = 1
# '3c50f4c3-a9c1-4872-9f18-883af014380b',
# 检测像素间隔
pix_interval=4
# 构建地图单元格大小单位米
cell_size = 1
ship_code_video_dict = {
    '3c50f4c3-a9c1-4872-9f18-883af014380a':'https://open.ys7.com/v3/openlive/F77671789_1_2.m3u8?expire=1656031904&id=329906774170845184&t=edf269f932bc49f1bd29178401a3285d5e53fd311953fda4a00fd402ffcf0fbb&ev=100',
    '3c50f4c3-a9c1-4872-9f18-883af014380b':'https://open.ys7.com/v3/openlive/F77671789_1_2.m3u8?expire=1656031904&id=329906774170845184&t=edf269f932bc49f1bd29178401a3285d5e53fd311953fda4a00fd402ffcf0fbb&ev=100',
    '3c50f4c3-a9c1-4872-9f18-883af014380c':'https://open.ys7.com/v3/openlive/D50551834_1_2.m3u8?expire=1656031846&id=329906531464740864&t=f46ffbe870ee3cb6e078d333ed517132cad8752f8162868c6811e27ac9ec6a21&ev=100'
}

