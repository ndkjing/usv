"""
存放船编号
"""
import os
ship_code = 'XXLJC4LCGSCSD1DA006'
video_code = 'C99929838'
ship_code_video_dict = {
    ship_code: video_code,
}
root_path = os.path.dirname(os.path.abspath(__file__))
# 保存视频播放地址
save_token_path = os.path.join(root_path, 'statics', 'save_token.json')
