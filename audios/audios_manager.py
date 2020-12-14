"""
声音播放管理
gps : GPS接收不到信号
network：无法连接服务器
register： 船还没有注册
battery：电池电量过低
"""

import os
import sys
from playsound import playsound
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config


def play_audio(audio_name=None,b_backend=False):
    """

    :param audio_name: 播放音乐名称
    :param b_backend: 是否后台播放
    :return:
    """
    base_dir = os.path.join(config.root_path, 'audios')
    if audio_name is None:
        path_list=[os.path.join(base_dir,i)for i in ['chunjie.mp3','huan.mp3','gps.mp3','network.mp3','register.mp3','battery.mp3']]
        # 开机音乐
        # path_list=[os.path.join(base_dir,i)for i in ['huan.mp3']]
    else:
        path_list=os.path.join(base_dir, audio_name)
    for i in path_list:
        try:
            playsound(i)
        except Exception as e:
            if b_backend:
                os.system('mpg321 %s &' % i)
            else:
                os.system('mpg321 %s' % i)


if __name__ == '__main__':
    play_audio()