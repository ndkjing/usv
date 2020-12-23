"""
声音播放管理
gps : GPS接收不到信号
network：无法连接服务器
register： 船还没有注册
battery：电池电量过低
"""

import config
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
audios_base_dir = os.path.join(config.root_path, 'audios')
path_list = [
            os.path.join(
                audios_base_dir,
                i)for i in [
                'huan.mp3',
                'chunjie.mp3',
                'gps.mp3',
                'network.mp3',
                'register.mp3',
                'battery.mp3']]


def play_audio(audio_index=0, b_backend=False):
    """

    :param audio_name: 播放音乐名称
    :param b_backend: 是否后台播放
    :return:
    """
    if audio_index>=len(path_list):
        audio_index=-1
    try:
        from playsound import playsound
        playsound(path_list[audio_index])
    except Exception as e:
        if b_backend:
            os.system('mpg321 %s &' % path_list[audio_index])
        else:
            os.system('mpg321 %s' % path_list[audio_index])


if __name__ == '__main__':
    play_audio()
