"""
声音播放管理
gps : GPS接收不到信号
network：无法连接服务器
register： 船还没有注册
battery：电池电量过低
"""
import os,sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config

import sys
import random

# sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
audios_base_dir = os.path.join(config.root_path, 'audios')

"""
0 开机
1 随机音乐库中音乐
2 网络
3 电量提醒
4 漏水
5 gps
6 靠近提醒
7 注册
"""
audio_dict={
    0:'setup.mp3',
    1:['chunjie.mp3','huan.mp3','music1.mp3','music2.mp3'],
    2:'network.mp3',
    3:'battery.mp3',
    4:'',
    5:'gps.mp3',
    6:'',
    7:'register.mp3',
}

def play_audio(audio_index=0, b_backend=False):
    """

    :param audio_name: 类型索引
    :param b_backend: 是否后台播放
    :return:
    """
    try:
        from playsound import playsound
        if audio_index == 1:
            playsound(os.path.join(audios_base_dir,audio_dict[audio_index][random.randint(0,len(audio_dict[1])-1)]))
        else:
            playsound(os.path.join(audios_base_dir,audio_dict[audio_index]))
    except Exception as e:
        print('e',e)
        if b_backend:
            if audio_index == 1:
                pass
                # os.system('mpg321 %s &' % os.path.join(audios_base_dir,audio_dict[audio_index][random.randint(0,len(audio_dict[1])))
            else:
                os.system('mpg321 %s &' % os.path.join(audios_base_dir,audio_dict[audio_index]))
        else:
            if audio_index == 1:
                os.system('mpg321 %s' % os.path.join(audios_base_dir,audio_dict[audio_index][random.randint(0,len(audio_dict[1])-1)]))
            else:
                os.system('mpg321 %s' % os.path.join(audios_base_dir,audio_dict[audio_index]))

if __name__ == '__main__':
    play_audio(1)
