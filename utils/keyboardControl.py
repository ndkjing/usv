"""
备用键盘上上下左右方向键控制
"""
import time
from pynput.keyboard import Key,Listener
import logging
logging.basicConfig(format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                    level=logging.DEBUG)
from dataGetSend import com_data
import config
# import enum
# class Direction(enum.Enum):
#     UP      = 1
#     DOWN    = 2
#     LEFT    = 3
#     RIGHT   = 4


class Control():
    def __init__(self):
        self.dir_ = str(360) # dir一定要用成员变量，不然没办法在on_press中修改

    def getdir(self):
        self.dir_ = None    # 如果是不是上下左右则返回None
        def on_press(key):
            if key == Key.up:self.dir_ = str(0)
            elif key == Key.down:self.dir_ = str(180)
            elif key == Key.left:self.dir_ = str(90)
            elif key == Key.right:self.dir_ = str(270)
            return False
        listener = Listener(on_press=on_press) # 创建监听器
        listener.start()    # 开始监听，每次获取一个键
        listener.join()     # 加入线程
        listener.stop()     # 结束监听，没有这句也行，直接随函数终止
        return self.dir_




if __name__ == '__main__':
    serial_obj = com_data.SerialData(config.port, config.baud, timeout=1/config.com2pi_interval)
    key_obj = Control()
    i = 0
    while True:
        i+=1
        key_input = key_obj.getdir()
        if key_input:
            logging.info('A%sZ' % (key_input))
            serial_obj.send_data('A%sZ'%(key_input))

