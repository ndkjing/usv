"""
树莓派直接控制电机
"""
import sys
import os

root_dir =  os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)

sys.path.append(
    os.path.join(
        root_dir,
        'dataGetSend'))

sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'pathPlanning'))

from utils import log
logger = log.LogHandler('main_log')
import config
# if config.current_platform == 'l':
import pigpio
import time

class PiControl:
    def __init__(self):
        self.left_pwm = 150
        self.right_pwm = 150
        self.pi = pigpio.pi()
        # gpio脚的编号顺序依照Broadcom number顺序，请自行参照gpio引脚图里面的“BCM编码”，
        self.pi.set_PWM_frequency(14, 50)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_frequency(15, 50)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_range(14, 2000)
        self.pi.set_PWM_range(15, 2000)
        self.init_motor()

    def forward(self,left_pwm=None,right_pwm=None):
        if left_pwm is None:
            left_pwm = 1700
        if right_pwm is None:
            right_pwm = 1300
        self.set_pwm(left_pwm,right_pwm)

    def backword(self,left_pwm=None,right_pwm=None):
        if left_pwm is None:
            left_pwm = 1350
        if right_pwm is None:
            right_pwm = 1650
        self.set_pwm(left_pwm, right_pwm)

    def left(self,left_pwm=None,right_pwm=None):
        if left_pwm is None:
            left_pwm = 1350
        if right_pwm is None:
            right_pwm = 1350
        self.set_pwm(left_pwm, right_pwm)

    def right(self,left_pwm=None,right_pwm=None):
        if left_pwm is None:
            left_pwm = 1650
        if right_pwm is None:
            right_pwm = 1650
        self.set_pwm(left_pwm, right_pwm)

    def stop(self):
        self.set_pwm(1500,1500)

    def init_motor(self):
        self.set_pwm(1500,1500)
        time.sleep(5)

    def set_pwm(self,left_pwm,right_pwm):
        """
        设置pwm波数值
        :param left_pwm:
        :param right_pwm:
        :return:
        """
        # 判断是否大于阈值
        if left_pwm>=config.max_pwm:
            left_pwm = config.max_pwm
        if left_pwm<=config.min_pwm:
            left_pwm = config.min_pwm
        if right_pwm >= config.max_pwm:
            right_pwm = config.max_pwm
        if right_pwm <= config.min_pwm:
            right_pwm = config.min_pwm

        left_pwm = int(left_pwm/10)
        right_pwm = int(right_pwm/10)
        sleep_time=0.01
        delta_time = 0.13/50.0
        # while self.left_pwm!=left_pwm or self.right_pwm!=right_pwm:
        while abs(self.left_pwm-left_pwm)>2 or abs(self.right_pwm!=right_pwm)>2:
            self.pi.set_PWM_dutycycle(14, self.left_pwm)  # 1000=2000*50%
            self.pi.set_PWM_dutycycle(15, self.right_pwm)  # 1000=2000*50%
            if abs(left_pwm - self.left_pwm)<=2:
                pass
            else:
                self.left_pwm = self.left_pwm + (left_pwm - self.left_pwm) // abs(left_pwm - self.left_pwm) * 1.1

            if abs(right_pwm - self.right_pwm)<=2:
                pass
            else:
                self.right_pwm = self.right_pwm + (right_pwm - self.right_pwm) // abs(right_pwm - self.right_pwm) * 1.1
            sleep_time = sleep_time+delta_time
            time.sleep(sleep_time)
        self.pi.set_PWM_dutycycle(14, self.left_pwm)  # 1000=2000*50%
        self.pi.set_PWM_dutycycle(15, self.right_pwm)  # 1000=2000*50%
        print('left_pwm:',self.left_pwm,'right_pwm:',self.right_pwm)

if __name__ == '__main__':
    pi_obj = PiControl()

