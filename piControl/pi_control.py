"""
树莓派直接控制电机
"""
import sys
import os

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
import pigpio
import time

class PiControl:
    def __init__(self):
        self.left_pwm = 1500
        self.right_pwm = 1500
        self.pice = 20000
        self.diff = int(20000 / self.pice)
        self.hz = 50

        self.pi = pigpio.pi()

        # gpio脚的编号顺序依照Broadcom number顺序，请自行参照gpio引脚图里面的“BCM编码”，
        self.pi.set_PWM_frequency(config.left_pwm_pin, self.hz)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_frequency(config.right_pwm_pin, self.hz)  # 设定14号引脚产生的pwm波形的频率为50Hz
        self.pi.set_PWM_range(config.left_pwm_pin, self.pice)
        self.pi.set_PWM_range(config.right_pwm_pin, self.pice)
        self.init_motor()

        self.save = [1, 161, 150, 157, 152, 142, 101]
        self.set = 1
        self.tick_0 = [None, None, None, None, None, None, None]
        self.tick_1 = [None, None, None, None, None, None, None]
        self.temp_read = [[150 for col in range(21)] for row in range(7)]
        self.count = [1, 0, 0, 0, 0, 0, 0]

        self.channel1_input_pwm = 0
        self.channel3_input_pwm = 0
        # 当前是否是遥控器控制 True 为遥控器控制 False 为网页控制
        self.b_remote_control = True

        self.cb1 = self.pi.callback(config.channel_1_pin, pigpio.EITHER_EDGE, self.mycallback)
        self.cb2 = self.pi.callback(config.channel_3_pin, pigpio.EITHER_EDGE, self.mycallback)

    def mycallback(self, gpio, level, tick):
        if level == 0:
            if int(gpio) == int(config.channel_1_pin):
                self.tick_0[0] = tick
                if self.tick_1[0] is not None:
                    diff = pigpio.tickDiff(self.tick_1[0], tick)
                    self.channel1_input_pwm = int(diff)
                    self.temp_read[0][self.count[0]] = diff
                    self.save[0] = int(self.temp_read[0][self.count[0]])
            if int(gpio) == int(config.channel_3_pin):
                self.tick_0[1] = tick
                if self.tick_1[1] is not None:
                    diff = pigpio.tickDiff(self.tick_1[1], tick)
                    self.channel3_input_pwm = int(diff)
                    self.temp_read[1][self.count[1]] = diff
                    self.save[1] = int(self.temp_read[1][self.count[1]])
        else:
            if gpio == int(config.channel_1_pin):
                self.tick_1[0] = tick
            if gpio == int(config.channel_3_pin):
                self.tick_1[1] = tick

    def forward(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1850
        if right_pwm is None:
            right_pwm = 1850
        self.set_pwm(left_pwm, right_pwm)

    def backword(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1300
        if right_pwm is None:
            right_pwm = 1300
        self.set_pwm(left_pwm, right_pwm)

    def left(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1300
        if right_pwm is None:
            right_pwm = 1700
        self.set_pwm(left_pwm, right_pwm)

    def right(self, left_pwm=None, right_pwm=None):
        if left_pwm is None:
            left_pwm = 1700
        if right_pwm is None:
            right_pwm = 1300
        self.set_pwm(left_pwm, right_pwm)

    def stop(self):
        self.set_pwm(1500, 1500)

    def init_motor(self):
        self.set_pwm(1500, 1500)
        time.sleep(config.motor_init_time)

    def set_pwm(self, left_pwm, right_pwm):
        """
        设置pwm波数值
        :param left_pwm:
        :param right_pwm:
        :return:
        """
        # 判断是否大于阈值
        if left_pwm >= config.max_pwm:
            left_pwm = config.max_pwm
        if left_pwm <= config.min_pwm:
            left_pwm = config.min_pwm
        if right_pwm >= config.max_pwm:
            right_pwm = config.max_pwm
        if right_pwm <= config.min_pwm:
            right_pwm = config.min_pwm

        # 如果有反桨叶反转电机pwm值
        if config.left_motor_cw == 1:
            left_pwm = 1500 - (left_pwm - 1500)
        if config.right_motor_cw == 1:
            right_pwm = 1500 - (right_pwm - 1500)

        left_pwm = int(left_pwm / (20000 / self.pice) / (50 / self.hz))
        right_pwm = int(right_pwm / (20000 / self.pice) / (50 / self.hz))
        sleep_time = 0.001
        delta_time = 0.0001 / 500.0
        while abs(self.left_pwm - left_pwm) != 0 or abs(self.right_pwm != right_pwm) != 0:
            if abs(left_pwm - self.left_pwm) == 0:
                pass
            else:
                self.left_pwm = self.left_pwm + (left_pwm - self.left_pwm) // abs(left_pwm - self.left_pwm) * 1
            if abs(right_pwm - self.right_pwm) == 0:
                pass
            else:
                self.right_pwm = self.right_pwm + (right_pwm - self.right_pwm) // abs(right_pwm - self.right_pwm) * 1
            self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
            self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
            time.sleep(sleep_time)
            sleep_time = sleep_time + delta_time
        # self.pi.set_PWM_dutycycle(config.left_pwm_pin, self.left_pwm)  # 1000=2000*50%
        # self.pi.set_PWM_dutycycle(config.right_pwm_pin, self.right_pwm)  # 1000=2000*50%
        # 不支持输出获取pwm状态，以后再调试
        # print('left_pwm:',self.left_pwm,self.pi.get_PWM_dutycycle(config.left_pwm_pin),'right_pwm:',self.right_pwm,self.pi.get_PWM_dutycycle(config.right_pwm_pin))
        print(time.time(), 'left_pwm:', self.left_pwm, 'right_pwm:', self.right_pwm)


if __name__ == '__main__':
    pi_obj = PiControl()
