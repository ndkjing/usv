import time
import pigpio
import os
import sys
import copy

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
sys.path.append(
    os.path.join(
        root_path,
        'baiduMap'))
sys.path.append(
    os.path.join(
        root_path,
        'dataGetSend'))
sys.path.append(
    os.path.join(
        root_path,
        'utils'))
sys.path.append(
    os.path.join(
        root_path,
        'piControl'))
import config

pi = pigpio.pi()
save = [1, 161, 150, 157, 152, 142, 101]
set = 1
tick_0 = [None, None, None, None, None, None, None]
tick_1 = [None, None, None, None, None, None, None]
temp_read = [[150 for col in range(21)] for row in range(7)]
count = [1, 0, 0, 0, 0, 0, 0]

channel1_pwm = 0
channel3_pwm = 0


def in_callback(argu, gpio, level, tick):
    if level == 0:
        tick_0[argu] = tick
        if tick_1[argu] is not None:
            diff = pigpio.tickDiff(tick_1[argu], tick)
            temp_read[argu][count[argu]] = diff
            save[argu] = temp_read[argu][count[argu]]
    else:
        tick_1[argu] = tick


def mycallback(gpio, level, tick):
    # print('level',level,gpio)
    if level == 0:
        tick_0[1] = tick
        if tick_1[1] is not None:
            diff = pigpio.tickDiff(tick_1[1], tick)
            temp_read[1][count[1]] = diff
            save[1] = int(temp_read[1][count[1]])
            if gpio == int(22):
                print('channel1', diff)
            if gpio == int(27):
                print('channel3', diff)
            if gpio == int(17):
                print('channel5', diff)
    else:
        tick_1[1] = tick


def mycallback3(gpio, level, tick):
    print('level', level, gpio)
    if level == 0:
        tick_0[1] = tick
        if tick_1[1] is not None:
            diff = pigpio.tickDiff(tick_1[1], tick)
            temp_read[1][count[1]] = diff
            save[1] = int(temp_read[1][count[1]])
            print('channel3', int(save[1]))
    else:
        tick_1[1] = tick


# def mycallback3(gpio, level, tick):
#     # in_callback(2, gpio, level, tick)
#     # print('level',level)
#     if level == 0:
#         tick_0[3] = tick
#         if tick_1[3] is not None:
#             diff = pigpio.tickDiff(tick_1[3], tick)
#             print('channel[3]', int(diff))
#             temp_read[3][count[3]] = diff
#             save[3] = int(temp_read[3][count[3]])
#     else:
#         tick_1[1] = tick


if __name__ == "__main__":
    # cb1 = pi.callback(config.channel_1_pin, pigpio.EITHER_EDGE, mycallback)
    # cb3 = pi.callback(config.channel_3_pin, pigpio.EITHER_EDGE, mycallback)
    cb1 = pi.callback(22, pigpio.EITHER_EDGE, mycallback)
    cb3 = pi.callback(27, pigpio.EITHER_EDGE, mycallback)
    cb5 = pi.callback(17, pigpio.EITHER_EDGE, mycallback)
    print('cb1', cb1, config.channel_1_pin)
    print('cb3', cb3, config.channel_3_pin)
    print('cb5', cb3, config.channel_3_pin)
    # cb2 = pi.callback(17, pigpio.EITHER_EDGE, mycallback2)
    # cb3 = pi.callback(27, pigpio.EITHER_EDGE, mycallback3)
    # cb4 = pi.callback(22, pigpio.EITHER_EDGE, mycallback4)
    # cb5 = pi.callback(10, pigpio.EITHER_EDGE, mycallback5)
    # cb6 = pi.callback(9, pigpio.EITHER_EDGE, mycallback6)
    time.sleep(1000)
    set = eval(input("If you want to cancel please type 1 and press enter"))
    if set == 1:
        cb1.cancel()
        # cb2.cancel()  # cancel callback
        # cb3.cancel()
        # cb4.cancel()
        # cb5.cancel()
        # cb6.cancel()
    else:
        cb1.cancel()
        # cb2.cancel()  # cancel callback
        # cb3.cancel()
        # cb4.cancel()
        # cb5.cancel()
        # cb6.cancel()
