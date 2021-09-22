import time


def dump_energy_cal(adc):
    """
    输入ADC采集电压返回剩余电量
    电量与电压对应关系  各个阶段之内用线性函数计算  下表为实验数据
    电量     电压      6S电池     ADC采集数值
    100%----4.20V     25.2      3900
    90%-----4.06V     24.36     3755
    80%-----3.98V     23.88     3662
    70%-----3.92V     23.52     3561
    60%-----3.87V     23.22     3536
    50%-----3.82V     22.92     3535
    40%-----3.79V▲    22.74     3520
    30%-----3.77V     22.62     3432
    20%-----3.74V     22.44     3461
    0%-----3.7V       22.2      3318
    """
    adc_list = [3900, 3755, 3662, 3561, 3536, 3520, 3432, 3318]
    cap_list = [100, 90, 80, 70, 60, 40, 20, 1]
    # for index, adc_item in enumerate(adc_list):
    #     if index == 0:
    #         if adc > adc_item:
    #             return_cap = cap_list[index]
    #             break
    #     elif index == (len(adc_list) - 1):
    #         if adc < adc_item:
    #             return_cap = cap_list[index]
    #             break
    #     else:
    #         if adc_list[index + 1] < adc < adc_list[index]:
    #             return_cap = cap_list[index + 1] + (adc - adc_list[index + 1]) / (
    #                         adc_list[index] - adc_list[index + 1])
    if adc_list[0] <= adc:
        return_cap = cap_list[0]
    elif adc_list[1] <= adc < adc_list[0]:
        return_cap = cap_list[1] + (adc - adc_list[1]) * (cap_list[0] - cap_list[1]) / (adc_list[0] - adc_list[1])
    elif adc_list[2] <= adc < adc_list[1]:
        return_cap = cap_list[2] + (adc - adc_list[2]) * (cap_list[1] - cap_list[2]) / (adc_list[1] - adc_list[2])
    elif adc_list[3] <= adc < adc_list[2]:
        return_cap = cap_list[3] + (adc - adc_list[3]) * (cap_list[2] - cap_list[3]) / (adc_list[2] - adc_list[3])
    elif adc_list[4] <= adc < adc_list[3]:
        return_cap = cap_list[4] + (adc - adc_list[4]) * (cap_list[3] - cap_list[4]) / (adc_list[3] - adc_list[4])
    elif adc_list[5] <= adc < adc_list[4]:
        return_cap = cap_list[5] + (adc - adc_list[5]) * (cap_list[4] - cap_list[5]) / (adc_list[4] - adc_list[5])
    elif adc_list[6] <= adc < adc_list[5]:
        return_cap = cap_list[6] + (adc - adc_list[6]) * (cap_list[5] - cap_list[6]) / (adc_list[5] - adc_list[6])
    elif adc_list[7] <= adc < adc_list[6]:
        return_cap = cap_list[7] + (adc - adc_list[7]) * (cap_list[6] - cap_list[7]) / (adc_list[6] - adc_list[7])
    else:
        return_cap = 1  # 小于最小电量置为1
    return int(return_cap)


from utils import log

log_obj = log.LogHandler('test.log')


def log_func(log_obj, msg, level=1):
    """
    定时记录log
    @return:
    """
    log_obj(msg)


# while True:
#     # log_obj.info({'time': time.time()})
#     log_func(log_obj.info,{'time': time.time()})
#     time.sleep(1)
import numpy as np

a = np.array([[1,0],[3,4]])
b = np.array([])


if isinstance(a,np.ndarray):
    print(a.all())
    print(a.any())

