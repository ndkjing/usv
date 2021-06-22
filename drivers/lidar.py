"""
第一步 安装包 >pip install hokuyolx
运行下面示例 输出数据如下
(array([2853, 2854, 2853, ...,  147,  143,  139], dtype=uint32), 1623898425874, 0)
array中包含1080个数据0 到1079 对应-135度到135度 值为mm   0 2853对应为 -135度方向障碍物距离为2.853m

示例
###########
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt

DMAX = 10000
laser = HokuyoLX()
timestamp, scan = laser.get_dist()
print(laser.get_filtered_dist())
for timestamp in laser.iter_dist():
    print(timestamp)
##################

"""

