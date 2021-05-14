from math import *
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import time

img = Image.open("./statics/2.png")
white, black = 255, 0
img = img.point(lambda x: white if x > 200 else black)
img = img.convert('1')
img = np.array(img)
ob = []
obstacle = []
print(len(img), len(img[0]))
for i in range(len(img)):
    temp = np.ones(len(img[0]))
    for j in range(len(img[0])):
        if img[i][j] == True:
            temp[j] = 0
        else:
            temp[j] = 1
            obstacle.append([i, j])
        ob.append(temp)

ob = np.asarray(ob)
obstacle = np.asarray(obstacle)

startpoint = [3, 3]  # 起始点
endpoint = [200, 600]  # 目标点


def Round(n):  # 自定义四舍五入函数
    n = round(n, 2)
    xs = (n - floor(n)) * 10
    if (xs >= 5):
        new = round(n) + 1
    else:
        new = round(n)
    return new


def caculatebeta(s, e):  # 角度计算，一到四象限
    dy = e[1] - s[1]
    dx = e[0] - s[0]
    if dx == 0:
        beta = pi / 2
    else:
        beta = atan(dy / dx)
        if (dx < 0):
            if (dy > 0):
                beta = pi - abs(beta)
            else:
                beta = pi + abs(beta)
        else:
            if (dy < 0):
                beta = 2 * pi - abs(beta)
    return beta


def howmany(c1, c2):  # 扇区数目
    n = 72
    dif = min([abs(c1 - c2), abs(c1 - c2 - n), abs(c1 - c2 + n)])
    return dif
# plt.subplot(2,2,1)

# plt.show()
step = 10  # 步数
f = 5   # 度数单元
dmax = 200  # 激光超声波最大范围
smax = 18  # 扇区最大个数
b = 2.5  # 参数b
a = 1 + b * (dmax ** 2)  # 参数a
C = 15  # cv值  信度值  certainty value
alpha = np.deg2rad(f)  # 度数增量的弧度表示
n = 360 / f  # 划分扇区  72
threshold = 1300  # 可以通过阈值
rsafe = 5  # 机器人自身约束
robot = startpoint  # 自身位置
kb = 90 / f  #一个象限内扇区数量
kt = Round(caculatebeta(robot, endpoint) / alpha)  # 当前到目标方向所属扇区索引
if (kt == 0):
    kt = n
ffff = np.zeros(73)
robottotal = [robot]
plt.figure()
plt.plot(obstacle[:, 0], obstacle[:, 1], '.k')
plt.grid("on")
# plt.hold("on")
plt.plot(startpoint[0], startpoint[1], '.b')
# plt.hold("on")
plt.title("VFH path planning")
plt.plot(endpoint[0], endpoint[1], '.r')
# plt.hold("on")
while (np.linalg.norm(np.array(robot) - np.array(endpoint), 2) != 0):
    if (np.linalg.norm(np.array(robot) - np.array(endpoint), 2) > step * 2):
        i = 0
        mag = np.zeros(int(n + 3))
        his = np.zeros(int(n + 3))
        while (i < len(obstacle)):
            d = np.linalg.norm(obstacle[i, :] - robot)
            if ((d < dmax) and (d > rsafe)):
                beta = caculatebeta(robot, obstacle[i, :])
                rangle = asin(rsafe / d)
                k = int(Round(beta / alpha))  # 所在位置扇区索引
                if (k == 0):
                    k = int(n)
                h = np.zeros(int(n + 3))
                if ((5 * (k) > np.rad2deg(beta) - np.rad2deg(rangle)) and (
                        5 * (k) < np.rad2deg(beta) + np.rad2deg(rangle))):
                    h[k] = 1
                else:
                    h[k] = 0
                m = (C ** 2) * (a - b * (d ** 2))
                mag[k] = mag[k]+m * h[k]
                i = i + 1
            else:
                i = i + 1
        his = mag
        j = 1
        q = 1
        c = np.zeros(int(n))
        while (q <= n):
            if (his[q] < threshold):
                kr = q
                kl = q
                while ((q <= n) and (his[q] < threshold)):
                    kl = q
                    q = q + 1
                # print(kl,kr)
                if (kl - kr > smax):   # 判断是否是宽波谷
                    c[j] = Round(kl - smax / 2)
                    c[j + 1] = Round(kr + smax / 2)
                    j = j + 2
                    if (kt >= kr and kt <= kl):
                        c[j] = kt
                        j = j + 1
                elif (kl - kr > smax / 5):
                    c[j] = Round((kr + kl) / 2.0)
                    j = j + 1
            else:
                q = q + 1
        g = np.zeros(j)
        how = []
        for i in range(1, j):
            g[i] = c[i]
            howtemp = 5 * howmany(g[i], kt) + 2 * howmany(g[i], kb) + 2 * howmany(g[i], kb)
            how.append(howtemp)
        ft = how.index(min(how))
        fk = ft + 1
        kb = g[int(fk)]
        robot[0] = robot[0] + step * cos(kb * alpha)
        robot[1] = robot[1] + step * sin(kb * alpha)
        robottotal.append(robot)
        print(robot[0], robot[1])
        # plt.hold("on")
        plt.plot(robot[0], robot[1], '.b')
        # plt.hold("on")
        plt.show()
        kt = Round(caculatebeta(robot, endpoint) / alpha)
        if (kt == 0):
            kt = n
    else:
        break

robottotal = np.asarray(robottotal)
plt.show()

