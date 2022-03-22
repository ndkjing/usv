import cv2
import numpy as np

minx=1
maxx=100
numsx=100
def func(x):
    if x<(minx+maxx)/2:
        return abs((x)) ** 0.2
    else:
        return abs((100-x)) ** 0.21


x = np.linspace(minx, maxx, numsx)
dx = (maxx - minx) / numsx
y = [func(i) for i in x]
area = np.sum(np.asarray(y) * dx)
y_list = [round(i, 2) for i in y]
print('面积和:', area)
print('y :', y)
sorted_id = sorted(range(len(y)), key=lambda k: y[k], reverse=False)
print('元素索引序列：', sorted_id)
deep_dict = {'x': x.tolist(),
             'y': y_list,
             'index': sorted_id,
             'area': area}
import json

with open('deep_data.json', 'w') as f:
    json.dump(deep_dict, f)
