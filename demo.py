import numpy as np
import json


def gen_data():
    minx = 1
    maxx = 100
    numsx = 100

    def func(x):
        if x < (minx + maxx) / 2:
            return abs((x)) ** 0.2
        else:
            return abs((100 - x)) ** 0.21

    x = np.linspace(minx, maxx, numsx)
    x_list = x.tolist()
    dx = (maxx - minx) / numsx
    y = [func(i) for i in x_list]
    area = np.sum(np.asarray(y) * dx)
    y_list = [round(i, 2) for i in y]
    y_list[-1]=1.5
    y_list[-2]=1.5
    y_list[-3]=1.5
    y_list[-4]=1.5
    y_list[-5]=1.5
    y_list[-6]=1.5
    y_list[-7]=1.5
    sorted_id = sorted(range(len(y_list)), key=lambda k: y_list[k], reverse=False)
    print('元素索引序列：', sorted_id)
    sorted_id2 = sorted(range(len(sorted_id)), key=lambda k: sorted_id[k], reverse=False)
    print('元素索引序列2：', sorted_id2)
    x_use = []
    # 声明字典
    key_value = {}
    # 初始化
    for index, value in enumerate(sorted_id):
        key_value.update({value: y[index]})
    print("按值(value)排序:")
    sort_dict = sorted(key_value.items(), key=lambda kv: (kv[1], kv[0]))
    for i in sort_dict:
        x_use.append(i[0])
    print('面积和:', area)
    print('y :', y)
    sorted_id = sorted(range(len(y)), key=lambda k: y[k], reverse=False)
    print('元素索引序列：', sorted_id)
    print('x_use：', x_use)
    # 数据样式 x x轴数据  y  y轴数据  index排序索引  area 检测面积（平方米）
    print(len(x_list),len(y_list))
    deep_dict = {'x': x_list,
                 'y': y_list,
                 'index': sorted_id2,
                 'area': area}
    with open('deep_data.json', 'w') as f:
        json.dump(deep_dict, f)


def demo():
    y_list = [0.5, 1.0, 2.1, 0.7, 0.1]
    sorted_id = sorted(range(len(y_list)), key=lambda k: y_list[k], reverse=False)
    print('元素索引序列：', sorted_id)
    sorted_id2 = sorted(range(len(sorted_id)), key=lambda k: sorted_id[k], reverse=False)
    print('元素索引序列2：', sorted_id2)
    x_use = []
    # 声明字典
    key_value = {}
    # 初始化
    for index, value in enumerate(sorted_id):
        key_value.update({value: y_list[value]})
    print('key_value',key_value)
    sort_dict = sorted(key_value.items(), key=lambda item:item[1])
    print('sort_dict',sort_dict)
    for i in sort_dict:
        x_use.append(i[0])
    print('y :', y_list)
    sorted_id = sorted(range(len(y_list)), key=lambda k: y_list[k], reverse=False)
    print('x_use：', x_use)


if __name__ == '__main__':
    # demo()
    gen_data()