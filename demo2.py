ship_id = 10
a = 'XXLJC4LCGSCSD1DA%03d' % ship_id
b = [[1, 1], [2, 2], [3, 3]]
b.reverse()
print('b', b)
import uuid
import math

print(math.atan2(1, 3) * 180 / 3.14)
print(math.atan2(1, -1) * 180 / 3.14)
print(math.atan2(-1, 1) * 180 / 3.14)
print(math.atan2(-1, -1) * 180 / 3.14)
del_index_list = []
e = [[114.123, 32.3213], [114.123, 32.3213], [114.123, 32.3213]]
for index, item in enumerate(e):
    item[0] = 1111
    print('item', item)
    if index == 1:
        del_index_list.append(index)

for i in del_index_list:
    del e[i]
print('e', e)
p = [0, 0]
