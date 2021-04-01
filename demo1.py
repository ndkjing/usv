import math
import numpy as np
print('a32' in 'a1232132')
# print(math.sin(math.radians(0.9)))
a = np.zeros((5,5))
b = a[2:4,1:3].tolist()
print(0 in a[2:4,1:3])
print(b)
print(b[0].count(0))
print(0 in b[0])
print(1 in b[0])
print(b.count(1))
print(1 in a[2:4,1:3])