from collections import deque
a = deque(maxlen=10)
a.append(1)
a.append(2)
a.append(3)
a.append(1)
print(sum(a))
print(a.clear())
print(a)