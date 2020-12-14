import threading
import random
import time
"""

class A:
    def __init__(self):
        self.a=1
        self.b=[1,2,3]
        self.c = {1:'1',2:'2','3':3}
        self.d = {1:'1',2:'2','3':3,4:{1:1,2:2,3:3}}
    def print(self):
        print(self.a)
        print(self.b)
        print(self.c)

class B:
    def __init__(self):
        self.a_obj = A()

    def set(self):
        while True:
            self.a_obj.a = random.randint(1,6)
            self.a_obj.b[1] = random.randint(1,6)
            self.a_obj.c.update({'3':random.randint(1,6)})
            self.a_obj.c.update({2:str(random.randint(1,6))})
            self.a_obj.d[4].update({1:str(random.randint(1,6))})
            time.sleep(2)

    def get(self):
        while True:
            print('a',self.a_obj.a)
            print('b',self.a_obj.b)
            print('c',self.a_obj.c)
            print('d',self.a_obj.d)
            time.sleep(3)

if __name__ == '__main__':
    b_obj = B()
    t_get = threading.Thread(target=b_obj.get)
    t_set = threading.Thread(target=b_obj.set)

    t_get.start()
    t_set.start()

    t_set.join()
    t_get.join()

"""

# import numpy as np
# a = [[1,2],[3.32,4.421]]
# str_a = str(a)
#
# print(str_a,type(str_a))
# array_a = np.array(str_a)
# # print(type(array_a),array_a,array_a.shape,array_a[0])
# new_array =[]
# for i in array_a:
#     new_array.append([i[0]*100,i[1]*100])
#
# print(new_array)
# import re
# s = 'AAA231.31'
#
# print(s.split('AAA'))
# print('dsa,ds,asdsa,wq,3123'.count(','))


