import time

a = '0,0,1,48.0Z\r\nA50,50,47,50,80,01,00,00,00,0,0,0,1,48.0Z\r\n'
b = 'A50,50,47,50,80,01,00'
a_list = b.split('\r\n')
print(a_list)
print([len(i) for i in a_list])
