a = [1,1,1]
import time
print(time.time())
import binascii

s = 'A0Z'
str_16 = str(binascii.b2a_hex(s.encode('utf-8')))[2:-1]  # 字符串转16进制
print(str_16)
print(type(str_16))

