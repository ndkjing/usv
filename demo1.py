import crcmod.predefined
a = 'H  0Z'
print(len(a))
b = bin(int('0',16))[2:]
b="%04s"%b
print(b,type(b),len(b))
print(b[0]=='1')
print(b[1]=='1')
print(b[2]=='1')
print(b[3]=='1')
# print(int(b[0:2]))
# crc8 = crcmod.predefined.Crc('crc-8')
# crc8.update(bytes().fromhex('0'+a[1:4]))
# print(crc8.crcValue)
# print(hex(crc8.crcValue))
# print()
# print(crc8.crcValue==int(a[4:6],16))
print("%04d" % 5)
#方法二

# crc8_func = crcmod.predefined.mkCrcFun('crc-8-maxim')
#
# crc8_func_value = crc8_func(bytes().fromhex('011200'))
#
# print(hex(crc8_func_value))
