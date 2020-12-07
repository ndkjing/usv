"""
串口数据收发
"""
import serial, time
import binascii
import time

class SerialData:
    def __init__(self, com='com10', bot=38400, timeout=0.5):
        self.com = com
        self.bot = bot
        try:
            self.serial_com = serial.Serial(self.com, self.bot, timeout=timeout)
        except Exception as e:
            print('error',e)
        self.data=None

    def read(self):
        while True:
            row_data = str(binascii.b2a_hex(self.serial_com.read(100)))
            if len(row_data)>3:
                print('time',time.time())
                print('row_data', len(row_data),row_data)
                self.data=row_data

class ComData:
    def __init__(self):
        self.serial_obj = SerialData()

    # 发送数据到单片机
    def send_data(self,data):
        pass

    # 从单片机接收数据
    def get_data(self):
        pass






if __name__ == '__main__':
    obj = SerialData()
    obj.read()

