"""
串口数据收发
"""
import sys
import os

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)

sys.path.append(
    os.path.join(
        root_dir,
        'dataGetSend'))

sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))
import serial
from serial.tools import list_ports
import binascii
import time
import copy
from threading import Thread

from utils import log

logger = log.LogHandler('test_com')

class ComData:
    def __init__(self, com, baud, timeout, logger=None):
        if logger is None:
            import logging
            logging.basicConfig(
                format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s',
                level=logging.DEBUG)
            self.logger = logging
        else:
            self.logger = logger
        self.port = com
        self.baud = baud
        self.timeout = timeout
        try:
            # 打开串口，并得到串口对象
            self.uart = serial.Serial(
                self.port, self.baud, timeout=self.timeout)
            # 判断是否打开成功
            if not (self.uart.is_open):
                self.logger.error('无法打开串口')
        except Exception as e:
            self.logger.error({"串口连接异常：": e})

    # 打印设备基本信息
    def print_info(self):
        info_dict = {
            'name': self.uart.name,  # 设备名字
            'port': self.uart.port,  # 读或者写端口
            'baudrate': self.uart.baudrate,  # 波特率
            'bytesize': self.uart.bytesize,  # 字节大小
            'parity': self.uart.parity,  # 校验位
            'stopbits': self.uart.stopbits,  # 停止位
            'timeout': self.uart.timeout,  # 读超时设置
            'writeTimeout': self.uart.writeTimeout,  # 写超时
            'xonxoff': self.uart.xonxoff,  # 软件流控
            'rtscts': self.uart.rtscts,  # 软件流控
            'dsrdtr': self.uart.dsrdtr,  # 硬件流控
            'interCharTimeout': self.uart.interCharTimeout,  # 字符间隔超时
        }
        self.logger.info(info_dict)
        return info_dict

    # 打开串口
    def open_serial(self):
        self.uart.open()

    # 关闭串口
    def close_Engine(self):
        self.uart.close()
        print(self.uart.is_open)  # 检验串口是否打开

    # 打印可用串口列表
    @staticmethod
    def print_ssed_com():
        port_list = list(list_ports.comports())
        print(port_list)

    # 接收指定大小的数据
    # 从串口读size个字节。如果指定超时，则可能在超时后返回较少的字节；如果没有指定超时，则会一直等到收完指定的字节数。
    def read_size(self, size):
        return self.uart.read(size=size)

    # 接收一行数据
    # 使用readline()时应该注意：打开串口时应该指定超时，否则如果串口没有收到新行，则会一直等待。
    # 如果没有超时，readline会报异常。
    def readline(self):
        data_read = self.uart.readline()
        return data_read
        # if str(data_read).count(',') > 2:
        #     return str(data_read)[2:-5]
        # else:
        #     return data_read

    # 发数据
    def send_data(self, data, b_hex=False):
        if b_hex:
            self.uart.write(bytes.fromhex(data))
        else:
            self.uart.write(data.encode())

    # 更多示例
    # self.main_engine.write(chr(0x06).encode("utf-8"))  # 十六制发送一个数据
    # print(self.main_engine.read().hex())  #  # 十六进制的读取读一个字节
    # print(self.main_engine.read())#读一个字节
    # print(self.main_engine.read(10).decode("gbk"))#读十个字节
    # print(self.main_engine.readline().decode("gbk"))#读一行
    # print(self.main_engine.readlines())#读取多行，返回列表，必须匹配超时（timeout)使用
    # print(self.main_engine.in_waiting)#获取输入缓冲区的剩余字节数
    # print(self.main_engine.out_waiting)#获取输出缓冲区的字节数
    # print(self.main_engine.readall())#读取全部字符。

    # 接收数据
    # 一个整型数据占两个字节
    # 一个字符占一个字节

    def recive_data(self, way):
        # 循环接收数据，此为死循环，可用线程实现
        print("开始接收数据：")
        while True:
            try:
                # 一个字节一个字节的接收
                if self.uart.in_waiting:
                    if (way == 0):
                        for i in range(self.uart.in_waiting):
                            print("接收ascii数据：" + str(self.read_size(1)))
                            data1 = self.read_size(1).hex()  # 转为十六进制
                            data2 = int(
                                data1, 16)  # 转为十进制print("收到数据十六进制："+data1+"  收到数据十进制："+str(data2))
                    if (way == 1):
                        # 整体接收
                        # data =
                        # self.main_engine.read(self.main_engine.in_waiting).decode("utf-8")#方式一
                        data = self.uart.read_all()  # 方式二print("接收ascii数据：", data)
            except Exception as e:
                print("异常报错：", e)


if __name__ == '__main__':
    import config

    b_compass = False
    b_alt = True
    b_com_data = False
    if b_compass:
        com_obj = ComData(config.compass_port,
                                config.compass_baud,
                                timeout=0.7,
                                logger=logger)
        com_obj1 = ComData(config.compass_port1,
                             config.compass_baud1,
                             timeout=0.7,
                             logger=logger)
        while True:
            # 校准罗盘  C0  开始  C1 结束
            key_input = input('input:')
            com_obj.send_data(key_input, b_hex=True)
            com_obj1.send_data(key_input, b_hex=True)
            print('0号罗盘数据:',com_obj.readline(),com_obj.readline(),com_obj.readline(),com_obj.readline())
            print('1号罗盘数据:',com_obj1.readline(),com_obj1.readline(),com_obj1.readline(),com_obj1.readline())

    elif b_com_data:
        serial_obj = ComData(config.stc_port,
                                config.stc_baud,
                                timeout=0.7,
                                logger=logger)
        while True:
            print(serial_obj.readline())
            print(serial_obj.readline())
            print(serial_obj.readline())
            print(serial_obj.readline())
            key_input = input('input:')
            try:
                i = int(key_input)
                if i == 1:
                    serial_obj.send_data('A1Z')
                else:
                    serial_obj.send_data('A0Z')
            except Exception as e:
                print({'error':e})
    elif b_alt:
        serial_obj = ComData('com4',
                             9600,
                             timeout=0.2,
                             logger=logger)
        print(serial_obj)
        while True:
            try:
                data = serial_obj.read_size(4)
                print(data)
                # print(time.time(), 'len(data)', len(data), 'data', data)
                if len(data) == 4:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    distance = int(str_data[2:-2], 16) / 1000
                    print(time.time(),'distance',distance)

                elif len(data) > 4:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    print('str_data', str_data)
                    print(r'str_data.split', str_data.split('ff')[0][:4])
                    print(r'str_data.split', int(str_data.split('ff')[0][:4], 16))
                    distance = int(str_data.split('ff')[0][:4], 16) / 1000
            except Exception as e:
                print({'error': e})
    else:
        serial_obj = ComData('com3',
                                '9600',
                                timeout=0.7,
                                logger=logger)
        while True:
            data = serial_obj.read_size(4)
            # print(time.time(),type(data),data)
            str_data = str(binascii.b2a_hex(data))[2:-1]
            # print(str_data)
            print(int(str_data[2:-2], 16) / 1000)
    # while True:
    #     # serial_obj.send_data('31',b_hex=True)
    #
    #     data = serial_obj.readline()
    #     str_data = data.decode('ascii')
    #     if str_data.startswith('$GNGGA'):
    #         data_list = str_data.split(',')
    #         print(data_list)
    #         lng,lat = float(data_list[4][:3])+float(data_list[4][3:])/60,float(data_list[2][:2])+float(data_list[2][2:])/60
    #         print('经纬度',lng,lat)
    #         print('误差',data_list[8])
    #     # 角度

    # str_data = data.decode('ascii')[:-3]
    # # print('str_data',str_data,type(str_data))
    # if len(str_data)<2:
    #     continue
    # # str_data = str_data.encode('utf8')
    # # print(str_data.split('.'))
    # float_data = float(str_data)
    # print(time.time(),'float_data', float_data,type(float_data))
    # time.sleep(0.1)
    # t1 = Thread(target=get_com_data)
    # t2 = Thread(target=send_com_data)
    # t1.start()
    # t2.start()
    # t1.join()
    # t2.join()
    # print(str(obj.Read_Line())[2:-5])
