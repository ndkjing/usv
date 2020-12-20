"""
串口数据收发
"""
import serial
import binascii
import time
import copy
from threading import Thread

from utils import log

logger = log.LogHandler('test_com')

class SerialData:
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
    def Print_Used_Com():
        port_list = list(serial.tools.list_ports.comports())
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
        # self.logger.info({'单片机读取数据':data_read})
        # 通过
        if str(data_read).count(',') > 2:

            # self.logger.info({'单片机读取数据处理后':str(data_read)[2:-5]})
            return str(data_read)[2:-5]
        else:
            return None
    # 发数据

    def send_data(self, data):
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

    serial_obj = SerialData(
        'com8',
        115200,
        timeout=1 /
        config.com2pi_interval,
        logger=logger)

    def get_com_data():
        while True:
            com_data_read = serial_obj.readline()
            # 解析串口发送过来的数据
            if com_data_read is None:
                continue
            com_data_list = com_data_read.split(',')
            # 角度，TDS，温度，经度，纬度，距离1，距离2
            # 当前朝向
            ship_current_direction = com_data_list[0]
            # 经纬度
            current_lng_lat = [
                float(
                    com_data_list[3]), float(
                    com_data_list[4])]
            # 左右侧的超声波检测距离
            l_distance, r_distance = com_data_list[5], com_data_list[6]
            # 水质数据
            # 水温
            water_temperature = com_data_list[2]
            # TDO
            TD = com_data_list[1]
            print('TD', TD)

    # 读取函数会阻塞 必须使用线程
    def send_com_data():
        ship_move_direction = [0, 90, 180, 270, 360]
        i = 0
        count = 0
        # 切换秒数
        change_s = 5
        change_count = config.pi2com_interval * change_s
        while True:
            if count < change_count:
                i = 0
            elif count >= change_count and count < change_count * 2:
                i = 1
            elif count >= 2 * change_count and count < 3 * change_count:
                i = 2
            elif count >= 3 * change_count and count < 4 * change_count:
                i = 3
            else:
                count = 0
            # 间隔指定秒数控制
            data_send_com = 'A%sZ' % str(ship_move_direction[i])
            # 监听mqtt控制
            # data_send_com = 'A%sZ' % str(mqtt_obj.move_direction)
            serial_obj.send_data(data_send_com)
            # 打印传输给单片机的控制数据
            logger.info('move_direction: ' + data_send_com)
            logger.info('i: ' + str(i))
            count += 1
            time.sleep(1 / config.pi2com_interval)

    t1 = Thread(target=get_com_data)
    t2 = Thread(target=send_com_data)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    # print(str(obj.Read_Line())[2:-5])
