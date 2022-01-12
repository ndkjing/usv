import time
import pigpio
import os
import sys
from collections import deque
import binascii
import re
import config

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)

sys.path.append(
    os.path.join(
        root_dir,
        'dataGetSend'))
sys.path.append(
    os.path.join(
        root_dir,
        'utils'))
sys.path.append(
    os.path.join(
        root_dir,
        'piControl'))


class PiSoftuart(object):
    def __init__(self, pi, rx_pin, tx_pin, baud, time_out=0.1):
        self._rx_pin = rx_pin
        self._tx_pin = tx_pin
        self.baud = baud
        self._pi = pi
        self._pi.set_mode(self._rx_pin, pigpio.INPUT)
        self._pi.set_mode(self._tx_pin, pigpio.OUTPUT)
        self.distance = 0
        # ATTR
        self._thread_ts = time_out
        self.flushInput()
        self.last_send = None
        self.last_lora_data = None  # 数据一次没有收完等待下次数据用于拼接
        self.dump_energy_queue = deque(maxlen=25)

    def flushInput(self):
        pigpio.exceptions = False  # fatal exceptions off (so that closing an unopened gpio doesn't error)
        self._pi.bb_serial_read_close(self._rx_pin)
        pigpio.exceptions = True
        # self._pi.bb_serial_read_open(self._rx_pin, config.ultrasonic_baud,
        #                              8)  # open a gpio to bit bang read, 1 byte each time.
        self._pi.bb_serial_read_open(self._rx_pin, self.baud,
                                     8)  # open a gpio to bit bang read, 1 byte each time.

    def read_ultrasonic(self, len_data=None):
        if len_data is None:
            len_data = 4
            try:
                time.sleep(self._thread_ts / 2)
                count, data = self._pi.bb_serial_read(self._rx_pin)
                print(time.time(), 'count', count, 'data', data)
                if count == len_data:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    distance = int(str_data[2:-2], 16) / 1000
                    # print(time.time(),'distance',distance)
                    # 太近进入了盲区 返回 -1
                    if distance <= 0.25:
                        return -1
                    else:
                        return distance
                elif count > len_data:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    # print('str_data', str_data)
                    print(r'str_data.split', str_data.split('ff'))
                    # print(r'str_data.split', int(str_data.split('ff')[0][:4], 16))
                    distance = 0
                    for i in str_data.split('ff'):
                        if i:
                            distance = int(i[:4], 16) / 1000
                    # print(str_data.split('ff')[0][:4])
                    if distance <= 0.25:
                        return -1
                    else:
                        return distance
                time.sleep(self._thread_ts)
            except Exception as e:
                print({'error': e})
                time.sleep(self._thread_ts / 2)
                return None

    def read_compass(self, send_data='31', len_data=None, debug=False):
        theta = None
        if len_data is None:
            len_data = 4
        try:
            self.write_data(send_data)
        except Exception as write_data_e:
            print({'error read_compass write_data_e': write_data_e})
        time.sleep(self._thread_ts)
        count, data = self._pi.bb_serial_read(self._rx_pin)
        if debug:
            print(time.time(), 'count', count, 'data', data)
        try:
            if count > len_data:
                str_data = data.decode('utf-8')[2:-1]
                theta = 360 - float(str_data)
        except Exception as decode_e:
            print({'error read_compass decode': decode_e})
        return theta

    def read_weite_compass(self, send_data=None, len_data=None, debug=False):
        if len_data is None:
            len_data = 20
            try:
                if send_data:
                    send_data = send_data.encode('utf-8')
                    print('send_data', send_data)
                    self.write_data(send_data,b_weite=True)
                    time.sleep(self._thread_ts)
                    count, data = self._pi.bb_serial_read(self._rx_pin)
                    time.sleep(self._thread_ts*2)
                    count, data1 = self._pi.bb_serial_read(self._rx_pin)
                    time.sleep(self._thread_ts*3)
                    count, data2 = self._pi.bb_serial_read(self._rx_pin)
                    if debug:
                        print('cel data######################################',time.time(), count, data,data1,data2)
                time.sleep(self._thread_ts)
                count, data3 = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), count,data3)
                if count > len_data:
                    str_data = str(data3)[15:-5]
                    res = re.findall(r'Y[!aw\d][aw\d]:(.*?)\\', str_data)
                    if len(res) > 0:
                        theta = float(res[0])+180
                    else:
                        theta=None
                    if debug:
                        print(time.time(),'float res', theta)
                        print(time.time(), type(str_data), '############data3    str_data', data3, str_data)
                    return theta
                # time.sleep(self._thread_ts)
            except Exception as e:
                print({'error read_compass': e})
                return None

    def read_gps(self, len_data=None, debug=False):
        """
        读取gps数据返回经纬度 误差  速度
        @param len_data:数据长度
        @param debug:True 答应获取到的数据
        @return: [经度, 纬度, 误差（米）, 速度, 航向, 磁偏角]
        """
        # 测量频率等待时间
        time.sleep(self._thread_ts)
        lng = None
        lat = None
        lng_lat_error = None
        speed = None
        course = None
        magnetic_declination = None
        if len_data is None:
            len_data = 4
        try:
            count, data = self._pi.bb_serial_read(self._rx_pin)
        except Exception as read_error:
            print({'error read_gps read_error': read_error})
            return None
        if debug:
            print(time.time(), 'count', count, 'data', data)
        if count > len_data:
            str_data = data.decode('utf-8', errors='ignore')
            data_list = str_data.split('$')
            if debug:
                print({'data_list': data_list})
            for gps_data in data_list:
                gps_data = gps_data.strip()
                if gps_data.startswith('GPGGA') or gps_data.startswith('GNGGA'):
                    data_list = gps_data.split(',')
                    if len(data_list) < 8:
                        continue
                    if data_list[2] and data_list[4]:
                        try:
                            lng = round(float(data_list[4][:3]) + float(data_list[4][3:]) / 60, 6)
                            lat = round(float(data_list[2][:2]) + float(data_list[2][2:]) / 60, 6)
                            if lng < 1 or lat < 1:  # 太小可能是假数据直接跳过
                                lng = None
                                lat = None
                        except Exception as convert_lng_lat_error:
                            if debug:
                                print({'error read_gps convert_lng_lat_error': convert_lng_lat_error})
                        try:
                            lng_lat_error = float(data_list[8])
                        except Exception as convert_lng_lat_error:
                            if debug:
                                print({'error read_gps convert_lng_lat_error': convert_lng_lat_error})
                if gps_data.startswith('GPRMC'):
                    data_list = gps_data.split(',')
                    if len(data_list) < 8:
                        continue
                    try:
                        speed = round(float(data_list[7]) * 1.852 / 3.6, 2)  # 将速度单位节转换为 m/s
                    except Exception as convert_speed_error:
                        if debug:
                            print({'error read_gps convert_speed_error': convert_speed_error})
                    try:
                        course = float(data_list[8])  # 航向
                    except Exception as convert_course_error:
                        if debug:
                            print({'error read_gps convert_course_error': convert_course_error})
                    try:
                        magnetic_declination = float(data_list[10])  # 磁偏角
                    except Exception as convert_magnetic_declination_error:
                        if debug:
                            print({
                                      'error read_gps convert_magnetic_declination_error': convert_magnetic_declination_error})
            return [lng, lat, lng_lat_error, speed, course, magnetic_declination]

    def read_laser(self, send_data=None):
        try:
            if send_data:
                self.write_data(send_data, baud=115200)
                time.sleep(self._thread_ts * 4)
            count, data = self._pi.bb_serial_read(self._rx_pin)
            # print(time.time(), type(data), count, data)
            if count == 0:
                time.sleep(1 / config.laser_hz)
                return 0
            str_data = str(binascii.b2a_hex(data))[2:-1]
            # print('str_data', str_data, 'len(str_data)', len(str_data))
            for i in str_data.split('aa'):
                if len(i) == 14 and '07' in i:
                    distance = int(i[6:12], 16) / 1000
                    # 超出量程返回None
                    if distance > 40:
                        return 0
                        # print(time.time(), type(data), count, data)
                        # print(str_data)
                    return distance
            time.sleep(1 / config.laser_hz)
        except Exception as e:
            time.sleep(1 / config.laser_hz)
            print({'error read_laser': e})
            return 0

    def read_sonar(self):
        len_data = 10
        try:
            time.sleep(self._thread_ts / 2)
            count, data = self._pi.bb_serial_read(self._rx_pin)
            print(time.time(), 'count', count, 'data', data)
            if count == len_data:
                str_data = str(binascii.b2a_hex(data))[2:-1]
                distance = int(str_data[2:-2], 16) / 1000
                # print(time.time(),'distance',distance)
                # 太近进入了盲区 返回 -1
                if distance <= 0.25:
                    return -1
                else:
                    return distance
            elif count > len_data:
                str_data = str(binascii.b2a_hex(data))[2:-1]
                # print('str_data', str_data)
                print(r'str_data.split', str_data.split('ff'))
                # print(r'str_data.split', int(str_data.split('ff')[0][:4], 16))
                distance = 0
                for i in str_data.split('ff'):
                    if i:
                        distance = int(i[:4], 16) / 1000
                # print(str_data.split('ff')[0][:4])
                if distance <= 0.25:
                    return -1
                else:
                    return distance
            time.sleep(self._thread_ts)
        except Exception as e:
            print({'error': e})
            time.sleep(self._thread_ts / 2)
            return None

    def pin_stc_read(self, debug=False):
        """
        软串口单片机数据读取
        :return:
        """
        count, data = self._pi.bb_serial_read(self._rx_pin)
        if debug:
            print(time.time(), 'count', count, 'data', data)

    def pin_stc_write(self, stc_write_data, debug=False):
        """
        软串口单片机数据发送
        :param stc_write_data:
        :param debug
        :return:
        """
        str_16_stc_write_data = str(binascii.b2a_hex(stc_write_data.encode('utf-8')))[2:-1]  # 字符串转16进制字符串
        self.write_data(str_16_stc_write_data, baud=self.baud, debug=debug)

    @staticmethod
    def split_lora_data(data: str) -> list:
        item_data = data[1:-1]
        item_data_list = item_data.split(',')
        if len(item_data_list) >= 13:
            left_row = int(item_data_list[1])
            left_col = int(item_data_list[0])
            right_row = int(item_data_list[3])
            right_col = int(item_data_list[2])
            fine_tuning = int(item_data_list[4])
            button_10 = int(item_data_list[9])
            button_11 = int(item_data_list[10])
            button_12 = int(item_data_list[11])
            button_13 = int(item_data_list[12])
            lever_6 = int(item_data_list[5])
            lever_7 = int(item_data_list[6])
            lever_8 = int(item_data_list[7])
            lever_9 = int(item_data_list[8])
            return_list = [left_col,
                           left_row,
                           right_col,
                           right_row,
                           fine_tuning,
                           lever_6,
                           lever_7,
                           lever_8,
                           lever_9,
                           button_10,
                           button_11,
                           button_12,
                           button_13,
                           ]
            return return_list
        else:
            return []

    def read_remote_control(self, len_data=None, debug=False):
        """
        读取自己做的lora遥控器数据
        :param len_data:限制接受数据最短长度
        :param debug:是否是调试  调试则print打印输出数据
        :return:
        """
        if len_data is None:
            try:
                time.sleep(self._thread_ts)
                return_list = None
                # 发送数据让遥控器接受变为绿灯
                s = 'S9'
                str_16 = str(binascii.b2a_hex(s.encode('utf-8')))[2:-1]  # 字符串转16进制字符串
                # str_16 = '41305a'
                if self.last_send is None:
                    self.write_data(str_16, baud=self.baud, debug=debug)
                    self.last_send = time.time()
                else:
                    if time.time() - self.last_send > 0.5:
                        self.write_data(str_16, baud=self.baud, debug=debug)
                        self.last_send = time.time()
                count, data = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), 'count', count, 'data', data)
                if count > 40:
                    # 转换数据然后按照换行符分隔
                    str_data = str(data, encoding="utf8")
                    data_list = str_data.split('\r\n')
                    if debug:
                        print(time.time(), 'str_data', str_data, 'data_list', data_list)
                    for item in data_list:
                        temp_data = item.strip()
                        if len(temp_data) < 2:
                            continue
                        # 开头结尾都存在是完整的一帧数据
                        if temp_data[0] == 'A' and temp_data[-1] == 'Z':
                            return_list = PiSoftuart.split_lora_data(temp_data)
                        # 数据不够一次完整的数据查看是否能拼接上次数据
                        # 开头存在，结尾不存在保存为遗留数据
                        elif temp_data[0] == 'A' and temp_data[-1] != 'Z':
                            self.last_lora_data = temp_data
                        # 开头不存在，结尾存在，看是否存在遗留数据可以拼接
                        elif temp_data[0] != 'A' and temp_data[-1] == 'Z':
                            if self.last_lora_data is not None:
                                concate_lora_data = self.last_lora_data + temp_data
                                if concate_lora_data[0] == 'A' and concate_lora_data[-1] == 'Z':
                                    return_list = PiSoftuart.split_lora_data(concate_lora_data)
                elif count > 0:
                    str_data = str(data, encoding="utf8")
                    data_list = str_data.split('\r\n')
                    for item in data_list:
                        temp_data = item.strip()
                        if len(temp_data) < 2:
                            continue
                        if temp_data[0] == 'A' and temp_data[-1] != 'Z':
                            self.last_lora_data = temp_data
                            # 开头不存在，结尾存在，看是否存在遗留数据可以拼接
                        elif temp_data[0] != 'A' and temp_data[-1] == 'Z':
                            if self.last_lora_data is not None:
                                concate_lora_data = self.last_lora_data + temp_data
                                if concate_lora_data[0] == 'A' and concate_lora_data[-1] == 'Z':
                                    print("################condate data")
                                    return_list = PiSoftuart.split_lora_data(concate_lora_data)
                return return_list
            except Exception as e:
                time.sleep(self._thread_ts)
                print({'error read_remote_control': e})
                return None

    def write_data(self, msg, baud=None, debug=False, b_weite=False):
        if debug:
            pass
            # print('send data', msg)
        self._pi.wave_clear()
        if b_weite:
            if baud:
                self._pi.wave_add_serial(self._tx_pin, baud, msg)
            else:
                self._pi.wave_add_serial(self._tx_pin, 9600, msg)
        else:
            if baud:
                self._pi.wave_add_serial(self._tx_pin, baud, bytes.fromhex(msg))
            else:
                self._pi.wave_add_serial(self._tx_pin, 9600, bytes.fromhex(msg))
        data = self._pi.wave_create()
        self._pi.wave_send_once(data)
        if self._pi.wave_tx_busy():
            pass
        self._pi.wave_delete(data)

    def set_thread_ts(self, thread_ts):
        self._thread_ts = thread_ts

    def get_thread_ts(self):
        return self._thread_ts

    def read_millimeter_wave(self, len_data=None, debug=False):
        """

        @param len_data:
        @param debug:
        @return:None或者{索引:[距离，角度，速度]}
        """
        if len_data is None:
            len_data = 4
        time.sleep(self._thread_ts)
        count, data = self._pi.bb_serial_read(self._rx_pin)
        if debug:
            print(time.time(), 'count', count, 'data', data)
        data_dict = {}
        if count > len_data:
            str_data = str(binascii.b2a_hex(data))[2:-1]
            split_str = 'aaaa'
            data_list = str_data.split(split_str)
            if debug:
                print({'str_data:': str_data, 'data_list:': data_list})
            for i in data_list:
                # 正常数据长度
                if len(i) % 12 == 0:
                    # 目标检测数据
                    if i.startswith('0c07'):
                        try:
                            index = int(i[4:6], 16)
                            distance = 0.01 * (int(i[8:10], 16) * 256 + int(i[10:12], 16))
                            angle = 2 * int(i[12:14], 16) - 90
                            speed = 0.05 * (int(i[14:16], 16) * 256 + int(i[16:18], 16)) - 35
                            data_dict.update({index: [distance, angle, speed]})
                            if debug:
                                print('data',i)
                                print('index:{}distance:{},angle:{},speed:{}'.format(index,distance, angle, speed))
                        except Exception as read_millimeter_wave_e:
                            if debug:
                                print({'read_millimeter_wave nomal data': read_millimeter_wave_e})
                    # 目标检测状态数据
                    elif i.startswith('0b07'):
                        try:
                            target_id = int(i[7:8], 16)
                            if debug:
                                print({'target_id': target_id})
                        except Exception as read_millimeter_id_e:
                            if debug:
                                print({'read_millimeter_wave read_millimeter_id_e nomal data': read_millimeter_id_e})
                # 收到字符出错，多帧粘黏在一起了
                elif (len(i) - 24) % 28 == 0:
                    # 计算含有多少帧数据
                    frame_count = int(1 + (len(i) - 24) / 28)
                    for sub_i in range(frame_count):
                        if sub_i == 0:
                            data = i[0:24]
                        else:
                            data = i[sub_i * 28:sub_i * 28 + 24]
                        # 目标检测数据
                        if data.startswith('0c07'):
                            try:
                                index = int(data[4:6], 16)
                                distance = 0.01 * (int(data[8:10], 16) * 256 + int(data[10:12], 16))
                                angle = 2 * int(data[12:14], 16) - 90
                                speed = 0.05 * (int(data[14:16], 16) * 256 + int(data[16:18], 16)) - 35
                                data_dict.update({index: [distance, angle, speed]})
                                if debug:
                                    print('data',data)
                                    print('##################拼接毫米波数据')
                                    print('index:{}distance:{},angle:{},speed:{}'.format(index,distance, angle, speed))
                            except Exception as read_millimeter_wave_e:
                                if debug:
                                    print({'read_millimeter_wave nomal data': read_millimeter_wave_e})
                        # 目标检测状态数据
                        elif data.startswith('0b07'):
                            try:
                                id = int(i[7:8], 16)
                                if debug:
                                    print('data', data)
                                    print('##################拼接毫米波数据')
                                    print({'target_id': id})
                            except Exception as more_data_e:
                                if debug:
                                    print({'read_millimeter_wave more_data_e data': more_data_e})
        return data_dict

    def send_stc_data(self, send_data):
        try:
            self.pin_stc_write(send_data)
            time.sleep(self._thread_ts)
            return None
            # time.sleep(self._thread_ts)
        except Exception as e:
            print({'error send_stc_data': e})
            return None

    def read_stc_data(self, debug=False):
        try:
            # time.sleep(self._thread_ts)
            count, data = self._pi.bb_serial_read(self._rx_pin)
            return_dict = {}
            water_data_list = None
            if debug:
                print('read_stc_data', count, data)
            if count > 4:
                # 转换数据然后按照换行符分隔
                str_data = str(data, encoding="utf8")
                data_list = str_data.split('\r\n')
                if debug:
                    print(time.time(), 'str_data', str_data, 'data_list', data_list)
                for item in data_list:
                    temp_data = item.strip()
                    if len(temp_data) < 2:  # 去除空字符串
                        continue
                    # 开头结尾都存在是完整的一帧数据
                    if temp_data[0] == 'G' and temp_data[-1] == 'Z':
                        int_data = int(temp_data[1:-1])
                        self.dump_energy_queue.append(int_data)
                    elif str_data.startswith('F') and temp_data[-1] == 'Z':
                        water_data_str = str_data[1:-1]
                        water_data_list = [float(i) for i in water_data_str.split(',')]
            if debug:
                if len(self.dump_energy_queue) >= 1:
                    print('dump_energy_queue,average', self.dump_energy_queue,
                          sum(self.dump_energy_queue) / len(self.dump_energy_queue))
                if return_dict:
                    print('return_dict', return_dict)
            if len(self.dump_energy_queue) > 20:
                return_dict.update(
                    {'dump_energy': [round(sum(self.dump_energy_queue) / len(self.dump_energy_queue), 1)]})
            if water_data_list is not None:
                return_dict.update({'water': water_data_list})
            return return_dict
        except Exception as e:
            print({'error read_stc_data': e})
            return {}
