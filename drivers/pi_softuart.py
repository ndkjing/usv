import time
import pigpio
import os
import sys
import binascii
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
        if len_data is None:
            len_data = 4
            try:
                self.write_data(send_data)
                time.sleep(self._thread_ts)
                count, data = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), 'count', count, 'data', data)
                if count > len_data:
                    str_data = data.decode('utf-8')[2:-1]
                    theta = float(str_data)
                    return 360 - theta
            except Exception as e:
                print({'error read_compass': e})
                return None

    def read_weite_compass(self, send_data=None, len_data=None, debug=False):
        if len_data is None:
            len_data = 4
            try:
                if send_data:
                    print('send_data', send_data)
                    self.write_data(send_data)
                time.sleep(self._thread_ts)
                time.sleep(0.1)
                count, data1 = self._pi.bb_serial_read(self._rx_pin)
                time.sleep(0.1)
                count, data2 = self._pi.bb_serial_read(self._rx_pin)
                time.sleep(0.1)
                count, data3 = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print('send_data', send_data)
                    print('self._rx_pin', self._rx_pin, self.baud)
                    print(time.time(), 'count', count, 'data', data1, 'data2', data2, 'data3', data3)
                if count > len_data:
                    str_data = data1.decode('utf-8')[2:-1]
                    theta = float(str_data)
                    return 360 - theta
                # time.sleep(self._thread_ts)
            except Exception as e:
                print({'error read_compass': e})
                return None

    def read_gps(self, len_data=None, debug=False):
        if len_data is None:
            len_data = 4
            try:
                count, data = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), 'count', count, 'data', data)
                if count > len_data:
                    str_data = data.decode('utf-8', errors='ignore')
                    for i in str_data.split('$'):
                        i = i.strip()
                        if i.startswith('GPGGA') or i.startswith('$GPGGA') or i.startswith('GNGGA') or i.startswith(
                                '$GNGGA'):
                            gps_data = i
                            data_list = gps_data.split(',')
                            if len(data_list) < 8:
                                continue
                            if data_list[2] and data_list[4]:
                                lng, lat = round(float(data_list[4][:3]) +
                                                 float(data_list[4][3:]) /
                                                 60, 6), round(float(data_list[2][:2]) +
                                                               float(data_list[2][2:]) /
                                                               60, 6)
                                if lng < 1 or lat < 1:
                                    pass
                                else:
                                    lng_lat_error = float(data_list[8])
                                    return [lng, lat, lng_lat_error]
                time.sleep(self._thread_ts * 10)
            except Exception as e:
                print({'error read_gps': e})
                return None

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

    def pin_stc_read(self,debug=False):
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

    def read_remote_control(self, len_data=None, debug=False):
        """
        读取自己做的lora遥控器数据
        :param len_data:限制接受数据最短长度
        :param debug:是否是调试  调试则print打印输出数据
        :return:
        """
        if len_data is None:
            len_data = 4
            try:
                # 发送数据让遥控器接受变为绿灯
                s = 'S9'
                str_16 = str(binascii.b2a_hex(s.encode('utf-8')))[2:-1]  # 字符串转16进制字符串
                # str_16 = '41305a'
                if self.last_send is None:
                    self.write_data(str_16, baud=self.baud, debug=debug)
                    self.last_send = time.time()
                else:
                    if time.time() - self.last_send > 1:
                        self.write_data(str_16, baud=self.baud, debug=debug)
                        self.last_send = time.time()
                count, data = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), 'count', count, 'data', data)
                if count > 40:
                    str_data = str(data, encoding="utf8")
                    data_list = str_data.split(r'\r\nZ')
                    if debug:
                        print(time.time(), 'str_data', str_data, 'data_list', data_list)
                    for item in data_list:
                        temp_data = item.strip()
                        if temp_data[0] == 'A' and temp_data[-1] == 'Z':
                            item_data = temp_data[1:-1]
                            item_data_list = item_data.split(',')
                            if len(item_data_list) == 14:
                                left_row = item_data_list[1]
                                left_col = item_data_list[0]
                                right_row = item_data_list[3]
                                right_col = item_data_list[2]
                                fine_tuning = item_data_list[4]
                                button_10 = item_data_list[9]
                                button_11 = item_data_list[10]
                                button_12 = item_data_list[11]
                                button_13 = item_data_list[12]
                                lever_6 = item_data_list[5]
                                lever_7 = item_data_list[6]
                                lever_8 = item_data_list[7]
                                lever_9 = item_data_list[8]
                                return [left_col,
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

                time.sleep(self._thread_ts)
            except Exception as e:
                time.sleep(self._thread_ts)
                print({'error read_remote_control': e})
                return None

    def write_data(self, msg, baud=None, debug=False):
        if debug:
            print('send data', msg)
        self._pi.wave_clear()
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
        if len_data is None:
            len_data = 4
            try:
                time.sleep(self._thread_ts)
                count, data = self._pi.bb_serial_read(self._rx_pin)
                if debug:
                    print(time.time(), 'count', count, 'data', data)
                if count > len_data:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    split_str = 'aaaa'
                    data_dict = {}
                    for i in str_data.split(split_str):
                        if i.startswith('0c07'):
                            index = int(i[4:6], 16)
                            distance = 0.01 * (int(i[8:10], 16) * 256 + int(i[10:12], 16))
                            angle = 2 * int(i[12:14], 16) - 90
                            speed = 0.05 * (int(i[14:16], 16) * 256 + int(i[16:18], 16)) - 35
                            data_dict.update({index: [distance, angle, speed]})
                            # print('distance:{},angle:{},speed:{}'.format(distance,angle,speed))
                    return data_dict
                else:
                    return None
            except Exception as e:
                # print({'read_millimeter_wave':e})
                time.sleep(self._thread_ts)
                return None

    def send_stc_data(self, send_data):
        try:
            self.pin_stc_write(send_data)
            time.sleep(self._thread_ts)
            return None
            # time.sleep(self._thread_ts)
        except Exception as e:
            print({'error send_stc_data': e})
            return None

    def read_stc_data(self):
        try:
            count, data = self._pi.bb_serial_read(self._rx_pin)
            print(time.time(), 'count', count, 'data', data)
            str_data = data.decode('utf-8', errors='ignore')
            com_data_read = str(data)[2:-5]
            print('com_data_read', com_data_read)
            time.sleep(self._thread_ts * 10)
        except Exception as e:
            print({'error read_gps': e})
            return None
