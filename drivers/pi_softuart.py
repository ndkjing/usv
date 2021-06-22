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
                # time.sleep(self._thread_ts)
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
                    print(time.time(), 'count', count, 'data', data1,'data2',data2,'data3',data3)
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

    def write_data(self, msg, baud=None):
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
            self.write_data(send_data, baud=115200)
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
