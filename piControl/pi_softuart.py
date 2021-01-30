import os
import time
import pigpio
import threading
import os
import sys
import binascii

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
import config

class PiSoftuart(object):
    def __init__(self, pi, rx_pin, tx_pin, baud):
        self._rx_pin = rx_pin
        self._tx_pin = tx_pin
        self._pi = pi
        self._pi.set_mode(self._rx_pin, pigpio.INPUT)
        self._pi.set_mode(self._tx_pin, pigpio.OUTPUT)
        self.distance = 0
        # ATTR
        self._thread_ts = 0.1
        self.flushInput()

    def flushInput(self):
        pigpio.exceptions = False  # fatal exceptions off (so that closing an unopened gpio doesn't error)
        self._pi.bb_serial_read_close(self._rx_pin)
        pigpio.exceptions = True
        self._pi.bb_serial_read_open(self._rx_pin, config.ultrasonic_baud, 8)  # open a gpio to bit bang read, 1 byte each time.

    def read(self, len_data=None):
        if len_data is None:
            len_data = 4
            try:
                count, data = self._pi.bb_serial_read(self._rx_pin)

                print(time.time(), 'count',count,'data',data)
                if count==len_data:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    distance = int(str_data[2:-2], 16) / 1000
                    # print(time.time(),'distance',distance)
                    # 太近进入了盲区
                    if distance <= 0.30:
                        self.distance = -1
                    else:
                        self.distance = distance
                    return distance
                elif count>len_data:
                    str_data = str(binascii.b2a_hex(data))[2:-1]
                    print('str_data',str_data)
                    print(r'str_data.split',str_data.split('ff')[0][:4])
                    print(r'str_data.split',int(str_data.split('ff')[0][:4], 16))
                    distance = int(str_data.split('ff')[0][:4], 16) / 1000
                    return distance
                time.sleep(self._thread_ts)
            except Exception as e:
                print({'error': e})
                return None

    def write_data(self, msg):
        self._pi.wave_clear()
        self._pi.wave_add_serial(self._tx_pin, config.baud, msg)
        data = self._pi.wave_create()
        self._pi.wave_send_once(data)
        if self._pi.wave_tx_busy():
            pass
        self._pi.wave_delete(data)

    def set_thread_ts(self, thread_ts):
        self._thread_ts = thread_ts

    def get_thread_ts(self):
        return self._thread_ts


if __name__ == '__main__':
    pi = pigpio.pi()
    left_distance_obj = PiSoftuart(pi=pi, rx_pin=config.left_rx, tx_pin=config.left_tx, baud=config.ultrasonic_baud)
    right_distance_obj = PiSoftuart(pi=pi, rx_pin=config.right_rx, tx_pin=config.right_tx, baud=config.ultrasonic_baud)
    while True:
        l_distance = left_distance_obj.read()
        r_distance = right_distance_obj.read()
        if l_distance is not  None:
            print('l_distance',l_distance)
        if r_distance is not None:
            print('r_distance',r_distance)
    # while True:
        # if thread_left_distance.is_alive():
        #     print(time.time(),'softuart_obj', softuart_obj.left_distance)
        #     time.sleep(0.2)
        # else:
        #     print(thread_left_distance.is_alive())
        #     time.sleep(1)
        #     # recvbuf = bytearray(softuart_obj.read(7))
        #     # b1 = int(recvbuf[3])
        #     # b0 = int(recvbuf[4])
        #     # result = (b1 << 8) | b0
        #     # print(result / 10.0)
        #     # time.sleep(.05)

