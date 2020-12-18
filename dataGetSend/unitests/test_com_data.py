import random
from utils import log
from dataGetSend.server_data import ServerData
from dataGetSend.com_data import SerialData

import time
import threading

logger = log.LogHandler('test')
import config
from dataGetSend import data_define
from dataGetSend.data_define import DataDefine
from dataGetSend.server_data import MqttSendGet

# obj = ServerData()
# logger = log.LogHandler('server_data_test')
mqtt_obj = MqttSendGet(logger)
data_define_obj = DataDefine()
# 启动后自动订阅话题
for topic, qos in data_define_obj.topics:
    logger.info(topic + '    ' + str(qos))
    mqtt_obj.subscribe_topic(topic=topic, qos=qos)
# http发送检测数据给服务器
# while True:
#     mqtt_obj.publish_topic(topic='status_data_%s' % (data_define.ship_code), data=data_define.status_data(), qos=1)
#     mqtt_obj.publish_topic(topic='detect_data_%s' % (data_define.ship_code), data=data_define.detect_data(), qos=1)
#     time.sleep(config.pi2mqtt_interval)
# Engine1 = Communication("com12", 115200, 0.5)
# if (Ret):
#     Engine1.Recive_data(0)

serial_obj = SerialData('com8', 115200, timeout=1 / config.com2pi_interval, logger=logger)


def get_com_data():
    while True:
        com_data_read = serial_obj.read_Line()
        ## 解析串口发送过来的数据
        # 经纬度
        # lng_lat = com_data_read
        # # 左右侧的超声波检测距离
        # l_distance, r_distance = com_data_read
        # ## 水质数据
        # # 水温
        # water_temperature = com_data_read
        # # TDO
        # water_td = com_data_read


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
        # data_send_com = 'A%sZ' % str(ship_move_direction[i])
        # 监听mqtt控制
        data_send_com = 'A%sZ' % str(mqtt_obj.move_direction)
        serial_obj.send_data(data_send_com.encode())
        # 打印传输给单片机的控制数据
        logger.info('move_direction: ' + data_send_com)
        logger.info('i: ' + str(i))
        count += 1
        time.sleep(1 / config.pi2com_interval)


def read():
    while 1:
        read_data = serial_obj.read_Line()
        print('读取', read_data, str(read_data)[2:-5])


def send():
    while 1:
        # data_send = 'a123z'.encode('ascii')
        data_send = b'A90Z'
        print('发送', type(data_send), data_send)
        serial_obj.send_data(data_send)
        time.sleep(1)


t1 = threading.Thread(target=get_com_data)
t2 = threading.Thread(target=send_com_data)
t1.start()
t2.start()
t1.join()
t2.join()