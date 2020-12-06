"""
main
"""
from dataGetSend.data_manager import DataManager
import time


def main():
    obj = DataManager()

    while True:
        # 接收控制数据
        move_direction = obj.server_data_obj.mqtt_send_get_obj.move_direction
        # 向串口发送数据
        obj.send(method='com', data=move_direction)
        # 向mqtt服务器发送数据
        send_mqtt_data="test123"
        obj.server_data_obj.send_server_mqtt_data(topic='test', data=send_mqtt_data, qos=1)
        obj.log.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)


if __name__ == '__main__':
    main()
