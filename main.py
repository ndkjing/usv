"""
main
"""
from dataGetSend.data_manager import DataManager
import time


def main():
    # 创建管理对象
    obj = DataManager()
    # 订阅话题
    obj.server_data_obj.mqtt_send_get_obj.subscribe_topic('control_data',qos=1)
    while True:
        # 接收控制数据
        move_direction = obj.server_data_obj.mqtt_send_get_obj.move_direction
        # 向串口发送数据
        # obj.send(method='com', data=move_direction)
        # 向mqtt服务器发送数据
        send_mqtt_data="test123"
        send_mqtt_data1 = obj.data_dict
        obj.send(method='mqtt',topic='all_data', data=send_mqtt_data1, qos=1)

        obj.log.info('move_direction: %f' % (float(move_direction)))
        time.sleep(2)


if __name__ == '__main__':
    main()
