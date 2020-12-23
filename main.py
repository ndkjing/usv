"""
入口函数
"""
import threading
import config
from utils import log
from dataGetSend.data_manager import DataManager
from audios import audios_manager
import sys
import os

sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'baiduMap'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'dataGetSend'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'pathPlanning'))


def main():
    if config.b_play_audio:
        audios_manager.play_audio(0)
    logger = log.LogHandler('main_log')

    # 数据处理对象
    data_manager_obj = DataManager()

    # 查询改船是否注册 若未注册直接退出
    try:
        binding_data = data_manager_obj.send(
            method='http', data="", url=config.http_binding, http_type='GET')
        if int(binding_data['flag']) != 1:
            if config.b_play_audio:
                audios_manager.play_audio('register.mp3')
            # TODO 未注册暂时跳过
            logger.error({'binding status': binding_data['flag']})
        logger.info({'binding status': binding_data['flag']})
    except Exception as e:
        logger.error({'binding_data error': e})

    # 启动串口数据收发和mqtt数据收发
    get_com_data_thread = threading.Thread(
        target=data_manager_obj.get_com_data)
    send_mqtt_data_thread = threading.Thread(
        target=data_manager_obj.send_mqtt_data)
    send_com_data_thread = threading.Thread(
        target=data_manager_obj.send_com_data)
    check_status_thread = threading.Thread(
        target=data_manager_obj.check_status)

    get_com_data_thread.setDaemon(True)
    send_mqtt_data_thread.setDaemon(True)
    send_com_data_thread.setDaemon(True)
    check_status_thread.setDaemon(True)

    get_com_data_thread.start()
    send_mqtt_data_thread.start()
    send_com_data_thread.start()
    check_status_thread.start()

    get_com_data_thread.join()
    send_mqtt_data_thread.join()
    send_com_data_thread.join()
    check_status_thread.join()

    #
    while True:
        pass


if __name__ == '__main__':
    main()
