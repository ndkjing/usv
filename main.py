"""
入口函数
"""
import threading
import time
import config
from utils import log
from dataGetSend.data_manager import DataManager
from audios import audios_manager
import sys
import os
time.sleep(10)

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

logger = log.LogHandler('main_log')
def main():
    if config.b_play_audio:
        audios_manager.play_audio(0)

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
    if (config.current_platform == "l"):
        get_com_data_thread = threading.Thread(
            target=data_manager_obj.get_com_data)
        send_com_data_thread = threading.Thread(
            target=data_manager_obj.send_com_data)
        compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
        gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)

    else:
        pass
    check_status_thread = threading.Thread(
        target=data_manager_obj.check_status)

    send_mqtt_data_thread = threading.Thread(
        target=data_manager_obj.send_mqtt_data)


    # send_com_heart_thread = threading.Thread(
    #     target=data_manager_obj.send_com_heart_data)

    check_status_thread.setDaemon(True)
    if (config.current_platform == "l"):
        get_com_data_thread.setDaemon(True)
        send_com_data_thread.setDaemon(True)
        compass_thread.setDaemon(True)
        gps_thread.setDaemon(True)
    send_mqtt_data_thread.setDaemon(True)

    # send_com_heart_thread.setDaemon(True)

    check_status_thread.start()
    if (config.current_platform == "l"):
        get_com_data_thread.start()
        send_com_data_thread.start()
        compass_thread.start()
        gps_thread.start()
    send_mqtt_data_thread.start()

    # send_com_heart_thread.start()

    # get_com_data_thread.join()
    # send_mqtt_data_thread.join()
    # send_com_data_thread.join()
    # check_status_thread.join()
    # send_com_heart_thread.join()

    while True:
        #  判断线程是否死亡并重启线程
        if (config.current_platform == "l"):
            if not get_com_data_thread.is_alive():
                if config.home_debug:
                    time.sleep(10)
                    pass
                else:
                    logger.error('restart get_com_data_thread')
                    try:
                        data_manager_obj.com_data_obj.uart.close()
                        data_manager_obj.com_data_obj =data_manager_obj.get_serial_obj()
                    except Exception as e:
                        logger.error({'串口关闭失败':111, 'error': e})
                    get_com_data_thread = threading.Thread(
                        target=data_manager_obj.get_com_data)
                    get_com_data_thread.setDaemon(True)
                    get_com_data_thread.start()
                    time.sleep(10)
                if not send_com_data_thread.is_alive():
                    logger.error('restart send_com_data_thread')
                    send_com_data_thread = threading.Thread(
                        target=data_manager_obj.send_com_data)
                    send_com_data_thread.setDaemon(True)
                    send_com_data_thread.start()
                    time.sleep(10)
            if not compass_thread.is_alive():
                logger.error('restart send_mqtt_data_thread')
                compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
                compass_thread.setDaemon(True)
                compass_thread.start()
                time.sleep(10)

            if not gps_thread.is_alive():
                logger.error('restart send_mqtt_data_thread')
                gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)
                gps_thread.setDaemon(True)
                gps_thread.start()
                time.sleep(10)

        # gps_thread.setDaemon(True)
        if not send_mqtt_data_thread.is_alive():
            logger.error('restart send_mqtt_data_thread')
            send_mqtt_data_thread = threading.Thread(
                target=data_manager_obj.send_mqtt_data)
            send_mqtt_data_thread.setDaemon(True)
            send_mqtt_data_thread.start()
            time.sleep(10)

        elif not check_status_thread.is_alive():
            logger.error('restart check_status_thread')
            check_status_thread = threading.Thread(
                target=data_manager_obj.check_status)
            check_status_thread.setDaemon(True)
            check_status_thread.start()
            time.sleep(10)

        # elif not send_com_heart_thread.is_alive():
        #     logger.error('restart send_com_heart_thread')
        #     send_com_heart_thread = threading.Thread(
        #         target=data_manager_obj.send_com_heart_data)
        #     send_com_heart_thread.setDaemon(True)
        #     send_com_heart_thread.start()
        #     time.sleep(10)
        else:
            # print('get_com_data_thread',get_com_data_thread.is_alive())
            # print('send_mqtt_data_thread',send_mqtt_data_thread.is_alive())
            # print('send_mqtt_data_thread',send_com_data_thread.is_alive())
            # print('send_mqtt_data_thread',check_status_thread.is_alive())
            # print('send_mqtt_data_thread',send_com_heart_thread.is_alive())
            time.sleep(1)

if __name__ == '__main__':
    while True:
        try:
            main()
        except Exception as e:
            time.sleep(5)
            logger.error({'main error':e})
