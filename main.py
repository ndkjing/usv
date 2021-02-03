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

if config.home_debug:
    time.sleep(config.start_sleep_time)

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
    except Exception as e1:
        logger.error({'binding_data error': e1})

    # 启动串口数据收发和mqtt数据收发
    if (config.current_platform == "l"):
        if os.path.exists(config.port):
            get_com_data_thread = threading.Thread(target=data_manager_obj.get_com_data)
        send_com_data_thread = threading.Thread(target=data_manager_obj.send_com_data)
        if os.path.exists(config.compass_port):
            compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
        if os.path.exists(config.compass_port1):
            compass_thread1 = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass1_data)
        if os.path.exists(config.gps_port):
            gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)
        if config.b_use_remote_control:
            remote_control_thread = threading.Thread(target=data_manager_obj.pi_main_obj.remote_control)
        if config.b_use_ultrasonic:
            left_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_left_distance)
            right_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_right_distance)
    else:
        pass
    check_status_thread = threading.Thread(
        target=data_manager_obj.check_status)

    send_mqtt_data_thread = threading.Thread(
        target=data_manager_obj.send_mqtt_data)

    check_status_thread.setDaemon(True)
    if (config.current_platform == "l"):
        if os.path.exists(config.port):
            get_com_data_thread.setDaemon(True)

        send_com_data_thread.setDaemon(True)
        if os.path.exists(config.compass_port):
            compass_thread.setDaemon(True)
        if os.path.exists(config.compass_port1):
            compass_thread1.setDaemon(True)
        if os.path.exists(config.gps_port):
            gps_thread.setDaemon(True)

        if config.b_use_remote_control:
            remote_control_thread.setDaemon(True)
        if config.b_use_ultrasonic:
            left_distance_thread.setDaemon(True)
            right_distance_thread.setDaemon(True)
    send_mqtt_data_thread.setDaemon(True)

    # send_com_heart_thread.setDaemon(True)

    check_status_thread.start()
    if (config.current_platform == "l"):
        if os.path.exists(config.port):
            get_com_data_thread.start()
        send_com_data_thread.start()
        if os.path.exists(config.compass_port):
            compass_thread.start()
        if os.path.exists(config.compass_port1):
            time.sleep(0.15)
            compass_thread1.start()
        if os.path.exists(config.gps_port):
            gps_thread.start()
        if config.b_use_remote_control:
            remote_control_thread.start()
        if config.b_use_ultrasonic:
            left_distance_thread.start()
            right_distance_thread.start()
    send_mqtt_data_thread.start()

    # send_com_heart_thread.start()
    # get_com_data_thread.join()
    # send_mqtt_data_thread.join()
    # send_com_data_thread.join()
    # check_status_thread.join()
    # send_com_heart_thread.join()
    thread_restart_time = 3
    while True:
        #  判断线程是否死亡并重启线程
        if (config.current_platform == "l"):
            if os.path.exists(config.port) and not get_com_data_thread.is_alive():
                logger.error('restart get_com_data_thread')
                try:
                    if data_manager_obj.com_data_obj.uart.is_open():
                        data_manager_obj.com_data_obj.uart.close()
                    data_manager_obj.com_data_obj = data_manager_obj.get_serial_obj(port=config.port, baud=config.baud)
                except Exception as e:
                    logger.error({'串口关闭失败': 111, 'error': e})
                get_com_data_thread = threading.Thread(target=data_manager_obj.get_com_data)
                get_com_data_thread.setDaemon(True)
                get_com_data_thread.start()
                time.sleep(thread_restart_time)

            if not send_com_data_thread.is_alive():
                logger.error('restart send_com_data_thread')
                send_com_data_thread = threading.Thread(
                    target=data_manager_obj.send_com_data)
                send_com_data_thread.setDaemon(True)
                send_com_data_thread.start()
                time.sleep(thread_restart_time)

            if os.path.exists(config.compass_port):
                if not compass_thread.is_alive():
                    logger.error('restart compass_thread')
                    try:
                        if data_manager_obj.pi_main_obj.compass_obj.uart.is_open():
                            data_manager_obj.pi_main_obj.compass_obj.uart.close()
                        data_manager_obj.com_data_obj = data_manager_obj.pi_main_obj.get_compass_obj(
                            port=config.compass_port, baud=config.compass_baud)
                    except Exception as e:
                        logger.error({'串口关闭失败': 111, 'error': e})

                    compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
                    compass_thread.setDaemon(True)
                    compass_thread.start()
                    time.sleep(thread_restart_time)

            if os.path.exists(config.compass_port1):
                if not compass_thread1.is_alive():
                    logger.error('restart compass_thread1')
                    try:
                        if data_manager_obj.pi_main_obj.compass_obj1.uart.is_open():
                            data_manager_obj.pi_main_obj.compass_obj1.uart.close()
                        data_manager_obj.com_data_obj = data_manager_obj.pi_main_obj.get_compass_obj(
                            port=config.compass_port1, baud=config.compass_baud1)
                    except Exception as e:
                        logger.error({'串口关闭失败': 111, 'error': e})

                    compass_thread1 = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass1_data)
                    compass_thread1.setDaemon(True)
                    compass_thread1.start()
                    time.sleep(thread_restart_time)
            if config.b_use_ultrasonic and not left_distance_thread.is_alive():
                logger.error('restart left_distance_thread')
                try:
                    data_manager_obj.pi_main_obj.left_ultrasonic_obj = data_manager_obj.pi_main_obj.get_left_ultrasonic_obj()
                except Exception as e:
                    logger.error({'restart left_distance_thread失败': 111, 'error': e})
                left_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_left_distance)
                left_distance_thread.setDaemon(True)
                left_distance_thread.start()
                time.sleep(thread_restart_time)

            if config.b_use_ultrasonic and not right_distance_thread.is_alive():
                logger.error('restart left_distance_thread')
                try:
                    data_manager_obj.pi_main_obj.right_ultrasonic_obj = data_manager_obj.pi_main_obj.get_left_ultrasonic_obj()
                except Exception as e:
                    logger.error({'restart left_distance_thread失败': 111, 'error': e})
                right_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_right_distance)
                right_distance_thread.setDaemon(True)
                right_distance_thread.start()
                time.sleep(thread_restart_time)

            if os.path.exists(config.compass_port):
                if not gps_thread.is_alive():
                    logger.error('restart gps_thread')
                    try:
                        if data_manager_obj.pi_main_obj.gps_obj.uart.is_open():
                            data_manager_obj.pi_main_obj.gps_obj.uart.close()
                        data_manager_obj.com_data_obj = data_manager_obj.pi_main_obj.get_gps_obj(port=config.gps_port,
                                                                                                 baud=config.gps_baud)
                    except Exception as e:
                        logger.error({'串口关闭失败': 111, 'error': e})
                    gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)
                    gps_thread.setDaemon(True)
                    gps_thread.start()
                    time.sleep(thread_restart_time)

            if config.b_use_remote_control:
                if not remote_control_thread.is_alive():
                    logger.error('restart remote_control_thread')
                    remote_control_thread = threading.Thread(target=data_manager_obj.pi_main_obj.remote_control)
                    remote_control_thread.setDaemon(True)
                    remote_control_thread.start()
                    time.sleep(thread_restart_time)

        if not send_mqtt_data_thread.is_alive():
            logger.error('restart send_mqtt_data_thread')
            send_mqtt_data_thread = threading.Thread(
                target=data_manager_obj.send_mqtt_data)
            send_mqtt_data_thread.setDaemon(True)
            send_mqtt_data_thread.start()
            time.sleep(thread_restart_time)

        elif not check_status_thread.is_alive():
            logger.error('restart check_status_thread')
            check_status_thread = threading.Thread(
                target=data_manager_obj.check_status)
            check_status_thread.setDaemon(True)
            check_status_thread.start()
            time.sleep(thread_restart_time)
        else:
            time.sleep(1)


if __name__ == '__main__':
    while True:
        try:
            main()
        except Exception as e:
            time.sleep(5)
            logger.error({'main error': e})
