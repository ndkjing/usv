"""
入口函数
"""
import threading
import time
import config
from utils import log
from messageBus import data_manager
from drivers import audios_manager
import sys
import os

if config.current_platform == config.CurrentPlatform.pi:
    time.sleep(config.start_sleep_time)

sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'drivers'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'externalConnect'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'messageBus'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'moveControl'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'statics'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'storage'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))

logger = log.LogHandler('main_log')


def main():
    config.update_setting()
    if config.b_play_audio:
        audios_manager.play_audio(audio_index=audios_manager.AudioType.start)
    # 数据处理对象
    data_manager_obj = data_manager.DataManager()
    # 查询改船是否注册 若未注册直接退出
    try:
        binding_data = data_manager_obj.send(
            method='http', data="", url=config.http_binding, http_type='GET')
        if int(binding_data['flag']) != 1:
            if config.b_play_audio:
                audios_manager.play_audio('register.mp3')
            logger.error({'binding status': binding_data['flag']})
        logger.info({'binding status': binding_data['flag']})
    except Exception as e1:
        logger.error({'binding_data error': e1})

    # 启动串口数据收发和mqtt数据收发  树莓派对象数据处理
    if config.current_platform == config.CurrentPlatform.pi:
        change_pwm_thread = threading.Thread(target=data_manager_obj.pi_main_obj.loop_change_pwm)
        if os.path.exists(config.stc_port):
            get_com_data_thread = threading.Thread(target=data_manager_obj.get_com_data)
        if config.b_pin_gps:
            soft_gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)
        if config.b_pin_compass:
            soft_compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
        if config.b_laser:
            get_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_distance_dict)
        elif config.b_millimeter_wave:
            get_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_distance_dict_millimeter)
        if config.b_laser or config.b_millimeter_wave:
            send_distacne_thread = threading.Thread(target=data_manager_obj.send_distacne)
        if config.b_laser or config.b_pin_stc:
            stc_data_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_stc_data)

    move_control_thread = threading.Thread(target=data_manager_obj.move_control)
    check_status_thread = threading.Thread(target=data_manager_obj.check_status)
    send_mqtt_data_thread = threading.Thread(target=data_manager_obj.send_mqtt_data)
    update_ship_gaode_thread = threading.Thread(target=data_manager_obj.update_ship_gaode_lng_lat)
    update_lng_lat_thread = threading.Thread(target=data_manager_obj.update_lng_lat)
    update_config_thread = threading.Thread(target=data_manager_obj.update_config)
    check_ping_delay_thread = threading.Thread(target=data_manager_obj.check_ping_delay)

    # check_status_thread.setDaemon(True)
    # move_control_thread.setDaemon(True)
    # send_mqtt_data_thread.setDaemon(True)
    # update_ship_gaode_thread.setDaemon(True)
    # update_lng_lat_thread.setDaemon(True)
    # update_config_thread.setDaemon(True)
    # check_ping_delay_thread.setDaemon(True)
    # send_distacne_thread.setDaemon(True)

    # if config.current_platform == config.CurrentPlatform.pi:
    #     change_pwm_thread.setDaemon(True)
    #     if os.path.exists(config.stc_port):
    #         get_com_data_thread.setDaemon(True)
    #     if os.path.exists(config.compass_port):
    #         compass_thread.setDaemon(True)
    #     if os.path.exists(config.compass_port1):
    #         compass_thread1.setDaemon(True)
    #     if os.path.exists(config.gps_port):
    #         gps_thread.setDaemon(True)
    #     if config.b_use_ultrasonic:
    #         left_distance_thread.setDaemon(True)
    #         right_distance_thread.setDaemon(True)
    #     if config.b_pin_gps:
    #         soft_compass_thread.setDaemon(True)
    #         soft_gps_thread.setDaemon(True)
    #     if config.b_laser:
    #         get_distance_thread.setDaemon(True)
    check_status_thread.start()
    send_mqtt_data_thread.start()
    move_control_thread.start()
    update_ship_gaode_thread.start()
    update_lng_lat_thread.start()
    update_config_thread.start()
    check_ping_delay_thread.start()

    if config.current_platform == config.CurrentPlatform.pi:
        change_pwm_thread.start()
        if os.path.exists(config.stc_port):
            get_com_data_thread.start()
        if config.b_pin_gps:
            soft_compass_thread.start()
            soft_gps_thread.start()
        if config.b_laser or config.b_millimeter_wave:
            get_distance_thread.start()
        if config.b_millimeter_wave:
            send_distacne_thread.start()
        if config.b_pin_stc:
            stc_data_thread.start()

    print('home_debug', config.home_debug)
    thread_restart_time = 1
    while True:
        #  判断线程是否死亡并重启线程
        if config.current_platform == config.CurrentPlatform.pi:
            if not change_pwm_thread.is_alive():
                logger.error('restart get_com_data_thread')
                change_pwm_thread = threading.Thread(target=data_manager_obj.pi_main_obj.loop_change_pwm)
                change_pwm_thread.setDaemon(True)
                change_pwm_thread.start()

            if os.path.exists(config.stc_port) and not get_com_data_thread.is_alive():
                logger.error('restart get_com_data_thread')
                try:
                    if data_manager_obj.com_data_obj.uart.is_open():
                        data_manager_obj.com_data_obj.uart.close()
                    data_manager_obj.com_data_obj = data_manager_obj.pi_main_obj.get_com_obj(port=config.stc_port,
                                                                                             baud=config.stc_baud,
                                                                                             logger_=None)
                except Exception as e1:
                    logger.error({'串口关闭失败': 111, 'error': e1})
                get_com_data_thread = threading.Thread(target=data_manager_obj.get_com_data)
                get_com_data_thread.setDaemon(True)
                get_com_data_thread.start()
                time.sleep(thread_restart_time)

            if config.b_pin_gps and not soft_gps_thread.is_alive():
                logger.error('restart soft_gps_thread')
                try:
                    data_manager_obj.pi_main_obj.gps_obj = data_manager_obj.pi_main_obj.get_gps_obj()
                except Exception as e:
                    logger.error({'restart soft_gps_thread': 111, 'error': e})
                soft_gps_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_gps_data)
                soft_gps_thread.setDaemon(True)
                soft_gps_thread.start()
                time.sleep(thread_restart_time)

            if config.b_pin_compass and not soft_compass_thread.is_alive():
                logger.error('restart soft_compass_thread')
                try:
                    data_manager_obj.pi_main_obj.compass_obj = data_manager_obj.pi_main_obj.get_gps_obj()
                except Exception as e:
                    logger.error({'restart soft_compass_thread': 111, 'error': e})
                soft_compass_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_compass_data)
                soft_compass_thread.setDaemon(True)
                soft_compass_thread.start()
                time.sleep(thread_restart_time)

            if config.b_pin_stc and not stc_data_thread.is_alive():
                logger.error('restart stc_data_thread')
                try:
                    data_manager_obj.pi_main_obj.stc_obj = data_manager_obj.pi_main_obj.get_stc_obj()
                except Exception as e:
                    logger.error({'restart stc_data_thread': 111, 'error': e})
                stc_data_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_stc_data)
                stc_data_thread.start()
                time.sleep(thread_restart_time)

            if not get_distance_thread.is_alive():
                logger.error('restart get_distance_thread')
                try:
                    data_manager_obj.pi_main_obj.laser_obj = data_manager_obj.pi_main_obj.get_laser_obj()
                except Exception as e:
                    logger.error({'restart get_distance_thread': 111, 'error': e})
                if config.b_laser:
                    get_distance_thread = threading.Thread(target=data_manager_obj.pi_main_obj.get_distance_dict)
                elif config.b_millimeter_wave:
                    get_distance_thread = threading.Thread(
                        target=data_manager_obj.pi_main_obj.get_distance_dict_millimeter)
                get_distance_thread.setDaemon(True)
                get_distance_thread.start()
                time.sleep(thread_restart_time)

            if not send_distacne_thread.is_alive():
                logger.error('restart send_distacne_thread')
                send_distacne_thread = threading.Thread(
                    target=data_manager_obj.send_distacne)
                send_distacne_thread.setDaemon(True)
                send_distacne_thread.start()
        if not send_mqtt_data_thread.is_alive():
            logger.error('restart send_mqtt_data_thread')
            send_mqtt_data_thread = threading.Thread(
                target=data_manager_obj.send_mqtt_data)
            send_mqtt_data_thread.setDaemon(True)
            send_mqtt_data_thread.start()
            time.sleep(thread_restart_time)

        if not check_status_thread.is_alive():
            logger.error('restart check_status_thread')
            check_status_thread = threading.Thread(
                target=data_manager_obj.check_status)
            check_status_thread.setDaemon(True)
            check_status_thread.start()
            time.sleep(thread_restart_time)

        if not move_control_thread.is_alive():
            logger.error('restart move_control_thread')
            move_control_thread = threading.Thread(
                target=data_manager_obj.move_control)
            move_control_thread.setDaemon(True)
            move_control_thread.start()
            time.sleep(thread_restart_time)

        if not update_ship_gaode_thread.is_alive():
            logger.error('restart update_ship_gaode_thread')
            update_ship_gaode_thread = threading.Thread(
                target=data_manager_obj.update_ship_gaode_lng_lat)
            update_ship_gaode_thread.setDaemon(True)
            update_ship_gaode_thread.start()

        if not update_lng_lat_thread.is_alive():
            logger.error('restart update_lng_lat_thread')
            update_lng_lat_thread = threading.Thread(
                target=data_manager_obj.update_lng_lat)
            update_lng_lat_thread.setDaemon(True)
            update_lng_lat_thread.start()

        if not update_config_thread.is_alive():
            logger.error('restart update_config_thread')
            update_config_thread = threading.Thread(
                target=data_manager_obj.update_config)
            update_config_thread.setDaemon(True)
            update_config_thread.start()

        if not check_ping_delay_thread.is_alive():
            logger.error('restart check_ping_delay_thread')
            check_ping_delay_thread = threading.Thread(
                target=data_manager_obj.check_ping_delay)
            check_ping_delay_thread.setDaemon(True)
            check_ping_delay_thread.start()
        else:
            time.sleep(thread_restart_time)


if __name__ == '__main__':
    main()
