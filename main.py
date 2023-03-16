"""
入口函数
"""
import threading
import time
import config
import tcp_server
from utils import log
from messageBus import data_manager
import multi_web_server
logger = log.LogHandler('main_log', level=20)
semaphore = threading.Semaphore(0)

class Main:
    def __init__(self,semaphore_=None):
        self.tcp_server_obj = tcp_server.TcpServer(self,semaphore=semaphore_)  # tcp发送数据对象
        self.damanager_dict = {}
        self.is_close = 0


class WebServerManager:
    def __init__(self):
        self.web_server_obj = multi_web_server.WebServer()  # tcp发送数据对象

    def start_web_server(self):
        find_pool_thread = threading.Thread(target=self.web_server_obj.find_pool)
        get_plan_path_thread = threading.Thread(target=self.web_server_obj.get_plan_path)
        send_bank_distance_thread = threading.Thread(target=self.web_server_obj.send_bank_distance)
        check_online_ship_thread = threading.Thread(target=self.web_server_obj.check_online_ship)
        check_reconnrct_thread = threading.Thread(target=self.web_server_obj.check_reconnrct)
        get_ezviz_thread = threading.Thread(target=self.web_server_obj.get_ezviz_alarm_image)
        find_pool_thread.start()
        get_plan_path_thread.start()
        send_bank_distance_thread.start()
        check_online_ship_thread.start()
        check_reconnrct_thread.start()
        # get_ezviz_thread.start()
        while True:
            if not find_pool_thread.is_alive():
                find_pool_thread = threading.Thread(target=self.web_server_obj.find_pool)
                find_pool_thread.start()
            if not get_plan_path_thread.is_alive():
                get_plan_path_thread = threading.Thread(target=self.web_server_obj.get_plan_path)
                get_plan_path_thread.start()
            if not send_bank_distance_thread.is_alive():
                send_bank_distance_thread = threading.Thread(target=self.web_server_obj.send_bank_distance)
                send_bank_distance_thread.start()
            if not check_online_ship_thread.is_alive():
                check_online_ship_thread = threading.Thread(target=self.web_server_obj.check_online_ship)
                check_online_ship_thread.start()
            if not check_reconnrct_thread.is_alive():
                check_reconnrct_thread = threading.Thread(target=self.web_server_obj.check_reconnrct)
                check_reconnrct_thread.start()
            time.sleep(1)


def main():
    config.update_setting()
    main_obj = Main(semaphore_=semaphore)
    start_server_thread = threading.Thread(target=main_obj.tcp_server_obj.start_server)
    start_server_thread.setDaemon(True)
    start_server_thread.start()
    web_server_manager_obj = WebServerManager()
    start_web_server_thread = threading.Thread(target=web_server_manager_obj.start_web_server)
    start_web_server_thread.setDaemon(True)
    start_web_server_thread.start()
    ship_thread_dict = {}
    while True:
        try:
            for ship_id in list(main_obj.tcp_server_obj.client_dict.keys()):
                if ship_id not in main_obj.damanager_dict:
                    # 判断是否是在线船只
                    if 'XXLJC4LCGSCSD1DA%03d' % ship_id not in config.ship_code_type_dict:
                        continue
                    logger.info({'新船上线': ship_id})
                    main_obj.damanager_dict[ship_id] = data_manager.DataManager(ship_id=ship_id,
                                                                                tcp_server_obj=main_obj.tcp_server_obj,
                                                                                semaphore=semaphore)
                    ship_thread = threading.Thread(target=main_obj.damanager_dict[ship_id].thread_control)
                    ship_thread.setDaemon(True)
                    ship_thread.start()
                    ship_thread_dict[ship_id] = ship_thread
            # 删除失效船只
            for ship_id in ship_thread_dict:
                if not ship_thread_dict.get(ship_id).is_alive():
                    if ship_id in list(main_obj.damanager_dict.keys()):
                        logger.info({'删除船号对象': ship_id})
                        del main_obj.damanager_dict[ship_id]
            time.sleep(1)
        except KeyboardInterrupt as e:
            while True:
                try:
                    # logger.info("KeyboardInterrupt 主线程中关闭socket客户端")
                    # main_obj.tcp_server_obj.close()
                    main_obj.tcp_server_obj.tcp_server_socket.detach()
                    main_obj.is_close = 1
                    logger.info({'主动结束...': e})
                    time.sleep(5)
                    return
                except Exception as e1:
                    print('不要多次CTRL+C...')


if __name__ == '__main__':
    main()
