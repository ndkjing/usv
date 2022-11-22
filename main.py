"""
入口函数
"""
import threading
import time
import config
import tcp_server
from utils import log
from messageBus import data_manager

logger = log.LogHandler('main_log', level=20)


class Main:
    def __init__(self):
        self.tcp_server_obj = tcp_server.TcpServer(self)  # tcp发送数据对象
        self.damanager_dict = {}
        self.is_close = 0


def main():
    config.update_setting()
    main_obj = Main()
    start_server_thread = threading.Thread(target=main_obj.tcp_server_obj.start_server)
    start_server_thread.setDaemon(True)
    start_server_thread.start()
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
                                                                                tcp_server_obj=main_obj.tcp_server_obj)
                    ship_thread = threading.Thread(target=main_obj.damanager_dict[ship_id].thread_control)
                    ship_thread.setDaemon(True)
                    ship_thread.start()
                    ship_thread_dict[ship_id] = ship_thread
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
