import copy
import re
import socket
import server_config
import config
import json
import time
import requests
import threading
from utils import log


def singleton(cls):
    _instance = {}

    def _singleton(*args, **kargs):
        if cls not in _instance:
            _instance[cls] = cls(*args, **kargs)
        return _instance[cls]

    return _singleton


server_logger = log.LogHandler('server_log', level=20)


@singleton
class TcpServer:
    def __init__(self):
        self.bind_ip = server_config.tcp_server_ip  # 监听所有可用的接口
        self.bind_port = server_config.tcp_server_port  # 非特权端口号都可以使用
        # AF_INET：使用标准的IPv4地址或主机名，SOCK_STREAM：说明这是一个TCP服务器
        self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 服务器监听的ip和端口号
        self.tcp_server_socket.bind((self.bind_ip, self.bind_port))
        print("[*] Listening on %s:%d" % (self.bind_ip, self.bind_port))
        self.tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        # 最大连接数
        self.tcp_server_socket.listen(128)
        # 是否有链接上
        self.b_connect = 0
        self.client = None
        self.client_dict = {}  # {船号:client}
        self.ship_status_data_dict = {}  # 船状态数据
        self.ship_draw_dict = {}  # 船抽水状态数据
        self.ship_detect_data_dict = {}  # 船检测数据
        self.ship_obstacle_data_dict = {}  # 船检测数据
        self.receive_confirm_data = ""
        self.disconnect_client_list = []  # 断线了的船号
        self.base_setting_dict = {}  # 船只返航避障设置
        self.ship_id_time_dict = {}  # 船号最近收到的消息
        self.ship_id_deep_dict = {}  # 船号检测到的深度
        self.ship_id_send_dict = {}  # 船号对应要发送数据

    def wait_connect(self):
        # 等待客户连接，连接成功后，将socket对象保存到client，将细节数据等保存到addr
        client, addr = self.tcp_server_socket.accept()
        print("客户端的ip地址和端口号为:", addr)
        self.b_connect = 1
        self.client = client

    def start_server(self):
        while True:
            # 等待客户连接，连接成功后，将socket对象保存到client，将细节数据等保存到addr
            client, addr = self.tcp_server_socket.accept()
            server_logger.info({time.time(): ["客户端的ip地址和端口号为:", addr]})
            # 代码执行到此，说明客户端和服务端套接字建立连接成功
            client_handler = threading.Thread(target=self.handle_client, args=(client, addr))
            # 子线程守护主线程
            client_handler.setDaemon(True)
            client_handler.start()
            time.sleep(0.5)

    # 客户处理线程
    def handle_client(self, client, addr):
        addr_dict = {}
        last_send_time = time.time()  # 上次接收到避障消息时间
        ship_id = None
        while True:
            try:
                recv_data = client.recv(1024, 0)
                if recv_data:
                    recv_content = recv_data.decode("gbk")
                    ship_id_list = re.findall('[ABCD](\d+)', recv_content)
                    if len(ship_id_list) > 0:
                        ship_id = int(ship_id_list[0])
                        self.ship_id_time_dict.update({ship_id: time.time()})
                        if recv_content.startswith('A'):
                            rec_list = recv_content.split(',')
                            if len(rec_list) >= 7:
                                if ship_id not in self.client_dict:
                                    self.client_dict.update({ship_id: client})
                                if ship_id not in addr_dict:
                                    addr_dict.update({addr: ship_id})
                                if ship_id in self.disconnect_client_list:
                                    self.disconnect_client_list.remove(ship_id)
                                lng = float(rec_list[1]) / 1000000.0
                                lat = float(rec_list[2]) / 1000000.0
                                dump_energy = round(float(rec_list[3]), 1)
                                current_angle = round(float(rec_list[4]), 1)
                                current_mode = int(rec_list[5])
                                angle_error = int(rec_list[6])
                                if len(rec_list) == 8:
                                    speed = int(rec_list[7].split('Z')[0])
                                    self.ship_status_data_dict.update(
                                        {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                   speed]})
                                elif len(rec_list) == 9:
                                    speed = int(rec_list[7])
                                    deep = int(rec_list[8].split('Z')[0])
                                    deep = round(deep / 100.0 + config.deep_recoup, 2)
                                    self.ship_id_deep_dict.update({ship_id: deep})
                                    self.ship_status_data_dict.update(
                                        {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                   speed]})
                                # print(time.time(),'rec_list',rec_list)
                            else:
                                server_logger.info({ship_id: [time.time(), "接收客户端的状态数据:", recv_content]})
                        elif recv_content.startswith('B'):
                            rec_list = recv_content.split(',')
                            if len(rec_list) == 6:
                                wt = round(float(rec_list[1]) / 10.0, 2)
                                ph = round(float(rec_list[2]) / 100.0, 2)
                                doDo = round(float(rec_list[3]) / 100.0, 2)
                                ec = round(float(rec_list[4]) / 10.0, 2)
                                td = round(float(rec_list[5].split('Z')[0]) / 100.0, 2)
                                self.ship_detect_data_dict.update(
                                    {ship_id: [wt, ph, doDo, ec, td]})
                                server_logger.info({'检测深度数据反馈消息 wt, ph, doDo, ec, td': [wt, ph, doDo, ec, td]})
                            elif len(rec_list) == 2:
                                deep = int(rec_list[1].split('Z')[0])
                                self.ship_detect_data_dict.update(
                                    {ship_id: [deep]})
                                server_logger.info('深度数据反馈消息%s\r\n' % recv_content.strip())
                        elif recv_content.startswith('C'):
                            rec_list = recv_content.split(',')
                            if len(rec_list) == 5:
                                bottle_id = int(rec_list[1])
                                draw_status = int(rec_list[2])
                                dump_draw_time = int(rec_list[3])
                                full_draw_time = int(rec_list[4].split('Z')[0])
                                self.ship_draw_dict.update(
                                    {ship_id: [bottle_id, draw_status, dump_draw_time, full_draw_time]})
                            # print('抽水反馈消息：%s\r\n' % recv_content.strip())
                        elif recv_content.startswith('D'):
                            rec_list = recv_content.split(',')
                            if len(rec_list) >= 4:
                                obj_id = int(rec_list[1])
                                obj_angle = 2 * int(rec_list[2]) - 90
                                obj_distance = int(rec_list[3].split('Z')[0]) / 100.0
                                if obj_distance > 15.0:  # 不处理大于15米障碍物
                                    continue
                                if ship_id not in self.ship_obstacle_data_dict:
                                    self.ship_obstacle_data_dict.update(
                                        {ship_id: {obj_id: [obj_angle, obj_distance]}})
                                else:
                                    self.ship_obstacle_data_dict.get(ship_id).update(
                                        {obj_id: [obj_angle, obj_distance]})
                                # print(time.time(), '障碍物检测反馈消息', recv_content.strip())
                                last_send_time = time.time()
                            # print('self.ship_obstacle_data_dict', self.ship_obstacle_data_dict)
                    if recv_content.startswith('S') and ship_id:
                        server_logger.info({time.time(): ['接收客户端的确认数据:%s\r\n' % recv_content]})
                        if "Operating" in recv_content:
                            server_logger.error({"树莓派重启": time.asctime(time.localtime(time.time()))})
                        self.receive_confirm_data = recv_content
                        # print('self.ship_id_send_dict', self.ship_id_send_dict,ship_id)
                        if self.ship_id_send_dict.get(ship_id):
                            temp_info_list = copy.copy(self.ship_id_send_dict.get(ship_id))
                            # print('11111111111111self.ship_id_send_dict', self.ship_id_send_dict)
                            for index, info in enumerate(temp_info_list):
                                if info and 'S8' not in info and info in recv_content:
                                    temp_info_list[index] = ""  # 接收到确认消息清空消息
                                    print('清空设置', index, info)
                            self.ship_id_send_dict[ship_id] = temp_info_list
                            # print('22222222222222222self.ship_id_send_dict', self.ship_id_send_dict)
                        # print('3333333333333333333self.ship_id_send_dict.get(ship_id)', self.ship_id_send_dict)
                        if time.time() - last_send_time > 2:
                            self.ship_obstacle_data_dict.update({ship_id: {}})
                if addr_dict.get(addr) in self.disconnect_client_list:
                    return
            except ValueError as e1:
                server_logger.error({'tcp解析数据报错..': e1})
            except IndexError as e2:
                server_logger.error({'tcp接受数据报错..': e2})
            except Exception as e:
                server_logger.error({'tcp接受数据报错..': e})
                if addr_dict.get(addr):
                    if addr_dict.get(addr) not in self.disconnect_client_list:
                        self.disconnect_client_list.append(addr_dict.get(addr))
                    if addr_dict.get(addr) in self.client_dict:
                        del self.client_dict[addr_dict.get(addr)]
                time.sleep(5)
                return

    def close(self):
        self.tcp_server_socket.close()

    def write_data(self, ship_id, data):
        try:
            if ship_id in self.client_dict.keys():
                if not data.startswith('S8') and not data.startswith('S3'):
                    server_logger.info('tcp发送数据%s\r\n' % data)
                # server_logger.info('tcp发送数据%s\r\n' % data)
                if self.ship_id_time_dict.get(ship_id) and time.time() - self.ship_id_time_dict.get(ship_id) > 10:
                    server_logger.error('tcp超时主动断开连接')
                    raise Exception
                # print('tcp发送数据', data, self.client_dict)
                self.client_dict.get(ship_id).send(data.encode())
        except Exception as e:
            if ship_id not in self.disconnect_client_list:
                self.disconnect_client_list.append(ship_id)
            if ship_id in self.client_dict:
                del self.client_dict[ship_id]
            server_logger.error({'tcp发送数据终端断开连接': e})


def te_():
    while True:
        for i in range(4):
            obj.write_data(i, str(i))
        time.sleep(1)


if __name__ == '__main__':
    obj = TcpServer()
    te_handler = threading.Thread(target=te_)
    # 子线程守护主线程
    te_handler.setDaemon(True)
    te_handler.start()
    obj.start_server()
