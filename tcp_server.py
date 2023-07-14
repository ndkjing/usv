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
from utils import lng_lat_calculate


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
    def __init__(self, main_obj, semaphore):
        self.main_obj = main_obj
        self.bind_ip = server_config.tcp_server_ip  # 监听所有可用的接口
        self.bind_port = server_config.tcp_server_port  # 非特权端口号都可以使用
        # AF_INET：使用标准的IPv4地址或主机名，SOCK_STREAM：说明这是一个TCP服务器
        self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 服务器监听的ip和端口号
        try:
            self.tcp_server_socket.bind((self.bind_ip, self.bind_port))
        except OSError as e:
            print("连接报错")
            # 设置socket参数使端口可以再次使用
            self.tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        self.receive_confirm_data = ""  # 返回的确认消息
        self.disconnect_client_list = []  # 断线了的船号
        self.base_setting_dict = {}  # 船只返航避障设置
        self.ship_id_time_dict = {}  # 船号最近收到的消息
        self.ship_id_deep_dict = {}  # 船号检测到的深度
        self.ship_id_send_dict = {}  # 船号对应要发送数据
        self.gps_millimeter_wave_online = {}  # GPS和毫米波是否正常连接，0：没连接  1：正常连接
        self.peripherals_status = {}  # 船外设连接状态，0：异常  1：正常 2:未装备
        self.semaphore = semaphore
        self.obstacle_lng_lat_list_dict = {}  # 需要删除障碍物列表和概率列表中障碍物索引
        self.obstacle_p_list_dict = {}
        self.del_index_list_dict = {}
        #     obstacle_lng_lat_list = []  # 需要删除障碍物列表和概率列表中障碍物索引
        # obstacle_p_list = []
        # del_index_list = []

    # def wait_connect(self):
    #     # 等待客户连接，连接成功后，将socket对象保存到client，将细节数据等保存到addr
    #     if self.main_obj.is_close:
    #         return
    #     client, addr = self.tcp_server_socket.accept()
    #     self.b_connect = 1
    #     self.client = client

    def start_server(self):
        self.tcp_server_socket.settimeout(3)
        print('tcp超时时间:', self.tcp_server_socket.gettimeout())
        while True:
            # 等待客户连接，连接成功后，将socket对象保存到client，将细节数据等保存到addr
            try:
                client, addr = self.tcp_server_socket.accept()
                # client.settimeout(3)
                server_logger.info({time.time(): ["客户端的ip地址和端口号为:", addr]})
                # 代码执行到此，说明客户端和服务端套接字建立连接成功
                client_handler = threading.Thread(target=self.handle_client, args=(client, addr))
                # 子线程守护主线程
                client_handler.setDaemon(True)
                client_handler.start()
                time.sleep(0.5)
            except socket.timeout as e:
                if self.main_obj.is_close:
                    server_logger.info({"socket.timeout socket 关闭": e})
                    self.close()
                    return
            except OSError as e1:
                server_logger.error({'OSError socket 关闭': e1})
                self.close()
                return

    # 客户处理线程
    def handle_client(self, client, addr):
        addr_dict = {}
        last_send_time = time.time()  # 上次接收到避障消息时间
        ship_id = None
        pre_log_info_time = 0
        pre_log_detect_time = 0

        while True:
            try:
                if self.main_obj.is_close:
                    print('handle_client 退出')
                    time.sleep(1)
                    return
                recv_data = client.recv(1024, 0)
                # print('recv_data', recv_data)
                if recv_data:
                    recv_content_all = recv_data.decode("gbk")
                    # print(time.time(), 'recv_content_all', recv_content_all)
                    recv_content_all_list = recv_content_all.split('\r\n')
                    for recv_content in recv_content_all_list:
                        ship_id_list = re.findall('[ABCDE](\d+)', recv_content)
                        if len(ship_id_list) > 0:
                            ship_id = int(ship_id_list[0])
                            self.ship_id_time_dict.update({ship_id: time.time()})
                            if recv_content.startswith('A'):
                                rec_list = recv_content.split(',')
                                if len(rec_list) >= 7:
                                    self.client_dict.update({ship_id: client})
                                    if ship_id not in addr_dict:
                                        addr_dict.update({addr: ship_id})
                                    if ship_id in self.disconnect_client_list:
                                        self.disconnect_client_list.remove(ship_id)
                                    # 如果有信号量 则释放信号量
                                    if self.semaphore is not None and ship_id == 7:
                                        # if self.semaphore is not None:
                                        self.semaphore.release()
                                    lng = float(rec_list[1]) / 1000000.0
                                    lat = float(rec_list[2]) / 1000000.0
                                    dump_energy = round(float(rec_list[3]), 1)
                                    current_angle = round(float(rec_list[4]), 1)
                                    current_mode = int(rec_list[5])
                                    angle_error = int(rec_list[6])
                                    if len(rec_list) == 8:
                                        speed = int(rec_list[7].split('Z')[0])
                                        speed = round(speed / 1000.0 / 3.6, 1)
                                        self.ship_status_data_dict.update(
                                            {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                       speed]})
                                        if current_angle or lng:
                                            self.gps_millimeter_wave_online.update(
                                                {ship_id: {"gps": 1}})
                                    elif len(rec_list) == 9:
                                        speed = int(rec_list[7])
                                        speed = round(speed / 1000.0 / 3.6, 1)
                                        deep = int(rec_list[8].split('Z')[0])
                                        deep = round(deep / 100.0 + config.deep_recoup, 2)
                                        self.ship_id_deep_dict.update({ship_id: deep})
                                        self.ship_status_data_dict.update(
                                            {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                       speed]})
                                        if current_angle or lng:
                                            self.gps_millimeter_wave_online.update(
                                                {ship_id: {"gps": 1}})
                                    elif len(rec_list) == 10:
                                        speed = int(rec_list[7])
                                        speed = round(speed / 1000.0 / 3.6, 1)
                                        deep = int(rec_list[8])
                                        if deep != 0:  # 有深度的情况下才添加补偿
                                            deep = round(deep / 100.0 + config.deep_recoup, 2)
                                        height = int(rec_list[9].split('Z')[0])
                                        # print('海拔:',height)
                                        self.ship_id_deep_dict.update({ship_id: deep})
                                        self.ship_status_data_dict.update(
                                            {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                       speed, height]})
                                        # 判断毫米波和GPS是否在线(获取到数据)
                                        if current_angle or lng:
                                            self.gps_millimeter_wave_online.update(
                                                {ship_id: {"gps": 1}})
                                    elif len(rec_list) == 11:
                                        speed = int(rec_list[7])
                                        speed = round(speed / 1000.0 / 3.6, 1)
                                        deep = int(rec_list[8])
                                        if deep != 0:  # 有深度的情况下才添加补偿
                                            deep = round(deep / 100.0 + config.deep_recoup, 2)
                                        height = int(rec_list[9])
                                        current_line_deep = int(rec_list[10].split('Z')[0])
                                        # print('current_line_deep:',current_line_deep)
                                        self.ship_id_deep_dict.update({ship_id: deep})
                                        self.ship_status_data_dict.update(
                                            {ship_id: [lng, lat, dump_energy, current_angle, current_mode, angle_error,
                                                       speed, height, current_line_deep]})
                                        # 判断毫米波和GPS是否在线(获取到数据)
                                        if current_angle or lng:
                                            self.gps_millimeter_wave_online.update(
                                                {ship_id: {"gps": 1}})
                                    if self.obstacle_lng_lat_list_dict.get(ship_id):
                                        for index, item in enumerate(
                                                self.obstacle_lng_lat_list_dict.get(ship_id)):
                                            lon = self.ship_status_data_dict.get(ship_id)[0]
                                            lat = self.ship_status_data_dict.get(ship_id)[1]
                                            current_theta = self.ship_status_data_dict.get(ship_id)[3]
                                            obstacle_ship_angle = lng_lat_calculate.angleFromCoordinate(lon,
                                                                                                        lat,
                                                                                                        item[0],
                                                                                                        item[1])
                                            obstacle_ship_distance = lng_lat_calculate.distanceFromCoordinate(
                                                lon, lat,
                                                item[0],
                                                item[1])
                                            is_inside = 0  # 障碍物是否在视野范围内 0：不在  1：在
                                            if current_theta - 45 < 0:
                                                if 0 < obstacle_ship_angle < current_theta + 45:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                    is_inside = 1
                                                elif 360 + current_theta - 45 < obstacle_ship_angle < 360:
                                                    is_inside = 1
                                            elif current_theta + 45 > 360:
                                                if current_theta-45 < obstacle_ship_angle < 360:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                    is_inside = 1
                                                elif 0 < obstacle_ship_angle < current_theta + 45 - 360:
                                                    is_inside = 1
                                            else:
                                                if current_theta - 45 < obstacle_ship_angle < current_theta + 45:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                    is_inside = 1
                                            if is_inside==1:
                                                pass
                                                    # self.obstacle_p_list_dict.get(ship_id)[index] -= 1
                                            if self.obstacle_p_list_dict.get(ship_id)[index] < 0:
                                                if self.del_index_list_dict.get(ship_id) is None:
                                                    self.del_index_list_dict.update({ship_id: []})
                                                self.del_index_list_dict.get(ship_id).append(index)
                                            else:
                                                self.ship_obstacle_data_dict.get(ship_id).update(
                                                    {index: [obstacle_ship_distance, obstacle_ship_angle]})
                                        if self.del_index_list_dict.get(ship_id):
                                            for del_index in self.del_index_list_dict.get(ship_id):
                                                print('删除障碍物索引', del_index,
                                                      self.obstacle_lng_lat_list_dict.get(ship_id)[del_index],
                                                      self.del_index_list_dict.get(ship_id)[del_index])
                                                del self.obstacle_lng_lat_list_dict.get(ship_id)[del_index]
                                                del self.obstacle_p_list_dict.get(ship_id)[del_index]
                                        print('状态数据更新障碍物经纬度列表', self.obstacle_lng_lat_list_dict.get(ship_id),
                                              self.obstacle_p_list_dict.get(ship_id))

                                    # print('self.ship_status_data_dict',self.ship_status_data_dict)
                                    if time.time() - pre_log_info_time > 10:
                                        server_logger.info({"船状态数据": recv_content})
                                        pre_log_info_time = time.time()
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
                                    if doDo < 0.1:  # 说明在空气中
                                        pass
                                    else:
                                        self.ship_detect_data_dict.update(
                                            {ship_id: [wt, ph, doDo, ec, td]})
                                    if time.time() - pre_log_detect_time > 10:
                                        server_logger.info({'检测深度数据反馈消息 wt, ph, doDo, ec, td': [wt, ph, doDo, ec, td]})
                                        pre_log_detect_time = time.time()
                                if len(rec_list) == 8:
                                    wt = round(float(rec_list[1]) / 10.0, 2)
                                    ph = round(float(rec_list[2]) / 100.0, 2)
                                    doDo = round(float(rec_list[3]) / 100.0, 2)
                                    ec = round(float(rec_list[4]) / 10.0, 2)
                                    td = round(float(rec_list[5]) / 10.0, 2)
                                    cod = round(float(rec_list[6]) / 10.0, 2)
                                    nh3nh4 = round(float(rec_list[7].split('Z')[0]) / 100.0, 2)
                                    # if doDo < 0.1:  # 说明在空气中
                                    #     pass
                                    # else:
                                    #     self.ship_detect_data_dict.update(
                                    #         {ship_id: [wt, ph, doDo, ec, td, cod, nh3nh4]})
                                    self.ship_detect_data_dict.update(
                                        {ship_id: [wt, ph, doDo, ec, td, cod, nh3nh4]})
                                    if time.time() - pre_log_detect_time > 10:
                                        server_logger.info({'检测深度数据反馈消息 wt, ph, doDo, ec, td cod nh3nh4': [wt, ph, doDo,
                                                                                                           ec, td, cod,
                                                                                                           nh3nh4]})
                                        pre_log_detect_time = time.time()
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
                                    obj_angle = 2 * int(rec_list[2]) - 90  # 左正右负

                                    obj_distance = int(rec_list[3].split('Z')[0]) / 100.0
                                    if obj_distance:  # 标志毫米波雷达数据有了
                                        self.gps_millimeter_wave_online.update(
                                            {ship_id: {"millimeter_wave": 1}})
                                    if obj_distance > 40.0:  # 不处理大于40米障碍物
                                        continue
                                    print(time.time(), '障碍物检测反馈消息', recv_content.strip())
                                    last_send_time = time.time()
                                    use_id = 0  # 1：使用id发送障碍物  0：使用概率
                                    if use_id:
                                        if ship_id not in self.ship_obstacle_data_dict:
                                            self.ship_obstacle_data_dict.update(
                                                {ship_id: {obj_id: [obj_angle, obj_distance]}})
                                        else:
                                            self.ship_obstacle_data_dict.get(ship_id).update(
                                                {obj_id: [obj_angle, obj_distance]})
                                    else:
                                        if self.ship_status_data_dict.get(ship_id) and len(
                                                self.ship_status_data_dict.get(ship_id)) > 3:
                                            lon = self.ship_status_data_dict.get(ship_id)[0]
                                            lat = self.ship_status_data_dict.get(ship_id)[1]
                                            current_theta = self.ship_status_data_dict.get(ship_id)[3]
                                            need_angle = current_theta + obj_angle
                                            # 转换障碍物为绝对角度 北为0 逆时针为真
                                            if need_angle < 0:
                                                need_angle += 360
                                            elif need_angle > 360:
                                                need_angle -= 360
                                            # 计算障碍物经纬度
                                            obstacle_lng_lat = lng_lat_calculate.one_point_diatance_to_end(lon, lat,
                                                                                                           need_angle,
                                                                                                           obj_distance / 100.0)
                                            print('障碍物经纬度', obstacle_lng_lat)
                                            if self.obstacle_lng_lat_list_dict.get(
                                                    ship_id) is None:
                                                self.obstacle_lng_lat_list_dict.update({ship_id: []})
                                                self.obstacle_lng_lat_list_dict.get(ship_id).append(obstacle_lng_lat)
                                                self.obstacle_p_list_dict.update({ship_id: []})
                                                self.obstacle_p_list_dict.get(ship_id).append(5)
                                                # obstacle_lng_lat_list.append(obstacle_lng_lat)
                                                # obstacle_p_list.append(5)
                                            # 判断障碍物经纬度是否和以前重合
                                            else:
                                                is_point_add = 1  # 点是否要添加到列表中 （与其他的都不重合）
                                                for index, item in enumerate(
                                                        self.obstacle_lng_lat_list_dict.get(ship_id)):
                                                    obstacle_distance = lng_lat_calculate.distanceFromCoordinate(
                                                        item[0],
                                                        item[1],
                                                        obstacle_lng_lat[
                                                            0],
                                                        obstacle_lng_lat[
                                                            1])
                                                    print('距离', obstacle_distance)
                                                    if obstacle_distance < 1:  # 距离为1认为为同一点
                                                        item[0] = (item[0] + obstacle_lng_lat[0]) / 2  # 更新经纬度和概率
                                                        item[1] = (item[1] + obstacle_lng_lat[1]) / 2
                                                        self.obstacle_p_list_dict.get(ship_id)[index] += 5
                                                        is_point_add = 0
                                                        print('两个障碍物距离近', obstacle_distance, item, obstacle_lng_lat)
                                                    obstacle_ship_angle = lng_lat_calculate.angleFromCoordinate(lon,
                                                                                                                lat,
                                                                                                                item[0],
                                                                                                                item[1])
                                                    obstacle_ship_distance = lng_lat_calculate.distanceFromCoordinate(
                                                        lon, lat,
                                                        item[0],
                                                        item[1])
                                                    is_inside = 0  # 障碍物是否在视野范围内 0：不在  1：在
                                                    if current_theta - 45 < 0:
                                                        if 0 < obstacle_ship_angle < current_theta + 45:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                            is_inside = 1
                                                        elif 360 + current_theta - 45 < obstacle_ship_angle < 360:
                                                            is_inside = 1
                                                    elif current_theta + 45 > 360:
                                                        if current_theta - 45 < obstacle_ship_angle < 360:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                            is_inside = 1
                                                        elif 0 < obstacle_ship_angle < current_theta + 45 - 360:
                                                            is_inside = 1
                                                    else:
                                                        if current_theta - 45 < obstacle_ship_angle < current_theta + 45:  # 障碍物在船视野范围内 就将障碍物的概率-1
                                                            is_inside = 1
                                                    if is_inside == 1:
                                                        self.obstacle_p_list_dict.get(ship_id)[index] -= 1
                                                    if self.obstacle_p_list_dict.get(ship_id)[index] < 0:
                                                        if self.del_index_list_dict.get(ship_id) is None:
                                                            self.del_index_list_dict.update({ship_id: []})
                                                        self.del_index_list_dict.get(ship_id).append(index)
                                                    else:
                                                        self.ship_obstacle_data_dict.get(ship_id).update(
                                                            {index: [obstacle_ship_distance, obstacle_ship_angle]})
                                                if is_point_add:
                                                    self.obstacle_lng_lat_list_dict.get(ship_id).append(
                                                        obstacle_lng_lat)
                                                    self.obstacle_p_list_dict.get(ship_id).append(5)
                                                for del_index in self.del_index_list_dict.get(ship_id):
                                                    print('删除障碍物索引', del_index,
                                                          self.obstacle_lng_lat_list_dict.get(ship_id)[del_index],
                                                          self.del_index_list_dict.get(ship_id)[del_index])
                                                    del self.obstacle_lng_lat_list_dict.get(ship_id)[del_index]
                                                    del self.obstacle_p_list_dict.get(ship_id)[del_index]
                                                print('障碍物经纬度列表', self.obstacle_lng_lat_list_dict.get(ship_id),
                                                      self.obstacle_p_list_dict.get(ship_id))
                            elif recv_content.startswith('E'):
                                rec_list = recv_content.split(',')
                                if len(rec_list) == 7:
                                    gps_status = int(rec_list[1])
                                    millimeter_wave_status = int(rec_list[2])
                                    water_data_status = int(rec_list[3])
                                    deep_sensor_status = int(rec_list[4])
                                    imu_status = int(rec_list[5])
                                    deep_motor_status = int(rec_list[6].split('Z')[0])
                                    self.peripherals_status.update(
                                        {ship_id: [gps_status, millimeter_wave_status, water_data_status,
                                                   deep_sensor_status, imu_status, deep_motor_status]})
                                # print('抽水反馈消息：%s\r\n' % recv_content.strip())
                        if recv_content.startswith('S') and ship_id:
                            server_logger.info({time.time(): ['接收客户端的确认数据:%s\r\n' % recv_content]})
                            self.receive_confirm_data = recv_content
                            # print('self.ship_id_send_dict', self.ship_id_send_dict,ship_id)
                            if self.ship_id_send_dict.get(ship_id):
                                temp_info_list = copy.copy(self.ship_id_send_dict.get(ship_id))
                                # print('11111111111111self.ship_id_send_dict', self.ship_id_send_dict)
                                for index, info in enumerate(temp_info_list):
                                    if info and 'S8' not in info and info in recv_content:  # 接受到确认数据就清空需要发送数据
                                        temp_info_list[index] = ""  # 接收到确认消息清空消息
                                        print('清空设置', index, info)
                                self.ship_id_send_dict[ship_id] = temp_info_list
                                # print('22222222222222222self.ship_id_send_dict', self.ship_id_send_dict)
                            # print('3333333333333333333self.ship_id_send_dict.get(ship_id)', self.ship_id_send_dict)
                        if time.time() - last_send_time > 2:  # 超时清除障碍物数据
                            self.ship_obstacle_data_dict.update({ship_id: {}})
                        if "Operating" in recv_content:
                            server_logger.error({"树莓派重启": time.asctime(time.localtime(time.time()))})
                        if recv_content and recv_content[0] not in ['S', 'A', 'B', 'C', 'D']:
                            server_logger.info({'其他类型消息..': recv_content})
                if addr_dict.get(addr) in self.disconnect_client_list:
                    return
            except ValueError as e1:
                server_logger.error({'tcp解析数据报错ValueError..': e1})
            except IndexError as e2:
                server_logger.error({'tcp接受数据报错IndexError..': e2})
            except ConnectionResetError as e3:
                server_logger.error({'tcp接受数据连接重置..': e3})
                time.sleep(2)
                continue
            except TimeoutError as e4:
                server_logger.error({'tcp接受数据TimeoutError..': e4})
            except Exception as e:
                server_logger.error({'tcp接受数据报错Exception..': e})
                if addr_dict.get(addr):
                    if addr_dict.get(addr) not in self.disconnect_client_list:
                        self.disconnect_client_list.append(addr_dict.get(addr))
                    if addr_dict.get(addr) in self.client_dict:
                        del self.client_dict[addr_dict.get(addr)]
                time.sleep(2)
                continue

    def close(self):
        # self.tcp_server_socket.shutdown(0)
        self.tcp_server_socket.close()
        print('close tcp 关闭')

    def write_data(self, ship_id, data):
        if self.main_obj.is_close:
            return
        try:
            if ship_id in self.client_dict.keys():
                if not data.startswith('S8'):
                    server_logger.info('tcp发送数据%s\r\n' % data)
                # if self.ship_id_time_dict.get(ship_id) and time.time() - self.ship_id_time_dict.get(ship_id) > 300:
                #     server_logger.error('tcp超时300主动断开连接')
                # raise Exception  # TODO 暂时不抛出异常
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
