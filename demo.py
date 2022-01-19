#! /bin/env python3
# import os
# import sys
# import time
#
# def like_cheese():
#     print(time.time(),"That's awesome!")
#     time.sleep(1)
#     exit()
#
# if __name__ == '__main__':
#     like_cheese()
#     time.sleep(1)
#     # os.execv(__file__,["1"])  # Run a new iteration of the current script, providing any command line args from the current iteration.
#     os.execv(sys.executable, ['python'] + sys.argv)
import socket
import threading
import time


class TcpServer:
    # 客户处理线程
    def handle_client(self, client_socket):
        while True:
            recv_data = client_socket.recv(1024)
            print("[*] Received: %s" % recv_data)
            # 向客户端返回数据
            data_len = len(recv_data)
            print("接收的数据长度为:", data_len)
            # 对二进制数据进行解码
            recv_content = recv_data.decode("gbk")
            print("接收客户端的数据:", recv_content)
            client_socket.send("hi~".encode())
            if recv_content == 'bye':
                break
        client_socket.close()

    def __init__(self):
        self.bind_ip = "0.0.0.0"  # 监听所有可用的接口
        self.bind_port = 3389  # 非特权端口号都可以使用
        # AF_INET：使用标准的IPv4地址或主机名，SOCK_STREAM：说明这是一个TCP服务器
        self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 服务器监听的ip和端口号
        self.tcp_server_socket.bind((self.bind_ip, self.bind_port))
        print("[*] Listening on %s:%d" % (self.bind_ip, self.bind_port))

        self.tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        # 最大连接数
        self.tcp_server_socket.listen(128)

    def start_server(self):
        while True:
            # 等待客户连接，连接成功后，将socket对象保存到client，将细节数据等保存到addr
            client, addr = self.tcp_server_socket.accept()
            print("客户端的ip地址和端口号为:", addr)
            print('time:', time.time())
            # 代码执行到此，说明客户端和服务端套接字建立连接成功
            print("[*] Acception connection from %s:%d" % (addr[0], addr[1]))
            client_handler = threading.Thread(target=self.handle_client, args=(client,))
            client_handler.start()


if __name__ == '__main__':
    obj = TcpServer()
    obj.start_server()
