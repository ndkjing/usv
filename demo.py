"""
Companion routing UDP client example.

This script connects to a endpoint at the Companion exposed using the IP 0.0.0.0
and a given port. It then sends some data so the server knows it has a client
and starts relaying the serial data back at it.
You should run this at your topside computer.
"""
import socket
import time

UDP_IP = "192.168.2.2" # Remote (Companion's) IP to connect to
UDP_PORT = 8888        # Remote (Companion's) port to connect to
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    # Send something so the server knows where to reply to
    # sent = sock.sendto(b"hello", (UDP_IP, UDP_PORT))
    # Loop receiving data
    while True:
        data, server = sock.recvfrom(4096)
        print(data.decode())
        time.sleep(0.01)
except Exception as e:
    print(e)
finally:
    sock.close()

