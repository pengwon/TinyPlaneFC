import socket
import time
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_ip = '192.168.20.123'
server_port = 3333
server_address = (server_ip, server_port)

for i in range(4096):
    client_socket.sendto(i.to_bytes(4, "little") + i.to_bytes(4, "little"), server_address)
    data, _ = client_socket.recvfrom(1024)
    data1, data2 = struct.unpack("<ii", data)
    print(i, data1, data2)
    time.sleep(1)

client_socket.close()
