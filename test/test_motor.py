import socket
import time
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_ip = '192.168.4.1'
server_port = 3333
server_address = (server_ip, server_port)

for i in range(4096):
    client_socket.sendto(i.to_bytes(4, "little") + i.to_bytes(4, "little"), server_address)
    data, _ = client_socket.recvfrom(1024)
    dat = struct.unpack("<iiiiiiiiiiiiiiiii", data)
    for d in dat:
        print(i, d)
    time.sleep(1)

client_socket.close()
