import socket
import struct
import time
import threading
import keyboard

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Server IP and port
server_ip = '192.168.1.79'
server_port = 3333
server_address = (server_ip, server_port)

i = 0

def increase_i():
    global i
    i += 100

def decrease_i():
    global i
    i -= 100

def send_data():
    global i
    while True:
        # Send two integers to the server
        client_socket.sendto(i.to_bytes(4, "little") + i.to_bytes(4, "little"), server_address)
        # Wait for 100 ms
        time.sleep(0.1)

def receive_data():
    while True:
        # Receive the response from the server
        data, _ = client_socket.recvfrom(1024)
        # Unpack the data
        dat = struct.unpack("<iiiiiiiffffffffff", data)
        # Print the unpacked data
        for d in dat:
            print(d)

# Register the keyboard hooks
keyboard.on_press_key("up", lambda _: increase_i())
keyboard.on_press_key("down", lambda _: decrease_i())

# Create threads
send_thread = threading.Thread(target=send_data)
receive_thread = threading.Thread(target=receive_data)

# Start threads
send_thread.start()
receive_thread.start()

# Wait for both threads to finish
send_thread.join()
receive_thread.join()
