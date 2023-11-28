import socket
import struct
import time
import threading
import keyboard

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Server IP and port
server_ip = "192.168.4.1"
server_port = 3333
server_address = (server_ip, server_port)

throttle = 1000
yaw = 1000
pitch = 1000
roll = 1000


def increase_i():
    global throttle, yaw, pitch, roll
    throttle += 100


def decrease_i():
    global throttle, yaw, pitch, roll
    throttle -= 100
    if throttle < 10:
        throttle = 0


def start():
    global throttle, yaw, pitch, roll
    throttle = 1500


def stop():
    global throttle, yaw, pitch, roll
    throttle = 1000


def send_data():
    global throttle, yaw, pitch, roll
    while True:
        # Send two integers to the server
        client_socket.sendto(
            throttle.to_bytes(4, "little")
            + yaw.to_bytes(4, "little")
            + pitch.to_bytes(4, "little")
            + roll.to_bytes(4, "little"),
            server_address
        )
        # Wait for 100 ms
        time.sleep(0.1)


def receive_data():
    while True:
        # Receive the response from the server
        data, _ = client_socket.recvfrom(128)
        # Unpack the data
        dat = struct.unpack("<iiiiiiiiffffffffff", data)
        # Print the unpacked data
        # for d in dat:
        print(dat)


def arm():
    thro = 1951
    client_socket.sendto(
        thro.to_bytes(4, "little")
        + thro.to_bytes(4, "little")
        + thro.to_bytes(4, "little")
        + thro.to_bytes(4, "little"),
        server_address,
    )


# Register the keyboard hooks
keyboard.on_press_key("up", lambda _: increase_i())
keyboard.on_press_key("down", lambda _: decrease_i())
keyboard.on_press_key("enter", lambda _: arm())
keyboard.on_press_key("space", lambda _: stop())

# Create threads
send_thread = threading.Thread(target=send_data)
receive_thread = threading.Thread(target=receive_data)

# Start threads
send_thread.start()
receive_thread.start()

# Wait for both threads to finish
send_thread.join()
receive_thread.join()
