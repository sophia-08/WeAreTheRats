import socket
import json

def connect_and_receive():
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server_address = '/tmp/hid_device_socket'
    sock.connect(server_address)
    
    try:
        while True:
            data = sock.recv(1024).decode()
            if not data:
                break
            try:
                json_data = json.loads(data)
                print(f"Received HID report: {json_data}")
            except json.JSONDecodeError:
                print(f"Received non-JSON data: {data}")
    finally:
        sock.close()

if __name__ == "__main__":
    connect_and_receive()