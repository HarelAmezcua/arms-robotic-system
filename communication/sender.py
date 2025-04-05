# sender.py
import socket
import json
import time

host = '192.168.118.154'  # Localhost
port = 5000

data_to_send = {
    "object_detected": 1,
    "position": [1.2, 3.4, 5.6],
    "orientation": [0.0, 0.0, 0.0]
}

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((host, port))
    
    # Receive handshake
    ack = sock.recv(1024)
    if ack != b'ACK':
        print("Handshake failed.")
        exit(1)
    print("Handshake successful.")

    # Send JSON periodically
    for i in range(5):  # You can loop forever or trigger on events
        data_to_send["timestamp"] = time.time()
        json_str = json.dumps(data_to_send)
        sock.sendall(json_str.encode())
        print("Sent:", json_str)
        time.sleep(2)  # Simulate time between messages