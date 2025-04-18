# receiver.py
import socket
import json

host = '0.0.0.0'  # Listen on all interfaces
port = 5000

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("Waiting for connection...")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    # Handshake response
    conn.sendall(b'ACK')  # Simple handshake response

    while True:
        data = conn.recv(4096)
        if not data:
            break
        try:
            message = json.loads(data.decode())
            print("Received JSON:", message)
        except json.JSONDecodeError:
            print("Invalid JSON received.")