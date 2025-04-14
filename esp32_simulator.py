import socket
import time
import json
import datetime

# Inside your loop, where you build 'data' dict
now = datetime.datetime.utcnow()
formatted_time = now.strftime("%H:%M:%S.%f")[:-3]  # ':-3' trims microseconds to milliseconds

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 4210      # Port number

def generate_dummy_gps():
    current_time = time.time()
    return {
        "latitude": 37.7749 + (current_time % 1) * 0.01,
        "longitude": -122.4194 + (current_time % 1) * 0.01,
        "altitude": 50.0 + (current_time % 10),
        "speed": round(current_time % 100, 3),
        "satellites": 12,
        "timestamp": formatted_time
    }

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"Simulated ESP32 server running on {HOST}:{PORT}")

        while True:
            print("Waiting for a client to connect...")
            conn, addr = server_socket.accept()
            with conn:
                print(f"Connected by {addr}")
                try:
                    while True:
                        dummy_data = generate_dummy_gps()
                        json_data = json.dumps(dummy_data) + "\n"  # Add newline for client readline()
                        conn.sendall(json_data.encode())
                        time.sleep(0.1)  # Adjust frequency: 10Hz is 0.1s (not 0.04s!)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    print(f"Client {addr} disconnected. Waiting for new connection...\n")
                    continue

if __name__ == "__main__":
    start_server()
