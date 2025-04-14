import socket
import time
import json

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 4210        # Same port as your ESP32

# Sample GPS data structure matching NetworkGPSData
def generate_dummy_gps():
    return {
        "lat": 37.7749 + (time.time() % 1) * 0.01, 
        "lon": -122.4194 + (time.time() % 1) * 0.01,
        "alt": 50.0 + (time.time() % 10),
        "spd": (time.time() % 100),
        "sat": 12,
        "ts": int(time.time() * 1000.0)
    }

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Simulated ESP32 server running on {HOST}:{PORT}")
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            # Generate and send JSON data at 10Hz
            dummy_data = generate_dummy_gps()
            json_data = json.dumps(dummy_data) + "\n"  # Add newline delimiter
            conn.sendall(json_data.encode())
            time.sleep(0.04)  # 10Hz = 0.1s delay