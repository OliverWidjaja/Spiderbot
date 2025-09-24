import socket, json, time


UDP_IP = "127.0.0.1"
UDP_PORT = 5059

position = [0, 0, 0]  # pos_x, pos_y, pos_z
orientation = [0, 0, 0]  # yaw, pitch, roll

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Non-blocking

def update_drone_pose():
    global position, orientation
    try:
        data, _ = sock.recvfrom(1024)
        msg = json.loads(data.decode())
        position = msg["pos"]
        orientation = msg["orientation"]
    except BlockingIOError:
        pass  # No new data 

while True:
    update_drone_pose()
    print(f"Position: {position}, Orientation: {orientation}")
    time.sleep(0.1)
    