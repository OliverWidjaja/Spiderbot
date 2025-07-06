import socket, json, time

UDP_IP = "127.0.0.1"
UDP_PORT = 5059

drone_pos_x = 0
drone_pos_y = 0
drone_pos_z = 0
drone_rot_x = 0
drone_rot_y = 0
drone_rot_z = 0
drone_rot_w = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Non-blocking

def update_drone_pose():
    global drone_pos_x, drone_pos_y, drone_pos_z
    global drone_rot_x, drone_rot_y, drone_rot_z, drone_rot_w
    try:
        data, _ = sock.recvfrom(1024)
        msg = json.loads(data.decode())
        drone_pos_x, drone_pos_y, drone_pos_z = msg["pos"]
        drone_rot_x, drone_rot_y, drone_rot_z, drone_rot_w = msg["rot"]
    except BlockingIOError:
        pass  # No new data

while True:
    update_drone_pose()
    print(f"pos_x: {drone_pos_x}, pos_y: {drone_pos_y}, pos_z: {drone_pos_z}, rot_w: {drone_rot_w}, rot_x: {drone_rot_x}, rot_y: {drone_rot_y}, rot_z: {drone_rot_z}")
    time.sleep(0.2)