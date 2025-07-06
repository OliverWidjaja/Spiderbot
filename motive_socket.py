from natnet_client import DataDescriptions, DataFrame, NatNetClient
import socket, json, time

UDP_IP = "127.0.0.1"
UDP_PORT = 5059
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a global mapping from rigid body ID to name
rigid_body_id_to_name = {}

def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    # Populate the mapping
    for rigid_body in desc.rigid_bodies:
        rigid_body_id_to_name[rigid_body.id_num] = rigid_body.name
        #print(f"RigidBody Name: {rigid_body.name}, ID: {rigid_body.id_num}")

def receive_new_frame(data_frame: DataFrame):
    global num_frames
    num_frames += 1
    # Print name and coordinates for each rigid body in the frame
    for rigid_body in getattr(data_frame, "rigid_bodies", []):
        name = rigid_body_id_to_name.get(rigid_body.id_num, "Unknown")
        pos = rigid_body.pos
        rot = rigid_body.rot
        pos_str = f"({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})"
        rot_str = f"({rot[0]:.4f}, {rot[1]:.4f}, {rot[2]:.4f}, {rot[3]:.4f})"
        #print(f"Frame: {num_frames} RigidBody Name: {name}, Position: {pos_str}, Rotation (quat): {rot_str}")

        if name == "TestSubject":
            data = {
                "pos": [float(f"{p:.4f}") for p in pos],
                "rot": [float(f"{r:.4f}") for r in rot]
            }
            sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT))

num_frames = 0
if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()

        while True:
            time.sleep(0.5) # How often to check for new frames in seconds
            streaming_client.update_sync()
            print(f"Received {num_frames} frames.")
