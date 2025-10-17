from scipy.optimize import least_squares
import numpy as np
import time
import asyncio
from bleak import BleakClient
from pyvesc.protocol.interface import encode, encode_request, decode
from pyvesc.VESC.messages import SetPosition, GetValues

# Bluetooth configuration
ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Parameters
g = 9.81
a1, a2 = np.array([-1000/1000, 1040/1000]), np.array([1000/1000, 1040/1000])  # Anchor positions in global frame (m), centred at center of workspace
b1, b2 = np.array([-117.5/1000, 35/1000]), np.array([117.5/1000, 35/1000])  # Cable extrusion points in local frame (m)

# Variables
mass = 2
endp = np.array([0/1000, 750/1000])  # x, y (m, m)
startp = np.array([0/1000, 35/1000])  # x, y (m, m)
t_total = 8
dt = 0.01

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic, tx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
        self.tx_characteristic = tx_characteristic
        self._buffer = bytearray()
        
        # Pre-encode messages for efficiency
        self._primary_request_msg = encode_request(GetValues())
        self._secondary_request_msg = encode_request(GetValues(can_id=119))
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))

        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)
    
    async def get_position(self, can_id=None):
        """Get current position from VESC (primary or secondary)"""
        try:
            # Clear buffer before new request
            self._buffer.clear()
            
            # Send request
            if can_id is not None:
                await self.client.write_gatt_char(self.rx_characteristic, self._secondary_request_msg)
            else:
                await self.client.write_gatt_char(self.rx_characteristic, self._primary_request_msg)
            
            # Wait for response with timeout
            response = await self._wait_for_response(timeout=1.0)
            
            if response is not None:
                # Extract position from response
                if hasattr(response, 'pid_pos_now'):
                    return response.pid_pos_now
                else:
                    print(f"No position data in response")
                    return None
            else:
                print("No response received")
                return None
                
        except Exception as e:
            print(f"Error getting position: {e}")
            return None
    
    async def _wait_for_response(self, timeout=1.0):
        """Wait for and decode response with timeout"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if len(self._buffer) > 0:
                try:
                    response, consumed = decode(bytes(self._buffer))
                    if consumed > 0:
                        self._buffer = self._buffer[consumed:]
                        return response
                except Exception as e:
                    print(f"Error decoding response: {e}")
                    self._buffer.clear()
                    return None
            
            await asyncio.sleep(0.01)
        
        print("Timeout waiting for response")
        return None
    
    async def notification_handler(self, sender, data):
        """Handle incoming notifications (responses from VESCs)"""
        # Add new data to buffer
        self._buffer.extend(data)

def solve_length(x, y, phi):
    rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    c1 = a1 - (rot_mat @ b1 + [x, y])
    c2 = a2 - (rot_mat @ b2 + [x, y])
    return np.linalg.norm(c1), np.linalg.norm(c2) # vector of cable 1 and 2 solution in meters

def solve_force_torque(vars, x, y, m):
    tens1, tens2, phi = vars[0], vars[1], vars[2]
    rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    
    c1 = a1 - (rot_mat @ b1 + [x, y])
    c2 = a2 - (rot_mat @ b2 + [x, y])
    c1_norm = c1 / np.linalg.norm(c1)
    c2_norm = c2 / np.linalg.norm(c2)
    
    fx = tens1 * c1_norm[0] + tens2 * c2_norm[0]
    fy = tens1 * c1_norm[1] + tens2 * c2_norm[1] - m * g
    
    # Torque balance (z-component only)
    b1_rot = np.append(rot_mat @ b1, 0)
    b2_rot = np.append(rot_mat @ b2, 0)
    f1 = np.append(c1_norm * tens1, 0)
    f2 = np.append(c2_norm * tens2, 0)
    torque = np.cross(b1_rot, f1)[2] + np.cross(b2_rot, f2)[2]

    return np.array([fx, fy, torque])

def solve_ik(x, y, m=mass, guess=np.array([10, 10, 1e-3])):
    bounds = ([0, 0, -np.pi/2],  # Lower bounds tens1, tens2, and φ
              [44, 44, np.pi/2])   # Upper bounds tens1, tens2, and φ
    result = least_squares(lambda vars: solve_force_torque(vars, x, y, m), guess, bounds=bounds)
    tens1, tens2, phi = result.x
    cable1, cable2 = solve_length(x, y, phi)
    residuals = solve_force_torque([tens1, tens2, phi], x, y, m)

    return {
        'tensions': np.array([tens1, tens2]),
        'found_lengths': np.array([cable1, cable2]),
        'phi': phi,
        'residuals': residuals,
        'feasible': tens1 > 0 and tens2 > 0 and np.linalg.norm(residuals) < 1e-2
    }

def solve_traj(startp, endp, t_total, dt, print_details=False):
    num_of_steps = int(t_total / dt) + 1
    time_steps = np.arange(0, num_of_steps) * dt
    p = startp + (endp - startp) * (time_steps / t_total).reshape(-1, 1)
    cable_lengths = np.zeros((num_of_steps, 2))

    for i in range(num_of_steps):
        x, y = p[i]
        sol = solve_ik(x, y)

        if (print_details == True):
            print(f"Point ({x:.2f}, {y:.2f}):")
            print(f"  Cable lengths: {sol['found_lengths']} m")
            print(f"  Tensions: {sol['tensions']} N")
            print(f"  φ: {sol['phi']} rad ({np.degrees(sol['phi'])}°)")
            print(f"  Residuals: {sol['residuals']} (N, N, Nm)")
            print(f"  Feasible: {sol['feasible']}\n")

        cable_lengths[i] = sol['found_lengths']

    return cable_lengths

async def run_traj(motor: BluetoothVESC, solution, dt):
    # Get current positions from VESCs to use as offsets
    print("Getting current positions from VESCs...")
    
    angle_1_offset = await motor.get_position(can_id=None)  # Primary VESC
    if angle_1_offset is None:
        print("Warning: Could not get position from primary VESC, using 0 as offset")
        angle_1_offset = 0
    else:
        print(f"Primary VESC current position: {angle_1_offset:.2f}°")
    
    angle_2_offset = await motor.get_position(can_id=119)  # Secondary VESC
    if angle_2_offset is None:
        print("Warning: Could not get position from secondary VESC, using 0 as offset")
        angle_2_offset = 0
    else:
        print(f"Secondary VESC current position: {angle_2_offset:.2f}°")
    
    print(f"Using offsets - Primary: {angle_1_offset:.2f}°, Secondary: {angle_2_offset:.2f}°")

    displaced_length = np.zeros_like(solution)

    for i in range(1, len(solution)):
        displaced_length[i] = (solution[i] - solution[0]) * 1000 # mm
        
    displaced_angle = displaced_length * (360 / 200)  # deg/ mm 
    print(displaced_angle)
    curr_angle_1 = angle_1_offset
    curr_angle_2 = angle_2_offset

    print("Executing trajectory with VESC motors...")
    for i in range(len(displaced_angle)):
        target_angle_1 = curr_angle_1 + displaced_angle[i][0]
        target_angle_2 = curr_angle_2 + displaced_angle[i][1]
        
        await motor.set_pos(target_angle_1)
        await motor.set_pos(target_angle_2, can_id=119)
        await asyncio.sleep(dt)

    print("Trajectory execution complete!")

async def main():
    async with BleakClient(ADDRESS) as client:
        print(f"Connected: {client.is_connected}")

        # Try pairing (might not be necessary)
        try:
            paired = await client.pair(protection_level=2)
            print(f"Paired: {paired}")
        except Exception as e:
            print(f"Pairing failed or not required: {e}")

        motor = BluetoothVESC(client, RX_CHARACTERISTIC, TX_CHARACTERISTIC)
        print("Motor object created")

        # Set up notification handler for receiving responses
        await client.start_notify(TX_CHARACTERISTIC, motor.notification_handler)
        print("Notification handler set up")

        # Calculate trajectory
        goingUp = solve_traj(startp, endp, t_total, dt)
        goingDown = solve_traj(endp, startp, t_total, dt)

        # Run trajectory with real position offsets
        await run_traj(motor=motor, solution=goingUp, dt=dt)
        await run_traj(motor=motor, solution=goingDown, dt=dt)

        # Clean up
        await client.stop_notify(TX_CHARACTERISTIC)

if __name__ == "__main__":
    asyncio.run(main())