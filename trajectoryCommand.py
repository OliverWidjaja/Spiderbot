from pyvesc import VESC
from scipy.optimize import least_squares
import numpy as np
import time

# Parameters
g = 9.81
a1, a2 = np.array([80/1000, 440/1000]), np.array([1120/1000, 440/1000])  # Anchor positions in global frame (m)
b1, b2 = np.array([-117.5/1000, 35/1000]), np.array([117.5/1000, 35/1000])  # Cable extrusion points in local frame (m)

# Variables
mass = 2
endp = np.array([600/1000, 150/1000])  # x, y (m, m)
startp = np.array([600/1000, 35/1000])  # x, y (m, m)
t_total = 4
dt = 0.01

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
    bounds = ([0, 0, -np.pi/2],  # Lower bounds for tens1, tens2, and φ
              [44, 44, np.pi/2])   # Upper bounds for tens1, tens2, and φ
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

def run_traj(motor: VESC, solution):
    displaced_length = np.zeros_like(solution)

    for i in range(1, len(solution)):
        displaced_length[i] = (solution[i] - solution[0]) * 1000 # mm
        if abs(displaced_length[i][0] - displaced_length[i-1][0]) < 0.1 or \
           abs(displaced_length[i][1] - displaced_length[i-1][1]) < 0.1:
            print("Step diff < 0.1 mm! Too small to execute.")
            return
        
    displaced_angle = displaced_length * (360 / 200)  # deg/ mm 

    while True:
        try:
            angle_1_offset = motor.get_pos()
            angle_2_offset = motor.get_pos(can_id=119)
            break
        except Exception as e:
            print(f"Error getting initial motor positions: {e}")
            print("Retrying ...")
            time.sleep(0.01)

    curr_angle_1 = angle_1_offset
    curr_angle_2 = angle_2_offset

    print("Executing trajectory with VESC motors...")
    for i in range(len(displaced_angle)):
        motor.set_pos(curr_angle_1 + displaced_angle[i][0])
        motor.set_pos(curr_angle_2 + displaced_angle[i][1], can_id=119)
        time.sleep(dt)

    print("Trajectory execution complete!")

def hold_motor(motor: VESC, duration):
    print(f"Holding motor position for {duration} seconds...")

    while True:
        try:
            angle_1 = motor.get_pos()
            angle_2 = motor.get_pos(can_id=119)
            break
        except Exception as e:
            print(f"Error getting motor positions: {e}")
            print("Retrying ...")
            time.sleep(0.01)
    
    start_time = time.time()
    while (time.time() - start_time < duration):
        loop_start = time.time()
        motor.set_pos(angle_1)
        motor.set_pos(angle_2, can_id=119)
        elapsed = time.time() - loop_start
        time.sleep(max(0, 0.01 - elapsed))  # Adjust sleep to maintain timing

    print("Hold complete.")

if __name__ == "__main__":
    motor = VESC(serial_port='COM3')

    # Calculate solution in-advance
    traj1 = solve_traj(startp, endp, t_total, dt)
    traj2 = solve_traj(endp, startp, t_total, dt)

    # Execute trajectories
    run_traj(motor=motor, solution=traj1)
    hold_motor(motor=motor, duration=5)
    run_traj(motor=motor, solution=traj2)
