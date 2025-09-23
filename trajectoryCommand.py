from pyvesc import VESC
from scipy.optimize import least_squares
from vescCommands import move_motors
import numpy as np
import time


# Constants
m, g = 2, 9.81  # Mass (kg), Gravity (m/s^2)
a1, a2 = np.array([80/1000, 440/1000]), np.array([1120/1000, 440/1000])  # Anchor positions in global frame (m)
b1, b2 = np.array([-117.5/1000, 35/1000]), np.array([117.5/1000, 35/1000])  # Cable extrusion points in local frame (m)

# Trajectory parameters
endp = np.array([500/1000, 370/1000])  # x, y (m, m)
startp = np.array([600/1000, 35/1000])  # x, y (m, m)

# Time parameters
t_total = 5
dt = 1
num_of_steps = int(t_total / dt) + 1
time_steps = np.arange(0, num_of_steps) * dt

# Generate trajectory
p = startp + (endp - startp) * (time_steps / t_total).reshape(-1, 1)

def cable_lengths(x, y, phi):
    rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    c1 = a1 - (rot_mat @ b1 + [x, y])
    c2 = a2 - (rot_mat @ b2 + [x, y])
    return np.linalg.norm(c1), np.linalg.norm(c2) # returns vector of cable 1 and 2 lengths in meters

def force_torque_eq(vars, x, y):
    tens1, tens2, phi = vars[0], vars[1], vars[2]
    rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    
    # Cable vectors and norms
    c1 = a1 - (rot_mat @ b1 + [x, y])
    c2 = a2 - (rot_mat @ b2 + [x, y])
    c1_norm = c1 / np.linalg.norm(c1)
    c2_norm = c2 / np.linalg.norm(c2)
    
    # Force balance
    fx = tens1 * c1_norm[0] + tens2 * c2_norm[0]
    fy = tens1 * c1_norm[1] + tens2 * c2_norm[1] - m * g
    
    # Torque balance (z-component only)
    b1_rot = np.append(rot_mat @ b1, 0)
    b2_rot = np.append(rot_mat @ b2, 0)
    f1 = np.append(c1_norm * tens1, 0)
    f2 = np.append(c2_norm * tens2, 0)
    torque = np.cross(b1_rot, f1)[2] + np.cross(b2_rot, f2)[2]

    return np.array([fx, fy, torque]) # residuals for force and torque balance

def solve_ik(x, y, guess=np.array([10, 10, 1e-3])):
    # Add reasonable bounds: tensions > 0, small φ range
    bounds = ([0, 0, -np.pi/2], [np.inf, np.inf, np.pi/2])  # φ ∈ [-90°, 90°]
    result = least_squares(lambda vars: force_torque_eq(vars, x, y), guess, bounds=bounds)
    tens1, tens2, phi = result.x
    cable1, cable2 = cable_lengths(x, y, phi)
    residuals = force_torque_eq([tens1, tens2, phi], x, y)

    return {
        'tensions': np.array([tens1, tens2]),
        'cable_lengths': np.array([cable1, cable2]),
        'phi': phi,
        'residuals': residuals,
        'feasible': tens1 > 0 and tens2 > 0 and np.linalg.norm(residuals) < 1e-2
    }

def execute_trajectory(motor: VESC):
    # Initialize arrays to store results
    cable_lengths_over_time = np.zeros((num_of_steps, 2))
    
    # Process each point in the trajectory
    for i in range(num_of_steps):
        x, y = p[i]
        sol = solve_ik(x, y)

        print(f"Step {i}: Pos: ({x:.3f}, {y:.3f}), φ: {sol['phi']:.2f} rad ({np.degrees(sol['phi']):.1f}°)")
        
        # Check feasibility and stop if infeasible
        if not sol['feasible']:
            print(f"\n❌ WARNING: Infeasible configuration detected at time {time_steps[i]:.1f}s!")
            print(f"Point ({x}, {y}):")
            print(f"  Cable Lengths: {sol['cable_lengths']} m")
            print(f"  Tensions: {sol['tensions']} N")
            print(f"  φ: {sol['phi']} rad ({np.degrees(sol['phi'])}°)")
            print(f"  Residuals: {sol['residuals']} (N, N, Nm)")
            print(f"  Feasible: {sol['feasible']}\n")
            return
        
        cable_lengths_over_time[i] = sol['cable_lengths']
    
    # Calculate cable length differences between consecutive time steps
    cable_length_differences = np.diff(cable_lengths_over_time, axis=0) 
    
    # Execute the trajectory using move_motors
    print("\nExecuting trajectory with VESC motors...")
    # for i in range(len(cable_length_differences)):
    #     print(f"Executing step {i+1}: ΔL1 = {cable_length_differences[i][0] * 1000:.3f} mm, ΔL2 = {cable_length_differences[i][1] * 1000:.3f} mm")
    #     move_motors(
    #         motor,
    #         length_1_mm=cable_length_differences[i][0] * 1000,  # Convert to mm
    #         length_2_mm=cable_length_differences[i][1] * 1000,  # Convert to mm
    #         T=dt,  # ~How many times move_motors is called per second
    #         dt=0.01,  # Fine control within each step
    #         can_id_2=119
    #     )

    print("Trajectory execution complete!")

def hold_motor(motor: VESC, duration=10):
    print(f"Holding motor position for {duration} seconds...")

    while True:
        try:
            angle_1 = motor.get_pos()
            angle_2 = motor.get_pos(can_id=119)
            break
        except Exception as e:
            print(f"Error getting motor positions: {e}")
            print("Retrying ...")
            time.sleep(0.1)
    
    start_time = time.time()
    while (time.time() - start_time < duration):
        motor.set_pos(angle_1)
        motor.set_pos(angle_2, can_id=119)


    print("Hold complete.")

def test_solve_ik():
    test_points = [
        (600/1000, 35/1000),
        (800/1000, 100/1000),
        (1000/1000, 440/1000),
        (1200/1000, 300/1000),
        (1400/1000, 400/1000)
    ]
    
    for x, y in test_points:
        sol = solve_ik(x, y)
        print(f"Point ({x}, {y}):")
        print(f"  Cable Lengths: {sol['cable_lengths']} m")
        print(f"  Tensions: {sol['tensions']} N")
        print(f"  φ: {sol['phi']} rad ({np.degrees(sol['phi'])}°)")
        print(f"  Residuals: {sol['residuals']} (N, N, Nm)")
        print(f"  Feasible: {sol['feasible']}\n")

    exit(0)

if __name__ == "__main__":
    # motor = VESC(serial_port='COM3')

    # test_solve_ik()

    execute_trajectory(motor=None)
    # execute_trajectory(motor=motor)    
    # hold_motor(motor=motor, duration=10)
