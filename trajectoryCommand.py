from pyvesc import VESC
from scipy.optimize import least_squares
from vescCommands import move_motors
from matplotGraph import create_trajectory_animation
import matplotlib.pyplot as plt
import numpy as np
import time


# Constants
m, g = 2000, 9.81  # Mass (gram), Gravity (m/s^2)
a1, a2 = np.array([80, 440]), np.array([1120, 440])  # Anchor positions (global)
b1, b2 = np.array([-117.5, 35]), np.array([117.5, 35])  # Cable extrusion points (local)

# Trajectory parameters
endp = np.array([1000, 100, 0])  # x, y, phi (global)
startp = np.array([600, 35, 0])
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
    return np.linalg.norm(c1), np.linalg.norm(c2)

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
    
    return np.array([fx, fy, torque])

def solve_ik(x, y, guess=np.array([10, 10, 1e-3])):
    result = least_squares(lambda vars: force_torque_eq(vars, x, y), guess)
    tens1, tens2, phi = result.x
    cable1, cable2 = cable_lengths(x, y, phi)
    residuals = force_torque_eq([tens1, tens2, phi], x, y)

    return {
        'tensions': np.array([tens1, tens2]),
        'cable_lengths': np.array([cable1, cable2]),
        'residuals': residuals,
        'feasible': tens1 > 0 and tens2 > 0 and np.linalg.norm(residuals) < 1e-2
    }

def execute_trajectory(motor: VESC):
    # Initialize arrays to store results
    cable_lengths_over_time = np.zeros((num_of_steps, 2))
    tensions_over_time = np.zeros((num_of_steps, 2))
    feasibility_over_time = np.zeros(num_of_steps, dtype=bool)
    
    # Process each point in the trajectory
    for i in range(num_of_steps):
        x, y, phi = p[i]
        sol = solve_ik(x, y, phi)
        
        # Check feasibility and stop if infeasible
        if not sol['feasible']:
            print(f"\n❌ WARNING: Infeasible configuration detected at time {time_steps[i]:.1f}s!")
            print(f"   Position: ({x:.1f}, {y:.1f}, {phi:.1f})")
            print(f"   Tensions: {sol['tensions'][0]:.1f} N, {sol['tensions'][1]:.1f} N")
            print("   At least one cable has negative tension (physically impossible)")
            print("   The trajectory cannot be executed with this cable configuration.")
            return
        
        # Store results
        cable_lengths_over_time[i] = sol['cable_lengths']
        tensions_over_time[i] = sol['tensions']
        feasibility_over_time[i] = sol['feasible']

    # Print the results
    print("\nCable lengths over time:")
    print(cable_lengths_over_time)
    
    # Calculate cable length differences between consecutive time steps
    cable_length_differences = np.diff(cable_lengths_over_time, axis=0) 
    
    # Execute the trajectory using move_motors
    print("\nExecuting trajectory with VESC motors...")
    for i in range(len(cable_length_differences)):
        print(f"Executing step {i+1}: ΔL1 = {cable_length_differences[i][0]:.3f} mm, ΔL2 = {cable_length_differences[i][1]:.3f} mm")
        move_motors(
            motor,
            length_1_mm=cable_length_differences[i][0],
            length_2_mm=cable_length_differences[i][1],
            T=dt,  # ~How many times move_motors is called per second
            dt=0.01,  # Fine control within each step
            can_id_2=119
        )

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
        (600, 35),
        (800, 100),
        (1000, 200),
        (1200, 300),
        (1400, 400)
    ]
    
    for x, y in test_points:
        sol = solve_ik(x, y)
        print(f"Point ({x}, {y}):")
        print(f"  Cable Lengths: {sol['cable_lengths']}")
        print(f"  Tensions: {sol['tensions']}")
        print(f"  Residuals: {sol['residuals']}")
        print(f"  Feasible: {sol['feasible']}\n")
    exit(0)

if __name__ == "__main__":
    # motor = VESC(serial_port='COM3')
    test_solve_ik()

    animation, fig, ax = create_trajectory_animation(p, time_steps, a1, a2, b1, b2)
    plt.show()

    # execute_trajectory(motor=motor)
    
    # hold_motor(motor=motor, duration=10)
