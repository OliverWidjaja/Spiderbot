import numpy as np
from scipy.optimize import least_squares


# Constants
m, g = 2000, 9.81  # Mass (gram), Gravity (m/s^2)
a1, a2 = np.array([80, 440]), np.array([1120, 440])  # Anchor positions (global)
b1, b2 = np.array([-117.5, 35]), np.array([117.5, 35])  # Cable extrusion points (local)

# Trajectory parameters
endp = np.array([600, 50, 0])  # x, y, phi (global)
startp = np.array([600, 35, 0])
t_total = 5 # Total trajectory duration (s)
dt = 1 # ~Arbitrary checkpoints for trajectory
num_of_steps = int(t_total / dt) + 1
time_steps = np.arange(0, num_of_steps) * dt

p = startp + (endp - startp) * (time_steps / t_total).reshape(-1, 1)

def cable_lengths(x, y, phi):
    """Calculate cable lengths for given position and orientation"""
    rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    c1 = a1 - (rot_mat @ b1 + [x, y])
    c2 = a2 - (rot_mat @ b2 + [x, y])
    return np.linalg.norm(c1), np.linalg.norm(c2)

def force_torque_eq(vars, x, y, phi):
    """Force and torque balance equations"""
    tens1, tens2 = vars
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
    
    return [fx, fy, torque]

def solve_ik(x, y, phi, guess=[1000, 1000]):
    """Complete inverse kinematics solution"""
    result = least_squares(lambda vars: force_torque_eq(vars, x, y, phi), guess)
    tens1, tens2 = result.x
    cable1, cable2 = cable_lengths(x, y, phi)
    residuals = force_torque_eq([tens1, tens2], x, y, phi)
    
    return {
        'tensions': (tens1, tens2),
        'cable_lengths': (cable1, cable2),
        'residuals': residuals,
        'feasible': tens1 > 0 and tens2 > 0
    }

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
        exit(1)
    
    # Store results
    cable_lengths_over_time[i] = sol['cable_lengths']
    tensions_over_time[i] = sol['tensions']
    feasibility_over_time[i] = sol['feasible']
    
    # Print progress for each time step
    print(f"\nTime: {time_steps[i]:.1f}s, Position: ({x:.1f}, {y:.1f}, {phi:.1f})")
    print(f"  Cable lengths: {sol['cable_lengths'][0]:.3f} mm, {sol['cable_lengths'][1]:.3f} mm")
    print(f"  Tensions: {sol['tensions'][0]:.1f} N, {sol['tensions'][1]:.1f} N")
    print(f"  Feasible: {'Yes' if sol['feasible'] else 'No'}")

print("\nCable lengths over time:")
print(cable_lengths_over_time)

# Calculate cable length differences between consecutive time steps
cable_length_differences = np.diff(cable_lengths_over_time, axis=0)

# Print the results
print("\nCable length differences over time:")
for i in range(len(cable_length_differences)):
    print(f"Time interval {i+1}: ΔL1 = {cable_length_differences[i][0]:.3f} mm, ΔL2 = {cable_length_differences[i][1]:.3f} mm")
