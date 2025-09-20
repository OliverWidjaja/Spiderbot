import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

# Constants
m, g = 2000, 9.81  # Mass (gram), Gravity (m/s^2)
a1, a2 = np.array([80, 440]), np.array([1120, 440])  # Anchor positions (global)
b1, b2 = np.array([-117.5, 35]), np.array([117.5, 35])  # Cable extrusion points (local)

# Trajectory parameters
endp = np.array([600, 500, 0])  # x, y, phi (global)
startp = np.array([600, 35, 0])
t_total = 5  # Total trajectory duration (s)
dt = 1  # ~Arbitrary checkpoints for trajectory
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

def create_trajectory_animation(p, time_steps, a1, a2, b1, b2):
    """
    Create an animation of the robot trajectory with cable lengths
    
    Parameters:
    p: array of robot positions and orientations [x, y, phi] over time
    time_steps: array of time values
    a1, a2: anchor positions
    b1, b2: cable extrusion points (local)
    """
    num_of_steps = len(p)
    
    # Initialize arrays to store results
    cable_lengths_over_time = np.zeros((num_of_steps, 2))
    tensions_over_time = np.zeros((num_of_steps, 2))
    feasibility_over_time = np.zeros(num_of_steps, dtype=bool)
    robot_positions = np.zeros((num_of_steps, 2))
    cable_endpoints = np.zeros((num_of_steps, 8))  # x1, y1, x1_attach, y1_attach, x2, y2, x2_attach, y2_attach

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
        robot_positions[i] = [x, y]
        
        # Calculate cable endpoints for visualization
        rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        cable_endpoints[i] = [
            a1[0], a1[1],  # Anchor 1 position
            x + rot_mat[0, 0]*b1[0] + rot_mat[0, 1]*b1[1],  # Cable 1 attachment point on robot
            y + rot_mat[1, 0]*b1[0] + rot_mat[1, 1]*b1[1],
            a2[0], a2[1],  # Anchor 2 position
            x + rot_mat[0, 0]*b2[0] + rot_mat[0, 1]*b2[1],  # Cable 2 attachment point on robot
            y + rot_mat[1, 0]*b2[0] + rot_mat[1, 1]*b2[1]
        ]
        
        # Print progress for each time step
        print(f"\nTime: {time_steps[i]:.1f}s, Position: ({x:.1f}, {y:.1f}, {phi:.1f})")
        print(f"  Cable lengths: {sol['cable_lengths'][0]:.3f} mm, {sol['cable_lengths'][1]:.3f} mm")
        print(f"  Tensions: {sol['tensions'][0]:.1f} N, {sol['tensions'][1]:.1f} N")
        print(f"  Feasible: {'Yes' if sol['feasible'] else 'No'}")

    # Create animation
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 500)
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.set_title('Cable-Driven Robot Trajectory Animation')
    ax.grid(True)

    # Draw anchor points
    ax.plot(a1[0], a1[1], 'ro', markersize=10, label='Anchor 1')
    ax.plot(a2[0], a2[1], 'ro', markersize=10, label='Anchor 2')

    # Initialize cables and robot
    cable1_line, = ax.plot([], [], 'b-', linewidth=2, label='Cable 1')
    cable2_line, = ax.plot([], [], 'g-', linewidth=2, label='Cable 2')
    robot = Rectangle((0, 0), 235, 70, angle=0, fill=True, color='orange', alpha=0.7)
    ax.add_patch(robot)

    # Text annotations for cable lengths and time
    length_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12,
                         bbox=dict(facecolor='white', alpha=0.7))
    time_text = ax.text(0.02, 0.88, '', transform=ax.transAxes, fontsize=12,
                       bbox=dict(facecolor='white', alpha=0.7))

    # Draw the trajectory path
    trajectory_line, = ax.plot(robot_positions[:, 0], robot_positions[:, 1], 'r--', alpha=0.5, label='Trajectory')

    # Add legend
    ax.legend(loc='upper right')

    def init():
        cable1_line.set_data([], [])
        cable2_line.set_data([], [])
        robot.set_xy((0, 0))
        robot.angle = 0
        length_text.set_text('')
        time_text.set_text('')
        return cable1_line, cable2_line, robot, length_text, time_text

    def animate(i):
        # Update cables
        x1, y1, x1_attach, y1_attach, x2, y2, x2_attach, y2_attach = cable_endpoints[i]
        cable1_line.set_data([x1, x1_attach], [y1, y1_attach])
        cable2_line.set_data([x2, x2_attach], [y2, y2_attach])
        
        # Update robot position and orientation
        x, y, phi = p[i]
        robot.set_xy((x - 117.5, y - 35))  # Center the robot
        robot.angle = np.degrees(phi)
        
        # Update text
        length_text.set_text(f'Cable 1: {cable_lengths_over_time[i, 0]:.1f} mm\nCable 2: {cable_lengths_over_time[i, 1]:.1f} mm')
        time_text.set_text(f'Time: {time_steps[i]:.1f} s')
        
        return cable1_line, cable2_line, robot, length_text, time_text

    # Create animation
    ani = FuncAnimation(fig, animate, frames=num_of_steps,
                        init_func=init, blit=True, interval=500, repeat=True)

    plt.tight_layout()
    
    return ani, fig, ax

# Main execution
if __name__ == "__main__":
    animation, fig, ax = create_trajectory_animation(p, time_steps, a1, a2, b1, b2)
    plt.show()
