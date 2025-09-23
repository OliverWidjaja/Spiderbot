import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from trajectoryCommand import solve_ik, startp, endp, t_total, dt, a1, a2, b1, b2

def animate_trajectory(start_pos=startp, end_pos=endp, t_total=t_total, dt=dt):
    if start_pos is None:
        start_pos = startp
    if end_pos is None:
        end_pos = endp
    
    
    # Calculate trajectory points
    num_of_steps = int(t_total / dt) + 1
    time_steps = np.arange(0, num_of_steps) * dt
    p = start_pos + (end_pos - start_pos) * (time_steps / t_total).reshape(-1, 1)
    
    # Pre-calculate all positions and robot outlines
    positions = []
    cable_starts = []
    feasible_points = []
    robot_outlines = []  # Store the robot outline for each frame
    
    # Define robot outline points in local frame (rectangle around b1 and b2)
    # Assuming the robot is a rectangle with b1 and b2 as cable attachment points
    robot_width = abs(b1[0] - b2[0]) + 0.05  # Width with some padding
    robot_height = 0.07  # Arbitrary height for visualization
    robot_corners_local = np.array([
        [b1[0], b1[1] - robot_height],  # Bottom-left
        [b2[0], b2[1] - robot_height],  # Bottom-right
        [b2[0], b2[1]],  # Top-right
        [b1[0], b1[1]],  # Top-left
        [b1[0], b1[1] - robot_height]   # Close the rectangle
    ])
    
    for i in range(num_of_steps):
        x, y = p[i]
        sol = solve_ik(x, y)
        phi = sol['phi']
        
        # Calculate robot orientation vector
        rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        
        # Calculate cable attachment points on robot
        cable1_start = np.array([x, y]) + rot_mat @ b1
        cable2_start = np.array([x, y]) + rot_mat @ b2
        
        # Calculate robot outline in global frame
        robot_outline_global = []
        for corner in robot_corners_local:
            global_corner = np.array([x, y]) + rot_mat @ corner
            robot_outline_global.append(global_corner)
        robot_outline_global = np.array(robot_outline_global)
        
        positions.append([x, y])
        cable_starts.append([cable1_start, cable2_start])
        feasible_points.append(sol['feasible'])
        robot_outlines.append(robot_outline_global)
    
    # Convert to numpy arrays for easier indexing
    positions = np.array(positions)
    cable_starts = np.array(cable_starts)
    feasible_points = np.array(feasible_points)
    robot_outlines = np.array(robot_outlines)
    
    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Set up the plot limits with some margin
    margin = 0.1
    x_min = min(positions[:,0].min(), a1[0], a2[0]) - margin
    x_max = max(positions[:,0].max(), a1[0], a2[0]) + margin
    y_min = min(positions[:,1].min(), a1[1], a2[1]) - margin
    y_max = max(positions[:,1].max(), a1[1], a2[1]) + margin
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.grid(True)
    ax.set_aspect('equal')

    ax.set_title('Cable-Suspended Robot Trajectory Visualization', fontsize=16, pad=20)
    
    # Plot fixed elements (anchor points)
    ax.plot(a1[0], a1[1], 'ko', markersize=8, label='Anchor 1')
    ax.plot(a2[0], a2[1], 'ko', markersize=8, label='Anchor 2')
    ax.text(a1[0], a1[1], ' A1', verticalalignment='center')
    ax.text(a2[0], a2[1], ' A2', verticalalignment='center')
    
    # Initialize animated elements
    trajectory_line, = ax.plot([], [], 'b-', alpha=0.5, linewidth=2, label='Trajectory')
    robot_point, = ax.plot([], [], 'ro', markersize=10, label='Robot Center')
    cable1_line, = ax.plot([], [], 'g-', linewidth=2, label='Cable 1')
    cable2_line, = ax.plot([], [], 'r-', linewidth=2, label='Cable 2')
    robot_outline, = ax.plot([], [], 'k-', linewidth=2, label='Robot Outline')
    
    status_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                       verticalalignment='top')
    
    # Initialize the animation
    def init():
        trajectory_line.set_data([], [])
        robot_point.set_data([], [])
        cable1_line.set_data([], [])
        cable2_line.set_data([], [])
        robot_outline.set_data([], [])
        status_text.set_text('')
        return trajectory_line, robot_point, cable1_line, cable2_line, robot_outline, status_text
    
    # Animation update function
    def update(frame):
        # Update trajectory line (path up to current frame)
        trajectory_line.set_data(positions[:frame+1, 0], positions[:frame+1, 1])
        
        # Update robot position
        robot_point.set_data([positions[frame, 0]], [positions[frame, 1]])
        
        # Update robot outline
        robot_outline.set_data(robot_outlines[frame, :, 0], robot_outlines[frame, :, 1])
        
        # Update cables
        cable1_line.set_data([cable_starts[frame, 0, 0], a1[0]], 
                            [cable_starts[frame, 0, 1], a1[1]])
        cable2_line.set_data([cable_starts[frame, 1, 0], a2[0]], 
                            [cable_starts[frame, 1, 1], a2[1]])
        
        # Update time text and title
        current_time = frame * dt
        feasibility = "Feasible" if feasible_points[frame] else "Infeasible"
        status_text.set_text(f'Status: {feasibility}\nTime: {current_time:.2f}/{t_total} s')

        if feasibility == "Infeasible":
            robot_outline.set_color('brown')
        else:
            robot_outline.set_color('black')

        return trajectory_line, robot_point, cable1_line, cable2_line, robot_outline, status_text
    
    # Create the animation
    anim = animation.FuncAnimation(fig, update, frames=num_of_steps,
                                  init_func=init, blit=True, interval=dt*1000, repeat=True)
    
    # Add legend
    ax.legend(loc='lower right')
    
    plt.tight_layout()
    fig.canvas.manager.window.resizable(False, False)
    plt.show()
    
    return anim

if __name__ == "__main__":
    # startp = np.array([0.6, 0.035])
    # endp = np.array([0.5, 0.4])

    animate_trajectory(start_pos=startp, end_pos=endp, t_total=t_total, dt=dt)