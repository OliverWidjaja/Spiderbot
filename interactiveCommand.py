from pyvesc import VESC
from scipy.optimize import least_squares
from vescCommands import move_motors
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import numpy as np
import time
import tkinter as tk
from tkinter import ttk
import threading
import queue

# Constants
m, g = 2000, 9.81  # Mass (gram), Gravity (m/s^2)
a1, a2 = np.array([80, 440]), np.array([1120, 440])  # Anchor positions (global)
b1, b2 = np.array([-117.5, 35]), np.array([117.5, 35])  # Cable extrusion points (local)

class CableRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Cable Robot Control")
        
        # Motor connection
        self.motor = None
        self.is_running = False
        self.is_visualizing = False
        self.command_queue = queue.Queue()
        
        # Create GUI elements
        self.create_widgets()
        
        # Setup plotting
        self.setup_plot()
        
        # Animation for real-time updates
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        
    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Input fields
        ttk.Label(main_frame, text="Start Position (x, y, phi):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.startp_entry = ttk.Entry(main_frame, width=20)
        self.startp_entry.grid(row=0, column=1, pady=5)
        self.startp_entry.insert(0, "600, 35, 0")
        
        ttk.Label(main_frame, text="End Position (x, y, phi):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.endp_entry = ttk.Entry(main_frame, width=20)
        self.endp_entry.grid(row=1, column=1, pady=5)
        self.endp_entry.insert(0, "1000, 100, 0")
        
        ttk.Label(main_frame, text="Total Time (s):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.t_total_entry = ttk.Entry(main_frame, width=20)
        self.t_total_entry.grid(row=2, column=1, pady=5)
        self.t_total_entry.insert(0, "5")
        
        ttk.Label(main_frame, text="Time Step (s):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.dt_entry = ttk.Entry(main_frame, width=20)
        self.dt_entry.grid(row=3, column=1, pady=5)
        self.dt_entry.insert(0, "1")
        
        # Buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=10)
        
        self.visualize_button = ttk.Button(button_frame, text="Visualize", command=self.visualize_trajectory)
        self.visualize_button.grid(row=0, column=0, padx=5)
        
        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_trajectory)
        self.start_button.grid(row=0, column=1, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_trajectory, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=2, padx=5)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Ready")
        self.status_label.grid(row=5, column=0, columnspan=2, pady=5)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
    def setup_plot(self):
        # Create figure for plotting
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Initialize plot elements
        self.ax.set_xlim(0, 1200)
        self.ax.set_ylim(0, 500)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('Cable Robot Trajectory')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        
        # Plot anchors
        self.ax.plot(a1[0], a1[1], 'ro', markersize=10, label='Anchor 1')
        self.ax.plot(a2[0], a2[1], 'ro', markersize=10, label='Anchor 2')
        
        # Initialize platform and cables
        self.platform, = self.ax.plot([], [], 'bo-', linewidth=2, markersize=8, label='Platform')
        self.cable1, = self.ax.plot([], [], 'k-', linewidth=1)
        self.cable2, = self.ax.plot([], [], 'k-', linewidth=1)
        
        # Initialize target trajectory
        self.trajectory, = self.ax.plot([], [], 'g--', linewidth=1, label='Target Trajectory')
        
        # Initialize visualization trajectory
        self.viz_trajectory, = self.ax.plot([], [], 'm-', linewidth=2, alpha=0.7, label='Visualization')
        
        self.ax.legend()
        
        # Current position marker
        self.current_pos, = self.ax.plot([], [], 'rx', markersize=10, label='Current Position')
        
        # Configure grid weights for the plot
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)
    
    def update_plot(self, frame):
        # This function will be called periodically to update the plot
        if hasattr(self, 'current_platform_pos'):
            x, y, phi = self.current_platform_pos
            
            # Calculate platform corners
            rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
            platform_corners = np.array([
                rot_mat @ np.array([-117.5, 35]) + [x, y],
                rot_mat @ np.array([117.5, 35]) + [x, y],
                rot_mat @ np.array([117.5, -35]) + [x, y],
                rot_mat @ np.array([-117.5, -35]) + [x, y],
                rot_mat @ np.array([-117.5, 35]) + [x, y]  # Close the rectangle
            ])
            
            # Update platform
            self.platform.set_data(platform_corners[:, 0], platform_corners[:, 1])
            
            # Update cables
            cable1_start = a1
            cable1_end = rot_mat @ b1 + [x, y]
            self.cable1.set_data([cable1_start[0], cable1_end[0]], [cable1_start[1], cable1_end[1]])
            
            cable2_start = a2
            cable2_end = rot_mat @ b2 + [x, y]
            self.cable2.set_data([cable2_start[0], cable2_end[0]], [cable2_start[1], cable2_end[1]])
            
            # Update current position marker
            self.current_pos.set_data([x], [y])
        
        return self.platform, self.cable1, self.cable2, self.current_pos
    
    def cable_lengths(self, x, y, phi):
        rot_mat = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        c1 = a1 - (rot_mat @ b1 + [x, y])
        c2 = a2 - (rot_mat @ b2 + [x, y])
        return np.linalg.norm(c1), np.linalg.norm(c2)
    
    def force_torque_eq(self, vars, x, y, phi):
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
    
    def solve_ik(self, x, y, phi, guess=[1000, 1000]):
        result = least_squares(lambda vars: self.force_torque_eq(vars, x, y, phi), guess)
        tens1, tens2 = result.x
        cable1, cable2 = self.cable_lengths(x, y, phi)
        residuals = self.force_torque_eq([tens1, tens2], x, y, phi)
        
        return {
            'tensions': (tens1, tens2),
            'cable_lengths': (cable1, cable2),
            'residuals': residuals,
            'feasible': tens1 > 0 and tens2 > 0
        }
    
    def visualize_trajectory(self):
        # Parse user inputs
        try:
            startp = np.array([float(x) for x in self.startp_entry.get().split(',')])
            endp = np.array([float(x) for x in self.endp_entry.get().split(',')])
            t_total = float(self.t_total_entry.get())
            dt = float(self.dt_entry.get())
        except ValueError:
            self.status_label.config(text="Invalid input values")
            return
        
        # Generate trajectory
        num_of_steps = int(t_total / dt) + 1
        time_steps = np.arange(0, num_of_steps) * dt
        p = startp + (endp - startp) * (time_steps / t_total).reshape(-1, 1)
        
        # Plot target trajectory
        self.trajectory.set_data(p[:, 0], p[:, 1])
        
        # Check feasibility of the entire trajectory
        feasible = True
        infeasible_points = []
        
        for i in range(num_of_steps):
            x, y, phi = p[i]
            sol = self.solve_ik(x, y, phi)
            
            if not sol['feasible']:
                feasible = False
                infeasible_points.append((x, y, phi, sol['tensions']))
                break
        
        if feasible:
            self.status_label.config(text="Trajectory is feasible. Ready to execute.")
            self.viz_trajectory.set_data(p[:, 0], p[:, 1])
            
            # Animate the visualization
            self.is_visualizing = True
            self.viz_points = p
            self.viz_index = 0
            self.animate_visualization()
        else:
            error_msg = f"Infeasible configuration at position: ({infeasible_points[0][0]:.1f}, {infeasible_points[0][1]:.1f}, {infeasible_points[0][2]:.1f})"
            self.status_label.config(text=error_msg)
            self.viz_trajectory.set_data([], [])
        
        self.canvas.draw()
    
    def animate_visualization(self):
        if self.is_visualizing and self.viz_index < len(self.viz_points):
            x, y, phi = self.viz_points[self.viz_index]
            self.current_platform_pos = (x, y, phi)
            self.viz_index += 1
            self.root.after(50, self.animate_visualization)  # Schedule next frame
        else:
            self.is_visualizing = False
    
    def start_trajectory(self):
        # Parse user inputs
        try:
            startp = np.array([float(x) for x in self.startp_entry.get().split(',')])
            endp = np.array([float(x) for x in self.endp_entry.get().split(',')])
            t_total = float(self.t_total_entry.get())
            dt = float(self.dt_entry.get())
        except ValueError:
            self.status_label.config(text="Invalid input values")
            return
        
        # Connect to motor if not already connected
        if self.motor is None:
            try:
                self.motor = VESC(serial_port='COM3')  # Adjust port as needed
            except Exception as e:
                self.status_label.config(text=f"Motor connection failed: {e}")
                return
        
        # Disable buttons, enable stop button
        self.visualize_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_label.config(text="Running...")
        
        # Generate trajectory
        num_of_steps = int(t_total / dt) + 1
        time_steps = np.arange(0, num_of_steps) * dt
        p = startp + (endp - startp) * (time_steps / t_total).reshape(-1, 1)
        
        # Plot target trajectory
        self.trajectory.set_data(p[:, 0], p[:, 1])
        self.viz_trajectory.set_data([], [])  # Clear visualization trajectory
        self.canvas.draw()
        
        # Start trajectory execution in a separate thread
        self.is_running = True
        thread = threading.Thread(target=self.execute_trajectory, args=(p, dt))
        thread.daemon = True
        thread.start()
    
    def stop_trajectory(self):
        self.is_running = False
        self.is_visualizing = False
        self.status_label.config(text="Stopping...")
        
        # Re-enable buttons, disable stop button
        self.visualize_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
    
    def execute_trajectory(self, p, dt):
        # Initialize arrays to store results
        num_of_steps = len(p)
        cable_lengths_over_time = np.zeros((num_of_steps, 2))
        tensions_over_time = np.zeros((num_of_steps, 2))
        feasibility_over_time = np.zeros(num_of_steps, dtype=bool)
        
        # Process each point in the trajectory
        for i in range(num_of_steps):
            if not self.is_running:
                self.command_queue.put(("status", "Stopped"))
                return
                
            x, y, phi = p[i]
            sol = self.solve_ik(x, y, phi)
            
            # Update current position for plotting
            self.current_platform_pos = (x, y, phi)
            
            # Check feasibility and stop if infeasible
            if not sol['feasible']:
                self.command_queue.put((
                    "error", 
                    f"Infeasible configuration at time {i*dt:.1f}s! " +
                    f"Position: ({x:.1f}, {y:.1f}, {phi:.1f}) " +
                    f"Tensions: {sol['tensions'][0]:.1f} N, {sol['tensions'][1]:.1f} N"
                ))
                self.is_running = False
                return
            
            # Store results
            cable_lengths_over_time[i] = sol['cable_lengths']
            tensions_over_time[i] = sol['tensions']
            feasibility_over_time[i] = sol['feasible']
            
            # Update status
            self.command_queue.put(("status", f"Step {i+1}/{num_of_steps}"))
            
            # Small delay to allow GUI updates
            time.sleep(0.01)
        
        # Calculate cable length differences between consecutive time steps
        cable_length_differences = np.diff(cable_lengths_over_time, axis=0) 
        
        # Execute the trajectory using move_motors
        self.command_queue.put(("status", "Executing trajectory with VESC motors..."))
        
        for i in range(len(cable_length_differences)):
            if not self.is_running:
                self.command_queue.put(("status", "Trajectory execution stopped"))
                return
                
            self.command_queue.put((
                "status", 
                f"Executing step {i+1}: " +
                f"ΔL1 = {cable_length_differences[i][0]:.3f} mm, " +
                f"ΔL2 = {cable_length_differences[i][1]:.3f} mm"
            ))
            
            # Execute motor movement
            try:
                move_motors(
                    self.motor,
                    length_1_mm=cable_length_differences[i][0],
                    length_2_mm=cable_length_differences[i][1],
                    T=dt,
                    dt=0.01,
                    can_id_2=119
                )
            except Exception as e:
                self.command_queue.put(("error", f"Motor command failed: {e}"))
                self.is_running = False
                return
            
            # Update current platform position for the next step
            if i < len(p) - 1:
                self.current_platform_pos = p[i+1]
            
            # Small delay to allow GUI updates
            time.sleep(dt)
        
        self.command_queue.put(("status", "Trajectory execution complete!"))
        self.is_running = False
        
        # Re-enable buttons, disable stop button
        self.command_queue.put(("reset_buttons", None))
    
    def process_queue(self):
        # Process messages from the command queue
        try:
            while True:
                msg_type, msg_content = self.command_queue.get_nowait()
                
                if msg_type == "status":
                    self.status_label.config(text=msg_content)
                elif msg_type == "error":
                    self.status_label.config(text=msg_content)
                    self.visualize_button.config(state=tk.NORMAL)
                    self.start_button.config(state=tk.NORMAL)
                    self.stop_button.config(state=tk.DISABLED)
                elif msg_type == "reset_buttons":
                    self.visualize_button.config(state=tk.NORMAL)
                    self.start_button.config(state=tk.NORMAL)
                    self.stop_button.config(state=tk.DISABLED)
                    
        except queue.Empty:
            pass
        
        # Schedule this method to run again after 100ms
        self.root.after(100, self.process_queue)

def main():
    root = tk.Tk()
    app = CableRobotGUI(root)
    
    # Start processing the command queue
    root.after(100, app.process_queue)
    
    # Start the GUI
    root.mainloop()

if __name__ == "__main__":
    main()