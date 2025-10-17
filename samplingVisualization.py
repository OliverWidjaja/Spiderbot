import matplotlib.pyplot as plt
import numpy as np
from bleTraj import solve_ik
from matplotlib.patches import Patch

def analyze_workspace_feasibility(grid_resolution=25, x_min=-1.2, x_max=1.2, y_min=0, y_max=1.2, 
                                  show_plots=True):
    x_points = np.linspace(x_min, x_max, grid_resolution)
    y_points = np.linspace(y_min, y_max, grid_resolution)
    
    X, Y = np.meshgrid(x_points, y_points)
    
    feasible_points = np.zeros((grid_resolution, grid_resolution), dtype=bool)
    phi_data = np.full((grid_resolution, grid_resolution), np.nan)

    masses = [0.2, 2, 4, 5, 6]
    mass_feasibility = np.zeros((len(masses), grid_resolution, grid_resolution), dtype=bool)

    print(f"Workspace: x=[{x_min*1000:.0f}-{x_max*1000:.0f}]mm, y=[{y_min*1000:.0f}-{y_max*1000:.0f}]mm")
    print(f"Grid resolution: {grid_resolution}x{grid_resolution} points")
    print("Discretizing workspace . . .")    

    for i, y in enumerate(y_points):
        for j, x in enumerate(x_points):
            try:
                result = solve_ik(x, y)
                feasible_points[i, j] = result['feasible']
                
                if result['feasible']:
                    phi_data[i, j] = result['phi']
                    
                for mass_idx, mass in enumerate(masses):
                    mass_result = solve_ik(x, y, m=mass)
                    mass_feasibility[mass_idx, i, j] = mass_result['feasible']
                        
            except Exception as e:
                print(f"Error at point ({x*1000:.1f}, {y*1000:.1f}): {e}")
                feasible_points[i, j] = False
                mass_feasibility[:, i, j] = False

    if show_plots:
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 5))

        ax1.pcolormesh(X*1000, Y*1000, feasible_points, shading='auto', cmap='RdBu', alpha=0.7)
        ax1.set_title('IK Feasibility Map (m=2 kg)')
        ax1.set_xlabel('X Position (mm)')
        ax1.set_ylabel('Y Position (mm)')
        ax1.grid(True, alpha=0.3)

        phi_map = ax2.pcolormesh(X*1000, Y*1000, np.degrees(phi_data), shading='auto', cmap='coolwarm')
        ax2.set_title('Platform Angle φ (degrees)')
        ax2.set_xlabel('X Position (mm)')
        ax2.set_ylabel('Y Position (mm)')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(phi_map, ax=ax2)
        
        mass_colors = ['blue', 'green', 'orange', 'red', 'purple'][:len(masses)]
        for mass_idx, mass in enumerate(masses):
            mass_data = mass_feasibility[mass_idx].astype(float)
            contourf = ax3.contourf(X*1000, Y*1000, mass_data, levels=[0.5, 1.5],
                                  colors=[mass_colors[mass_idx]], alpha=0.3)
            contour = ax3.contour(X*1000, Y*1000, mass_data, levels=[0.5], 
                                colors=mass_colors[mass_idx], linewidths=1)
            ax3.clabel(contour, inline=True, fontsize=8, fmt=f'{mass} kg')

        ax3.set_title('Feasible Mass Regions (Overlaid)')
        ax3.set_xlabel('X Position (mm)')
        ax3.set_ylabel('Y Position (mm)')
        ax3.grid(True, alpha=0.3)

        legend_elements = [Patch(facecolor=mass_colors[i], alpha=0.3, label=f'{masses[i]} kg') 
                         for i in range(len(masses))]
        ax3.legend(handles=legend_elements, loc='best')

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    analyze_workspace_feasibility()
