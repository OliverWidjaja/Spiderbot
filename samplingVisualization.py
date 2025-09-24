import matplotlib.pyplot as plt
import numpy as np
from trajectoryCommand import solve_ik

def analyze_workspace_feasibility(grid_resolution=100, x_min=0, x_max=1.2, y_min=0, y_max=0.6, 
                                  show_plots=True, return_data=False):
    # Discretize the workspace
    x_points = np.linspace(x_min, x_max, grid_resolution)
    y_points = np.linspace(y_min, y_max, grid_resolution)
    
    # Create meshgrid for plotting
    X, Y = np.meshgrid(x_points, y_points)
    
    # Arrays to store feasibility results
    feasible_points = np.zeros((grid_resolution, grid_resolution), dtype=bool)
    tension_data = np.zeros((grid_resolution, grid_resolution, 2))  # Store both tensions
    phi_data = np.zeros((grid_resolution, grid_resolution))  # Store phi values
    
    print("Discretizing workspace and evaluating IK feasibility...")
    print(f"Workspace: x=[{x_min*1000:.0f}-{x_max*1000:.0f}]mm, y=[{y_min*1000:.0f}-{y_max*1000:.0f}]mm")
    print(f"Grid resolution: {grid_resolution}x{grid_resolution} points")
    
    # Evaluate IK at each grid point
    for i, y in enumerate(y_points):
        for j, x in enumerate(x_points):
            try:
                result = solve_ik(x, y)
                feasible_points[i, j] = result['feasible']
                tension_data[i, j] = result['tensions']
                phi_data[i, j] = result['phi']
            except Exception as e:
                print(f"Error at point ({x*1000:.1f}, {y*1000:.1f}): {e}")
                feasible_points[i, j] = False
    
    # Calculate feasibility statistics
    total_points = grid_resolution * grid_resolution
    feasible_count = np.sum(feasible_points)
    feasibility_percentage = (feasible_count / total_points) * 100
    
    print(f"\nFeasibility Results:")
    print(f"Total points evaluated: {total_points}")
    print(f"Feasible points: {feasible_count}")
    print(f"Feasibility rate: {feasibility_percentage:.1f}%")
    
    if show_plots:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
        
        feasible_map = ax1.pcolormesh(X*1000, Y*1000, feasible_points, 
                                     shading='auto', cmap='RdYlGn', alpha=0.7)
        ax1.set_title('IK Feasibility Map\n(Green = Feasible, Red = Infeasible)')
        ax1.set_xlabel('X Position (mm)')
        ax1.set_ylabel('Y Position (mm)')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(feasible_map, ax=ax1, label='Feasible')
        
        phi_map = ax2.pcolormesh(X*1000, Y*1000, np.degrees(phi_data), 
                                shading='auto', cmap='coolwarm')
        ax2.set_title('Platform Angle φ (degrees)')
        ax2.set_xlabel('X Position (mm)')
        ax2.set_ylabel('Y Position (mm)')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(phi_map, ax=ax2)
        
        # Overlay feasible points as scatter points on all plots
        feasible_x = X[feasible_points] * 1000
        feasible_y = Y[feasible_points] * 1000
        
        for ax in [ax1, ax2]:
            ax.scatter(feasible_x, feasible_y, color='black', s=10, alpha=0.7, 
                       label='Feasible Points')
            if ax == ax1:  # Only add legend to first plot
                ax.legend()
        
        plt.tight_layout()
        plt.show()
    
    if return_data:
        results = {
            'feasible_points': feasible_points,
            'tension_data': tension_data,
            'phi_data': phi_data,
            'X_grid': X,
            'Y_grid': Y,
            'statistics': {
                'total_points': total_points,
                'feasible_count': feasible_count,
                'feasibility_percentage': feasibility_percentage
            },
            'parameters': {
                'grid_resolution': grid_resolution,
                'x_bounds': (x_min, x_max),
                'y_bounds': (y_min, y_max)
            }
        }
        return results

# Example usage when run directly
if __name__ == "__main__":
    # Run with default parameters
    analyze_workspace_feasibility()
    
    # Example of custom usage:
    # results = analyze_workspace_feasibility(
    #     grid_resolution=50,
    #     x_min=0.1, x_max=1.0,
    #     y_min=0.1, y_max=0.5,
    #     show_plots=True,
    #     return_data=True
    # )
    