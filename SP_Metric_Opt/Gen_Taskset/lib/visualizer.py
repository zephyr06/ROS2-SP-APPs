import numpy as np
import matplotlib.pyplot as plt
import os

def plot_3d_execution_time_surface(cfgs: dict, task_param: dict, output_path: str = None, draw: bool = False, is_mixture: bool = False) -> np.ndarray:
    """Draws a 3D surface plot representing execution time over coordinates."""
    # Matplotlib imports inside function to avoid heavy initial imports
    from mpl_toolkits.mplot3d import Axes3D
    
    # Get bounds
    X_MAX = int(task_param['D1_MAX'])
    X_MIN = int(task_param['D1_MIN'])
    Y_MAX = X_MAX
    Y_MIN = X_MIN

    # Construct the coordinate grid
    Et_result = np.zeros((X_MAX - X_MIN + 1, Y_MAX - Y_MIN + 1))
    
    # Import conditional sampling functions
    from .gmm_model import GaussianComponent, GMMTaskModel
    
    # Reconstruct and pre-fill GMM models for sampling
    if is_mixture:
        weights = task_param['weights']
        components = [GaussianComponent(None, None, coeffs=c['coeffs']) for c in task_param['tasks']]
        task_model = GMMTaskModel(
            components=components,
            weights=weights,
            period=task_param['period'],
            d1_min=X_MIN, d1_max=X_MAX, d1_sigma=task_param['D1_sigma'],
            d2_min=task_param['D2_MIN'], d2_max=task_param['D2_MAX'], d2_sigma=task_param['D2_sigma'],
            et_mean=task_param['Et_mean'], et_sigma=task_param['Et_sigma']
        )
    else:
        # Single Gaussian Task Component
        coeffs = task_param['coeffs']
        component = GaussianComponent(None, None, coeffs=coeffs)

    g_final_Et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])

    for x in range(X_MIN, X_MAX + 1):
        for y in range(Y_MIN, Y_MAX + 1):
            if is_mixture:
                # Use mean_only = True for a smooth surface plot
                et = task_model.sample_execution_time(
                    d1=float(x), d2=float(y),
                    final_et_range=g_final_Et_over_period_range,
                    et_min_2sigma=True,
                    mean_only=True
                )
            else:
                et = component.sample_conditional_execution_time(float(x), float(y), mean_only=True)

                min_Et = component.et_mean - 2 * component.et_sigma
                if et < min_Et:
                    et = min_Et
                if et < 1.0:
                    et = 1.0

            Et_result[x + X_MAX, y + Y_MAX] = et

    if draw or output_path is not None:
        X, Y = np.meshgrid(np.arange(X_MIN, X_MAX + 1), np.arange(Y_MIN, Y_MAX + 1))
        fig = plt.figure(figsize=(8, 6))
        plt.clf()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Et_result, cmap='viridis')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Execution Time (ms)')
        ax.set_title('GMM Execution Time Surface Map')

        if output_path is not None:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path)
        
        if draw:
            plt.show()
        else:
            plt.close()

    return Et_result

def plot_moving_trajectory(steps: list, stops: list, cfgs: dict, output_path: str = None, draw: bool = False) -> None:
    """Draws the path trajectory on the coordinate grid."""
    X_MIN = cfgs['D1_RANGE'][0]
    X_MAX = cfgs['D1_RANGE'][1]
    Y_MIN = cfgs['D2_RANGE'][0]
    Y_MAX = cfgs['D2_RANGE'][1]

    grid_size = stops[1][0] - stops[0][0]
    if grid_size == 0:
        grid_size = stops[1][1] - stops[0][1]
    if grid_size < 0:
        grid_size = -grid_size
    if grid_size == 0:
        grid_size = int((X_MAX - X_MIN) * 0.1)

    x_values, y_values = zip(*steps)

    plt.figure(figsize=(6, 6))
    plt.clf()
    plt.plot(x_values, y_values, marker='o', color='b', linestyle='-', markersize=2, alpha=0.5)
    
    # Plot stops
    stop_x, stop_y = zip(*stops)
    plt.scatter(stop_x, stop_y, color='r', marker='s', s=40, zorder=3, label='Stop Waypoints')

    plt.xlim(X_MIN, X_MAX)
    plt.ylim(Y_MIN, Y_MAX)
    plt.grid(True)
    plt.xticks(np.arange(X_MIN, X_MAX + 1, grid_size))
    plt.yticks(np.arange(Y_MIN, Y_MAX + 1, grid_size))

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Simulated Robot Moving Trajectory')
    plt.legend()
    plt.tight_layout()

    if output_path is not None:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path)

    if draw:
        plt.show()
    else:
        plt.close()
