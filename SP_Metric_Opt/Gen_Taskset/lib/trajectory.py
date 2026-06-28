import numpy as np
import random

def generate_stops_in_map(cfgs: dict, x_step_ratio: float = 0.1, y_step_ratio: float = 0.1) -> list[tuple[int, int]]:
    """Generates stop stations/waypoints on the 2D grid."""
    X_MIN = cfgs['D1_RANGE'][0]
    X_MAX = cfgs['D1_RANGE'][1]
    Y_MIN = cfgs['D2_RANGE'][0]
    Y_MAX = cfgs['D2_RANGE'][1]

    X_STEP = int((X_MAX - X_MIN) * x_step_ratio)
    Y_STEP = int((Y_MAX - Y_MIN) * y_step_ratio)
    if X_STEP < 1:
        X_STEP = 1
    if Y_STEP < 1:
        Y_STEP = 1

    stops = []
    for x in range(X_MIN, X_MAX + 1, int(X_STEP)):
        for y in range(Y_MIN, Y_MAX + 1, int(Y_STEP)):
            stops.append((x, y))

    return stops

def _apply_step(coord: float, delta: float, min_bound: float, max_bound: float) -> float:
    """Apply a step delta to a coordinate, clamping to [min_bound, max_bound].

    This helper exists so the clamping logic is testable in isolation.
    The returned value is a float to preserve sub-integer progress.
    """
    return max(min_bound, min(max_bound, coord + delta))


def generate_path_only(cfgs: dict, stops: list, n_steps: int = 1000, reverse_prob: float = 0.1) -> list[tuple[float, float]]:
    """Simulates a step-by-step path (moving trajectory) on the grid between stops."""
    N = len(stops)
    X_MIN = cfgs['D1_RANGE'][0]
    X_MAX = cfgs['D1_RANGE'][1]
    Y_MIN = cfgs['D2_RANGE'][0]
    Y_MAX = cfgs['D2_RANGE'][1]

    # Speed adjustment: step size per simulation move (default to 5 for exploration)
    step_size = cfgs.get("ROBOT_STEP_SIZE", 5.0)

    src_stop = random.randint(0, N - 1)    
    dst_stop = random.randint(0, N - 1)
    if dst_stop == src_stop:
        dst_stop = (src_stop + 1) % N
        
    curr = stops[src_stop]
    dst = stops[dst_stop]

    steps = []
    i = 0
    while i < n_steps:
        # Snap to destination stop if close enough to avoid overshoot oscillations
        snapped = False
        if abs(curr[0] - dst[0]) < step_size and abs(curr[1] - dst[1]) < step_size:
            curr = dst
            snapped = True

        if snapped or curr == dst:
            steps.append(curr)
            i += 1
            if i >= n_steps:
                break
            src_stop = dst_stop
            dst_stop = random.randint(0, N - 1)
            if dst_stop == src_stop:
                dst_stop = (src_stop + 1) % N
            curr = stops[src_stop]
            dst = stops[dst_stop]
            continue
        
        dy = dst[1] - curr[1]
        dx = dst[0] - curr[0]
        
        # probability of moving in each direction: 0-left, 1-up, 2-right, 3-down
        move_direction_p = [0.0, 0.0, 0.0, 0.0]
        if dx > 0:
            move_direction_p[2] = float(dx)
            move_direction_p[0] = float(dx * reverse_prob)
        elif dx < 0:
            move_direction_p[0] = float(-dx)
            move_direction_p[2] = float(-dx * reverse_prob)
        
        if dy > 0:
            move_direction_p[1] = float(dy)
            move_direction_p[3] = float(dy * reverse_prob)
        elif dy < 0:
            move_direction_p[3] = float(-dy)
            move_direction_p[1] = float(-dy * reverse_prob)
        
        # Add perpendicular noise to avoid perfectly straight horizontal/vertical lines and force turns
        if dx == 0 and dy != 0:
            perp_noise = float(abs(dy) * reverse_prob)
            move_direction_p[0] = perp_noise
            move_direction_p[2] = perp_noise
        elif dy == 0 and dx != 0:
            perp_noise = float(abs(dx) * reverse_prob)
            move_direction_p[1] = perp_noise
            move_direction_p[3] = perp_noise
        
        total_prob = sum(move_direction_p)
        if total_prob > 0.0:
            for k in range(4):
                move_direction_p[k] /= total_prob
        else:
            move_direction_p = [0.25, 0.25, 0.25, 0.25]
        
        # Sample direction
        chosen_components = np.random.choice(4, size=1, p=move_direction_p)
        move_direction = chosen_components[0]

        if move_direction == 0:
            # left
            curr = (_apply_step(curr[0], -step_size, X_MIN, X_MAX), curr[1])
        elif move_direction == 1:
            # up
            curr = (curr[0], _apply_step(curr[1], step_size, Y_MIN, Y_MAX))
        elif move_direction == 2:
            # right
            curr = (_apply_step(curr[0], step_size, X_MIN, X_MAX), curr[1])
        elif move_direction == 3:
            # down
            curr = (curr[0], _apply_step(curr[1], -step_size, Y_MIN, Y_MAX))

        steps.append(curr)
        i += 1

    return steps
