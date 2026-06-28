import numpy as np
import os

def generate_execution_time_trace(
    path_xys: list[tuple[int, int]],
    period: int,
    ms_per_move: int,
    n_steps: int,
    task_param: dict,
    cfgs: dict,
    dump_path: str = None
) -> tuple[list[tuple[int, int, float]], tuple[float, float]]:
    """Generates execution time samples along a coordinate path for a GMM task."""
    steps = []
    Et_min = 100000000.0
    Et_max = -1.0

    i = 0
    t_ms = 0
    path_idx = 0
    n_path = len(path_xys)
    
    # Pre-read GMM parameters for faster execution
    weights = task_param['weights']
    components_dicts = task_param['tasks']
    g_final_Et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])

    # Reconstruct GaussianComponent objects from dicts for sampling
    from .gmm_model import GaussianComponent
    components = []
    for c in components_dicts:
        # Reconstruct component from coeffs dictionary
        components.append(GaussianComponent(None, None, coeffs=c['coeffs']))

    from .gmm_model import GMMTaskModel
    task_model = GMMTaskModel(
        components=components,
        weights=weights,
        period=period,
        d1_min=task_param.get('D1_MIN', 0.0),
        d1_max=task_param.get('D1_MAX', 100.0),
        d1_sigma=task_param.get('D1_sigma', 25.0),
        d2_min=task_param.get('D2_MIN', 0.0),
        d2_max=task_param.get('D2_MAX', 360.0),
        d2_sigma=task_param.get('D2_sigma', 90.0),
        et_mean=task_param.get('Et_mean', 10.0),
        et_sigma=task_param.get('Et_sigma', 5.0)
    )

    while i < n_steps:
        if t_ms >= ms_per_move:
            t_ms -= ms_per_move    
            path_idx += 1
            if path_idx >= n_path:
                path_idx = 0
        else:
            t_ms += period

        curr = path_xys[path_idx]
        x = curr[0]
        y = curr[1]

        # Pass Cartesian coordinates directly; GMM coefficients are computed in Cartesian space.
        d1_cartesian = x
        d2_cartesian = y

        # Sample execution time
        if task_param.get('env_dependent', False):
            # Env-dependent tasks: use the full GMM with spatial position
            et = task_model.sample_execution_time(
                d1=d1_cartesian,
                d2=d2_cartesian,
                final_et_range=g_final_Et_over_period_range,
                et_min_2sigma=True
            )
        else:
            # Non-env tasks: fixed execution time from UUniFast allocation.
            # No spatial or random variation — deterministic per design.
            et = task_param.get('Et_mean', task_model.et_mean)

        Et_min = min(Et_min, et)
        Et_max = max(Et_max, et)
        steps.append((x, y, et))    
        i += 1

    if dump_path is not None:
        os.makedirs(os.path.dirname(dump_path), exist_ok=True)
        with open(dump_path, "w") as f:
            for x, y, et in steps:
                # Coordinates are kept as floats internally but rounded to ints
                # for the trace file so the C++ loader (which expects int, int, float)
                # continues to work without modification.
                xi = int(round(x))
                yi = int(round(y))
                f.write(f"{xi},{yi},{et:.3f}\n")

    return steps, (float(Et_min), float(Et_max))
