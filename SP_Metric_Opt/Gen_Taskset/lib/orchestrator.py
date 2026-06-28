import os
import math
import sys
import numpy as np
import yaml
import matplotlib.pyplot as plt

from .generation_config_parser import load_generation_config
from .yaml_exporter import convert_taskset_parameters_to_cpp_yaml, export_taskset_to_yaml
from .taskset_generator import generate_taskset_parameters, load_and_fill_taskset_param_file
from .trajectory import generate_stops_in_map, generate_path_only
from .trace_generator import generate_execution_time_trace
from .visualizer import plot_moving_trajectory

# Interval settings
UPDATE_INTERVAL_S = 10


def _compute_hyper_period(periods_ms: list[int]) -> int:
    """Return the least common multiple (LCM) of task periods in milliseconds."""
    # Try native math.lcm (Python 3.9+), else manual fallback
    try:
        return math.lcm(*periods_ms)
    except AttributeError:
        hp = 1
        for p in periods_ms:
            hp = (hp * p) // math.gcd(hp, p)
        return hp


def generate_additional_execution_traces(
    cfgs: dict,
    dir_path: str,
    n_path_per_task: int,
    n_inst_per_path: int,
    path_idx: int = None,
    n_sec: int = 1000,
    add_perf_records: bool = True,
    interact: bool = False
) -> None:
    """Appends additional path and execution time trace files for an existing taskset."""
    update_interval_s = cfgs.get("UPDATE_INTERVAL_S", 10)
    n_ms = n_sec * 1000
    n_intervals = max(1, math.ceil(n_sec / update_interval_s))

    # 1. Figure out start path_idx (e.g. path_0.png, path_1.png)
    if path_idx is None:
        i = 0
        while True:
            fpath = os.path.join(dir_path, f'path_{i}.png')
            if not os.path.exists(fpath):
                break
            i += 1
        path_idx = i

    # 2. Reload parameters from taskset_param.yaml
    param_fpath = os.path.join(dir_path, 'taskset_param.yaml')
    params = load_and_fill_taskset_param_file(param_fpath)

    # 3. Generate stops on the map
    stops = generate_stops_in_map(cfgs, x_step_ratio=0.2, y_step_ratio=0.2)

    # Determine maximum period
    prd_max = 0
    for task in params['tasks']:
        prd = task['period']
        if prd > prd_max:
            prd_max = prd

    # Physical parameters: derive step size from speed so that
    #   step_m = speed_mps * (prd_max / 1000) [meters per step]
    #   total_distance = n_steps * step_m = (n_sec * 1000 / prd_max) * step_m
    #                  = n_sec * speed_mps
    speed_mps = cfgs.get("ROBOT_SPEED_MPS", 1.0)
    step_m = speed_mps * prd_max / 1000.0
    cfgs['ROBOT_STEP_SIZE'] = step_m

    # If continuing path trace generation, fill existing bounds
    if path_idx > 0:
        taskchar_fpath = os.path.join(dir_path, 'taskset_characteristics.yaml')
        if os.path.exists(taskchar_fpath):
            with open(taskchar_fpath, "r") as f:
                taskchar_param = yaml.safe_load(f)
            if taskchar_param is not None:
                n_tasks = len(taskchar_param['tasks'])
                for i in range(n_tasks):
                    if 'execution_time_min' in taskchar_param['tasks'][i] and 'execution_time_max' in taskchar_param['tasks'][i]:
                        params['tasks'][i]['Et_min'] = taskchar_param['tasks'][i]['execution_time_min']
                        params['tasks'][i]['Et_max'] = taskchar_param['tasks'][i]['execution_time_max']

    n_tasks = len(params['tasks'])
    task_Ets = []  # To compute average task stats per interval
    for i in range(n_tasks):
        task_Ets.append([])
        for _ in range(n_intervals):
            task_Ets[i].append({'Ets': [], 'Et_mean': 0.0, 'Et_sigma': 0.0, 'Et_min': 0.0, 'Et_max': 0.0})

    cpu_util_lst = []
    draw = interact

    # 4. Generate paths and task execution trace files
    for k in range(path_idx, n_path_per_task + path_idx):
        cpu_util_lst_1 = []
        pic_path = os.path.join(dir_path, f'path_{k}.png') if dir_path is not None else None
        
        # Simulate trajectory
        path_xys = generate_path_only(cfgs, stops, n_steps=int(n_ms / prd_max) + 1, reverse_prob=0.05)
        if pic_path is not None or draw:
            plot_moving_trajectory(path_xys, stops, cfgs, output_path=pic_path, draw=draw)

        for si in range(n_inst_per_path):
            cpu_utils = [0.0] * math.ceil(n_sec / update_interval_s)
            
            for i in range(n_tasks):
                task = params['tasks'][i]
                dump_path = os.path.join(dir_path, f"path_Et_task_{i}_{k}_{si}.txt") if dir_path is not None else None
                prd = task['period']
                n_steps = int(n_ms / prd)
                
                print(f'generating Et for task {i}, {n_steps} steps, {k}th path, {si}th instance')
                steps, task_Et_min_max = generate_execution_time_trace(
                    path_xys=path_xys,
                    period=prd,
                    ms_per_move=prd_max,
                    n_steps=n_steps,
                    task_param=task,
                    cfgs=cfgs,
                    dump_path=dump_path
                )

                if 'Et_min' not in params['tasks'][i] or 'Et_max' not in params['tasks'][i]:
                    params['tasks'][i]['Et_min'] = task_Et_min_max[0]
                    params['tasks'][i]['Et_max'] = task_Et_min_max[1]
                else:
                    if task_Et_min_max[0] < params['tasks'][i]['Et_min']:
                        params['tasks'][i]['Et_min'] = task_Et_min_max[0]
                    if task_Et_min_max[1] > params['tasks'][i]['Et_max']:
                        params['tasks'][i]['Et_max'] = task_Et_min_max[1]

                # Accumulate CPU utilization
                for ii in range(len(steps)):
                    f = steps[ii][2]
                    idx = int(ii * prd / (1000 * update_interval_s))
                    if idx < len(cpu_utils):
                        cpu_utils[idx] += f
                        task_Ets[i][idx]['Ets'].append(f)

            # Normalize CPU util per interval
            n_cores = cfgs.get("N_CORES", 1)
            for idx in range(len(cpu_utils)):
                # Average-per-core utilization as a percentage:
                #  (sum of execution times in ms) / (n_cores * interval_ms) * 100
                cpu_utils[idx] = (
                    cpu_utils[idx] / (n_cores * 1000 * update_interval_s)
                ) * 100.0

            cpu_util_lst_1.append(cpu_utils)

        cpu_util_lst.append(cpu_util_lst_1)

    # 5. Plot expected CPU utilization and per-task utilization over time
    if dir_path is not None:
        nn = math.ceil(n_sec / update_interval_s)
        xx = [update_interval_s * idx for idx in range(nn)]
        n_cores = cfgs.get("N_CORES", 1)

        # --- aggregate CPU util ---
        fig, ax = plt.subplots()
        k_idx = 0
        for c in cpu_util_lst:
            si = 0
            for cc in c:
                ax.plot(xx, cc, label=f"path_{k_idx}_inst_{si}", alpha=0.6, marker='o', markersize=3)
                si += 1
            k_idx += 1
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Average Per-Core CPU Utilization (%)')
        ax.set_title('Expected CPU Utilization')
        ax.legend()
        plt.savefig(os.path.join(dir_path, "cpu_util.png"))
        plt.close()

        # --- per-task util ---
        for i in range(n_tasks):
            task = params['tasks'][i]
            task_name = task.get('name', f'task_{i+1}')
            prd = task['period']

            if task.get('env_dependent', False):
                task_type = 'env-dependent'
            elif task.get('time_limit_task', False):
                task_type = 'perf-dependent'
            else:
                task_type = 'normal'

            fig, ax = plt.subplots()
            task_util = [0.0] * nn
            for idx in range(nn):
                ets = task_Ets[i][idx]['Ets']
                if ets:
                    # Average ET over all runs, divided by period => true scheduling util
                    mean_et = sum(ets) / len(ets)
                    task_util[idx] = (mean_et / prd) * 100.0

            ax.plot(xx, task_util, marker='o', markersize=3, alpha=0.7)
            ax.set_xlabel('time (s)')
            ax.set_ylabel('Task Utilization (%)')
            ax.set_title(
                f'{task_name} ({task_type}) — period={prd}ms\n'
                f'Expected Utilization: {task["Et_mean"] / prd * 100:.1f}%'
            )
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(os.path.join(dir_path, f"task_{i}_util.png"))
            plt.close()

    # 6. Re-calculate metrics and write taskset_characteristics_[k].yaml
    perf_sel = None
    for k_val in range(n_intervals):
        bad_tasks = []  # (task_idx, n_samples) for diagnostics
        for i in range(n_tasks):
            interval_ets = task_Ets[i][k_val]['Ets']
            n = len(interval_ets)
            if n >= 2:
                task_Ets[i][k_val]['Et_mean'] = float(np.mean(interval_ets))
                task_Ets[i][k_val]['Et_sigma'] = float(np.std(interval_ets))
                task_Ets[i][k_val]['Et_min'] = float(np.min(interval_ets))
                task_Ets[i][k_val]['Et_max'] = float(np.max(interval_ets))
                if task_Ets[i][k_val]['Et_sigma'] < 1e-6:
                    task_Ets[i][k_val]['Et_sigma'] = 1.0
            else:
                bad_tasks.append((i, n))

        if not bad_tasks:
            # All tasks in this interval are well-sampled
            for i in range(n_tasks):
                params['tasks'][i]['Et_actual'] = task_Ets[i][k_val]

            if dir_path is not None:
                old_task_char, perf_sel = convert_taskset_parameters_to_cpp_yaml(
                    params,
                    cfgs,
                    n_sec=n_sec,
                    add_perf_records=add_perf_records,
                    perf_sel=perf_sel
                )
                dump_yml_fpath = os.path.join(dir_path, f"taskset_characteristics_{k_val}.yaml")
                export_taskset_to_yaml(old_task_char, dump_yml_fpath)

                # Write global taskset_characteristics.yaml for backward compatibility
                if k_val == 0:
                    export_taskset_to_yaml(old_task_char, os.path.join(dir_path, "taskset_characteristics.yaml"))

                # Write processor-specific files for CSPSimulation_2 compatibility
                n_cores = cfgs.get("N_CORES", 1)
                for pp in range(n_cores):
                    old_task_char_p, perf_sel = convert_taskset_parameters_to_cpp_yaml(
                        params,
                        cfgs,
                        n_sec=n_sec,
                        add_perf_records=add_perf_records,
                        perf_sel=perf_sel,
                        iprocessorId=pp
                    )
                    dump_yml_fpath_p = os.path.join(dir_path, f"taskset_characteristics_i{k_val}_p{pp}.yaml")
                    export_taskset_to_yaml(old_task_char_p, dump_yml_fpath_p)
        else:
            bad_str = ', '.join(f'task {idx} ({cnt} samples)' for idx, cnt in bad_tasks)
            msg = (
                f"Interval {k_val} has insufficient samples: {bad_str}. "
                f"Each task interval needs >=2 samples. Consider increasing "
                f"n_sec={n_sec} or n_path_per_task={n_path_per_task}."
            )
            if k_val == 0:
                # Fatal: primary output (taskset_characteristics.yaml) cannot be produced
                raise ValueError(f"Cannot compute primary taskset_characteristics: {msg}")
            else:
                # Secondary interval: warn and skip, but keep processing later intervals
                print(f'Warning: skipping interval {k_val}: {msg}')
                continue

def run_full_generation_pipeline(
    cfg_file: str,
    n_sec: int = 1000,
    dir_path: str = None,
    add_perf_records: bool = True,
    interact: bool = False,
    n_path_per_task: int = 4,
    n_inst_per_path: int = 1
) -> None:
    """Coordinates the full pipeline: parameter generation, stops/path generation, and trace generation."""
    OPT_SP_PROJECT_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    # Resolve paths
    if not cfg_file.startswith('/'):
        cfg_file = os.path.join(OPT_SP_PROJECT_PATH, cfg_file)
    if not os.path.exists(cfg_file):
        raise FileNotFoundError(f"Configuration file not found: {cfg_file}")

    cfgs = load_generation_config(cfg_file)

    if dir_path is None:
        cfg_file_name = os.path.basename(cfg_file)
        dir_path = os.path.join(OPT_SP_PROJECT_PATH, 'TaskData', cfg_file_name.replace('.json', '_gen_1'))
    else:
        if not dir_path.startswith('/'):
            dir_path = os.path.join(OPT_SP_PROJECT_PATH, dir_path)
            
    os.makedirs(dir_path, exist_ok=True)

    # 1. Generate core taskset parameters
    taskset_params = generate_taskset_parameters(cfgs, n_sec=n_sec)

    # Validate that total simulated time covers at least 2 hyper-periods
    periods_ms = [int(t['period']) for t in taskset_params['tasks']]
    hyper_period_ms = _compute_hyper_period(periods_ms)
    required_sim_time_ms = 2 * hyper_period_ms
    n_sec_ms = n_sec * 1000
    if n_sec_ms < required_sim_time_ms:
        raise ValueError(
            f"Total simulated time ({n_sec}s = {n_sec_ms}ms) must be at least "
            f"2× the hyper-period ({required_sim_time_ms}ms). "
            f"Hyper-period of periods {periods_ms} = {hyper_period_ms}ms. "
            f"Increase n_sec to >= {math.ceil(required_sim_time_ms / 1000.0)}s."
        )

    # 2. Export taskset_param.yaml
    export_taskset_to_yaml(taskset_params, os.path.join(dir_path, "taskset_param.yaml"))

    # 3. Generate path traces and characteristics
    generate_additional_execution_traces(
        cfgs=cfgs,
        dir_path=dir_path,
        n_path_per_task=n_path_per_task,
        n_inst_per_path=n_inst_per_path,
        path_idx=0,
        n_sec=n_sec,
        add_perf_records=add_perf_records,
        interact=interact
    )
