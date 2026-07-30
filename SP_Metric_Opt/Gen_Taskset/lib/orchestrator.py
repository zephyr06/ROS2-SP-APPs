import os
import math
import sys
import json
import numpy as np
import yaml
import matplotlib.pyplot as plt

from .generation_config_parser import load_generation_config
from .yaml_exporter import convert_taskset_parameters_to_cpp_yaml, export_taskset_to_yaml
from .taskset_generator import generate_taskset_parameters, load_and_fill_taskset_param_file
from .trajectory import generate_stops_in_map, generate_path_only
from .trace_generator import generate_execution_time_trace
from .visualizer import plot_moving_trajectory
from .feasibility_clamp import clamp_avg_et_to_period
from .important_task_rta import (
    _load_emitted_tasks_by_gid,
    _wcets_from_loaded_tasks,
    important_tasks_schedulable,
)

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


# ---------------------------------------------------------------------------
# Trajectory-layer config integrity (the 2nd gate).
# ---------------------------------------------------------------------------
# ``validate_config_integrity`` in taskset_generator.py gates the *generation*
# parameters read by ``generate_taskset_parameters`` (GMM / ET / SP / period
# params). The orchestrator reads a second, smaller set of params for path +
# trace generation that ``generate_taskset_parameters`` never touches:
#
#   - ROBOT_SPEED_MPS : robot speed; orchestrator derives ROBOT_STEP_SIZE from it
#                       (step_m = speed_mps * prd_max / 1000) and trajectory.py
#                       consumes ROBOT_STEP_SIZE. A forgotten ROBOT_SPEED_MPS
#                       previously fell through to the 1.0 default below, so a
#                       config that meant to set a non-unit speed could silently
#                       run at 1.0 m/s. Required here so that cannot happen.
#
# UPDATE_INTERVAL_S and ROBOT_STEP_SIZE are intentionally NOT required:
#   - UPDATE_INTERVAL_S is a *runtime* knob injected by run_sim_experiments /
#     compare_optimizers (``config_dict["UPDATE_INTERVAL_S"] =
#     scheduler_trigger_interval``) and is also a legitimate standalone-call
#     default (10 s) when the orchestrator is invoked directly; the .get(10)
#     below is the documented runtime default, not a forgotten-value mask.
#   - ROBOT_STEP_SIZE is *derived* (computed from ROBOT_SPEED_MPS in
#     generate_additional_execution_traces and written into cfgs); trajectory.py
#     keeps a .get(5.0) fallback only for standalone trajectory tests that pass
#     a minimal cfgs without going through the orchestrator.
TRAJECTORY_REQUIRED_CONFIG_PARAMS = [
    {"key": "ROBOT_SPEED_MPS", "suggest": 1.0, "desc": "robot speed (m/s); ROBOT_STEP_SIZE is derived from it"},
]


def validate_trajectory_config(cfgs: dict, config_path: str = None) -> dict:
    """Ensure every trajectory-layer parameter the orchestrator reads is set.

    A thinner, layer-specific companion to
    :func:`taskset_generator.validate_config_integrity`: it only covers the
    params read in this module (``generate_additional_execution_traces`` /
    ``run_full_generation_pipeline``) that the generation gate does not cover,
    so a forgotten ``ROBOT_SPEED_MPS`` cannot silently fall back to 1.0 m/s.

    Same interactive behavior as the generation gate: prompts on a TTY (writing
    resolved values back to ``config_path``) and raises ``ValueError`` listing
    the missing key(s) otherwise. Idempotent: a complete config returns
    unchanged.
    """
    missing = [e for e in TRAJECTORY_REQUIRED_CONFIG_PARAMS if e["key"] not in cfgs]
    if not missing:
        return cfgs

    if not sys.stdin.isatty() or config_path is None:
        lines = ["Trajectory config is missing required parameters:"]
        for e in missing:
            lines.append(f"  - {e['key']} (suggested: {e['suggest']!r}) -- {e['desc']}")
        where = f" {config_path}" if config_path else " your config file"
        lines.append(f"Add the missing keys to{where} (or run interactively to be prompted).")
        raise ValueError("\n".join(lines))

    # Interactive: reuse the generation gate's prompt + write-back helpers so
    # the two gates behave identically (single source of truth for the UX).
    from .taskset_generator import _parse_prompted_value, _write_back_resolved_keys
    print("\nTrajectory config is missing required parameters.")
    print("Suggested values are the former silent defaults -- press Enter to accept,")
    print("or type a value (JSON: int/float/list/bool/null).\n")
    resolved = {}
    for e in missing:
        key, desc, suggest = e["key"], e["desc"], e["suggest"]
        print(f"{key} -- {desc}")
        print(f"  suggested: {suggest!r}")
        try:
            raw = input(f"  {key} [Enter to accept]: ")
        except EOFError:
            raw = ""
        value = _parse_prompted_value(raw, suggest)
        cfgs[key] = value
        resolved[key] = value
        print()
    _write_back_resolved_keys(config_path, resolved)
    return cfgs


# ---------------------------------------------------------------------------
# Shared path + config resolution helpers (single source of truth for the
# pipeline entry points — ``run_full_generation_pipeline`` (shell),
# ``_run_pipeline_with_cfgs`` (body), and the P0.8 important-task gate all
# resolve paths identically). Extracted to kill triplicated scaffolding and the
# divergence bug it caused (the split once dropped the ``dir_path=None`` default
# from the shell, breaking the canonical CLI's no-``--dir_path`` invocation).
# ---------------------------------------------------------------------------

# Project root (this file is <root>/Gen_Taskset/lib/orchestrator.py). Module-level
# so the three entry points share one computation, not three identical copies.
OPT_SP_PROJECT_PATH = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _resolve_config_path(cfg_file: str) -> str:
    """Resolve a config path to an absolute, existing file path.

    Relative paths are anchored at :data:`OPT_SP_PROJECT_PATH` (the project
    root), matching the historical behavior of the pipeline entry points.
    """
    if not cfg_file.startswith('/'):
        cfg_file = os.path.join(OPT_SP_PROJECT_PATH, cfg_file)
    if not os.path.exists(cfg_file):
        raise FileNotFoundError(f"Configuration file not found: {cfg_file}")
    return cfg_file


def _load_and_validate_cfgs(cfg_file: str) -> dict:
    """Load a generation config and run both integrity gates.

    Thin composition of :func:`load_generation_config` + the trajectory-layer
    gate (:func:`validate_trajectory_config`, the second gate — covers
    ``ROBOT_SPEED_MPS`` that the generation gate does not). Shared by the
    canonical shell and the P0.8 gate so they cannot diverge on which gates a
    loaded cfgs passes.
    """
    cfgs = load_generation_config(cfg_file)
    validate_trajectory_config(cfgs, config_path=cfg_file)
    return cfgs


def _resolve_dir_path(dir_path, cfg_file: str) -> str:
    """Resolve the pipeline output directory to a concrete, created path.

    ``dir_path=None`` (the CLI default) resolves to
    ``TaskData/<cfg_name>_gen_1``; relative paths anchor at the project root.
    Creates the directory (``exist_ok=True``). ``cfg_file`` is the already-
    resolved config path (its basename seeds the default name).
    """
    if dir_path is None:
        cfg_file_name = os.path.basename(cfg_file)
        dir_path = os.path.join(OPT_SP_PROJECT_PATH, 'TaskData', cfg_file_name.replace('.json', '_gen_1'))
    elif not dir_path.startswith('/'):
        dir_path = os.path.join(OPT_SP_PROJECT_PATH, dir_path)
    os.makedirs(dir_path, exist_ok=True)
    return dir_path


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
    # UPDATE_INTERVAL_S is a runtime knob, not a generation param: it is injected
    # by run_sim_experiments / compare_optimizers (config_dict["UPDATE_INTERVAL_S"]
    # = scheduler_trigger_interval) and is also a legitimate standalone default
    # when the orchestrator is called directly. The .get(10) here is the
    # documented runtime default, intentionally NOT in TRAJECTORY_REQUIRED.
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
    speed_mps = cfgs["ROBOT_SPEED_MPS"]  # presence enforced by validate_trajectory_config
    step_m = speed_mps * prd_max / 1000.0
    cfgs['ROBOT_STEP_SIZE'] = step_m  # derived; trajectory.py reads it (with a 5.0 standalone fallback)

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
            n_cores = cfgs["N_CORES"]  # presence enforced by validate_config_integrity
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
        n_cores = cfgs["N_CORES"]  # presence enforced by validate_config_integrity

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
                dump_yml_fpath = os.path.join(dir_path, f"taskset_characteristics_interval_{k_val}.yaml")
                export_taskset_to_yaml(old_task_char, dump_yml_fpath)

                # Write global taskset_characteristics.yaml for backward compatibility
                if k_val == 0:
                    export_taskset_to_yaml(old_task_char, os.path.join(dir_path, "taskset_characteristics.yaml"))

                # Write processor-specific files for CSPSimulation_2 compatibility
                n_cores = cfgs["N_CORES"]  # presence enforced by validate_config_integrity
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
    """Coordinates the full pipeline: parameter generation, stops/path generation, and trace generation.

    Thin shell: resolves + loads the config (applying ``standardize_config`` +
    both integrity gates), then delegates to :func:`_run_pipeline_with_cfgs`.
    The ~8 existing callers of this function are untouched.

    The cfgs-loading + body were split so the P0.8 important-task gate
    (:func:`run_full_generation_pipeline_with_important_task_gate`) can hold ONE
    loaded cfgs, advance ``cfgs["RANDOM_SEED"]`` per retry, and call
    ``_run_pipeline_with_cfgs`` directly. The gate MUST NOT call this shell: it
    reloads cfgs from the config FILE each call, which would discard the
    advanced seed and produce a byte-identical taskset every retry (a no-op
    retry — the exact silent hole the gate exists to prevent).
    """
    cfg_file = _resolve_config_path(cfg_file)
    cfgs = _load_and_validate_cfgs(cfg_file)
    dir_path = _resolve_dir_path(dir_path, cfg_file)

    _run_pipeline_with_cfgs(
        cfgs,
        n_sec=n_sec,
        dir_path=dir_path,
        add_perf_records=add_perf_records,
        interact=interact,
        n_path_per_task=n_path_per_task,
        n_inst_per_path=n_inst_per_path,
    )


def _run_pipeline_with_cfgs(
    cfgs: dict,
    n_sec: int,
    dir_path: str,
    add_perf_records: bool,
    interact: bool,
    n_path_per_task: int,
    n_inst_per_path: int,
) -> None:
    """Run the generation pipeline against a PRE-LOADED ``cfgs`` (required, no default).

    Body of the former ``run_full_generation_pipeline``. Takes the cfgs the
    caller already loaded + validated (so this fn does NOT reload from file,
    does NOT re-run the integrity gates). The caller is responsible for cfgs
    validity — ``run_full_generation_pipeline`` loads+validates before calling
    here; the P0.8 gate loads once, then mutates ``cfgs["RANDOM_SEED"]`` per
    retry before calling here.

    ``dir_path`` is required (no default): the shell resolves a default when
    ``None``; the gate always supplies a concrete dir. Forcing a concrete dir
    here means a caller that forgets to set one fails loudly instead of
    silently writing into a project-relative default.
    """
    if dir_path is None:
        raise ValueError(
            "_run_pipeline_with_cfgs requires a concrete dir_path (the shell "
            "run_full_generation_pipeline resolves a default; pass one explicitly)."
        )
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

    # 4. Feasibility clamp (P1.8, fix F1): pull any non-perf task whose avg ET
    # exceeds 0.95*period back inside the period across every emitted
    # characteristics YAML, and relabel its deadline to the period. Removes the
    # WCET/mu > deadline/period substrate the generator emits (no feasibility
    # guard in taskset_generator.py) that let INCR_WCET beat INCR on
    # catastrophically-unschedulable tasksets. Perf-record tasks are skipped
    # (their min/max are TL-grid bounds). Runs AFTER all characteristics YAMLs
    # are on disk, BEFORE the pipeline returns. See feasibility_clamp.py for
    # the FiniteDist-truncates-at-max trace behind the mu+max+min clamp.
    clamp_avg_et_to_period(dir_path, et_over_period_cap=0.95)


# ---------------------------------------------------------------------------
# P0.8 — important-task gate (generation-time seed certification)
# ---------------------------------------------------------------------------
# After the canonical pipeline emits its characteristics YAMLs (and the
# feasibility clamp runs — D5: clamp first, then RTA), the gate derives each
# task's WCET per D2 and runs ``important_tasks_schedulable``. On failure it
# re-runs the pipeline with an ADVANCED seed so each retry actually
# re-samples. Budget 20 (D4), then a LOUD raise (NEVER silently emit an
# unschedulable taskset — re-creates the P1.8 substrate).
#
# WHY THE GATE CANNOT CALL ``run_full_generation_pipeline``: that shell reloads
# ``cfgs`` from the config FILE every call (``load_generation_config`` at the
# top of the shell), so mutating an in-memory ``cfgs["RANDOM_SEED"]`` between
# retries would be discarded — every retry would re-read the file's original
# seed and produce a byte-identical taskset (a no-op retry, the exact silent
# hole the gate exists to prevent). The gate therefore loads cfgs ONCE, mutates
# ``cfgs["RANDOM_SEED"]`` per attempt, and calls ``_run_pipeline_with_cfgs``
# (the body) directly, so the advanced seed actually takes effect.

# D4: maximum re-sampling attempts before the gate gives up and raises loudly.
# 20 is generous — a config that cannot draw a schedulable important subset in
# 20 seeded attempts is almost certainly mis-specified (utilization too high,
# deadlines too tight), not unlucky.
IMPORTANT_TASK_GATE_MAX_ATTEMPTS = 20


def run_full_generation_pipeline_with_important_task_gate(
    cfg_file: str,
    n_sec: int = 1000,
    dir_path: str = None,
    add_perf_records: bool = True,
    interact: bool = False,
    n_path_per_task: int = 4,
    n_inst_per_path: int = 1,
    max_attempts: int = IMPORTANT_TASK_GATE_MAX_ATTEMPTS,
) -> dict:
    """Generation-time gate: certify the emitted taskset is schedulable for the
    important tasks under DM-with-top-priority-lock at the seed point (P0.8).

    Wraps the canonical pipeline (:func:`_run_pipeline_with_cfgs`) with a
    seed-advancing retry loop. Each attempt:

      1. Advances ``cfgs["RANDOM_SEED"]`` to ``base_seed + attempt`` (the cfgs
         is loaded ONCE, before the loop — see the module-level note on why the
         shell cannot be used).
      2. Runs ``_run_pipeline_with_cfgs`` (generate → export → traces → clamp).
         Interval files use fixed names + ``"w"`` overwrite, so a re-run with
         the SAME ``dir_path`` overwrites the prior attempt's files — no
         accumulation, no stale-data mixing into the WCET/RTA read.
      3. Reads the emitted tasks (:func:`_load_emitted_tasks_by_gid`, which
         also normalizes the emitted ``important`` key to the RTA's
         ``is_important``) and derives WCETs (``_wcets_from_loaded_tasks``, D2).
      4. Runs :func:`important_tasks_schedulable` (per-core DM-within-important
         fixed-priority RTA, D3).

    On PASS: returns immediately with a report. On FAIL: advances the seed and
    retries, up to ``max_attempts``. On exhaustion: raises ``RuntimeError``
    with the final culprits + attempt count (D4: NEVER silent — an
    unschedulable taskset is never emitted without a loud failure).

    Args:
        cfg_file: path to the generation config JSON (loaded once; the seed is
            read from ``cfgs["RANDOM_SEED"]`` and advanced per attempt).
        n_sec, dir_path, add_perf_records, interact, n_path_per_task,
            n_inst_per_path: forwarded to ``_run_pipeline_with_cfgs`` (same
            semantics as :func:`run_full_generation_pipeline`).
        max_attempts: D4 retry budget (default 20). The gate raises loudly
            after this many unschedulable draws.

    Returns:
        ``{"schedulable": True, "attempts_used": int, "culprits": []}`` on
        success. ``culprits`` is empty (the final, passing attempt had no
        misses). The report is for logging/diagnostics; the gate's CONTRACT is
        that it returns only on a schedulable draw (otherwise it raises).

    Raises:
        ValueError: if ``cfgs["RANDOM_SEED"]`` is absent (the gate cannot
            advance a seed that isn't there — a non-reproducible gate is a
            config error, never silently papered over).
        RuntimeError: if the budget is exhausted (D4 loud raise; message
            includes the final culprits + attempt count).
    """
    cfg_file = _resolve_config_path(cfg_file)
    cfgs = _load_and_validate_cfgs(cfg_file)
    dir_path = _resolve_dir_path(dir_path, cfg_file)

    base_seed = cfgs.get("RANDOM_SEED")
    if base_seed is None:
        raise ValueError(
            f"Config {cfg_file!r} has no RANDOM_SEED. The important-task gate "
            "advances the seed per retry to re-sample; a non-reproducible "
            "config (no seed) cannot be advanced. Set RANDOM_SEED in the config."
        )

    et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    tl_grid_upper = et_over_period_range[1]

    culprits = []
    for attempt in range(max_attempts):
        # Advance the seed INSIDE the held cfgs so the re-seed inside
        # generate_taskset_parameters picks up a fresh draw each attempt.
        cfgs["RANDOM_SEED"] = base_seed + attempt

        _run_pipeline_with_cfgs(
            cfgs,
            n_sec=n_sec,
            dir_path=dir_path,
            add_perf_records=add_perf_records,
            interact=interact,
            n_path_per_task=n_path_per_task,
            n_inst_per_path=n_inst_per_path,
        )

        # Single disk read: tasks + WCETs from the same loaded dicts (the
        # reader normalizes `important` → `is_important` so the RTA sees the
        # flag — see _load_emitted_tasks_by_gid's docstring).
        tasks_by_gid = _load_emitted_tasks_by_gid(dir_path)
        wcets = _wcets_from_loaded_tasks(tasks_by_gid, tl_grid_upper)
        # RTA takes a parallel list; order by gid for determinism.
        gids = sorted(tasks_by_gid.keys())
        tasks = [tasks_by_gid[g] for g in gids]
        wcet_list = [wcets[g] for g in gids]

        ok, culprits = important_tasks_schedulable(tasks, wcet_list)
        if ok:
            return {
                "schedulable": True,
                "attempts_used": attempt + 1,
                "culprits": [],
            }

    # Budget exhausted (D4): NEVER silently emit the unschedulable taskset.
    raise RuntimeError(
        f"Important-task gate: taskset from {cfg_file!r} failed schedulability "
        f"for the important tasks under DM-with-top-priority-lock after "
        f"{max_attempts} seed-advancing attempts (seeds {base_seed}.."
        f"{base_seed + max_attempts - 1}). The last attempt's culprits:\n"
        f"{json.dumps(culprits, indent=2)}\n"
        "This is almost certainly a mis-specified config (utilization too "
        "high or deadlines too tight for the important subset), not bad luck. "
        "Loosen CPU_UTIL_RANDOM_RANGE / increase deadlines / reduce "
        "IMPORTANT_TASK_RATIO, then re-run."
    )
