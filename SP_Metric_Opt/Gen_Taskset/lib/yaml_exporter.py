import yaml
import os
import random

class SpaceSeparatedListDumper(yaml.Dumper):
    """Custom YAML dumper to format lists as space-separated strings."""
    def increase_indent(self, flow=False, indentless=False):
        return super(SpaceSeparatedListDumper, self).increase_indent(flow=True)

def format_list_to_space_separated_string(data_list: list) -> str:
    """Formats a list of numbers into a space-separated string."""
    return " ".join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in data_list)

def convert_taskset_parameters_to_cpp_yaml(
    inp_params: dict,
    cfgs: dict,
    n_sec: int = None,
    add_perf_records: bool = True,
    perf_sel: list = None,
    iprocessorId: int = None
) -> tuple[dict, list]:
    """Converts the internal taskset parameter representation to C++ compatible YAML format."""
    out = {'tasks': []}
    n = len(inp_params['tasks'])
    n_ms = (n_sec * 1000) if n_sec is not None else 100000

    # Read GMM/SP configurations from cfgs
    g_sel_n_perf_record_task = cfgs.get("N_ENV_DEPENDENT_TASKS", 2)
    g_min_prd_with_perf_records = cfgs.get("MIN_PERIOD_WITH_PERFORMANCE_RECORDS", 100)
    g_final_Et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    
    # 1. Identify which tasks will have performance records (soft tasks)
    if perf_sel is None:
        perf_sel = []
        # Find candidates (period >= min period threshold)
        candidates = []
        for i in range(n):
            task_period = inp_params['tasks'][i].get('period', inp_params['tasks'][i].get('period'))
            if task_period >= g_min_prd_with_perf_records:
                candidates.append(i)
        
        # Select randomly up to g_sel_n_perf_record_task
        if len(candidates) >= g_sel_n_perf_record_task:
            perf_sel = random.sample(candidates, g_sel_n_perf_record_task)
        else:
            perf_sel = candidates.copy()

    total_weights = 0.0
    local_id = 0

    # 2. Build task dictionaries
    for i in range(n):
        task_data = inp_params['tasks'][i]
        
        task_processorId = task_data.get('processorId', 0)
        if iprocessorId is not None and task_processorId != iprocessorId:
            continue
            
        t = {'id': local_id, 'gid': i}
        local_id += 1
        
        # Handle if task_data has Et_actual (historical run statistics) or not
        if 'Et_actual' in task_data:
            t['execution_time_mu'] = task_data['Et_actual']['Et_mean']
            t['execution_time_sigma'] = task_data['Et_actual']['Et_sigma']
            t['execution_time_min'] = task_data['Et_actual']['Et_min']
            t['execution_time_max'] = task_data['Et_actual']['Et_max']
        else:
            t['execution_time_mu'] = task_data['Et_mean']
            t['execution_time_sigma'] = task_data['Et_sigma']
            # Calculate bounds
            t['execution_time_min'] = task_data.get('Et_min', max(1.0, task_data['Et_mean'] - 2.0 * task_data['Et_sigma']))
            t['execution_time_max'] = task_data.get('Et_max', task_data['period'] * g_final_Et_over_period_range[1])

        t['period'] = task_data['period']
        t['deadline'] = int(round(t['period'] * random.uniform(0.5, 1.0)))
        t['processorId'] = task_processorId
        t['name'] = task_data.get('name', f'task_{i+1}')
        t['sp_threshold'] = task_data['sp_threshold']
        
        # Assign base weights: 1 for normal, 2 for performance records
        if add_perf_records and i in perf_sel:
            t['sp_weight'] = 2.0
        else:
            t['sp_weight'] = 1.0

        total_weights += t['sp_weight']
        t['total_running_time'] = n_ms

        # 3. Add performance records for soft tasks
        if add_perf_records and i in perf_sel:
            max_time_limit_options = cfgs.get("MAX_TIME_LIMIT_OPTIONS", 10)
            t['execution_time_max'] = t['period'] * g_final_Et_over_period_range[1]
            t['execution_time_min'] = t['period'] * g_final_Et_over_period_range[0]
            n_steps = max_time_limit_options - 1
            step = (t['execution_time_max'] - t['execution_time_min']) / n_steps
            t_s = t['execution_time_min']

            performance_records_time = []
            performance_records_perf = []
            for _ in range(max_time_limit_options):
                performance_records_time.append(t_s)
                performance_records_perf.append(len(performance_records_perf) * 0.1 + 0.1)
                t_s += step

            t['performance_records_time'] = " ".join(f"{x:.3f}" for x in performance_records_time)
            t['performance_records_perf'] = " ".join(f"{x:.1f}" for x in performance_records_perf)

        out['tasks'].append(t)

    # 4. Normalize weights to g_total_weights if specified
    g_total_weights = cfgs.get("SP_WEIGHTS_SUM", 5.0)
    if total_weights > 0.0:
        for j in range(len(out['tasks'])):
            out['tasks'][j]['sp_weight'] *= g_total_weights / total_weights

    return out, perf_sel

def export_taskset_to_yaml(taskset_data: dict, output_path: str) -> None:
    """Formats and writes the taskset parameters to a YAML file using SpaceSeparatedListDumper."""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(
            taskset_data, 
            f, 
            sort_keys=False, 
            default_flow_style=False, 
            width=float("inf"), 
            Dumper=SpaceSeparatedListDumper
        )
