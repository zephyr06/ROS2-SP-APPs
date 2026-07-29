import yaml
import os

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
    """Converts the internal taskset parameter representation to C++ compatible YAML format.

    This function is intentionally a *pure format transformer*.
    All semantic parameter generation (execution bounds, performance records,
    weight normalization, deadline assignment) lives in taskset_generator.py.
    """
    out = {'tasks': []}
    n = len(inp_params['tasks'])
    g_final_Et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])

    # 1. Identify which tasks will have performance records (soft tasks)
    if perf_sel is None:
        perf_sel = [
            i for i in range(n)
            if inp_params['tasks'][i].get('time_limit_task', False)
        ]

    local_id = 0

    # 2. Build task dictionaries — map generator fields to C++ field names
    for i in range(n):
        task_data = inp_params['tasks'][i]

        task_processorId = task_data.get('processorId', 0)
        if iprocessorId is not None and task_processorId != iprocessorId:
            continue

        t = {'id': local_id, 'gid': i}
        local_id += 1

        # Handle actual execution statistics from prior simulation runs
        if 'Et_actual' in task_data:
            t['execution_time_mu'] = task_data['Et_actual']['Et_mean']
            t['execution_time_sigma'] = task_data['Et_actual']['Et_sigma']
            t['execution_time_min'] = task_data['Et_actual']['Et_min']
            t['execution_time_max'] = task_data['Et_actual']['Et_max']
        else:
            t['execution_time_mu'] = task_data['Et_mean']
            t['execution_time_sigma'] = task_data['Et_sigma']
            t['execution_time_min'] = task_data.get('execution_time_min', max(1.0, task_data['Et_mean'] - 2.0 * task_data['Et_sigma']))
            t['execution_time_max'] = task_data.get('execution_time_max', task_data['period'] * g_final_Et_over_period_range[1])

        t['period'] = task_data['period']
        t['deadline'] = task_data.get('deadline', int(round(t['period'] * 0.75)))
        t['processorId'] = task_processorId
        t['name'] = task_data.get('name', f'task_{i+1}')
        t['sp_threshold'] = task_data['sp_threshold']
        t['sp_weight'] = task_data.get('sp_weight', 1.0)
        # P0.6/P0.7/P0.8: important-task label, set by the generator (top
        # IMPORTANT_TASK_RATIO by sp_weight). Emitted here so C++ ReadTaskSet
        # reads it onto Task::is_important. Defaults false for tasksets that
        # predate the label (e.g. reloaded taskset_param.yaml without it).
        t['important'] = bool(task_data.get('is_important', False))
        t['total_running_time'] = task_data.get('total_running_time', (n_sec * 1000) if n_sec is not None else 100000)

        # 3. For performance-record tasks, ensure bounds span the full config range
        #    and attach the pre-generated performance records.
        if add_perf_records and i in perf_sel:
            t['execution_time_max'] = t['period'] * g_final_Et_over_period_range[1]
            t['execution_time_min'] = t['period'] * g_final_Et_over_period_range[0]
            if task_data.get('performance_records_time'):
                t['performance_records_time'] = task_data['performance_records_time']
            if task_data.get('performance_records_perf'):
                t['performance_records_perf'] = task_data['performance_records_perf']

        out['tasks'].append(t)

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
