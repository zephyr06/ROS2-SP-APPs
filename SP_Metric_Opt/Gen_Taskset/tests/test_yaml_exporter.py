import pytest
import yaml
from Gen_Taskset.lib.yaml_exporter import format_list_to_space_separated_string, convert_taskset_parameters_to_cpp_yaml

def test_format_list_to_space_separated_string():
    data = [1.2356, 2.0, 3]
    result = format_list_to_space_separated_string(data)
    assert result == "1.236 2.000 3"


def test_convert_emits_important_field():
    """P0.6/P0.7/P0.8: the C++-facing YAML carries the important-task label as
    `important` per task, read back by C++ ReadTaskSet onto Task::is_important.
    The exporter is a pure format transformer -- it must forward is_important
    from the internal task dict (defaulting false for tasksets that predate it).
    """
    inp_params = {
        "tasks": [
            {"period": 100, "deadline": 75, "sp_threshold": 0.5,
             "sp_weight": 2.0, "is_important": True,
             "Et_mean": 10.0, "Et_sigma": 1.0,
             "execution_time_min": 1.0, "execution_time_max": 90.0,
             "name": "task_1", "processorId": 0},
            {"period": 200, "deadline": 150, "sp_threshold": 0.5,
             "sp_weight": 1.0, "is_important": False,
             "Et_mean": 20.0, "Et_sigma": 2.0,
             "execution_time_min": 1.0, "execution_time_max": 180.0,
             "name": "task_2", "processorId": 1},
        ]
    }
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    out, _ = convert_taskset_parameters_to_cpp_yaml(inp_params, cfgs, add_perf_records=False)
    assert out["tasks"][0]["important"] is True
    assert out["tasks"][1]["important"] is False


def test_convert_defaults_important_false_when_absent():
    """A taskset_param reloaded from a YAML that predates the label has no
    is_important key; the exporter must default it to false (not raise), so
    legacy tasksets still load."""
    inp_params = {
        "tasks": [
            {"period": 100, "deadline": 75, "sp_threshold": 0.5,
             "sp_weight": 2.0,
             "Et_mean": 10.0, "Et_sigma": 1.0,
             "execution_time_min": 1.0, "execution_time_max": 90.0,
             "name": "task_1", "processorId": 0},
        ]
    }
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    out, _ = convert_taskset_parameters_to_cpp_yaml(inp_params, cfgs, add_perf_records=False)
    assert out["tasks"][0]["important"] is False


def test_important_field_round_trips_through_yaml():
    """The `important` bool survives a yaml dump/load round-trip as a YAML bool,
    which is what C++ YAML-cpp .as<bool>() parses."""
    inp_params = {
        "tasks": [
            {"period": 100, "deadline": 75, "sp_threshold": 0.5,
             "sp_weight": 2.0, "is_important": True,
             "Et_mean": 10.0, "Et_sigma": 1.0,
             "execution_time_min": 1.0, "execution_time_max": 90.0,
             "name": "task_1", "processorId": 0},
        ]
    }
    cfgs = {"FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}
    out, _ = convert_taskset_parameters_to_cpp_yaml(inp_params, cfgs, add_perf_records=False)
    dumped = yaml.safe_dump(out, default_flow_style=False)
    reloaded = yaml.safe_load(dumped)
    assert reloaded["tasks"][0]["important"] is True
    assert isinstance(reloaded["tasks"][0]["important"], bool)
