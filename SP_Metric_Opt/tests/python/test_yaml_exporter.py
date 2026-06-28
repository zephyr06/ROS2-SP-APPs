import random
import unittest
import os
import sys

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.yaml_exporter import (
    format_list_to_space_separated_string,
    convert_taskset_parameters_to_cpp_yaml
)

class TestYamlExporter(unittest.TestCase):

    def test_format_list_to_space_separated_string(self):
        self.assertEqual(format_list_to_space_separated_string([1, 2, 3]), "1 2 3")
        self.assertEqual(format_list_to_space_separated_string([1.5, 2.75]), "1.500 2.750")

    def test_convert_taskset_parameters_to_cpp_yaml_partitioning(self):
        # Mock inputs — include all fields produced by generate_taskset_parameters
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_A', 'processorId': 0, 'period': 100,
                    'Et_mean': 10.0, 'Et_sigma': 1.0, 'sp_threshold': 0.8,
                    'sp_weight': 1.0, 'deadline': 73,
                    'execution_time_min': 8.0, 'execution_time_max': 90.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                },
                {
                    'id': 1, 'name': 'task_B', 'processorId': 1, 'period': 200,
                    'Et_mean': 20.0, 'Et_sigma': 2.0, 'sp_threshold': 0.6,
                    'sp_weight': 1.0, 'deadline': 150,
                    'execution_time_min': 16.0, 'execution_time_max': 180.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                },
                {
                    'id': 2, 'name': 'task_C', 'processorId': 0, 'period': 300,
                    'Et_mean': 30.0, 'Et_sigma': 3.0, 'sp_threshold': 0.4,
                    'sp_weight': 1.0, 'deadline': 225,
                    'execution_time_min': 24.0, 'execution_time_max': 270.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                }
            ]
        }
        cfgs = {
            "SP_WEIGHTS_SUM": 5.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]
        }

        # Test partition processor 0
        out_p0, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params=inp_params,
            cfgs=cfgs,
            iprocessorId=0,
            add_perf_records=False
        )

        self.assertEqual(len(out_p0['tasks']), 2)
        # Verify local id assignment
        self.assertEqual(out_p0['tasks'][0]['id'], 0)
        self.assertEqual(out_p0['tasks'][0]['gid'], 0)
        self.assertEqual(out_p0['tasks'][0]['name'], 'task_A')
        self.assertEqual(out_p0['tasks'][0]['processorId'], 0)

        self.assertEqual(out_p0['tasks'][1]['id'], 1)
        self.assertEqual(out_p0['tasks'][1]['gid'], 2)
        self.assertEqual(out_p0['tasks'][1]['name'], 'task_C')
        self.assertEqual(out_p0['tasks'][1]['processorId'], 0)

        # Test partition processor 1
        out_p1, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params=inp_params,
            cfgs=cfgs,
            iprocessorId=1,
            add_perf_records=False
        )
        self.assertEqual(len(out_p1['tasks']), 1)
        self.assertEqual(out_p1['tasks'][0]['id'], 0)
        self.assertEqual(out_p1['tasks'][0]['gid'], 1)
        self.assertEqual(out_p1['tasks'][0]['name'], 'task_B')
        self.assertEqual(out_p1['tasks'][0]['processorId'], 1)

    def test_performance_records_are_passed_through(self):
        """The exporter must forward pre-generated performance records verbatim."""
        # Generator-produced strings for a task with 10 TL options
        perf_time_10 = "50.000 144.444 238.889 333.333 427.778 522.222 616.667 711.111 805.556 900.000"
        perf_perf_10 = "0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"

        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'slow_task', 'processorId': 0, 'period': 1000,
                    'Et_mean': 50.0, 'Et_sigma': 5.0, 'sp_threshold': 0.5,
                    'sp_weight': 2.0, 'deadline': 750,
                    'execution_time_min': 50.0, 'execution_time_max': 900.0,
                    'total_running_time': 100000, 'time_limit_task': True,
                    'performance_records_time': perf_time_10,
                    'performance_records_perf': perf_perf_10,
                },
                {
                    'id': 1, 'name': 'fast_task', 'processorId': 0, 'period': 50,
                    'Et_mean': 5.0, 'Et_sigma': 0.5, 'sp_threshold': 0.5,
                    'sp_weight': 1.0, 'deadline': 37,
                    'execution_time_min': 4.0, 'execution_time_max': 45.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                }
            ]
        }

        cfgs = {
            "SP_WEIGHTS_SUM": 5.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        }
        out, sel = convert_taskset_parameters_to_cpp_yaml(
            inp_params=inp_params,
            cfgs=cfgs,
            add_perf_records=True
        )
        self.assertEqual(len(sel), 1)
        self.assertEqual(sel[0], 0)

        perf_task = out['tasks'][0]
        time_rec = perf_task['performance_records_time'].split()
        perf_rec = perf_task['performance_records_perf'].split()
        self.assertEqual(len(time_rec), 10)
        self.assertEqual(len(perf_rec), 10)
        self.assertEqual(perf_task['performance_records_time'], perf_time_10)
        self.assertEqual(perf_task['performance_records_perf'], perf_perf_10)

        # Verify perf bounds are overridden to full range
        self.assertEqual(perf_task['execution_time_min'], 50.0)
        self.assertEqual(perf_task['execution_time_max'], 900.0)

        # Non-perf task must NOT have performance record keys
        non_perf = out['tasks'][1]
        self.assertNotIn('performance_records_time', non_perf)
        self.assertNotIn('performance_records_perf', non_perf)

    def test_add_perf_records_false_omits_records(self):
        """When add_perf_records=False, keys must be omitted even for time_limit_task."""
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'soft_task', 'processorId': 0, 'period': 200,
                    'Et_mean': 20.0, 'Et_sigma': 2.0, 'sp_threshold': 0.5,
                    'sp_weight': 2.0, 'deadline': 150,
                    'execution_time_min': 10.0, 'execution_time_max': 180.0,
                    'total_running_time': 100000, 'time_limit_task': True,
                    'performance_records_time': "10.000 95.000 180.000",
                    'performance_records_perf': "0.1 0.2 0.3",
                }
            ]
        }
        cfgs = {
            "SP_WEIGHTS_SUM": 5.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        }
        out, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )
        self.assertNotIn('performance_records_time', out['tasks'][0])
        self.assertNotIn('performance_records_perf', out['tasks'][0])

    def test_deadline_is_passed_through_verbatim(self):
        # The exporter must use the deadline provided by the generator verbatim.
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_A', 'processorId': 0, 'period': 100,
                    'Et_mean': 10.0, 'Et_sigma': 1.0, 'sp_threshold': 0.8,
                    'sp_weight': 1.0, 'deadline': 73,
                    'execution_time_min': 8.0, 'execution_time_max': 90.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                },
            ]
        }
        cfgs = {"SP_WEIGHTS_SUM": 5.0, "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}

        out, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        self.assertEqual(out['tasks'][0]['deadline'], 73)

    def test_deadline_fallback_when_not_provided(self):
        # If the generator somehow omits deadline, exporter falls back to period*0.75.
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_B', 'processorId': 0, 'period': 200,
                    'Et_mean': 20.0, 'Et_sigma': 2.0, 'sp_threshold': 0.6,
                    'sp_weight': 1.0,
                    'execution_time_min': 16.0, 'execution_time_max': 180.0,
                    'total_running_time': 100000, 'time_limit_task': False,
                },
            ]
        }
        cfgs = {"SP_WEIGHTS_SUM": 5.0, "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}

        out, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        self.assertEqual(out['tasks'][0]['deadline'], 150)

if __name__ == "__main__":
    unittest.main()
