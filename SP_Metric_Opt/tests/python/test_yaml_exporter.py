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
        # Mock inputs
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_A', 'processorId': 0, 'period': 100,
                    'Et_mean': 10.0, 'Et_sigma': 1.0, 'sp_threshold': 0.8
                },
                {
                    'id': 1, 'name': 'task_B', 'processorId': 1, 'period': 200,
                    'Et_mean': 20.0, 'Et_sigma': 2.0, 'sp_threshold': 0.6
                },
                {
                    'id': 2, 'name': 'task_C', 'processorId': 0, 'period': 300,
                    'Et_mean': 30.0, 'Et_sigma': 3.0, 'sp_threshold': 0.4
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

    def test_max_time_limit_options_cap(self):
        """Verify that MAX_TIME_LIMIT_OPTIONS caps the number of TL options."""
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'slow_task', 'processorId': 0, 'period': 1000,
                    'Et_mean': 50.0, 'Et_sigma': 5.0, 'sp_threshold': 0.5
                },
                {
                    'id': 1, 'name': 'fast_task', 'processorId': 0, 'period': 50,
                    'Et_mean': 5.0, 'Et_sigma': 0.5, 'sp_threshold': 0.5
                }
            ]
        }

        # Cap at 10 options (default)
        cfgs_10 = {
            "SP_WEIGHTS_SUM": 5.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "N_ENV_DEPENDENT_TASKS": 1,
            "MIN_PERIOD_WITH_PERFORMANCE_RECORDS": 100
        }
        out_10, sel_10 = convert_taskset_parameters_to_cpp_yaml(
            inp_params=inp_params,
            cfgs=cfgs_10,
            add_perf_records=True
        )
        # slow_task qualifies (period 1000 >= 100), fast_task does not
        self.assertEqual(len(sel_10), 1)
        self.assertEqual(sel_10[0], 0)
        perf_task_10 = out_10['tasks'][0]
        perf_records_time_10 = perf_task_10['performance_records_time'].split()
        perf_records_perf_10 = perf_task_10['performance_records_perf'].split()
        self.assertEqual(len(perf_records_time_10), 10)
        self.assertEqual(len(perf_records_perf_10), 10)

        # Cap at 5 options
        cfgs_5 = {
            "SP_WEIGHTS_SUM": 5.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "MAX_TIME_LIMIT_OPTIONS": 5,
            "N_ENV_DEPENDENT_TASKS": 1,
            "MIN_PERIOD_WITH_PERFORMANCE_RECORDS": 100
        }
        out_5, sel_5 = convert_taskset_parameters_to_cpp_yaml(
            inp_params=inp_params,
            cfgs=cfgs_5,
            add_perf_records=True
        )
        perf_task_5 = out_5['tasks'][0]
        perf_records_time_5 = perf_task_5['performance_records_time'].split()
        perf_records_perf_5 = perf_task_5['performance_records_perf'].split()
        self.assertEqual(len(perf_records_time_5), 5)
        self.assertEqual(len(perf_records_perf_5), 5)

        # Even with different caps, the first and last TL values should match
        # because they are derived from the same min/max = period * range.
        self.assertEqual(perf_records_time_10[0], perf_records_time_5[0])
        self.assertEqual(perf_records_time_10[-1], perf_records_time_5[-1])

    def test_deadline_deterministic_when_provided(self):
        """Bug: yaml_exporter ignores a pre-defined deadline and regenerates randomly.

        If task_data already contains a 'deadline', the exporter must use it verbatim.
        """
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_A', 'processorId': 0, 'period': 100,
                    'Et_mean': 10.0, 'Et_sigma': 1.0, 'sp_threshold': 0.8,
                    'deadline': 73  # pre-defined static deadline
                },
            ]
        }
        cfgs = {"SP_WEIGHTS_SUM": 5.0, "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}

        random.seed(123)
        out1, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        random.seed(999)
        out2, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        self.assertEqual(out1['tasks'][0]['deadline'], 73)
        self.assertEqual(out2['tasks'][0]['deadline'], 73)

    def test_deadline_random_when_not_provided(self):
        """Backward compat: missing deadline key triggers random generation."""
        inp_params = {
            'tasks': [
                {
                    'id': 0, 'name': 'task_B', 'processorId': 0, 'period': 200,
                    'Et_mean': 20.0, 'Et_sigma': 2.0, 'sp_threshold': 0.6,
                },
            ]
        }
        cfgs = {"SP_WEIGHTS_SUM": 5.0, "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]}

        random.seed(123)
        out1, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        random.seed(999)
        out2, _ = convert_taskset_parameters_to_cpp_yaml(
            inp_params, cfgs, add_perf_records=False
        )

        self.assertNotEqual(
            out1['tasks'][0]['deadline'], out2['tasks'][0]['deadline'],
            "Deadlines should be different when random seeds differ and no deadline key is provided"
        )

if __name__ == "__main__":
    unittest.main()
