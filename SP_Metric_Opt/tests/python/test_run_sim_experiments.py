import unittest
import os
import tempfile
import shutil
import sys

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.run_sim_experiments import (
    run_single_simulation,
    analyze_single_instance,
)
from simulation_experiments.utils import (
    compute_miss_rate,
    write_summary_and_plots,
)
import unittest.mock


class TestRunSimExperiments(unittest.TestCase):

    def test_compute_miss_rate_from_summary(self):
        """Fast path using miss_rate_summary.txt"""
        temp_dir = tempfile.mkdtemp()
        try:
            summary_path = os.path.join(temp_dir, "miss_rate_summary.txt")
            with open(summary_path, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("420,12,0.028571\n")
            rate = compute_miss_rate(temp_dir, {})
            self.assertAlmostEqual(rate, 0.028571)
        finally:
            shutil.rmtree(temp_dir)

    def test_compute_miss_rate_from_traces(self):
        """Fallback using response_times_task_*.txt"""
        temp_dir = tempfile.mkdtemp()
        try:
            # task 0: 2 jobs, rt=100 and rt=200, deadline=150
            with open(os.path.join(temp_dir, "response_times_task_0.txt"), "w") as f:
                f.write("jobId,release_time,start_time,finish_time,response_time,execution_time,is_overrun\n")
                f.write("0,100,100,200,100,100,0\n")
                f.write("1,500,500,700,200,200,0\n")
            # task 1: 2 jobs, rt=50 and rt=150, deadline=100
            with open(os.path.join(temp_dir, "response_times_task_1.txt"), "w") as f:
                f.write("jobId,release_time,start_time,finish_time,response_time,execution_time,is_overrun\n")
                f.write("0,0,0,50,50,50,0\n")
                f.write("1,200,200,350,150,150,0\n")

            task_deadlines = {0: 150.0, 1: 100.0}
            # Expected misses: task0 job1 (200>150), task1 job1 (150>100)
            rate = compute_miss_rate(temp_dir, task_deadlines)
            self.assertAlmostEqual(rate, 0.5)
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.run_sim_experiments.subprocess.run")
    def test_run_single_simulation_no_extra_args(self, mock_run):
        run_single_simulation(
            sim_bin_path="sim_bin",
            taskset_dir="taskset",
            sched_dir="sched",
            simt=1000,
            scheduler="INCR",
            inst=0,
        )
        mock_run.assert_called_once_with(
            ["sim_bin", "taskset", "sched", "INCR", "1000"],
            check=True,
            stdout=unittest.mock.ANY,
            stderr=unittest.mock.ANY,
        )

    @unittest.mock.patch("simulation_experiments.run_sim_experiments.subprocess.run")
    def test_run_single_simulation_with_export_args(self, mock_run):
        run_single_simulation(
            sim_bin_path="sim_bin",
            taskset_dir="taskset",
            sched_dir="sched",
            simt=1000,
            scheduler="BF",
            inst=1,
            verbose=0,
            export_level=2,
            sample_interval=60,
        )
        mock_run.assert_called_once_with(
            ["sim_bin", "taskset", "sched", "BF", "1000", "2", "60"],
            check=True,
            stdout=unittest.mock.ANY,
            stderr=unittest.mock.ANY,
        )

    def test_analyze_single_instance(self):
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "INCR"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

            # Write interval_sp_metrics.txt
            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                f.write("0,0.95\n")
                f.write("1,0.92\n")

            # Write miss_rate_summary.txt
            summary_file = os.path.join(sched_dir, "miss_rate_summary.txt")
            with open(summary_file, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("10,1,0.1\n")

            task_deadlines = {0: 1000.0}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "INCR")
            self.assertAlmostEqual(miss_rate, 0.1)
            self.assertEqual(sp_values_run, [0.95, 0.92])
            self.assertEqual(run_intervals_data, [(0, 0.95), (1, 0.92)])
            self.assertEqual(avg_sched_time, 0.0)
        finally:
            shutil.rmtree(temp_dir)

    def test_analyze_single_instance_with_exec_time(self):
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "BF"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

            # Write interval_sp_metrics.txt
            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                f.write("0,0.95\n")

            # Write miss_rate_summary.txt
            summary_file = os.path.join(sched_dir, "miss_rate_summary.txt")
            with open(summary_file, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("10,0,0.0\n")

            # Write scheduler_execution_time.txt
            exec_file = os.path.join(sched_dir, "scheduler_execution_time.txt")
            with open(exec_file, "w") as f:
                f.write("1.23456\n")

            task_deadlines = {0: 1000.0}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "BF")
            self.assertAlmostEqual(miss_rate, 0.0)
            self.assertEqual(sp_values_run, [0.95])
            self.assertAlmostEqual(avg_sched_time, 1.23456)
        finally:
            shutil.rmtree(temp_dir)

    def test_analyze_single_instance_with_exec_time_and_intervals(self):
        """Verify total exec time is divided by number of intervals."""
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "BF"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

            # Write interval_sp_metrics.txt
            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                f.write("0,0.95\n")

            # Write miss_rate_summary.txt
            summary_file = os.path.join(sched_dir, "miss_rate_summary.txt")
            with open(summary_file, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("10,0,0.0\n")

            # Write scheduler_execution_time.txt (total process time = 6.0s)
            exec_file = os.path.join(sched_dir, "scheduler_execution_time.txt")
            with open(exec_file, "w") as f:
                f.write("6.0\n")

            # Create 3 interval characteristic files
            for i in range(3):
                char_path = os.path.join(temp_dir, f"taskset_characteristics_{i}.yaml")
                with open(char_path, "w") as f:
                    f.write("tasks:\n")
                    f.write("  - id: 0\n")
                    f.write("    deadline: 1000\n")

            task_deadlines = {0: 1000.0}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "BF")
            self.assertAlmostEqual(avg_sched_time, 2.0)  # 6.0 / 3 intervals
        finally:
            shutil.rmtree(temp_dir)

    def test_analyze_single_instance_cfs_exec_time_is_zero(self):
        """CFS should report 0.0 regardless of execution time file."""
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "CFS"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                f.write("0,0.88\n")

            summary_file = os.path.join(sched_dir, "miss_rate_summary.txt")
            with open(summary_file, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("10,0,0.0\n")

            exec_file = os.path.join(sched_dir, "scheduler_execution_time.txt")
            with open(exec_file, "w") as f:
                f.write("45.0\n")

            task_deadlines = {0: 1000.0}
            _, _, _, _, avg_sched_time = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)
            self.assertEqual(avg_sched_time, 0.0)
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("matplotlib.pyplot.savefig")
    def test_write_summary_and_plots(self, mock_savefig):
        temp_dir = tempfile.mkdtemp()
        try:
            results_by_scheduler = {
                "INCR": {
                    "sp_values": [0.9, 0.95],
                    "miss_rates": [0.1, 0.2],
                    "intervals": {
                        0: [0.9],
                        1: [0.95]
                    }
                }
            }
            write_summary_and_plots(
                results_by_scheduler=results_by_scheduler,
                schedulers=["INCR"],
                output_dir_abs=temp_dir,
                horizon_granularity=10
            )
            # Verify CSV file
            csv_path = os.path.join(temp_dir, "comparison_summary.csv")
            self.assertTrue(os.path.exists(csv_path))
            with open(csv_path, 'r') as f:
                lines = f.readlines()
            self.assertEqual(len(lines), 2)
            self.assertEqual(lines[0].strip(), "Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,Std_Miss_Rate,Mean_Scheduler_Execution_Time_s")
            parts = lines[1].strip().split(',')
            self.assertEqual(parts[0], "INCR")
            # Mean SP = 0.925
            self.assertAlmostEqual(float(parts[1]), 0.925)
            # Mean miss rate = 0.15
            self.assertAlmostEqual(float(parts[3]), 0.15)

            # Verify plots path savefig was called
            mock_savefig.assert_called_once()
        finally:
            shutil.rmtree(temp_dir)


if __name__ == "__main__":
    unittest.main()
