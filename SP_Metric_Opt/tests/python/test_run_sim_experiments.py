import unittest
import os
import tempfile
import shutil
import sys

# Ensure project root and Visualize_SP_Metric is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from run_sim_experiments import (
    parse_sp_value_from_stdout,
    compute_miss_rate,
    combine_processor_results,
    run_single_simulation,
    analyze_single_instance,
    write_summary_and_plots
)
import unittest.mock

class TestRunSimExperiments(unittest.TestCase):

    def test_parse_sp_value_from_stdout_valid(self):
        stdout = (
            "Some initialization logs...\n"
            "SP-Metric: 1.800657\n"
            "Some final logs...\n"
        )
        val = parse_sp_value_from_stdout(stdout)
        self.assertAlmostEqual(val, 1.800657)

    def test_parse_sp_value_from_stdout_invalid(self):
        stdout = "No SP metric here"
        with self.assertRaises(ValueError):
            parse_sp_value_from_stdout(stdout)

    def test_compute_miss_rate(self):
        # Create temp file
        fd, temp_path = tempfile.mkstemp()
        try:
            with os.fdopen(fd, 'w') as f:
                # taskid, jobid, start_time, end_time, execution_time
                f.write("0,0,100,200,100\n")  # rt = 100
                f.write("0,1,500,700,200\n")  # rt = 200
                f.write("1,0,0,50,50\n")      # rt = 50
                f.write("1,1,200,350,150\n")    # rt = 150
            
            task_deadlines = {0: 150.0, 1: 100.0}
            # Expected misses:
            # task 0, job 1 (rt = 200 > 150.0) -> miss
            # task 1, job 1 (rt = 150 > 100.0) -> miss
            # total = 4 jobs, misses = 2, rate = 0.5
            rate = compute_miss_rate(temp_path, task_deadlines)
            self.assertAlmostEqual(rate, 0.5)
        finally:
            os.remove(temp_path)

    def test_combine_processor_results(self):
        # Create temp directory
        temp_dir = tempfile.mkdtemp()
        try:
            sched_dir = os.path.join(temp_dir, "scheduler_dir")
            os.makedirs(sched_dir)
            
            # Write core-local results for p0 and p1
            # Format: taskid, jobid, start_time, end_time, execution_time
            p0_res = os.path.join(sched_dir, "sim_res_TEST_0_p0.txt")
            with open(p0_res, 'w') as f:
                f.write("0,0,10,20,10\n")  # local task 0
                f.write("1,0,20,40,20\n")  # local task 1
            
            p1_res = os.path.join(sched_dir, "sim_res_TEST_0_p1.txt")
            with open(p1_res, 'w') as f:
                f.write("0,0,30,50,20\n")  # local task 0
                f.write("1,0,60,90,30\n")  # local task 1
            
            # Mock global tasks list
            # Global ID 0: task_A on p1 (local 0)
            # Global ID 1: task_B on p0 (local 0)
            # Global ID 2: task_C on p0 (local 1)
            # Global ID 3: task_D on p1 (local 1)
            yaml_tasks = [
                {"id": 0, "name": "task_A", "processorId": 1},
                {"id": 1, "name": "task_B", "processorId": 0},
                {"id": 2, "name": "task_C", "processorId": 0},
                {"id": 3, "name": "task_D", "processorId": 1},
            ]
            
            combined_res = os.path.join(sched_dir, "sim_res_TEST_0.txt")
            combined_log = os.path.join(sched_dir, "sim_log_TEST_0.txt")
            
            combine_processor_results(
                sched_dir=sched_dir,
                scheduler="TEST",
                inst=0,
                n_cores=2,
                yaml_tasks=yaml_tasks,
                res_file=combined_res,
                log_file=combined_log
            )
            
            # Verify mapping:
            # p0:
            #   local 0 -> global 1 (task_B)
            #   local 1 -> global 2 (task_C)
            # p1:
            #   local 0 -> global 0 (task_A)
            #   local 1 -> global 3 (task_D)
            with open(combined_res, 'r') as f:
                lines = f.readlines()
                
            self.assertEqual(len(lines), 4)
            # Order combined from p0 then p1:
            # p0 lines:
            self.assertEqual(lines[0].strip(), "1,0,10,20,10") # local 0 maps to global 1
            self.assertEqual(lines[1].strip(), "2,0,20,40,20") # local 1 maps to global 2
            # p1 lines:
            self.assertEqual(lines[2].strip(), "0,0,30,50,20") # local 0 maps to global 0
            self.assertEqual(lines[3].strip(), "3,0,60,90,30") # local 1 maps to global 3
            
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("run_sim_experiments.subprocess.run")
    def test_run_single_simulation(self, mock_run):
        run_single_simulation(
            sim_bin_path="sim_bin",
            taskset_dir="taskset",
            sched_dir="sched",
            simt=1000,
            scheduler="INCR",
            inst=0
        )
        mock_run.assert_called_once_with([
            "sim_bin",
            "--input_folder", "taskset",
            "--output_folder", "sched",
            "--simt", "1000",
            "--scheduler", "INCR",
            "--output_job_not_executed", "1",
            "--inst_idx", "0",
            "--verbose", "0"
        ], check=True, stdout=unittest.mock.ANY, stderr=unittest.mock.ANY)

    @unittest.mock.patch("run_sim_experiments.subprocess.run")
    def test_analyze_single_instance(self, mock_run):
        # Create temp folder representing taskset_dir
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "INCR"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler)
            os.makedirs(sched_dir)
            
            # Write a mock sim_res_INCR_0_p0.txt
            # Format: taskid, jobid, start_time, end_time, execution_time
            # task 0, job 0: start at 1000 ms, end at 1500 ms (exet 500 ms) -> rt = 500 ms
            res_file = os.path.join(sched_dir, f"sim_res_{scheduler}_{inst}_p0.txt")
            with open(res_file, 'w') as f:
                f.write("0,0,1000,1500,500\n")
            
            # Create a mock config dict
            config_dict = {"N_CORES": 1}
            # Create a mock yaml tasks list
            yaml_data = {
                "tasks": [
                    {"id": 0, "name": "MPC", "deadline": 1000.0}
                ]
            }
            task_deadlines = {0: 1000.0}
            
            # Mock subprocess.run for AnalyzeSP_Metric
            # It should return a completed process with stdout containing "SP-Metric: 0.95"
            mock_res = unittest.mock.MagicMock()
            mock_res.stdout = "SP-Metric: 0.95\n"
            mock_run.return_value = mock_res
            
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time = analyze_single_instance(
                taskset_dir=temp_dir,
                scheduler=scheduler,
                inst=inst,
                config_dict=config_dict,
                yaml_data=yaml_data,
                task_deadlines=task_deadlines,
                analyze_bin_path="analyze_bin",
                horizon_granularity=10
            )
            
            self.assertEqual(sched_ret, "INCR")
            self.assertAlmostEqual(miss_rate, 0.0) # rt (500) < deadline (1000)
            self.assertEqual(sp_values_run, [0.95])
            self.assertEqual(run_intervals_data, [(0, 0.95)])
            
            # Check files were copied / renamed
            self.assertTrue(os.path.exists(os.path.join(sched_dir, "response_times_0.csv")))
            self.assertTrue(os.path.exists(os.path.join(sched_dir, "simulation_log_0.txt")))
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
