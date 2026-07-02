import unittest
import os
import json
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
    _should_generate,
)
import simulation_experiments.run_sim_experiments as run_sim_experiments
from simulation_experiments.utils import (
    compute_miss_rate,
    compute_miss_rate_by_task,
    compute_important_task_miss_rate,
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
            interval_duration_ms=1000,
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
            interval_duration_ms=1000,
            scheduler="BF",
            inst=1,
            verbose=0,
            export_level=2,
        )
        mock_run.assert_called_once_with(
            ["sim_bin", "taskset", "sched", "BF", "1000", "2"],
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
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "INCR")
            self.assertAlmostEqual(miss_rate, 0.1)
            self.assertEqual(sp_values_run, [0.95, 0.92])
            self.assertEqual(run_intervals_data, [(0, 0.95), (1, 0.92)])
            self.assertEqual(avg_sched_time, 0.0)
            self.assertEqual(imp_miss, 0.0)
            self.assertEqual(non_imp_miss, 0.0)
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
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "BF")
            self.assertAlmostEqual(miss_rate, 0.0)
            self.assertEqual(sp_values_run, [0.95])
            self.assertAlmostEqual(avg_sched_time, 1.23456)
            self.assertEqual(imp_miss, 0.0)
            self.assertEqual(non_imp_miss, 0.0)
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
                char_path = os.path.join(temp_dir, f"taskset_characteristics_interval_{i}.yaml")
                with open(char_path, "w") as f:
                    f.write("tasks:\n")
                    f.write("  - id: 0\n")
                    f.write("    deadline: 1000\n")

            task_deadlines = {0: 1000.0}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            self.assertEqual(sched_ret, "BF")
            self.assertAlmostEqual(avg_sched_time, 2.0)  # 6.0 / 3 intervals
            self.assertEqual(imp_miss, 0.0)
            self.assertEqual(non_imp_miss, 0.0)
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
            _, _, _, _, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)
            self.assertEqual(avg_sched_time, 0.0)
            self.assertEqual(imp_miss, 0.0)
            self.assertEqual(non_imp_miss, 0.0)
        finally:
            shutil.rmtree(temp_dir)

    def test_analyze_single_instance_with_important_tasks(self):
        """Important-task miss rate computed when task_sp_weights provided."""
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

            # Write miss_rate_per_task.txt (detail level 1+)
            per_task_file = os.path.join(sched_dir, "miss_rate_per_task.txt")
            with open(per_task_file, "w") as f:
                f.write("task_id,total_jobs,missed_jobs,miss_rate\n")
                f.write("0,10,1,0.10\n")
                f.write("1,10,5,0.50\n")
                f.write("2,10,0,0.00\n")
                f.write("3,10,2,0.20\n")

            task_deadlines = {0: 1000.0, 1: 1000.0, 2: 1000.0, 3: 1000.0}
            task_sp_weights = {0: 0.5, 1: 0.3, 2: 0.15, 3: 0.05}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(
                    temp_dir, scheduler, inst, task_deadlines, 10,
                    task_sp_weights=task_sp_weights,
                    important_pct=0.25, min_important=1)

            self.assertEqual(sched_ret, "INCR")
            # Top 25% => 1 task (task 0 with weight 0.5). miss_rate = 0.10
            self.assertAlmostEqual(imp_miss, 0.10)
            # Remaining tasks 1,2,3 => (0.5 + 0.0 + 0.2) / 3 = 0.2333...
            self.assertAlmostEqual(non_imp_miss, 0.233333, places=5)
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.utils.save_figure")
    def test_write_summary_and_plots(self, mock_save):
        temp_dir = tempfile.mkdtemp()
        try:
            results_by_scheduler = {
                "INCR": {
                    "sp_values": [0.9, 0.95],
                    "miss_rates": [0.1, 0.2],
                    "intervals": {
                        0: [0.9],
                        1: [0.95]
                    },
                    "sched_times": [0.01, 0.02],
                    "important_miss_rates": [0.05, 0.10],
                    "non_important_miss_rates": [0.15, 0.25],
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
            header = "Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,Std_Miss_Rate,Mean_Scheduler_Execution_Time_s,Important_Miss_Rate,Non_Important_Miss_Rate"
            self.assertEqual(lines[0].strip(), header)
            parts = lines[1].strip().split(',')
            self.assertEqual(parts[0], "INCR")
            # Mean SP = 0.925
            self.assertAlmostEqual(float(parts[1]), 0.925)
            # Mean miss rate = 0.15
            self.assertAlmostEqual(float(parts[3]), 0.15)
            # Mean important miss rate = 0.075
            self.assertAlmostEqual(float(parts[6]), 0.075)
            # Mean non-important miss rate = 0.2
            self.assertAlmostEqual(float(parts[7]), 0.2)

            # Verify save_figure was called once (writes PNG + PDF) with the
            # comparison_plots stem.
            mock_save.assert_called_once()
            called_stem = mock_save.call_args.args[1]
            self.assertTrue(called_stem.endswith("comparison_plots"))
        finally:
            shutil.rmtree(temp_dir)

    def test_analyze_single_instance_ignores_path_files(self):
        """Path-specific YAML files (i0_p0) must NOT be counted as interval files."""
        temp_dir = tempfile.mkdtemp()
        try:
            scheduler = "BF"
            inst = 0
            sched_dir = os.path.join(temp_dir, scheduler, scheduler)
            os.makedirs(sched_dir, exist_ok=True)

            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                f.write("0,0.95\n")

            summary_file = os.path.join(sched_dir, "miss_rate_summary.txt")
            with open(summary_file, "w") as f:
                f.write("total_jobs,missed_jobs,miss_rate\n")
                f.write("10,0,0.0\n")

            exec_file = os.path.join(sched_dir, "scheduler_execution_time.txt")
            with open(exec_file, "w") as f:
                f.write("6.0\n")

            # 2 genuine interval files
            for i in range(2):
                char_path = os.path.join(
                    temp_dir, f"taskset_characteristics_interval_{i}.yaml"
                )
                with open(char_path, "w") as f:
                    f.write("tasks:\n")
                    f.write("  - id: 0\n")
                    f.write("    deadline: 1000\n")

            # 3 path-specific files that must be ignored
            for i in range(3):
                for pp in range(2):
                    path_file = os.path.join(
                        temp_dir, f"taskset_characteristics_i{i}_p{pp}.yaml"
                    )
                    with open(path_file, "w") as f:
                        f.write("tasks:\n")
                        f.write("  - id: 0\n")
                        f.write("    deadline: 1000\n")

            task_deadlines = {0: 1000.0}
            sched_ret, miss_rate, sp_values_run, run_intervals_data, avg_sched_time, imp_miss, non_imp_miss = \
                analyze_single_instance(temp_dir, scheduler, inst, task_deadlines, 10)

            # 6.0 / 2 intervals = 3.0, NOT 6.0 / (2 + 6) = 0.75
            self.assertEqual(sched_ret, "BF")
            self.assertAlmostEqual(avg_sched_time, 3.0)
            self.assertEqual(imp_miss, 0.0)
            self.assertEqual(non_imp_miss, 0.0)
        finally:
            shutil.rmtree(temp_dir)


class TestComputeMissRateByTask(unittest.TestCase):

    def test_from_per_task_file(self):
        temp_dir = tempfile.mkdtemp()
        try:
            per_task_file = os.path.join(temp_dir, "miss_rate_per_task.txt")
            with open(per_task_file, "w") as f:
                f.write("task_id,total_jobs,missed_jobs,miss_rate\n")
                f.write("0,10,1,0.1\n")
                f.write("1,10,2,0.2\n")
                f.write("2,10,0,0.0\n")
            result = compute_miss_rate_by_task(temp_dir, {})
            self.assertEqual(result, {0: 0.1, 1: 0.2, 2: 0.0})
        finally:
            shutil.rmtree(temp_dir)

    def test_from_trace_files(self):
        temp_dir = tempfile.mkdtemp()
        try:
            with open(os.path.join(temp_dir, "response_times_task_0.txt"), "w") as f:
                f.write("jobId,release_time,start_time,finish_time,response_time,execution_time,is_overrun\n")
                f.write("0,0,0,200,200,200,0\n")
                f.write("1,100,100,300,200,200,0\n")
            with open(os.path.join(temp_dir, "response_times_task_1.txt"), "w") as f:
                f.write("jobId,release_time,start_time,finish_time,response_time,execution_time,is_overrun\n")
                f.write("0,0,0,80,80,80,0\n")
                f.write("1,100,100,120,120,120,0\n")
            task_deadlines = {0: 150.0, 1: 100.0}
            result = compute_miss_rate_by_task(temp_dir, task_deadlines)
            # task 0: both jobs rt=200 > 150 => 1.0
            # task 1: first job rt=80 <= 100, second rt=120 > 100 => 0.5
            self.assertAlmostEqual(result[0], 1.0)
            self.assertAlmostEqual(result[1], 0.5)
        finally:
            shutil.rmtree(temp_dir)

    def test_no_data(self):
        temp_dir = tempfile.mkdtemp()
        try:
            result = compute_miss_rate_by_task(temp_dir, {0: 100.0})
            self.assertEqual(result, {})
        finally:
            shutil.rmtree(temp_dir)


class TestComputeImportantTaskMissRate(unittest.TestCase):

    def test_basic(self):
        per_task_mr = {0: 0.1, 1: 0.2, 2: 0.0, 3: 0.5}
        task_weights = {0: 0.5, 1: 0.3, 2: 0.15, 3: 0.05}
        imp, non_imp = compute_important_task_miss_rate(per_task_mr, task_weights, important_pct=0.25, min_important=1)
        # Top 1 task => task 0 (weight 0.5). miss_rate = 0.1
        self.assertAlmostEqual(imp, 0.1)
        # Non-important => tasks 1,2,3 => (0.2 + 0.0 + 0.5) / 3 = 0.2333...
        self.assertAlmostEqual(non_imp, 0.233333, places=5)

    def test_empty_input(self):
        self.assertEqual(compute_important_task_miss_rate({}, {}, 0.1, 1), (0.0, 0.0))
        self.assertEqual(compute_important_task_miss_rate({0: 0.1}, {}, 0.1, 1), (0.0, 0.0))
        self.assertEqual(compute_important_task_miss_rate({}, {0: 1.0}, 0.1, 1), (0.0, 0.0))

    def test_single_task(self):
        per_task_mr = {0: 0.15}
        task_weights = {0: 1.0}
        imp, non_imp = compute_important_task_miss_rate(per_task_mr, task_weights, important_pct=0.1, min_important=1)
        self.assertAlmostEqual(imp, 0.15)
        self.assertAlmostEqual(non_imp, 0.0)

    def test_min_important_enforced(self):
        per_task_mr = {0: 0.1, 1: 0.2, 2: 0.3, 3: 0.4}
        task_weights = {0: 0.4, 1: 0.3, 2: 0.2, 3: 0.1}
        # 10% of 4 = 0.4 => rounded up = 1, but min_important=2 overrides
        imp, non_imp = compute_important_task_miss_rate(per_task_mr, task_weights, important_pct=0.1, min_important=2)
        # Top 2 tasks => 0, 1 (weights 0.4, 0.3) => (0.1 + 0.2) / 2 = 0.15
        self.assertAlmostEqual(imp, 0.15)
        # Remaining => tasks 2,3 => (0.3 + 0.4) / 2 = 0.35
        self.assertAlmostEqual(non_imp, 0.35)

    def test_all_tasks_important(self):
        per_task_mr = {0: 0.1, 1: 0.2}
        task_weights = {0: 0.6, 1: 0.4}
        imp, non_imp = compute_important_task_miss_rate(per_task_mr, task_weights, important_pct=1.0, min_important=1)
        # All tasks important
        self.assertAlmostEqual(imp, 0.15)
        self.assertAlmostEqual(non_imp, 0.0)

    def test_missing_miss_rate_for_important_task(self):
        per_task_mr = {0: 0.1}  # task 1 missing
        task_weights = {0: 0.6, 1: 0.4}
        imp, non_imp = compute_important_task_miss_rate(per_task_mr, task_weights, important_pct=0.5, min_important=1)
        # Top task => task 0 (has data). miss_rate = 0.1
        self.assertAlmostEqual(imp, 0.1)
        # task 1 is non-important but has no miss rate data => non_important only from available
        self.assertAlmostEqual(non_imp, 0.0)


class TestShouldGenerate(unittest.TestCase):
    """_should_generate replaces the old needs_generation/skip_generation
    decision with config-change detection: a taskset generated under an older
    generator config is never silently reused when the config has since changed.
    """

    def _make_taskset(self, base_dir, config, with_intervals=True):
        """Create a taskset dir with interval files + a saved generator_config.json."""
        taskset_dir = os.path.join(base_dir, "taskset_0")
        os.makedirs(taskset_dir, exist_ok=True)
        if with_intervals:
            for i in range(3):
                with open(os.path.join(
                    taskset_dir, f"taskset_characteristics_interval_{i}.yaml"), "w"
                ) as f:
                    f.write("tasks: []\n")
        if config is not None:
            with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
                json.dump(config, f)
        return taskset_dir

    def _assert_prompt_regenerates_noninteractive(self, taskset_dir, new_cfg):
        """Assert the 'prompt' policy regenerates on a config change in a
        non-interactive session.

        ``_should_generate`` queries the real ``sys.stdin.isatty()`` to decide
        whether to prompt a human or fall back to regenerating. Under an
        interactive test runner (a TTY-attached terminal) ``isatty()`` is True
        and the code would call ``input()`` and hang forever. Patch stdin to
        be non-interactive so the test exercises the fallback path
        deterministically, regardless of how pytest is invoked.
        """
        with unittest.mock.patch.object(sys.stdin, "isatty", return_value=False):
            self.assertTrue(_should_generate(
                taskset_dir, new_cfg, 0,
                skip_if_exists=True, on_change_policy="prompt"))

    def test_no_intervals_on_disk_always_generate(self):
        """Empty taskset dir -> True regardless of skip_if_exists / policy."""
        temp_dir = tempfile.mkdtemp()
        try:
            taskset_dir = os.path.join(temp_dir, "taskset_0")
            os.makedirs(taskset_dir, exist_ok=True)
            self.assertTrue(_should_generate(
                taskset_dir, {"RANDOM_SEED": 100}, 0,
                skip_if_exists=True, on_change_policy="keep"))
        finally:
            shutil.rmtree(temp_dir)

    def test_forced_regenerate_when_skip_is_false(self):
        """skip_if_exists=False forces regeneration even if config matches."""
        temp_dir = tempfile.mkdtemp()
        try:
            cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            taskset_dir = self._make_taskset(temp_dir, cfg)
            self.assertTrue(_should_generate(
                taskset_dir, cfg, 0,
                skip_if_exists=False, on_change_policy="keep"))
        finally:
            shutil.rmtree(temp_dir)

    def test_config_matches_reuses(self):
        """Identical saved + new config -> False (reuse)."""
        temp_dir = tempfile.mkdtemp()
        try:
            cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            taskset_dir = self._make_taskset(temp_dir, cfg)
            self.assertFalse(_should_generate(
                taskset_dir, cfg, 0,
                skip_if_exists=True, on_change_policy="prompt"))
        finally:
            shutil.rmtree(temp_dir)

    def test_config_changed_regenerate_policy(self):
        """on_change_policy='regenerate' -> True on a config change."""
        temp_dir = tempfile.mkdtemp()
        try:
            old_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            taskset_dir = self._make_taskset(temp_dir, old_cfg)
            new_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 30}  # interval changed
            self.assertTrue(_should_generate(
                taskset_dir, new_cfg, 0,
                skip_if_exists=True, on_change_policy="regenerate"))
        finally:
            shutil.rmtree(temp_dir)

    def test_config_changed_keep_policy_reuses(self):
        """on_change_policy='keep' -> False on a config change (explicit opt-out)."""
        temp_dir = tempfile.mkdtemp()
        try:
            old_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            taskset_dir = self._make_taskset(temp_dir, old_cfg)
            new_cfg = {"RANDOM_SEED": 200, "UPDATE_INTERVAL_S": 10}  # seed changed
            self.assertFalse(_should_generate(
                taskset_dir, new_cfg, 0,
                skip_if_exists=True, on_change_policy="keep"))
        finally:
            shutil.rmtree(temp_dir)

    def test_config_changed_prompt_noninteractive_regenerates(self):
        """on_change_policy='prompt' with no TTY (CI/subprocess) -> regenerate."""
        temp_dir = tempfile.mkdtemp()
        try:
            old_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            taskset_dir = self._make_taskset(temp_dir, old_cfg)
            new_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 60}
            # Patched non-interactive stdin -> regenerates (not the live TTY).
            self._assert_prompt_regenerates_noninteractive(taskset_dir, new_cfg)
        finally:
            shutil.rmtree(temp_dir)

    def test_corrupt_saved_config_treated_as_change(self):
        """A malformed generator_config.json is treated as a config change."""
        temp_dir = tempfile.mkdtemp()
        try:
            taskset_dir = self._make_taskset(temp_dir, None)
            with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
                f.write("{not valid json")
            new_cfg = {"RANDOM_SEED": 100, "UPDATE_INTERVAL_S": 10}
            # Corrupt old config != new -> prompt policy regenerates (non-TTY).
            self._assert_prompt_regenerates_noninteractive(taskset_dir, new_cfg)
        finally:
            shutil.rmtree(temp_dir)


class _SentinelStop(Exception):
    """Raised by the patched resolver to short-circuit main() after the
    routing assertion point (avoids needing the C++ binary)."""


class TestNumTasksCliAcceptance(unittest.TestCase):
    """P13 Commit B: --num_tasks is no longer restricted to choices=[4,6,8].
    When set, any N >= 2 routes through resolve_taskset_config_path; when
    unset, the --config_file fallback path is used (resolver not called).
    """

    def _run_main_catching_resolver(self, cli_args):
        """Run run_sim_experiments.main(); expect the patched resolver to raise
        a sentinel. Returns the num_tasks the resolver saw (None if not called)."""
        seen = {}

        def fake_resolver(num_tasks, *a, **kw):
            seen["num_tasks"] = num_tasks
            raise _SentinelStop()

        with unittest.mock.patch.object(
            run_sim_experiments, "resolve_taskset_config_path", side_effect=fake_resolver
        ), unittest.mock.patch.object(sys, "argv", ["run_sim_experiments.py"] + cli_args):
            with self.assertRaises(_SentinelStop):
                run_sim_experiments.main()
        return seen.get("num_tasks")

    def test_num_tasks_10_accepted_and_routed(self):
        """--num_tasks 10 is accepted (no choices rejection) and routed to the
        resolver instead of the old hard-coded paper_10 lookup."""
        n = self._run_main_catching_resolver(
            ["--num_tasks", "10", "--n_tasksets", "1", "-v", "0"]
        )
        self.assertEqual(n, 10)

    def test_num_tasks_below_two_rejected_by_argparse(self):
        """--num_tasks 1 is rejected with a parser error (SystemExit)."""
        with unittest.mock.patch.object(
            sys, "argv", ["run_sim_experiments.py", "--num_tasks", "1", "-v", "0"]
        ):
            with self.assertRaises(SystemExit):
                run_sim_experiments.main()


if __name__ == "__main__":
    unittest.main()
