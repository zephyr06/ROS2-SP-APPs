"""Tests for simulation_experiments.run_end_to_end_experiments.

These tests mock ``subprocess.run`` so the orchestrator is exercised without
launching real simulations. They verify that the correct commands are
constructed for each stage, that the main+ablation scheduler lists are
unioned and de-duplicated, that ``--dry_run`` prints without executing, and
that the pipeline always runs all three stages in fixed order (there is no
``--steps`` flag -- simulate, sweep, and aggregate are dependent and always
run together).
"""
import os
import sys
import unittest
import unittest.mock

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.run_end_to_end_experiments as e2e
from simulation_experiments.experiment_config_loader import load_experiment_config


def _cfg(mode="test"):
    """Load a real config so the constructed commands reflect actual values."""
    return load_experiment_config(mode=mode)


class TestSchedulerUnion(unittest.TestCase):

    def test_union_dedup_preserves_order(self):
        cfg = {
            "main_scheduler_list": ["INCR", "BF", "RM", "CFS"],
            "ablation_scheduler_list": ["BF", "INCR", "INCR_NO_TL", "INCR_WCET"],
        }
        union = e2e.build_scheduler_union(cfg)
        # BF and INCR appear in both lists but only once in the union
        self.assertEqual(union, ["INCR", "BF", "RM", "CFS",
                                 "INCR_NO_TL", "INCR_WCET"])
        self.assertEqual(len(union), len(set(union)), "union contains duplicates")

    def test_union_from_real_config(self):
        cfg = _cfg("test")
        union = e2e.build_scheduler_union(cfg)
        # Every main and ablation scheduler is present, with no duplicates
        for s in cfg["main_scheduler_list"] + cfg["ablation_scheduler_list"]:
            self.assertIn(s, union)
        self.assertEqual(len(union), len(set(union)))


class TestBuildCommands(unittest.TestCase):

    def test_simulate_command_carries_config_values(self):
        cfg = _cfg("test")
        cfg["bin_dir"] = "release"
        run_root = e2e.build_run_root("/tmp/out", cfg)
        cmd = e2e.build_simulate_command(6, cfg, "/tmp/out", verbose=1,
                                         run_root=run_root)

        self.assertEqual(cmd[1], "-m")
        self.assertEqual(cmd[2], "simulation_experiments.compare_optimizers")
        # num_tasks, n_tasksets, n_sec, trigger interval, seed from config
        self.assertIn("--num_tasks", cmd)
        self.assertEqual(cmd[cmd.index("--num_tasks") + 1], "6")
        self.assertEqual(cmd[cmd.index("--n_tasksets") + 1],
                         str(cfg["num_tasksets_to_generate"]))
        self.assertEqual(cmd[cmd.index("--n_sec") + 1],
                         str(cfg["simulation_duration_seconds"]))
        self.assertEqual(cmd[cmd.index("--scheduler_trigger_interval") + 1],
                         str(cfg["scheduler_trigger_interval_seconds"]))
        self.assertEqual(cmd[cmd.index("--base_seed") + 1],
                         str(cfg["base_random_seed"]))
        # P23: --run_root is forwarded so sims land under <run_root>/sim/.
        self.assertIn("--run_root", cmd)
        self.assertEqual(cmd[cmd.index("--run_root") + 1], run_root)
        # Scheduler union is passed positionally after --schedulers
        sched_idx = cmd.index("--schedulers")
        schedulers_passed = cmd[sched_idx + 1:cmd.index("--bin_dir", sched_idx)]
        union = e2e.build_scheduler_union(cfg)
        self.assertEqual(schedulers_passed, union)
        # Important-task threshold flows through from the analysis block
        self.assertEqual(cmd[cmd.index("--important_task_pct") + 1],
                         str(cfg["analysis"]["important_task_top_percentage"]))

    def test_simulate_command_resume_flag(self):
        cfg = _cfg("test")
        cfg["bin_dir"] = "release"
        cfg["analysis"]["enable_resume_from_existing_results"] = True
        run_root = e2e.build_run_root("/tmp/out", cfg)
        cmd = e2e.build_simulate_command(4, cfg, "/tmp/out", verbose=1,
                                         run_root=run_root)
        self.assertIn("--resume", cmd)

    def test_simulate_command_no_resume_by_default(self):
        cfg = _cfg("test")
        cfg["bin_dir"] = "release"
        cfg["analysis"]["enable_resume_from_existing_results"] = False
        run_root = e2e.build_run_root("/tmp/out", cfg)
        cmd = e2e.build_simulate_command(4, cfg, "/tmp/out", verbose=1,
                                         run_root=run_root)
        self.assertNotIn("--resume", cmd)

    def test_simulate_command_num_workers(self):
        cfg = _cfg("test")
        cfg["bin_dir"] = "release"
        cfg["parallel_worker_processes"] = 8
        run_root = e2e.build_run_root("/tmp/out", cfg)
        cmd = e2e.build_simulate_command(8, cfg, "/tmp/out", verbose=1,
                                         run_root=run_root)
        self.assertEqual(cmd[cmd.index("--num_workers") + 1], "8")

    def test_simulate_command_keeps_prompt_policy(self):
        """P20: the simulate stage keeps compare_optimizers' default 'prompt'
        policy -- the orchestrator shares tasksets across stages, it does NOT
        silence the config-drift guard the user relies on.
        """
        cfg = _cfg("test")
        cfg["bin_dir"] = "release"
        run_root = e2e.build_run_root("/tmp/out", cfg)
        cmd = e2e.build_simulate_command(6, cfg, "/tmp/out", verbose=1,
                                         run_root=run_root)
        self.assertNotIn("--on_taskset_config_change", cmd)

    def test_sweep_command(self):
        cfg = _cfg("test")
        cmd = e2e.build_sweep_command(cfg, "/tmp/out", verbose=1)
        self.assertEqual(cmd[2], "simulation_experiments.interval_sweep")
        self.assertEqual(cmd[cmd.index("--mode") + 1], "test")
        self.assertEqual(cmd[cmd.index("--output_parent") + 1], "/tmp/out")

    def test_sweep_command_reuses_matching_interval(self):
        """P20: the sweep stage reuses the main step's tasksets for a matching
        interval. The regeneration policy is left at the sweep's default
        ('prompt') -- not overridden -- so drift still surfaces."""
        cfg = _cfg("test")
        cmd = e2e.build_sweep_command(cfg, "/tmp/out", verbose=1)
        # Reuse flag is present (store_true -- no value follows).
        self.assertIn("--reuse_matching_interval", cmd)
        # Policy is NOT overridden by the orchestrator (default 'prompt' kept).
        self.assertNotIn("--on_taskset_config_change", cmd)

    def test_aggregate_command(self):
        cfg = _cfg("prod")
        cmd = e2e.build_aggregate_command(cfg, "/tmp/out")
        self.assertEqual(cmd[2], "simulation_experiments.aggregate_across_tasks")
        self.assertEqual(cmd[cmd.index("--mode") + 1], "prod")
        self.assertEqual(cmd[cmd.index("--output_parent") + 1], "/tmp/out")


class TestRunCommand(unittest.TestCase):

    def test_dry_run_does_not_execute(self):
        with unittest.mock.patch.object(e2e.subprocess, "run") as mock_run:
            ok = e2e.run_command(["python", "-c", "print('hi')"], dry_run=True)
            self.assertTrue(ok)
            mock_run.assert_not_called()

    def test_real_run_calls_subprocess(self):
        with unittest.mock.patch.object(e2e.subprocess, "run") as mock_run:
            mock_run.return_value = unittest.mock.MagicMock(returncode=0)
            ok = e2e.run_command(["python", "-c", "print('hi')"], dry_run=False)
            self.assertTrue(ok)
            mock_run.assert_called_once()

    def test_failed_run_returns_false(self):
        with unittest.mock.patch.object(e2e.subprocess, "run") as mock_run:
            mock_run.side_effect = e2e.subprocess.CalledProcessError(1, "cmd")
            ok = e2e.run_command(["python", "-c", "print('hi')"], dry_run=False)
            self.assertFalse(ok)


class TestMainPipeline(unittest.TestCase):
    """Drive main() end-to-end with run_command / subprocess.run mocked."""

    def _run_main(self, extra_args, mode="test"):
        """Run main() with run_command mocked; return (exit_code, mock_rc).

        Counting ``run_command`` calls (one per constructed command) is the
        mode-independent way to assert how many commands were built -- in
        dry-run mode ``run_command`` returns early and never reaches
        ``subprocess.run``, so counting subprocess calls would be wrong.
        """
        argv = ["prog", "--mode", mode] + extra_args
        with unittest.mock.patch.object(sys, "argv", argv), \
             unittest.mock.patch.object(e2e, "run_command", return_value=True) as mock_rc, \
             unittest.mock.patch.object(e2e, "time") as mock_time:
            mock_time.time.return_value = 0.0
            try:
                e2e.main()
                exit_code = 0
            except SystemExit as exc:
                exit_code = exc.code
            return exit_code, mock_rc

    def test_dry_run_prints_and_does_not_execute(self):
        argv = ["prog", "--mode", "test", "--dry_run"]
        with unittest.mock.patch.object(sys, "argv", argv), \
             unittest.mock.patch.object(e2e.subprocess, "run") as mock_run, \
             unittest.mock.patch.object(e2e, "time") as mock_time:
            mock_time.time.return_value = 0.0
            try:
                e2e.main()
                exit_code = 0
            except SystemExit as exc:
                exit_code = exc.code
        self.assertEqual(exit_code, 0)
        # Dry run never invokes subprocess.run
        self.assertEqual(mock_run.call_count, 0)

    def test_all_stages_always_run(self):
        # There is no --steps flag: the pipeline always runs simulate (once per
        # task count) + sweep + aggregate. test_mode has 2 task counts ->
        # 2 simulate cmds + 1 sweep + 1 aggregate = 4 run_command calls.
        exit_code, mock_rc = self._run_main(["--dry_run"])
        self.assertEqual(exit_code, 0)
        cfg = _cfg("test")
        expected_simulate = len(cfg["num_tasks_for_cross_task_comparison"])
        self.assertEqual(mock_rc.call_count, expected_simulate + 2)

    def test_single_task_figures_zero_skips_sweep(self):
        """num_tasks_for_single_task_figures=0 opts out of the single-task
        sweep (Fig 2) and the single-task figures (Fig 1F/3). The sweep stage
        returns early without building a command, so run_command is called once
        FEWER (simulate + aggregate only, no sweep). The cross-task figures are
        unaffected -- aggregate still runs.
        """
        cfg = _cfg("test")
        cfg["num_tasks_for_single_task_figures"] = 0
        with unittest.mock.patch.object(e2e, "load_experiment_config",
                                        return_value=cfg), \
             unittest.mock.patch.object(sys, "argv",
                                        ["prog", "--mode", "test", "--dry_run"]), \
             unittest.mock.patch.object(e2e, "run_command", return_value=True) as mock_rc, \
             unittest.mock.patch.object(e2e, "time") as mock_time:
            mock_time.time.return_value = 0.0
            try:
                e2e.main()
                exit_code = 0
            except SystemExit as exc:
                exit_code = exc.code
        self.assertEqual(exit_code, 0)
        # simulate (once per cross-task count) + aggregate; sweep SKIPPED.
        expected = len(cfg["num_tasks_for_cross_task_comparison"]) + 1
        self.assertEqual(mock_rc.call_count, expected)
        # The aggregate command still runs (cross-task figures unaffected);
        # the sweep command is NOT built (early return). run_command(cmd, ...)
        # -> cmd is the first positional arg (an argv list), so join to a str.
        cmds = [" ".join(c.args[0]) for c in mock_rc.call_args_list]
        self.assertTrue(any("aggregate_across_tasks" in cmd for cmd in cmds),
                        "aggregate stage must still run")
        self.assertFalse(any("interval_sweep" in cmd for cmd in cmds),
                         "sweep stage must be skipped")

    def test_steps_flag_rejected(self):
        # --steps was removed; argparse must reject it (unrecognized arg).
        argv = ["prog", "--mode", "test", "--dry_run", "--steps", "simulate"]
        with unittest.mock.patch.object(sys, "argv", argv), \
             unittest.mock.patch.object(e2e, "time") as mock_time:
            mock_time.time.return_value = 0.0
            with self.assertRaises(SystemExit) as ctx:
                e2e.main()
        # argparse exits with code 2 on argument errors.
        self.assertEqual(ctx.exception.code, 2)

    def test_failed_stage_aborts(self):
        # The first stage (simulate for the first task count) fails; the
        # pipeline must abort immediately and not run sweep/aggregate.
        argv = ["prog", "--mode", "test", "--dry_run"]
        with unittest.mock.patch.object(sys, "argv", argv), \
             unittest.mock.patch.object(e2e, "run_command", return_value=False) as mock_rc, \
             unittest.mock.patch.object(e2e, "time") as mock_time:
            mock_time.time.return_value = 0.0
            with self.assertRaises(SystemExit) as ctx:
                e2e.main()
            self.assertNotEqual(ctx.exception.code, 0)
            # run_command should have been called once (first task count), then abort
            self.assertEqual(mock_rc.call_count, 1)


if __name__ == "__main__":
    unittest.main()
