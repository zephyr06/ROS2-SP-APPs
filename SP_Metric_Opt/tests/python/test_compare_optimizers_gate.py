"""P2.17 — ``compare_optimizers.py`` must route generation through P0.8's
important-task schedulability gate.

Background (see ``agents/active_tasks/P2_17_p06_p08_gate_consistency_gap/``):
``compare_optimizers.py`` used to call the UNGATED
``run_full_generation_pipeline`` for every taskset (lines 591/604) and had no
``--important_tasks_schedulability_check`` flag at all. So tasksets that are
unschedulable for the important tasks at the worst case were emitted freely,
reached the sim, and tripped P0.6's ``ComputeSafeFallback`` loud-fail
("No safe fallback exists for this task set — regenerate a new task set"),
which P1.15 layer-A turns into a hard run abort. This blocked P0.7's Step-3
SP-penalty A/B at N=6 (and the compare_against_bf N=4 run).

The fix mirrors ``run_sim_experiments.py``'s gate wiring exactly:
  - a ``--important_tasks_schedulability_check`` BooleanOptionalAction, default ON,
  - when ON, generation goes through
    ``run_full_generation_pipeline_with_important_task_gate`` AND the
    per-taskset base seed is spaced by ``IMPORTANT_TASK_GATE_MAX_ATTEMPTS``
    (20) so the gate's internal +0..19 retry window for taskset k cannot
    collide with taskset k+1's starting draw,
  - when OFF, the legacy ungated pipeline + the +1 per-taskset seed step
    (bit-identical to the pre-P2.17 behavior).

These tests pin BOTH the routing AND the seed step. They mock the two
generation entry points as attributes on the ``compare_optimizers`` module (so
the production code must import them by name) and the
``run_single_simulation`` sim path (no C++ binary needed). TDD-first: RED
against the ungated code, GREEN after the wiring lands.
"""
import os
import sys
import json
import shutil
import tempfile
import unittest
import unittest.mock

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.compare_optimizers as compare_optimizers
from simulation_experiments.compare_optimizers import main as compare_main
from Gen_Taskset.lib.orchestrator import IMPORTANT_TASK_GATE_MAX_ATTEMPTS


# The config dict main() builds when load_generation_config is patched to
# return this object (RANDOM_SEED + UPDATE_INTERVAL_S stamped per taskset).
# _should_generate compares it against the on-disk generator_config.json, so
# the saved file must deep-equal the stamped dict.
_FAKE_CONFIG = {
    "N_TASKS": 4,
    "N_CORES": 2,
    "RANDOM_SEED": 1000,       # stamped per-taskset in main()
    "UPDATE_INTERVAL_S": 10,   # stamped per-taskset in main()
}


def _write_taskset_characteristics(taskset_dir, num_tasks=4):
    """Write a minimal taskset_characteristics_interval_0.yaml so generation
    can be skipped and analyze can read deadlines/sp_weights."""
    import yaml
    os.makedirs(taskset_dir, exist_ok=True)
    tasks = []
    for i in range(num_tasks):
        tasks.append({
            "id": i, "period": 100, "deadline": 1000,
            "sp_threshold": 0.5, "sp_weight": 1.25, "name": f"task_{i+1}",
        })
    char_path = os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml")
    with open(char_path, "w") as f:
        yaml.safe_dump({"tasks": tasks}, f)


def _fake_run_single_success(sim_bin_path, taskset_dir, sched_dir,
                             interval_duration_ms, scheduler, inst,
                             verbose=1, export_level=None):
    """A ``run_single_simulation`` fake that writes the valid outputs the real
    binary would write (so analyze_single_instance succeeds). No subprocess."""
    nested = os.path.join(sched_dir, scheduler)
    os.makedirs(nested, exist_ok=True)
    with open(os.path.join(nested, "interval_sp_metrics.txt"), "w") as f:
        for i in range(3):
            f.write(f"{i},0.90\n")
    with open(os.path.join(nested, "miss_rate_summary.txt"), "w") as f:
        f.write("total_jobs,missed_jobs,miss_rate\n")
        f.write("30,1,0.033333\n")
    with open(os.path.join(nested, "scheduler_execution_time.txt"), "w") as f:
        f.write("0.05\n")


class _GateCallRecorder:
    """Wraps two fakes (gated + ungated generation) and records which was
    called and with what ``cfg_file``. The seed is read back from the temp
    config file main() writes (so the recorder sees the EXACT per-taskset
    seed the production code stamped, not a pre-patch value)."""

    def __init__(self):
        self.gated_calls = []      # list of cfg_file paths
        self.ungated_calls = []    # list of cfg_file paths

    def _record(self, bucket, cfg_file, **kwargs):
        seed = None
        try:
            with open(cfg_file) as f:
                seed = json.load(f).get("RANDOM_SEED")
        except (OSError, ValueError):
            pass
        bucket.append({"cfg_file": cfg_file, "seed": seed, "kwargs": kwargs})

    def gated(self, cfg_file, **kwargs):
        self._record(self.gated_calls, cfg_file, **kwargs)

    def ungated(self, cfg_file, **kwargs):
        self._record(self.ungated_calls, cfg_file, **kwargs)


class TestCompareOptimizersGateWiring(unittest.TestCase):
    """P2.17 D1: ``compare_optimizers.py`` must route generation through P0.8's
    gate by default and use the +20 per-taskset seed step."""

    def _run_main(self, tmp, extra_argv, n_tasksets=3):
        """Run compare_optimizers.main() with generation mocked (so no real
        taskset is generated) and the sim path mocked to succeed.

        Patches:
        - ``resolve_taskset_config_path`` -> a temp config path.
        - ``load_generation_config`` -> _FAKE_CONFIG (so the stamped config
          deep-equals the saved generator_config.json -> reuse, no regen via
          _should_generate... but we FORCE generation below to observe the
          routing).
        - ``_should_generate`` -> always True (force the generation branch so
          the recorder observes which entry point main() calls).
        - ``run_full_generation_pipeline`` / ``run_full_generation_pipeline_with_important_task_gate``
          -> recorder fakes.
        - ``run_single_simulation`` -> success fake.
        - ``MATPLOTLIB_AVAILABLE`` -> False.

        Returns the recorder.
        """
        base_output = os.path.join(tmp, "out")
        os.makedirs(base_output, exist_ok=True)
        # resolve_run_output_dir auto-name for num_tasks=4, n_sec=300,
        # interval=10, base_seed=1000 (the default).
        run_dir = os.path.join(base_output, "tasks4_dur300_interval10_seed1000")
        for idx in range(n_tasksets):
            _write_taskset_characteristics(os.path.join(run_dir, f"taskset_{idx}"))

        recorder = _GateCallRecorder()

        argv = [
            "compare_optimizers.py",
            "--num_tasks", "4",
            "--n_tasksets", str(n_tasksets),
            "--schedulers", "BF", "INCR_Reopt_10",
            "-o", base_output,
            "--scheduler_trigger_interval", "10",
            "--base_seed", "1000",
            "-v", "0",
        ] + extra_argv

        with unittest.mock.patch.object(
            compare_optimizers, "resolve_taskset_config_path",
            return_value=os.path.join(tmp, "fake_cfg.json"),
        ), unittest.mock.patch.object(
            compare_optimizers, "load_generation_config",
            return_value=dict(_FAKE_CONFIG),
        ), unittest.mock.patch.object(
            compare_optimizers, "_should_generate",
            return_value=True,
        ), unittest.mock.patch.object(
            compare_optimizers, "run_full_generation_pipeline",
            side_effect=recorder.ungated,
        ), unittest.mock.patch.object(
            compare_optimizers, "run_full_generation_pipeline_with_important_task_gate",
            side_effect=recorder.gated,
        ), unittest.mock.patch.object(
            compare_optimizers, "run_single_simulation",
            side_effect=_fake_run_single_success,
        ), unittest.mock.patch.object(
            compare_optimizers, "MATPLOTLIB_AVAILABLE", False,
        ), unittest.mock.patch.object(sys, "argv", argv):
            try:
                compare_main()
            except SystemExit:
                pass

        return recorder

    def test_gate_on_by_default_routes_through_gated_pipeline(self):
        """With NO flag passed, generation must go through the gated pipeline
        (the default-ON behavior that mirrors run_sim_experiments.py)."""
        tmp = tempfile.mkdtemp()
        try:
            recorder = self._run_main(tmp, extra_argv=[])
            self.assertEqual(len(recorder.gated_calls), 3,
                             "default-ON gate must route every taskset through "
                             "the gated pipeline")
            self.assertEqual(len(recorder.ungated_calls), 0,
                             "default-ON gate must NOT touch the ungated pipeline")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_gate_on_uses_widened_seed_step(self):
        """When the gate is ON, the per-taskset base seed must be spaced by
        IMPORTANT_TASK_GATE_MAX_ATTEMPTS (20) so the gate's +0..19 retry
        window for taskset k cannot collide with taskset k+1's draw.

        base_seed=1000 -> tasksets get seeds 1000, 1020, 1040 (NOT 1000,
        1001, 1002)."""
        tmp = tempfile.mkdtemp()
        try:
            recorder = self._run_main(tmp, extra_argv=[])
            seeds = [c["seed"] for c in recorder.gated_calls]
            expected = [1000 + i * IMPORTANT_TASK_GATE_MAX_ATTEMPTS
                        for i in range(len(seeds))]
            self.assertEqual(seeds, expected,
                             f"gate-ON seeds must step by {IMPORTANT_TASK_GATE_MAX_ATTEMPTS}; "
                             f"got {seeds}")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_gate_off_routes_through_ungated_pipeline(self):
        """``--no-important_tasks_schedulability_check`` must route generation
        through the ungated pipeline (the diagnostic opt-out)."""
        tmp = tempfile.mkdtemp()
        try:
            recorder = self._run_main(
                tmp, extra_argv=["--no-important_tasks_schedulability_check"])
            self.assertEqual(len(recorder.ungated_calls), 3,
                             "gate OFF must route every taskset through the "
                             "ungated pipeline")
            self.assertEqual(len(recorder.gated_calls), 0,
                             "gate OFF must NOT touch the gated pipeline")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_gate_off_uses_legacy_plus_one_seed_step(self):
        """When the gate is OFF, the per-taskset seed must use the legacy +1
        step (bit-identical to the pre-P2.17 behavior): 1000, 1001, 1002."""
        tmp = tempfile.mkdtemp()
        try:
            recorder = self._run_main(
                tmp, extra_argv=["--no-important_tasks_schedulability_check"])
            seeds = [c["seed"] for c in recorder.ungated_calls]
            expected = [1000 + i for i in range(len(seeds))]
            self.assertEqual(seeds, expected,
                             f"gate-OFF seeds must step by 1 (legacy); got {seeds}")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
