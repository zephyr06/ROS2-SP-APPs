"""P1.19 — ``--rerun_mode`` policy tests for the e2e orchestrator.

The end-to-end orchestrator gained a single up-front knob,
``--rerun_mode {reuse, clear_all}`` (default ``reuse``), that decides how to
treat prior run artifacts before the simulate/sweep/aggregate stages run. The
policy lives in :func:`apply_rerun_mode`; it runs BEFORE any stage so it
preempts each stage's own reuse/resume guards rather than fighting them.

These tests pin the destructive behavior directly (no subprocess, no real
simulation) so a future change to ``apply_rerun_mode`` cannot silently regress
the wipe semantics:

- ``reuse`` (default) is a TRUE no-op: a populated ``sim/`` tree is left
  intact. This is the contract that keeps the bare ``./run_end_to_end.sh``
  invocation unchanged.
- ``clear_all`` wipes the WHOLE ``<run_root>/sim/`` tree (generated tasksets +
  per-scheduler results + sweep variants) — exactly "remove all existing
  generated task sets to re-run." It must NOT touch the sibling ``figures/``
  tree (figures are terminal output, not tasksets/sim results).
- ``clear_all`` on a run root with no ``sim/`` directory is a no-op (not an
  error) — clearing never aborts the pipeline.
- ``dry_run=True`` prints the intent and touches nothing.
- An unknown mode warns and is a no-op (defensive; argparse choices prevent
  this in practice, but the function is also called directly by tests).
"""
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.run_end_to_end_experiments as e2e


class TestApplyRerunMode(unittest.TestCase):
    """Behavioral pin for ``apply_rerun_mode`` (the --rerun_mode policy)."""

    def setUp(self):
        # A throwaway run root: <tmp>/runs/<run_id>/ with sim/ + figures/ subtrees.
        self.tmp = tempfile.mkdtemp(prefix="p119_rerun_")
        self.run_root = os.path.join(self.tmp, "runs",
                                     "run_test_dur70_interval10_seed1000_tasks4x6")
        self.sim_dir = os.path.join(self.run_root, "sim")
        self.figures_dir = os.path.join(self.run_root, "figures")

        # Populate a realistic-ish sim tree: a taskset dir + a per-scheduler
        # result file + a sweep artifact. Plus a sibling figures/ tree that
        # MUST survive clear_all.
        os.makedirs(os.path.join(
            self.sim_dir, "tasks6_dur70_interval10_seed1000", "taskset_0", "INCR"))
        with open(os.path.join(self.sim_dir, "tasks6_dur70_interval10_seed1000",
                               "taskset_0", "INCR", "interval_sp_metrics.txt"), "w") as f:
            f.write("pretend metrics\n")
        with open(os.path.join(self.sim_dir, "sweep_dummy.txt"), "w") as f:
            f.write("pretend sweep artifact\n")
        os.makedirs(self.figures_dir)
        with open(os.path.join(self.figures_dir, "fig1.png"), "w") as f:
            f.write("pretend figure\n")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _sim_files_present(self):
        return os.path.isdir(self.sim_dir) and bool(os.listdir(self.sim_dir))

    # --- reuse (default) ---

    def test_reuse_leaves_sim_tree_intact(self):
        """``reuse`` is a true no-op: nothing removed, no error."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            ok = e2e.apply_rerun_mode("reuse", self.run_root,
                                      dry_run=False, verbose=1)
        self.assertTrue(ok)
        self.assertTrue(self._sim_files_present(),
                        "reuse must not remove any sim artifacts")
        # The planted files are still there.
        self.assertTrue(os.path.exists(os.path.join(
            self.sim_dir, "sweep_dummy.txt")))
        self.assertTrue(os.path.exists(os.path.join(
            self.sim_dir, "tasks6_dur70_interval10_seed1000",
            "taskset_0", "INCR", "interval_sp_metrics.txt")))
        # reuse prints nothing about removing (it is a no-op).
        self.assertNotIn("removing", buf.getvalue().lower())

    # --- clear_all ---

    def test_clear_all_wipes_sim_tree(self):
        """``clear_all`` removes the whole <run_root>/sim/ tree."""
        ok = e2e.apply_rerun_mode("clear_all", self.run_root,
                                  dry_run=False, verbose=1)
        self.assertTrue(ok)
        self.assertFalse(os.path.isdir(self.sim_dir),
                         "clear_all must remove the sim/ directory")

    def test_clear_all_keeps_figures_tree(self):
        """``clear_all`` must NOT touch the sibling figures/ tree."""
        e2e.apply_rerun_mode("clear_all", self.run_root,
                             dry_run=False, verbose=1)
        self.assertTrue(os.path.isdir(self.figures_dir),
                        "clear_all must not remove figures/")
        self.assertTrue(os.path.exists(os.path.join(self.figures_dir, "fig1.png")),
                        "clear_all must not remove figures/ contents")

    def test_clear_all_missing_sim_is_noop(self):
        """``clear_all`` on a run root with no sim/ is a no-op, not an error."""
        run_root_no_sim = os.path.join(self.tmp, "runs", "run_with_no_sim")
        os.makedirs(run_root_no_sim)  # exists, but no sim/ subdir
        buf = io.StringIO()
        with redirect_stdout(buf):
            ok = e2e.apply_rerun_mode("clear_all", run_root_no_sim,
                                      dry_run=False, verbose=1)
        self.assertTrue(ok, "a missing sim/ dir must not abort the pipeline")
        self.assertIn("nothing to clear", buf.getvalue())

    def test_clear_all_missing_run_root_is_noop(self):
        """``clear_all`` on a non-existent run root is a no-op, not an error."""
        missing = os.path.join(self.tmp, "runs", "never_created")
        self.assertFalse(os.path.exists(missing))
        ok = e2e.apply_rerun_mode("clear_all", missing,
                                  dry_run=False, verbose=1)
        self.assertTrue(ok)

    def test_clear_all_announces_removal_when_verbose(self):
        """``clear_all`` with verbose>=1 logs what it removes."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            e2e.apply_rerun_mode("clear_all", self.run_root,
                                 dry_run=False, verbose=1)
        self.assertIn("removing", buf.getvalue().lower())
        self.assertIn("sim", buf.getvalue())

    # --- dry_run ---

    def test_clear_all_dry_run_touches_nothing(self):
        """``dry_run=True`` prints the wipe intent and removes nothing."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            ok = e2e.apply_rerun_mode("clear_all", self.run_root,
                                      dry_run=True, verbose=1)
        self.assertTrue(ok)
        self.assertTrue(self._sim_files_present(),
                        "dry_run must not remove anything")
        self.assertIn("would remove", buf.getvalue())

    # --- unknown mode (defensive) ---

    def test_unknown_mode_warns_and_is_noop(self):
        """An unknown mode prints a warning and removes nothing."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            ok = e2e.apply_rerun_mode("bogus_mode", self.run_root,
                                      dry_run=False, verbose=1)
        self.assertTrue(ok, "an unknown mode must not abort the pipeline")
        self.assertTrue(self._sim_files_present(),
                        "an unknown mode must not remove anything")
        self.assertIn("warning", buf.getvalue().lower())
        self.assertIn("bogus_mode", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
