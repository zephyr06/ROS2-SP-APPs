#!/usr/bin/env python3
"""Unified end-to-end experiment orchestrator.

This is the single entry point that runs the *entire* simulation-experiment
pipeline from one config file. Instead of manually invoking the three
pipeline stages by hand (step 1: per-task-count simulations, step 2: interval
sweep, step 3: cross-task aggregation and figures), this script does all of
them, driven entirely by ``configs/experiment_config.json``.

Pipeline stages
---------------
The orchestrator runs three stages, each implemented by an existing module,
**always in this fixed order** -- there is no stage-selection flag, because the
stages are dependent: aggregate reads what simulate wrote, and sweep reuses
simulate's tasksets. Run the whole pipeline or don't run it at all.

1. **simulate** -- For each task count in
   ``num_tasks_for_cross_task_comparison``, run
   :mod:`simulation_experiments.compare_optimizers` with the *union* of the
   main and ablation scheduler lists (so one simulation pass produces data
   for both the main and ablation figures). This populates
   ``optimizer_comparison/runs/<run_id>/sim/tasks{N}_dur{D}_interval{I}_seed{S}/``
   (P23: raw sim output is co-located under the run root, next to the figures).
2. **sweep** -- Run :mod:`simulation_experiments.interval_sweep` to sweep
   ``scheduler_trigger_interval`` over ``interval_sweep_seconds_list`` and
   produce Figure 2. Results go to
   ``runs/<run_id>/sim/tasks{N}_sweep_interval{I}_...`` and Figure 2 to
   ``runs/<run_id>/figures/``.
3. **aggregate** -- Run :mod:`simulation_experiments.aggregate_across_tasks`
   to scan the simulated directories and emit Figures 1A-1F, the ablation
   figures, and Figure 3.

Usage
-----
    # Everything, fast smoke-test parameters:
    python3 -m simulation_experiments.run_end_to_end_experiments --mode test

    # Everything, paper-grade parameters:
    python3 -m simulation_experiments.run_end_to_end_experiments --mode prod

    # Print the commands that would run without executing them:
    python3 -m simulation_experiments.run_end_to_end_experiments --dry_run

Notes
-----
- Every parameter comes from the config file. No experiment parameter is
  hardcoded here; the only CLI flags select *mode* and *verbosity*.
- The binary ``release/tests/RunOrchestrator`` must exist (the simulate and
  sweep stages invoke it; validation is delegated to each subprocess).
"""
import argparse
import os
import subprocess
import sys
import time

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.experiment_config_loader import (
    load_experiment_config,
    build_run_root,
    DEFAULT_CONFIG_PATH,
)

DEFAULT_OUTPUT_PARENT = os.path.join(
    PROJECT_ROOT, "simulation_experiments", "optimizer_comparison"
)


def _print_header(title, width=70):
    """Print a visible section divider around *title*."""
    bar = "=" * width
    print(f"\n{bar}\n{title}\n{bar}")


def build_scheduler_union(cfg):
    """Return the de-duplicated union of main + ablation scheduler lists.

    A single simulation run over this combined list produces data for both
    the main-group and abation-group figures, avoiding a second pass.

    Parameters
    ----------
    cfg : dict
        Loaded experiment config (from :func:`load_experiment_config`).

    Returns
    -------
    list[str]
        Schedulers in main-list order, then ablation-only schedulers in
        ablation-list order, with duplicates removed (preserving first
        occurrence).
    """
    main = cfg.get("main_scheduler_list", [])
    ablation = cfg.get("ablation_scheduler_list", [])
    union = []
    seen = set()
    for s in list(main) + list(ablation):
        if s not in seen:
            union.append(s)
            seen.add(s)
    return union


def build_simulate_command(num_tasks, cfg, output_parent, verbose, run_root):
    """Construct the ``compare_optimizers`` command for one task count.

    Parameters
    ----------
    num_tasks : int
        Number of tasks (must be 4, 6, or 8 per the upstream CLI).
    cfg : dict
        Loaded experiment config.
    output_parent : str
        Absolute path to ``optimizer_comparison/``.
    verbose : int
        Verbosity level forwarded to the subprocess.
    run_root : str
        Absolute path to this run's co-located root (``<output_parent>/runs/
        <run_id>``). Forwarded as ``--run_root`` so compare_optimizers writes
        its raw sim output under ``<run_root>/sim/`` (P23), next to the figures.

    Returns
    -------
    list[str]
        The ``python -m simulation_experiments.compare_optimizers ...`` argv.
    """
    schedulers = build_scheduler_union(cfg)
    cmd = [
        sys.executable, "-m", "simulation_experiments.compare_optimizers",
        "--num_tasks", str(num_tasks),
        "--n_tasksets", str(cfg.get("num_tasksets_to_generate", 10)),
        "--n_sec", str(cfg.get("simulation_duration_seconds", 300)),
        "--scheduler_trigger_interval",
        str(cfg.get("scheduler_trigger_interval_seconds", 10)),
        "--n_inst",
        str(cfg.get("number_of_gmm_trace_instances_per_path", 1)),
        "--base_seed", str(cfg.get("base_random_seed", 1000)),
        "--schedulers", *schedulers,
        "--bin_dir", cfg.get("bin_dir", "release"),
        "--output_dir", output_parent,
        "--run_root", run_root,
        "--export_level",
        str(cfg.get("export_detail_level", 1)),
        "--important_task_pct",
        str(cfg.get("analysis", {}).get("important_task_top_percentage", 0.10)),
        "--verbose", str(verbose),
        # P20: the simulate stage does NOT override the regeneration policy --
        # compare_optimizers' default 'prompt' is kept so genuine config drift
        # (e.g. after the P19 refactor) still surfaces a [Y/n] prompt instead
        # of silently regenerating. The orchestrator only ensures stages SHARE
        # tasksets (see the sweep's --reuse_matching_interval); it does not
        # silence the drift guard. See agents/tasks.md (P20).
    ]
    num_workers = cfg.get("parallel_worker_processes")
    if num_workers is not None:
        cmd += ["--num_workers", str(num_workers)]
    if cfg.get("analysis", {}).get("enable_resume_from_existing_results", False):
        cmd.append("--resume")
    if not cfg.get("enable_execution_time_profiling", True):
        # export_level 0 disables execution-time export; only set when profiling
        # is explicitly disabled in the config.
        cmd[cmd.index("--export_level") + 1] = "0"
    return cmd


def build_sweep_command(cfg, output_parent, verbose):
    """Construct the ``interval_sweep`` command.

    Parameters
    ----------
    cfg : dict
        Loaded experiment config.
    output_parent : str
        Absolute path to ``optimizer_comparison/``.
    verbose : int
        Verbosity level forwarded to the subprocess.

    Returns
    -------
    list[str]
        The ``python -m simulation_experiments.interval_sweep ...`` argv.
    """
    cmd = [
        sys.executable, "-m", "simulation_experiments.interval_sweep",
        "--mode", cfg.get("_active_mode", "test"),
        "--output_parent", output_parent,
        "--verbose", str(verbose),
        # P20: reuse the main step's tasksets for any sweep interval that
        # matches it (no regeneration). The regeneration policy is left at
        # the sweep's default ('prompt') so config drift still surfaces a
        # [Y/n] prompt -- the orchestrator shares tasksets, it does not
        # silence the drift guard.
        "--reuse_matching_interval",
    ]
    config_path = cfg.get("_config_source_path")
    if config_path:
        cmd += ["--config_json", config_path]
    return cmd


def build_aggregate_command(cfg, output_parent):
    """Construct the ``aggregate_across_tasks`` command.

    Parameters
    ----------
    cfg : dict
        Loaded experiment config.
    output_parent : str
        Absolute path to ``optimizer_comparison/`` (where the run's experiment
        directories live and where ``runs/<run_id>/figures/`` is written).

    Returns
    -------
    list[str]
        The ``python -m simulation_experiments.aggregate_across_tasks ...`` argv.
    """
    cmd = [
        sys.executable, "-m", "simulation_experiments.aggregate_across_tasks",
        "--mode", cfg.get("_active_mode", "test"),
        "--output_parent", output_parent,
    ]
    config_path = cfg.get("_config_source_path")
    if config_path:
        cmd += ["--config_json", config_path]
    return cmd


def run_command(cmd, dry_run):
    """Run *cmd*, or just print it when *dry_run* is set.

    Parameters
    ----------
    cmd : list[str]
        Argv to execute.
    dry_run : bool
        If True, print the command without executing it.

    Returns
    -------
    bool
        True if the command succeeded (or was a dry run), False if it failed.
    """
    print("  Command:", " ".join(cmd))
    if dry_run:
        print("  [dry-run] not executed.")
        return True
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"  ERROR: stage failed with exit code {exc.returncode}.")
        return False
    return True


def stage_simulate(cfg, output_parent, run_root, verbose, dry_run):
    """Run the per-task-count simulation stage."""
    task_counts = cfg.get("num_tasks_for_cross_task_comparison", [4, 6])
    _print_header(
        f"STAGE 1/3: SIMULATE  -- task counts {task_counts} "
        f"(mode={cfg.get('_active_mode')})"
    )
    for num_tasks in task_counts:
        _print_header(f"Simulating {num_tasks} tasks", width=50)
        cmd = build_simulate_command(num_tasks, cfg, output_parent, verbose, run_root)
        if not run_command(cmd, dry_run):
            return False
    return True


def stage_sweep(cfg, output_parent, verbose, dry_run):
    """Run the trigger-interval sweep stage (Figure 2)."""
    intervals = cfg.get("interval_sweep_seconds_list", [5, 10])
    _print_header(
        f"STAGE 2/3: SWEEP  -- intervals {intervals} "
        f"(mode={cfg.get('_active_mode')})"
    )
    cmd = build_sweep_command(cfg, output_parent, verbose)
    return run_command(cmd, dry_run)


def stage_aggregate(cfg, output_parent, dry_run):
    """Run the cross-task aggregation and figure stage (Figs 1A-1F, 3)."""
    _print_header(
        f"STAGE 3/3: AGGREGATE  -- cross-task figures "
        f"(mode={cfg.get('_active_mode')})"
    )
    cmd = build_aggregate_command(cfg, output_parent)
    return run_command(cmd, dry_run)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Run the full simulation-experiment pipeline end-to-end from a "
            "single config file. Executes: simulate -> sweep -> aggregate."
        )
    )
    parser.add_argument(
        "--mode", choices=["test", "prod"], default="test",
        help="Experiment mode: test (fast/smoke) or prod (paper-grade). "
             "Selects the parameter block in experiment_config.json.",
    )
    parser.add_argument(
        "--config_json", default=DEFAULT_CONFIG_PATH,
        help="Path to the experiment_config.json. Defaults to the shipped "
             "config at configs/experiment_config.json.",
    )
    parser.add_argument(
        "--output_parent", default=DEFAULT_OUTPUT_PARENT,
        help="Base output directory for experiment results "
             "(default: simulation_experiments/optimizer_comparison).",
    )
    parser.add_argument(
        "--bin_dir", default=None,
        help="Directory containing C++ binaries (default: from config, "
             "usually 'release'). Overrides the config value when set.",
    )
    parser.add_argument(
        "--verbose", type=int, choices=[0, 1, 2], default=1,
        help="Verbosity level forwarded to all subprocess stages.",
    )
    parser.add_argument(
        "--dry_run", action="store_true",
        help="Print the commands that would run without executing them.",
    )
    args = parser.parse_args()

    cfg = load_experiment_config(mode=args.mode, config_path=args.config_json)

    # Optional CLI override for the binary directory (not an experiment
    # parameter, so it lives here rather than in the JSON config).
    if args.bin_dir is not None:
        cfg["bin_dir"] = args.bin_dir
    else:
        cfg.setdefault("bin_dir", "release")

    output_parent = (
        args.output_parent if os.path.isabs(args.output_parent)
        else os.path.join(PROJECT_ROOT, args.output_parent)
    )

    # P23: one co-located run root per config. All three stages derive their
    # paths from this: simulate writes sims under <run_root>/sim/, sweep
    # reuses/writes there + emits Fig 2 under <run_root>/figures/, aggregate
    # scans <run_root>/sim/ and writes Figs 1A-1F/3 under <run_root>/figures/.
    # Computed once here and threaded through so every stage agrees on the path.
    run_root = build_run_root(output_parent, cfg)

    # Pre-flight: validate the binary exists. The simulate and sweep stages
    # both invoke RunOrchestrator, and both always run (there is no
    # stage-selection flag), so the binary is always required unless this is a
    # dry run.
    if not args.dry_run:
        bin_dir = cfg["bin_dir"]
        bin_dir_abs = (
            bin_dir if os.path.isabs(bin_dir)
            else os.path.join(PROJECT_ROOT, bin_dir)
        )
        sim_bin = os.path.join(bin_dir_abs, "tests", "RunOrchestrator")
        if not os.path.exists(sim_bin):
            print(f"Error: RunOrchestrator binary not found at {sim_bin}.")
            print("Compile in release mode first, or pass --bin_dir.")
            sys.exit(1)

    start = time.time()
    _print_header(
        f"SP-Metric Optimization: End-to-End Pipeline "
        f"(mode={args.mode}, dry_run={args.dry_run})",
        width=70,
    )
    print(f"  Config: {cfg.get('_config_source_path')}")
    print(f"  Output: {output_parent}")
    print(f"  Tasks:  {cfg.get('num_tasks_for_cross_task_comparison')}")
    print(f"  Sweep:  {cfg.get('interval_sweep_seconds_list')}")

    # Stages always run together, in fixed order: simulate -> sweep ->
    # aggregate. They are dependent (aggregate reads what simulate wrote;
    # sweep reuses simulate's tasksets), so there is no stage-selection flag.
    # A failed stage aborts the pipeline.
    stages = [
        ("simulate", lambda: stage_simulate(cfg, output_parent, run_root, args.verbose, args.dry_run)),
        ("sweep", lambda: stage_sweep(cfg, output_parent, args.verbose, args.dry_run)),
        ("aggregate", lambda: stage_aggregate(cfg, output_parent, args.dry_run)),
    ]
    for name, stage_fn in stages:
        if not stage_fn():
            print(f"\nPipeline aborted at stage '{name}'.")
            sys.exit(1)

    elapsed = time.time() - start
    figures_dir = os.path.join(run_root, "figures")
    _print_header(
        f"Pipeline complete in {elapsed:.1f}s. "
        f"Figures: {figures_dir}",
        width=70,
    )


if __name__ == "__main__":
    main()
