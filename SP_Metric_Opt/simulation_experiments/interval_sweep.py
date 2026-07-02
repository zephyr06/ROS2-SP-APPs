#!/usr/bin/env python3
"""Trigger-interval sweep experiment and Figure 2 generation.

Orchestrates multiple runs of `compare_optimizers.py`, each with a
different ``scheduler_trigger_interval``, and produces a line chart
(Figure 2) showing how INCR mean SP varies with re-optimization
frequency.  BF and RM are shown as horizontal reference lines.

When ``analysis.normalize_sp`` is enabled, a second normalized Figure 2 is
emitted (``fig2_sp_vs_interval_normalized``) where each scheduler's SP is
divided by the **ideal** SP (theoretical ceiling ``Σ sp_weight ×
perf_coefficient`` -- the SP if every task met its deadline), so 1.0 = "all
deadlines met perfectly" and no point exceeds 1.0.

Usage
-----
    python interval_sweep.py --mode test --num_tasks 6
    python interval_sweep.py --mode prod
"""
import argparse
import csv
import os
import subprocess
import sys

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.plotting_config import (
    setup_publication_style,
    save_figure,
    get_scheduler_color_map,
    FIGSIZE_SINGLE,
)
from simulation_experiments.experiment_config_loader import (
    load_experiment_config,
    build_run_id,
    DEFAULT_CONFIG_PATH,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

DEFAULT_OUTPUT_PARENT = os.path.join(
    PROJECT_ROOT, "simulation_experiments", "optimizer_comparison"
)


def _main_step_dir(num_tasks, n_sec, interval_sec, base_seed, output_parent):
    """Return the canonical main-step experiment dir path for these params.

    The main step (compare_optimizers without --run_name) writes to
    ``tasks{N}_dur{D}_interval{I}_seed{S}`` via
    :func:`build_experiment_dir_name`. The sweep can reuse that dir's results
    when its interval matches the main step's, so it must reconstruct the same
    name to *find* it.
    """
    from simulation_experiments.experiment_config_loader import (
        build_experiment_dir_name,
    )
    return os.path.join(
        output_parent,
        build_experiment_dir_name(num_tasks, n_sec, interval_sec, base_seed),
    )


def _main_dir_is_fresh(num_tasks, n_tasksets, n_sec, interval_sec, base_seed,
                       output_parent):
    """Check whether the main-step dir has fresh, reusable tasksets.

    "Fresh" means: every ``taskset_{idx}`` inside it was generated under a
    config matching what the sweep would use (same source config + same
    ``RANDOM_SEED = base_seed + idx`` + same ``UPDATE_INTERVAL_S =
    interval_sec``). Reuses :func:`_should_generate`'s exact comparison so the
    sweep's freshness notion matches compare_optimizers' own.

    Returns the dir path if fresh and it has a comparison_summary.csv, else
    ``None``.
    """
    from Gen_Taskset.lib.generation_config_parser import (
        load_generation_config, resolve_taskset_config_path,
    )
    from simulation_experiments.run_sim_experiments import _should_generate

    main_dir = _main_step_dir(num_tasks, n_sec, interval_sec, base_seed,
                              output_parent)
    if not os.path.exists(os.path.join(main_dir, "comparison_summary.csv")):
        return None

    config_file_abs = resolve_taskset_config_path(num_tasks)
    try:
        base_config = load_generation_config(config_file_abs)
    except (OSError, ValueError):
        return None

    for idx in range(n_tasksets):
        taskset_dir = os.path.join(main_dir, f"taskset_{idx}")
        # Build the exact config compare_optimizers would compare against.
        config_dict = dict(base_config)
        config_dict["RANDOM_SEED"] = base_seed + idx
        config_dict["UPDATE_INTERVAL_S"] = interval_sec
        # _should_generate returns True iff the taskset is absent or its saved
        # config has drifted (stale) -- i.e. NOT reusable. We use the
        # "regenerate" policy purely as a detector here (it only *decides*;
        # the caller does the regenerating), with verbose=0 so it stays quiet.
        # "keep" would be wrong: it returns False (reuse) even when stale.
        if _should_generate(
            taskset_dir, config_dict, idx,
            skip_if_exists=True, on_change_policy="regenerate", verbose=0,
        ):
            return None
    return main_dir


def run_single_interval(
    num_tasks,
    n_tasksets,
    n_sec,
    interval_sec,
    base_seed,
    schedulers,
    bin_dir,
    output_parent,
    export_level,
    important_task_pct,
    num_workers,
    resume,
    verbose,
    reuse_matching_interval=False,
    on_taskset_config_change="prompt",
):
    """Launch compare_optimizers.py for a single trigger interval.

    Parameters
    ----------
    reuse_matching_interval : bool
        P20: when True, first look for the main-step experiment dir for this
        ``(N, n_sec, interval, seed)``. If it exists with fresh, reusable
        tasksets (every ``taskset_{idx}`` generated under a matching config)
        and a ``comparison_summary.csv``, skip running compare_optimizers
        entirely and return that dir -- its summary already contains every
        scheduler the sweep plots (the main step runs the union list). This
        avoids regenerating tasksets the main step already produced for the
        same interval, and never clobbers the main dir's union summary.
    on_taskset_config_change : str
        Forwarded to compare_optimizers as ``--on_taskset_config_change``.
        Defaults to ``"prompt"`` so the config-drift guard the user relies on
        stays visible (a [Y/n] prompt on a TTY; the non-interactive fallback in
        :func:`_should_generate` regenerates with a warning when there is no
        TTY). The e2e orchestrator does **not** override this -- it shares
        tasksets across stages (``reuse_matching_interval``), it does not
        silence the drift guard. See ``agents/tasks.md`` (P20).
    """
    output_dir = os.path.join(
        output_parent, f"tasks{num_tasks}_sweep_interval{interval_sec}_seed{base_seed}"
    )

    if reuse_matching_interval:
        reused = _main_dir_is_fresh(
            num_tasks, n_tasksets, n_sec, interval_sec, base_seed, output_parent
        )
        if reused is not None:
            if verbose >= 1:
                print(f"\n{'='*60}")
                print(f"SWEEP: interval={interval_sec}s (reused main-step dir)")
                print(f"{'='*60}")
                print(f"  Reusing {reused}; skipping compare_optimizers.")
            return reused

    cmd = [
        sys.executable,
        "-m", "simulation_experiments.compare_optimizers",
        "--num_tasks", str(num_tasks),
        "--n_tasksets", str(n_tasksets),
        "--n_sec", str(n_sec),
        "--scheduler_trigger_interval", str(interval_sec),
        "--base_seed", str(base_seed),
        "--schedulers", *schedulers,
        "--bin_dir", bin_dir,
        "--output_dir", output_parent,
        "--run_name", os.path.basename(output_dir),
        "--export_level", str(export_level),
        "--important_task_pct", str(important_task_pct),
        "--verbose", str(verbose),
        "--on_taskset_config_change", on_taskset_config_change,
    ]
    if num_workers is not None:
        cmd += ["--num_workers", str(num_workers)]
    if resume:
        cmd.append("--resume")

    if verbose >= 1:
        print(f"\n{'='*60}")
        print(f"SWEEP: interval={interval_sec}s")
        print(f"{'='*60}")
        print("Command:", " ".join(cmd))

    subprocess.run(cmd, check=True)
    return output_dir


def load_summary(summary_path):
    """Load comparison_summary.csv and return rows as list of dicts."""
    if not os.path.exists(summary_path):
        return []
    rows = []
    with open(summary_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def generate_interval_sweep_figure(data, cfg, output_path_stem, ideal_sp=None):
    """Generate Figure 2: Mean SP vs. trigger interval.

    Parameters
    ----------
    data : list[dict]
        Each dict has ``interval`` (int), ``scheduler``, ``mean_sp``, ``std_sp``.
    cfg : dict
        Loaded experiment config. When ``cfg["analysis"]["normalize_sp"]`` is
        true, a second normalized figure is emitted alongside the raw one,
        dividing each SP by the ideal (theoretical-ceiling) SP so 1.0 =
        "all deadlines met perfectly".
    output_path_stem : str
        Output path without extension; the normalized variant appends
        ``_normalized`` to the stem.
    ideal_sp : float | None
        The ideal-SP ceiling (``Σ sp_weight × perf_coefficient``) for the task
        count this sweep runs at. Required for the normalized variant; when
        ``None`` or <= 0 the normalized figure is skipped with a warning.
        Pass it in from :func:`compute_sp_upper_bound` so the normalizer does
        not re-read the YAML.
    """
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available; skipping Fig 2.")
        return

    _draw_interval_sweep_variant(data, cfg, output_path_stem,
                                 ylabel="Mean SP-Metric",
                                 title="INCR SP-Metric vs. Optimizer Invocation Interval")

    analysis = cfg.get("analysis", {})
    if analysis.get("normalize_sp", False):
        norm_data = _normalize_sweep_data(data, ideal_sp)
        if norm_data is not None:
            _draw_interval_sweep_variant(
                norm_data, cfg, output_path_stem + "_normalized",
                ylabel="Mean SP-Metric (normalized)",
                title="INCR SP-Metric vs. Optimizer Invocation Interval (Normalized)",
            )


def _normalize_sweep_data(data, ideal_sp):
    """Return sweep data with mean_sp/std_sp divided by the ideal-SP ceiling.

    The sweep runs at a single task count, so the ceiling is one constant
    (``ideal_sp`` = ``Σ sp_weight × perf_coefficient``, the SP if every task
    met its deadline). Dividing by a constant ceiling -- not a per-interval
    realized mean -- guarantees every point stays in [0, 1] with 1.0 = "all
    deadlines met perfectly". Returns ``None`` (caller keeps raw) when
    ``ideal_sp`` is missing or <= 0.
    """
    if ideal_sp is None or ideal_sp <= 0:
        print("Warning: ideal-SP ceiling unavailable for the sweep task count; "
              "Fig 2 normalization skipped (raw values kept).")
        return None
    out = []
    for d in data:
        out.append({
            "interval": d["interval"],
            "scheduler": d["scheduler"],
            "mean_sp": d["mean_sp"] / ideal_sp,
            "std_sp": d["std_sp"] / ideal_sp if d["std_sp"] else 0.0,
        })
    return out


def _draw_interval_sweep_variant(data, cfg, output_path_stem, ylabel, title):
    """Draw one Figure 2 variant (raw or normalized) and save it."""
    schedulers = cfg.get("main_scheduler_list", ["INCR", "BF", "RM", "CFS"])
    color_map = get_scheduler_color_map(schedulers)

    intervals = sorted({d["interval"] for d in data})

    fig, ax = plt.subplots(figsize=FIGSIZE_SINGLE)

    # Plot each scheduler
    for sched in schedulers:
        x = []
        y = []
        yerr = []
        for iv in intervals:
            points = [d for d in data if d["interval"] == iv and d["scheduler"] == sched]
            if points:
                x.append(iv)
                y.append(points[0]["mean_sp"])
                yerr.append(points[0]["std_sp"])
        if not x:
            continue
        if sched == "INCR":
            ax.errorbar(
                x, y, yerr=yerr, marker="o", markersize=8, linewidth=2,
                label=sched, color=color_map.get(sched, "gray"),
                capsize=4, zorder=3,
            )
        else:
            # Baselines — horizontal dashed lines across the full x-range
            mean_val = sum(y) / len(y)
            ax.axhline(
                mean_val, linestyle="--", linewidth=1.5,
                label=f"{sched} (avg)", color=color_map.get(sched, "gray"),
            )

    ax.set_xlabel("Trigger Interval (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xticks(intervals)
    ax.legend(loc="best")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_figure(fig, output_path_stem)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Run interval sweep experiment and generate Figure 2."
    )
    parser.add_argument(
        "--mode", choices=["test", "prod"], default="test",
        help="Experiment mode: test (fast) or prod (paper-grade).",
    )
    parser.add_argument(
        "--config_json", default=DEFAULT_CONFIG_PATH,
        help="Path to the experiment_config.json. Defaults to the shipped "
             "config at configs/experiment_config.json.",
    )
    parser.add_argument(
        "--num_tasks", type=int, default=None,
        help="Number of tasks (overrides config).",
    )
    parser.add_argument(
        "--n_tasksets", type=int, default=None,
        help="Number of tasksets (overrides config).",
    )
    parser.add_argument(
        "--n_sec", type=int, default=None,
        help="Simulation duration in seconds (overrides config).",
    )
    parser.add_argument(
        "--output_parent", default=DEFAULT_OUTPUT_PARENT,
        help="Base output directory for sweep subdirectories.",
    )
    parser.add_argument(
        "--verbose", type=int, choices=[0, 1, 2], default=1,
        help="Verbosity level.",
    )
    parser.add_argument(
        "--reuse_matching_interval", action="store_true", default=False,
        help=("P20: for each sweep interval, first look for the main-step "
              "experiment dir (tasks{N}_dur{D}_interval{I}_seed{S}) with fresh, "
              "reusable tasksets; if present, reuse its summary instead of "
              "running compare_optimizers. Avoids regenerating tasksets the "
              "main step already produced for the same interval. Default: off."),
    )
    parser.add_argument(
        "--on_taskset_config_change",
        choices=["prompt", "regenerate", "keep"], default="prompt",
        help=("Policy forwarded to compare_optimizers when an existing "
              "taskset's config has changed. Default 'prompt' keeps the "
              "drift guard visible (a [Y/n] prompt on a TTY)."),
    )
    args = parser.parse_args()

    cfg = load_experiment_config(mode=args.mode, config_path=args.config_json)

    num_tasks = args.num_tasks if args.num_tasks is not None else cfg.get("num_tasks_for_single_task_figures", 6)
    n_tasksets = args.n_tasksets if args.n_tasksets is not None else cfg.get("num_tasksets_to_generate", 2)
    n_sec = args.n_sec if args.n_sec is not None else cfg.get("simulation_duration_seconds", 30)
    interval_list = cfg.get("interval_sweep_seconds_list", [5, 10])
    base_seed = cfg.get("base_random_seed", 1000)
    main_schedulers = cfg.get("main_scheduler_list", ["INCR", "BF", "RM", "CFS"])
    export_level = cfg.get("export_detail_level", 1)
    important_task_pct = cfg.get("analysis", {}).get("important_task_top_percentage", 0.10)
    num_workers = cfg.get("parallel_worker_processes", None)
    resume = cfg.get("analysis", {}).get("enable_resume_from_existing_results", False)
    bin_dir = "release"

    plotting_cfg = cfg.get("plotting", {})
    setup_publication_style(
        font_base_size_points=plotting_cfg.get("font_base_size_points", 14),
        font_axis_label_size_points=plotting_cfg.get("font_axis_label_size_points", 16),
        font_title_size_points=plotting_cfg.get("font_title_size_points", 18),
        color_palette_name=plotting_cfg.get("color_palette_name", "colorblind"),
        grid_line_alpha=plotting_cfg.get("grid_line_alpha", 0.5),
    )

    all_data = []
    first_output_dir = None
    for interval_sec in interval_list:
        output_dir = run_single_interval(
            num_tasks=num_tasks,
            n_tasksets=n_tasksets,
            n_sec=n_sec,
            interval_sec=interval_sec,
            base_seed=base_seed,
            schedulers=main_schedulers,
            bin_dir=bin_dir,
            output_parent=args.output_parent,
            export_level=export_level,
            important_task_pct=important_task_pct,
            num_workers=num_workers,
            resume=resume,
            verbose=args.verbose,
            reuse_matching_interval=args.reuse_matching_interval,
            on_taskset_config_change=args.on_taskset_config_change,
        )
        if first_output_dir is None:
            first_output_dir = output_dir
        summary_rows = load_summary(os.path.join(output_dir, "comparison_summary.csv"))
        for row in summary_rows:
            try:
                all_data.append({
                    "interval": interval_sec,
                    "scheduler": row["Scheduler"],
                    "mean_sp": float(row["Mean_SP_Metric"]),
                    "std_sp": float(row["Std_SP_Metric"]),
                })
            except (KeyError, ValueError):
                continue

    if not all_data:
        print("No data collected; aborting Fig 2.")
        sys.exit(1)

    # The sweep runs at a single task count, so the ideal-SP ceiling is one
    # constant. Compute it from any sweep experiment dir (they all share N).
    from simulation_experiments.aggregate_across_tasks import compute_sp_upper_bound
    ideal_sp = (compute_sp_upper_bound(first_output_dir)
                if first_output_dir is not None else None)

    # Scope Figure 2 to this run so different runs don't clobber each other.
    run_id = build_run_id(cfg)
    figures_dir = os.path.join(args.output_parent, "runs", run_id, "figures")
    os.makedirs(figures_dir, exist_ok=True)
    fig2_path = os.path.join(figures_dir, "fig2_sp_vs_interval")
    generate_interval_sweep_figure(all_data, cfg, fig2_path, ideal_sp=ideal_sp)
    print(f"\nFigure 2 saved to: {fig2_path}.{{png,pdf}}")
    if cfg.get("analysis", {}).get("normalize_sp", False):
        print(f"Figure 2 (normalized) saved to: {fig2_path}_normalized.{{png,pdf}}")


if __name__ == "__main__":
    main()
