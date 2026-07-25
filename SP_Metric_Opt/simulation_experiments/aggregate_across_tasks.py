"""Aggregate results across task counts and generate cross-task-count figures.

This script scans ``simulation_experiments/optimizer_comparison/`` for the
experiment directories belonging to the *current* run (matched by duration /
trigger interval / base seed), loads each ``comparison_summary.csv``, and
produces line charts showing how scheduler performance scales with the number
of tasks.

Figures are written to a per-run folder so runs with different parameters do
not clobber each other::

    optimizer_comparison/runs/<run_id>/figures/

where ``<run_id>`` is built from the active config (mode, duration, interval,
seed, task-count list) by :func:`build_run_id`.

Usage
-----
    python aggregate_across_tasks.py --mode test
    python aggregate_across_tasks.py --mode prod

Figures produced
----------------
Main group (INCR, BF, RM, CFS) -- paper figures:
    fig1a_mean_sp_vs_tasks_main.{png,pdf}
    fig1c_mean_exec_time_vs_tasks_main.{png,pdf}    (log y-axis)

Main group -- debug figures:
    fig1b_std_sp_vs_tasks_main.{png,pdf}
    fig1d_mean_miss_rate_vs_tasks_main.{png,pdf}
    fig1e_std_miss_rate_vs_tasks_main.{png,pdf}

Main group -- optional (when analysis.normalize_sp is true):
    fig1a_mean_sp_normalized_vs_tasks_main.{png,pdf}
    fig1b_std_sp_normalized_vs_tasks_main.{png,pdf}   (debug)

Ablation group (BF, INCR, INCR_NO_TL, INCR_WCET):
    fig_ablation_mean_sp_vs_tasks.{png,pdf}
    fig_ablation_mean_exec_time_vs_tasks.{png,pdf}  (log y-axis)

Ablation group -- optional (when analysis.normalize_sp is true):
    fig_ablation_mean_sp_normalized_vs_tasks.{png,pdf}

Standalone distribution (single fixed task count):
    fig1f_sp_distribution_boxplot.{png,pdf}
    fig1f_sp_distribution_boxplot_normalized.{png,pdf}  (when normalize_sp)
"""
import argparse
import copy
import csv
import glob
import os
import re
import shutil
import sys

import numpy as np

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.plotting_config import (
    setup_publication_style,
    save_figure,
    get_scheduler_color_map,
    FIGSIZE_GROUPED,
)
from simulation_experiments.experiment_config_loader import (
    load_experiment_config,
    build_run_id,
    build_run_root,
    DEFAULT_CONFIG_PATH,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FuncFormatter, LogLocator
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    FuncFormatter = None
    LogLocator = None

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------
OPTIMIZER_COMPARISON_DIR = os.path.join(
    PROJECT_ROOT, "simulation_experiments", "optimizer_comparison"
)
FIGURES_OUTPUT_DIR = os.path.join(OPTIMIZER_COMPARISON_DIR, "figures")

# Regex to extract num_tasks from directory name like "tasks6_dur300_interval10_seed1000"
TASK_COUNT_PATTERN = re.compile(r"tasks(\d+)_")


def parse_task_count_from_dir_name(dir_name: str) -> int | None:
    """Extract the number of tasks from an experiment directory name.

    Parameters
    ----------
    dir_name
        Directory basename, e.g. ``"tasks6_dur300_interval10_seed1000"``.

    Returns
    -------
    int | None
        The task count if the name matches the expected pattern, else ``None``.
    """
    match = TASK_COUNT_PATTERN.search(dir_name)
    return int(match.group(1)) if match else None


def compute_sp_upper_bound(experiment_dir):
    """Compute the theoretical per-interval SP upper bound for one experiment dir.

    The SP-metric is a weighted sum over tasks of ``SP_Func(...) * sp_weight *
    perf_coefficient``, where ``SP_Func ∈ [0, 1]`` (see
    ``sources/Safety_Performance_Metric/SP_Metric.h``). The simulation exports
    the **node-only** term (``ObtainSP_TaskSet_And_TimeLimits``), so the
    achievable ceiling is ``Σ_i sp_weight_i * perf_coefficient_i`` -- reached
    when no task misses its deadline.

    ``perf_coefficient`` equals 1.0 for generated tasksets (they carry no
    ``timePerformancePairs``), so in practice the ceiling is the sum of
    ``sp_weight`` over tasks. The generator normalizes that sum to
    ``SP_WEIGHTS_SUM`` (5.0) regardless of N, which makes the ceiling flat
    across task counts -- the empirical reason cross-N raw-SP comparison is
    misleading and normalization is needed.

    Parameters
    ----------
    experiment_dir : str
        Absolute path to an experiment directory (contains ``taskset_*/``
        subdirs, each holding ``taskset_characteristics_interval_0.yaml``).

    Returns
    -------
    float | None
        The upper bound, or ``None`` if it cannot be determined (no YAML
        found / yaml unavailable). Callers treat ``None`` as "do not
        normalize this task count" and keep the raw values.
    """
    if not YAML_AVAILABLE:
        return None

    # Use the first taskset's interval-0 characteristics as representative.
    # The weight sum is identical across tasksets of the same N by construction.
    taskset_dirs = sorted(
        d for d in glob.glob(os.path.join(experiment_dir, "taskset_*"))
        if os.path.isdir(d)
    )
    for ts_dir in taskset_dirs:
        char_path = os.path.join(ts_dir, "taskset_characteristics_interval_0.yaml")
        if not os.path.exists(char_path):
            char_path = os.path.join(ts_dir, "taskset_characteristics.yaml")
        if not os.path.exists(char_path):
            continue
        try:
            with open(char_path, "r") as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError):
            continue
        tasks = data.get("tasks", []) if data else []
        if not tasks:
            continue
        ceiling = 0.0
        for t in tasks:
            # perf_coefficient is 1.0 for generated tasksets (they carry no
            # timePerformancePairs). sp_weight is REQUIRED: a missing weight
            # is a bug (the generator writes one for every task), so we raise
            # rather than silently defaulting. The generator normalizes the
            # weight sum to SP_WEIGHTS_SUM regardless of N, so the ceiling is
            # flat across task counts.
            if "sp_weight" not in t:
                raise KeyError(
                    f"Task missing required 'sp_weight' in {char_path}: {t!r}"
                )
            perf_coeff = 1.0
            weight = float(t["sp_weight"])
            ceiling += weight * perf_coeff
        # Add chain/path term if present (not used by the simulation export,
        # but included for completeness if a DAG config is ever aggregated).
        for c in data.get("chains", []) or []:
            ceiling += float(c.get("sp_weight", 1.0))
        return ceiling
    return None


def normalize_records_sp(records, cfg, upper_bound_by_task_count=None):
    """Return a new record list with SP normalized to [0, 1] per task count.

    Raw records are **not** mutated. Normalization is a reporting transform so
    the raw ``Mean_SP_Metric`` values in the CSV are preserved; this function
    exists to feed a normalized variant of the SP-vs-N figures.

    **Semantics of 1.0 (by design):** normalized SP = ``raw_SP / ideal_SP``,
    where ``ideal_SP`` is the best achievable with *infinite* computation --
    every task meets its deadline perfectly (``SP_Func = 1`` for all tasks)
    and the optimizer uses the best config. Because ``SP_Func ∈ [0, 1]`` with
    non-negative weights, ``raw_SP ≤ ideal_SP`` always, so the ratio is
    guaranteed ∈ [0, 1] with 1.0 = "all deadlines met perfectly". This is
    independent of any scheduler's realized value (BF included) -- dividing by
    a realized baseline would not bound the ratio at 1.0 and was the source of
    earlier >1.0 artifacts.

    Methods (selected by ``cfg["analysis"]["sp_normalization_method"]``):

    ``upper_bound`` (default -- "fraction of the ideal")
        Divide ``mean_sp`` and ``std_sp`` by the per-N theoretical ceiling
        (``compute_sp_upper_bound`` -- ``Σ sp_weight × perf_coefficient``,
        i.e. the SP if no task missed its deadline; constant at ~5.0 here
        because the generator normalizes the weight sum). The divisor is a
        constant ceiling, not a per-scheduler mean, so no figure can exceed
        1.0. Falls back to the raw value when no ceiling is available or the
        ceiling is <= 0.

    ``minmax_per_task_count`` (debug only -- relative ranking, NOT
    fraction-of-ideal)
        ``(sp - min_sp_at_N) / (max_sp_at_N - min_sp_at_N)`` across schedulers
        at each fixed N. Produces a clean [0, 1] of relative ranking but loses
        the "fraction of ideal" meaning, so 1.0 here is "best of the
        compared schedulers", not "best possible". The degenerate all-equal
        case maps to 1.0.

    Parameters
    ----------
    records : list[dict]
        Output of :func:`aggregate_data_from_directories`.
    cfg : dict
        Loaded experiment config (the ``analysis`` block is consulted).
    upper_bound_by_task_count : dict[int, float] | None
        Optional precomputed ceilings keyed by task count. Used only by the
        ``upper_bound`` method; when ``None`` the ceilings are read lazily
        via :func:`compute_sp_upper_bound` using the experiment-dir paths
        embedded in the records' ``_experiment_dir`` field.

    Returns
    -------
    list[dict]
        New list of records with ``mean_sp`` / ``std_sp`` replaced by their
        normalized values. Records for task counts that cannot be normalized
        (no ceiling available) are copied unchanged.
    """
    analysis = cfg.get("analysis", {}) if cfg else {}
    method = analysis.get("sp_normalization_method", "upper_bound")

    # Precompute ceilings for the upper_bound method if not supplied.
    if method == "upper_bound" and upper_bound_by_task_count is None:
        upper_bound_by_task_count = {}
        seen_dirs = {}
        for r in records:
            nt = r["num_tasks"]
            if nt in upper_bound_by_task_count:
                continue
            exp_dir = r.get("_experiment_dir")
            if exp_dir and exp_dir not in seen_dirs:
                seen_dirs[exp_dir] = compute_sp_upper_bound(exp_dir)
            ub = seen_dirs.get(exp_dir) if exp_dir else None
            upper_bound_by_task_count[nt] = ub

    # For minmax, gather the per-N min/max of mean_sp across schedulers.
    minmax_by_task_count = {}
    if method == "minmax_per_task_count":
        by_nt = {}
        for r in records:
            by_nt.setdefault(r["num_tasks"], []).append(r["mean_sp"])
        for nt, vals in by_nt.items():
            if vals:
                minmax_by_task_count[nt] = (min(vals), max(vals))

    out = []
    for r in records:
        nr = copy.deepcopy(r)
        nt = r["num_tasks"]
        if method == "upper_bound":
            ub = upper_bound_by_task_count.get(nt) if upper_bound_by_task_count else None
            if ub is not None and ub > 0:
                nr["mean_sp"] = r["mean_sp"] / ub
                nr["std_sp"] = r["std_sp"] / ub
        elif method == "minmax_per_task_count":
            lo_hi = minmax_by_task_count.get(nt)
            if lo_hi:
                lo, hi = lo_hi
                span = hi - lo
                if span > 0:
                    nr["mean_sp"] = (r["mean_sp"] - lo) / span
                    nr["std_sp"] = r["std_sp"] / span if r["std_sp"] else 0.0
                else:
                    # Degenerate: all schedulers equal at this N.
                    nr["mean_sp"] = 1.0 if r["mean_sp"] == hi else 0.0
                    nr["std_sp"] = 0.0
        out.append(nr)
    return out


def load_experiment_summaries(experiment_dir: str) -> list[dict]:
    """Load ``comparison_summary.csv`` from a single experiment directory.

    Parameters
    ----------
    experiment_dir
        Absolute path to an experiment directory (contains taskset_*/ subdirs).

    Returns
    -------
    list[dict]
        One dict per scheduler row, with the standard CSV columns.
    """
    csv_path = os.path.join(experiment_dir, "comparison_summary.csv")
    if not os.path.exists(csv_path):
        return []

    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def aggregate_data_from_directories(cfg=None, output_parent=None):
    """Scan the run's experiment directories and build the aggregate records.

    When *cfg* is provided, ingestion is **scoped to the current run**: only
    directories whose name starts with ``tasks{N}_dur{D}_interval{I}_seed{S}``
    (for each task count ``N`` in the config) are ingested. This excludes both
    the interval-sweep result dirs (``tasks{N}_sweep_interval{I}_...``) and
    stale directories from runs with a different duration / interval / seed,
    which previously overwrote the real points for a task count (the
    "last-write-wins" pollution bug noted in the dev log).

    When *cfg* is ``None`` (legacy direct callers / tests), the previous
    behaviour is preserved: ingest every ``tasks{N}_*`` directory except those
    containing ``sweep``.

    Parameters
    ----------
    cfg : dict | None
        Loaded experiment config. When given, used to build the allowed
        directory-name prefixes for run-scoped ingestion.
    output_parent : str | None
        Base directory to scan (defaults to :data:`OPTIMIZER_COMPARISON_DIR`).

    Returns
    -------
    list[dict]
        Each dict has keys:
        ``num_tasks``, ``scheduler``, ``mean_sp``, ``std_sp``,
        ``mean_miss_rate``, ``std_miss_rate``, ``mean_sched_time``,
        ``std_sched_time``, ``important_miss_rate``,
        ``non_important_miss_rate``.
    """
    # P23: when scoped to a run (cfg given), the simulate + sweep stages wrote
    # their dirs under <run_root>/sim/, not at the top of output_parent -- so
    # scan there. The unscoped/legacy path (cfg is None, direct callers/tests)
    # still scans output_parent itself.
    if cfg is not None:
        scan_parent = build_run_root(
            output_parent if output_parent is not None else OPTIMIZER_COMPARISON_DIR,
            cfg,
        )
        base_dir = os.path.join(scan_parent, "sim")
    else:
        base_dir = output_parent if output_parent is not None else OPTIMIZER_COMPARISON_DIR
    records = []
    if not os.path.isdir(base_dir):
        print(f"Warning: directory not found: {base_dir}")
        return records

    # Build the allowed directory-name prefixes for the current run, if scoped.
    allowed_prefixes = None
    if cfg is not None:
        dur = cfg.get("simulation_duration_seconds")
        interv = cfg.get("scheduler_trigger_interval_seconds")
        seed = cfg.get("base_random_seed")
        task_counts = cfg.get("num_tasks_for_cross_task_comparison", [])
        if dur is not None and interv is not None and seed is not None and task_counts:
            allowed_prefixes = [
                f"tasks{n}_dur{dur}_interval{interv}_seed{seed}"
                for n in task_counts
            ]

    for entry in os.listdir(base_dir):
        exp_path = os.path.join(base_dir, entry)
        if not os.path.isdir(exp_path):
            continue

        if allowed_prefixes is not None:
            # Run-scoped: only ingest dirs matching this run's parameters.
            if not any(entry.startswith(p) for p in allowed_prefixes):
                continue
        else:
            # Legacy: skip interval-sweep result dirs (tasks{N}_sweep_...).
            # They also match the tasks(\d+)_ pattern but represent a sweep
            # over a single task count, not a cross-task data point.
            if "sweep" in entry:
                continue

        num_tasks = parse_task_count_from_dir_name(entry)
        if num_tasks is None:
            continue

        summaries = load_experiment_summaries(exp_path)
        for row in summaries:
            try:
                records.append({
                    "num_tasks": num_tasks,
                    "scheduler": row["Scheduler"],
                    "mean_sp": float(row["Mean_SP_Metric"]),
                    "std_sp": float(row["Std_SP_Metric"]),
                    "mean_miss_rate": float(row.get("Mean_Miss_Rate", 0)),
                    "std_miss_rate": float(row.get("Std_Miss_Rate", 0)),
                    "mean_sched_time": float(row["Mean_Scheduler_Execution_Time_s"]),
                    "std_sched_time": 0.0,
                    "important_miss_rate": float(row.get("Important_Miss_Rate", 0)),
                    "non_important_miss_rate": float(row.get("Non_Important_Miss_Rate", 0)),
                    # Carried so normalize_records_sp can locate the
                    # taskset_characteristics YAML for the SP upper bound.
                    # Not a metric; ignored by the figure builders.
                    "_experiment_dir": exp_path,
                })
            except (KeyError, ValueError) as e:
                print(f"Skipping malformed row in {entry}: {e}")
                continue

    return records


def build_line_chart(
    records,
    scheduler_list,
    metric_key,
    std_key,
    ylabel,
    title,
    output_stem,
    figsize=FIGSIZE_GROUPED,
    log_y=False,
):
    """Generate a multi-line chart from aggregate records.

    One line per scheduler, plotted against the number of tasks (x-axis), with
    error bars from the std field. Replaces the previous grouped bar chart --
    line plots are used everywhere per the project convention.

    Parameters
    ----------
    records : list[dict]
        Output of :func:`aggregate_data_from_directories`.
    scheduler_list : list[str]
        Schedulers to include, in desired order (one line each).
    metric_key : str
        Key for the line value (e.g. ``"mean_sp"``).
    std_key : str
        Key for the error bar (e.g. ``"std_sp"``).
    ylabel : str
        Y-axis label string.
    title : str
        Figure title.
    output_stem : str
        Output path without extension.
    figsize : tuple
        Matplotlib figure size.
    log_y : bool
        If ``True``, set the y-axis to log scale. Use for metrics that span
        orders of magnitude across schedulers (e.g. execution time, where the
        optimizer search cost dwarfs fast baselines). A ``FuncFormatter`` labels
        the major (decade) ticks as plain numbers (``0.001`` not ``10^-3`` and
        not a fixed-decimal ``0.00``). The sub-decade (minor) ticks are labelled
        too, but SPARSELY -- only the 2x and 5x multiples per decade -- so a
        value that falls between two decades (e.g. ``0.062`` between ``0.01`` and
        ``0.1``) is still readable off the axis (``0.001 / 0.002 / 0.005 / 0.01
        / 0.02 / 0.05 / 0.1``) without the labels overlapping into an
        unreadable wall of numbers. Labelling every sub-decade (2x..9x) was tried
        and overlapped; only the two well-spaced 2x/5x ticks are kept.
    """
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available; skipping figure generation.")
        return

    # Bucket data by num_tasks -> scheduler -> (metric, std)
    num_tasks_set = sorted({r["num_tasks"] for r in records})
    data = {
        nt: {s: {"metric": 0.0, "std": 0.0} for s in scheduler_list}
        for nt in num_tasks_set
    }

    for r in records:
        nt = r["num_tasks"]
        sched = r["scheduler"]
        if nt in data and sched in data[nt]:
            data[nt][sched]["metric"] = r[metric_key]
            data[nt][sched]["std"] = r[std_key]

    # Color map
    color_map = get_scheduler_color_map(scheduler_list)

    fig, ax = plt.subplots(figsize=figsize)
    x = np.arange(len(num_tasks_set))

    for sched in scheduler_list:
        means = [data[nt][sched]["metric"] for nt in num_tasks_set]
        stds = [data[nt][sched]["std"] for nt in num_tasks_set]
        ax.errorbar(
            x,
            means,
            yerr=stds,
            marker="o",
            markersize=8,
            linewidth=2,
            capsize=4,
            label=sched,
            color=color_map.get(sched, "gray"),
            zorder=3,
        )

    ax.set_xticks(x)
    ax.set_xticklabels([str(nt) for nt in num_tasks_set])
    ax.set_xlabel("Number of Tasks")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(axis="y", linestyle="--", alpha=0.5)

    if log_y:
        ax.set_yscale("log")
        # Label major (decade) ticks as plain numbers via %g. ScalarFormatter
        # is avoided because it picks one fixed decimal place for the whole
        # axis, so small decades collapse to "0.00" when the range also
        # includes large values -- producing a stack of identical "0.00"
        # labels. %g formats each tick on its own scale (0.0001, 0.001, 0.01,
        # 0.1, 1).
        if FuncFormatter is not None:
            ax.yaxis.set_major_formatter(
                FuncFormatter(lambda val, pos=None: f"{val:g}")
            )

        # The sub-decade (minor) ticks are labelled too, but SPARSELY -- only
        # the 2x and 5x multiples per decade -- so a value that lands between two
        # decades (e.g. 0.062 between 0.01 and 0.1) is readable instead of
        # floating in an unlabelled gap (the old >=2-decade threshold left a
        # ~2.4-decade ET span like 0.00025..0.062 with just "0.01" and an empty
        # gap up to "0.1" -- the N>10 values were unreadable). Only 2x/5x are
        # labelled (not the full 2x..9x) so the labels never overlap into an
        # unreadable wall of numbers, regardless of the decade span.
        if LogLocator is not None:
            ax.yaxis.set_minor_locator(
                LogLocator(base=10.0, subs=[0.2, 0.5], numticks=12)
            )
            ax.yaxis.set_minor_formatter(
                FuncFormatter(lambda val, pos=None: f"{val:g}")
            )
        ax.grid(which="minor", axis="y", linestyle=":", alpha=0.3)
        ax.grid(which="major", axis="y", linestyle="--", alpha=0.5)

    plt.tight_layout()
    save_figure(fig, output_stem)
    plt.close(fig)


def generate_main_group_figures(records, cfg, figures_dir=None):
    """Produce Figs 1A–1E for the main scheduler group.

    When ``cfg["analysis"]["normalize_sp"]`` is true, an additional normalized
    Fig 1A variant (``fig1a_mean_sp_normalized_vs_tasks_main``) is emitted
    alongside the raw Fig 1A, so both views are available for the paper.
    """
    scheduler_list = cfg.get("main_scheduler_list", ["INCR_Reopt_10", "BF", "RM_FAST", "RM_SLOW", "CFS"])
    figures_dir = figures_dir if figures_dir is not None else FIGURES_OUTPUT_DIR
    os.makedirs(figures_dir, exist_ok=True)

    # Fig 1A: Mean SP (raw -- always produced)
    build_line_chart(
        records, scheduler_list,
        "mean_sp", "std_sp",
        "Mean SP-Metric",
        "Mean SP-Metric vs. Number of Tasks",
        os.path.join(figures_dir, "fig1a_mean_sp_vs_tasks_main"),
    )

    # Normalize SP to [0,1] per task count (when enabled). Computed once here
    # and reused for every normalized SP variant in this group so all SP
    # figures use the identical transform (raw_SP / ideal_SP, ideal_SP =
    # Σ sp_weight × perf_coefficient ~5.0 -> 1.0 = all deadlines met
    # perfectly). Raw SP scales with the (constant) weight-sum ceiling, so the
    # normalized view is the fair cross-N comparison. Raw values are unchanged
    # on disk.
    analysis = cfg.get("analysis", {})
    norm_records = (normalize_records_sp(records, cfg)
                    if analysis.get("normalize_sp", False) else None)

    # Fig 1A-norm: Mean SP normalized to [0,1] per task count (when enabled).
    if norm_records is not None:
        build_line_chart(
            norm_records, scheduler_list,
            "mean_sp", "std_sp",
            "Mean SP-Metric (normalized)",
            "Mean SP-Metric vs. Number of Tasks (Normalized)",
            os.path.join(figures_dir, "fig1a_mean_sp_normalized_vs_tasks_main"),
        )

    # Fig 1B: Std SP (debug)
    build_line_chart(
        records, scheduler_list,
        "std_sp", "std_sp",
        "Std SP-Metric",
        "Std SP-Metric vs. Number of Tasks (Debug)",
        os.path.join(figures_dir, "fig1b_std_sp_vs_tasks_main"),
    )

    # Fig 1B-norm: Std SP normalized (debug). normalize_records_sp scales
    # std_sp by the same per-N reference divisor as mean_sp, so the normalized
    # std is directly comparable across task counts.
    if norm_records is not None:
        build_line_chart(
            norm_records, scheduler_list,
            "std_sp", "std_sp",
            "Std SP-Metric (normalized)",
            "Std SP-Metric vs. Number of Tasks (Normalized, Debug)",
            os.path.join(figures_dir, "fig1b_std_sp_normalized_vs_tasks_main"),
        )

    # Fig 1C: Mean execution time (log y -- times span orders of magnitude
    # between the optimizer search cost and fast baselines like RM/CFS).
    build_line_chart(
        records, scheduler_list,
        "mean_sched_time", "std_sched_time",
        "Mean Execution Time (s)",
        "Mean Scheduler Execution Time vs. Number of Tasks",
        os.path.join(figures_dir, "fig1c_mean_exec_time_vs_tasks_main"),
        log_y=True,
    )

    # Fig 1D: Mean miss rate (debug)
    build_line_chart(
        records, scheduler_list,
        "mean_miss_rate", "std_miss_rate",
        "Mean Miss Rate",
        "Mean Miss Rate vs. Number of Tasks (Debug)",
        os.path.join(figures_dir, "fig1d_mean_miss_rate_vs_tasks_main"),
    )

    # Fig 1E: Std miss rate (debug)
    build_line_chart(
        records, scheduler_list,
        "std_miss_rate", "std_miss_rate",
        "Std Miss Rate",
        "Std Miss Rate vs. Number of Tasks (Debug)",
        os.path.join(figures_dir, "fig1e_std_miss_rate_vs_tasks_main"),
    )


def generate_important_task_miss_rate_figure(records, cfg, figures_dir=None):
    """Generate Fig 3: Important-task miss rate by scheduler.

    Uses the data collected at a single representative task count (default: the
    largest available or the one configured as ``num_tasks_for_single_task_figures``).
    """
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available; skipping Fig 3.")
        return

    target_tasks = cfg.get("num_tasks_for_single_task_figures", 6)
    scheduler_list = cfg.get("main_scheduler_list", ["INCR_Reopt_10", "BF", "RM_FAST", "RM_SLOW", "CFS"])
    figures_dir = figures_dir if figures_dir is not None else FIGURES_OUTPUT_DIR

    # Filter to target task count, sort schedulers in consistent order
    filtered = [r for r in records if r["num_tasks"] == target_tasks
                and r["scheduler"] in scheduler_list]
    if not filtered:
        print(f"No important-miss-rate data for {target_tasks} tasks; skipping Fig 3.")
        return

    fig, ax = plt.subplots(figsize=FIGSIZE_GROUPED)
    color_map = get_scheduler_color_map(scheduler_list)

    means = [next((r["important_miss_rate"] for r in filtered if r["scheduler"] == s), 0.0)
             for s in scheduler_list]
    # Use 0 std for now (per-taskset data not retained in summary CSV)
    stds = [0.0] * len(scheduler_list)

    # Line plot over scheduler index (categorical x-axis, connected markers).
    x = np.arange(len(scheduler_list))
    ax.errorbar(
        x, means, yerr=stds, marker="o", markersize=8, linewidth=2,
        capsize=4, zorder=3,
        color=color_map.get(scheduler_list[0], "gray"),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(scheduler_list)
    ax.set_ylabel("Important-Task Miss Rate")
    ax.set_title(f"Important-Task Miss Rate ({target_tasks} Tasks)")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_figure(fig, os.path.join(figures_dir, "fig3_important_task_miss_rate"))
    plt.close(fig)

    # Fig 3b: non-important-task miss rate (debug)
    fig, ax = plt.subplots(figsize=FIGSIZE_GROUPED)
    means_non = [next((r["non_important_miss_rate"] for r in filtered if r["scheduler"] == s), 0.0)
                 for s in scheduler_list]
    ax.errorbar(
        x, means_non, yerr=stds, marker="o", markersize=8, linewidth=2,
        capsize=4, zorder=3,
        color=color_map.get(scheduler_list[0], "gray"),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(scheduler_list)
    ax.set_ylabel("Non-Important-Task Miss Rate")
    ax.set_title(f"Non-Important-Task Miss Rate ({target_tasks} Tasks, Debug)")
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_figure(fig, os.path.join(figures_dir, "fig3b_non_important_task_miss_rate"))
    plt.close(fig)


def generate_ablation_group_figures(records, cfg, figures_dir=None):
    """Produce ablation-group figures (Ab-A, Ab-B).

    Mirrors the main group: log y on the execution-time figure, and an
    optional normalized SP variant when ``normalize_sp`` is enabled.
    """
    scheduler_list = cfg.get(
        "ablation_scheduler_list",
        ["BF", "INCR_Reopt_10", "INCR_NO_TL", "INCR_WCET"],
    )
    figures_dir = figures_dir if figures_dir is not None else FIGURES_OUTPUT_DIR
    os.makedirs(figures_dir, exist_ok=True)

    # Ab-A: Mean SP (raw -- always produced)
    build_line_chart(
        records, scheduler_list,
        "mean_sp", "std_sp",
        "Mean SP-Metric",
        "Mean SP-Metric vs. Number of Tasks (Ablation)",
        os.path.join(figures_dir, "fig_ablation_mean_sp_vs_tasks"),
    )

    # Ab-A-norm: normalized variant (when enabled). Same transform as the main
    # group so all SP figures are consistent (raw_SP / ideal_SP -> 1.0 = all
    # deadlines met perfectly).
    if cfg.get("analysis", {}).get("normalize_sp", False):
        norm_records = normalize_records_sp(records, cfg)
        build_line_chart(
            norm_records, scheduler_list,
            "mean_sp", "std_sp",
            "Mean SP-Metric (normalized)",
            "Mean SP-Metric vs. Number of Tasks (Ablation, Normalized)",
            os.path.join(figures_dir, "fig_ablation_mean_sp_normalized_vs_tasks"),
        )

    # Ab-B: Mean execution time (log y)
    build_line_chart(
        records, scheduler_list,
        "mean_sched_time", "std_sched_time",
        "Mean Execution Time (s)",
        "Mean Execution Time vs. Number of Tasks (Ablation)",
        os.path.join(figures_dir, "fig_ablation_mean_exec_time_vs_tasks"),
        log_y=True,
    )


def generate_distribution_boxplot(cfg, figures_dir=None, run_root=None):
    """Generate Fig 1F: SP distribution box plot for a single fixed task count.

    Reads raw ``interval_sp_metrics.txt`` files from a representative experiment
    directory (e.g., the first directory matching ``tasks{N}_*`` where N is
    ``cfg["num_tasks_for_single_task_figures"]``).

    When ``cfg["analysis"]["normalize_sp"]`` is true, an additional normalized
    variant is emitted: each SP value is divided by the ideal-SP ceiling
    (``Σ sp_weight × perf_coefficient``, ~5.0 -- the SP if every task met its
    deadline), so 1.0 = "all deadlines met perfectly" and every point stays
    in [0, 1]. The divisor is a constant ceiling, not a per-scheduler mean,
    so boxplot whiskers cannot exceed 1.0. Same transform the line figures
    apply, kept consistent across all SP-metric figures.

    Parameters
    ----------
    run_root : str | None
        P23: when set, the experiment dirs live under ``<run_root>/sim/``;
        scan there for the single-task dir. When None, fall back to the
        module-global :data:`OPTIMIZER_COMPARISON_DIR` (legacy/standalone path).
    """
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available; skipping Fig 1F.")
        return

    target_tasks = cfg.get("num_tasks_for_single_task_figures", 6)
    scheduler_list = cfg.get("main_scheduler_list", ["INCR_Reopt_10", "BF", "RM_FAST", "RM_SLOW", "CFS"])
    figures_dir = figures_dir if figures_dir is not None else FIGURES_OUTPUT_DIR

    # P23: sims now live under <run_root>/sim/; only the legacy/standalone path
    # scans the module-global OPTIMIZER_COMPARISON_DIR directly.
    scan_dir = os.path.join(run_root, "sim") if run_root else OPTIMIZER_COMPARISON_DIR

    # Find the first matching experiment directory
    candidate_dirs = []
    if os.path.isdir(scan_dir):
        candidate_dirs = [
            d for d in os.listdir(scan_dir)
            if d.startswith(f"tasks{target_tasks}_")
            and os.path.isdir(os.path.join(scan_dir, d))
        ]
    if not candidate_dirs:
        print(f"No experiment directory found for task count {target_tasks}; skipping Fig 1F.")
        return

    exp_dir = os.path.join(scan_dir, sorted(candidate_dirs)[0])

    # Collect per-scheduler SP values
    boxplot_data = []
    labels = []
    for sched in scheduler_list:
        sp_vals = []
        taskset_dirs = [
            os.path.join(exp_dir, d)
            for d in os.listdir(exp_dir)
            if d.startswith("taskset_") and os.path.isdir(os.path.join(exp_dir, d))
        ]
        for ts_dir in taskset_dirs:
            metrics_file = os.path.join(ts_dir, sched, sched, "interval_sp_metrics.txt")
            if os.path.exists(metrics_file):
                with open(metrics_file, "r") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        parts = line.split(",")
                        if len(parts) >= 2:
                            try:
                                sp_vals.append(float(parts[1]))
                            except ValueError:
                                pass
        if sp_vals:
            boxplot_data.append(sp_vals)
            labels.append(sched)

    if not boxplot_data:
        print(f"No interval SP data found in {exp_dir}; skipping Fig 1F.")
        return

    _draw_sp_boxplot(
        boxplot_data, labels, "SP-Metric Value",
        f"SP-Metric Distribution ({target_tasks} Tasks)",
        os.path.join(figures_dir, "fig1f_sp_distribution_boxplot"),
    )

    # Normalized variant (when enabled): divide every SP value by the ideal
    # (theoretical-ceiling) SP -- Σ sp_weight × perf_coefficient, i.e. the SP if
    # every task met its deadline. This is a *constant ceiling* (~5.0 here),
    # NOT a per-scheduler mean, so every point stays in [0, 1] with 1.0 =
    # "all deadlines met perfectly". (Dividing each per-interval point by a
    # realized mean -- the earlier approach -- pushed boxplot whiskers above
    # 1.0, since a point can legitimately exceed the mean; a constant ceiling
    # cannot be exceeded.)
    analysis = cfg.get("analysis", {})
    if analysis.get("normalize_sp", False):
        ideal_sp = compute_sp_upper_bound(exp_dir)
        if ideal_sp is not None and ideal_sp > 0:
            norm_data = [[v / ideal_sp for v in vals] for vals in boxplot_data]
            _draw_sp_boxplot(
                norm_data, labels, "SP-Metric Value (normalized)",
                f"SP-Metric Distribution ({target_tasks} Tasks, Normalized)",
                os.path.join(figures_dir, "fig1f_sp_distribution_boxplot_normalized"),
            )
        else:
            print(f"Could not compute ideal-SP ceiling for {exp_dir}; "
                  f"skipping normalized Fig 1F.")


def _draw_sp_boxplot(boxplot_data, labels, ylabel, title, output_stem):
    """Draw and save one SP distribution box plot."""
    fig, ax = plt.subplots(figsize=(10, 6))
    bp = ax.boxplot(boxplot_data, labels=labels, patch_artist=True)
    color_map = get_scheduler_color_map(labels)
    for patch, label in zip(bp["boxes"], labels):
        patch.set_facecolor(color_map.get(label, "lightgray"))
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    save_figure(fig, output_stem)
    plt.close(fig)


def _copy_config_into_run_root(cfg, run_root):
    """Copy the driving config JSON into the run root (P23).

    Makes a run self-describing: the exact config file that produced the run's
    sims and figures sits next to them as ``config.json``. Uses
    :func:`shutil.copy2` to preserve mtime. Skips silently when the source path
    is unknown, or when the source already resolves to the destination (so a
    re-run never clobbers or errors). The run root must already exist.
    """
    src = cfg.get("_config_source_path")
    if not src:
        return
    src_abs = os.path.abspath(src)
    dst = os.path.join(run_root, "config.json")
    if os.path.abspath(dst) == src_abs:
        # The config already lives in the run root -- nothing to copy.
        return
    try:
        shutil.copy2(src_abs, dst)
        print(f"Copied config -> {dst}")
    except OSError as exc:
        # Non-fatal: the run still produced figures; just note the miss.
        print(f"Warning: could not copy config into run root: {exc}")


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate results across task counts and generate publication figures."
    )
    parser.add_argument(
        "--mode",
        choices=["test", "prod"],
        default="test",
        help="Experiment mode: test (fast/smoke) or prod (paper-grade).",
    )
    parser.add_argument(
        "--config_json",
        default=DEFAULT_CONFIG_PATH,
        help="Path to the paper_simulation_config.json. Defaults to the shipped "
             "config at configs/paper_simulation_config.json.",
    )
    parser.add_argument(
        "--output_parent",
        default=OPTIMIZER_COMPARISON_DIR,
        help="Base output directory holding the run's experiment directories "
             "(default: simulation_experiments/optimizer_comparison). "
             "Figures are written under runs/<run_id>/figures/ inside it.",
    )
    args = parser.parse_args()

    cfg = load_experiment_config(mode=args.mode, config_path=args.config_json)
    plotting_cfg = cfg.get("plotting", {})
    setup_publication_style(
        font_base_size_points=plotting_cfg.get("font_base_size_points", 14),
        font_axis_label_size_points=plotting_cfg.get("font_axis_label_size_points", 16),
        font_title_size_points=plotting_cfg.get("font_title_size_points", 18),
        color_palette_name=plotting_cfg.get("color_palette_name", "colorblind"),
        grid_line_alpha=plotting_cfg.get("grid_line_alpha", 0.5),
    )

    # Resolve output_parent relative to the project root if not absolute.
    output_parent = (
        args.output_parent if os.path.isabs(args.output_parent)
        else os.path.join(PROJECT_ROOT, args.output_parent)
    )

    # P23: co-locate this run's figures and raw sim output under one run root.
    # The simulate + sweep stages wrote their dirs under <run_root>/sim/; this
    # stage scans there and writes figures under <run_root>/figures/.
    run_root = build_run_root(output_parent, cfg)
    run_id = build_run_id(cfg)
    figures_dir = os.path.join(run_root, "figures")
    os.makedirs(figures_dir, exist_ok=True)

    print(f"Run id: {run_id}")
    print(f"Figures dir: {figures_dir}")

    # P23: copy the driving config JSON into the run root so the run is
    # self-describing -- the exact config that produced these figures and sim
    # dirs sits next to them. Aggregate is the natural home: it runs last (the
    # run root already exists) and this is a benign, non-clobbering input copy.
    _copy_config_into_run_root(cfg, run_root)

    print("Scanning experiment directories ...")
    records = aggregate_data_from_directories(cfg=cfg, output_parent=output_parent)
    if not records:
        # Aggregate is read-only: it ingests comparison_summary.csv files that
        # the simulate stage wrote. With no matching records there is nothing
        # to plot, and aggregate deliberately does NOT auto-run simulate (that
        # would couple a read-only stage to the C++ binary + generation and
        # could surprise a user who expected a quick re-plot with a long sim).
        # Show exactly what was looked for so the gap is obvious, then point
        # at the entry point that runs the dependent stages in order.
        sim_dir = os.path.join(run_root, "sim")
        print("No experiment records found for this run's parameters.")
        print(f"(Looked under: {sim_dir})")
        dur = cfg.get("simulation_duration_seconds")
        interv = cfg.get("scheduler_trigger_interval_seconds")
        seed = cfg.get("base_random_seed")
        task_counts = cfg.get("num_tasks_for_cross_task_comparison", [])
        if dur is not None and interv is not None and seed is not None and task_counts:
            expected = [
                f"tasks{n}_dur{dur}_interval{interv}_seed{seed}"
                for n in task_counts
            ]
            print("Expected experiment directories (none found on disk):")
            for name in expected:
                print(f"  - {os.path.join(sim_dir, name)}")
            print("(These are created by the simulate stage, which writes a "
                  "comparison_summary.csv into each.)")
        print(
            "Run the pipeline end-to-end so simulate runs before aggregate:\n"
            "  ./scripts/run_simulation_plot_eval_ns.sh"
        )
        sys.exit(1)

    print(f"Loaded {len(records)} scheduler records.")

    print("Generating main-group figures ...")
    generate_main_group_figures(records, cfg, figures_dir=figures_dir)

    print("Generating ablation-group figures ...")
    generate_ablation_group_figures(records, cfg, figures_dir=figures_dir)

    print("Generating distribution box plot (Fig 1F) ...")
    generate_distribution_boxplot(cfg, figures_dir=figures_dir, run_root=run_root)

    print("Generating Fig 3: Important-Task Miss Rate ...")
    generate_important_task_miss_rate_figure(records, cfg, figures_dir=figures_dir)

    print(f"\nAll figures saved to: {figures_dir}")


if __name__ == "__main__":
    main()
