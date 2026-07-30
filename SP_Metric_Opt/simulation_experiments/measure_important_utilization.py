#!/usr/bin/env python3
"""Measure per-core important-task WCET-utilization at the D=T seed point.

Context
-------
The P0.8 important-task gate certifies that the important subset is schedulable
under DM-with-top-priority-lock at the seed point. The RTA's utilization guard
returns ``math.inf`` when ``Σ WCET/period >= 1.0`` on a core -- a deadline-
independent overload (Mode 2). This script measures, for the current paper
config, how the important subset's WCET-utilization is distributed per core, so
we can see whether (and by how much) it exceeds 1.0 and which WCET lever
(perf cap vs non-perf et_mean+2σ) drives it.

Faithfulness
------------
It calls the REAL ``generate_taskset_parameters`` and computes WCET EXACTLY as
the gate does: perf = ``execution_time_mu`` (the task's et_mean — the faithful
operating-point ET; the assigned TL is a downward cap, never an inflator, so
runtime ET <= et_mean is a sound WCET); non-perf = ``execution_time_max``
(= ``et_mean + 2*sigma`` as emitted). Only the expensive trace generation is
skipped (the gate reads the emitted bounds off disk, not the traces). Deadlines
are read as-emitted (D=T under DEADLINE_MODE=implicit).

Output
------
Per-N, per-seed: each important task's (period, deadline, wcet, wcet/period,
perf?, core), per-core important WCET-util sums, and the max-core util. A JSON
record under ``simulation_experiments/important_utilization/<tag>/``.

Usage
-----
    python3 -m simulation_experiments.measure_important_utilization
    python3 -m simulation_experiments.measure_important_utilization --ns 4 8 16 --seeds 5
    python3 -m simulation_experiments.measure_important_utilization --ns 16 --deadline implicit
"""
import argparse
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib import orchestrator as orch  # noqa: E402
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters  # noqa: E402

_PAPER_CONFIGS = {
    4: "Gen_Taskset/task_sets_config/taskset_cfg_paper_4.json",
    8: "Gen_Taskset/task_sets_config/taskset_cfg_paper_8.json",
    16: "Gen_Taskset/task_sets_config/taskset_cfg_paper_16.json",
}

_BASE_TEMPLATE = os.path.join(
    PROJECT_ROOT, "Gen_Taskset", "task_sets_config",
    "templates", "taskset_cfg_paper_base.json",
)

_BASE_SEED_OFFSET = 1000
_BASE_SEED_STEP = 100


def _write_sample_config(n_tasks: int, base_seed: int, deadline_mode: str,
                         tmp_dir: str, overrides: dict = None) -> str:
    """Write a self-contained temp config with the given base seed + deadline mode
    + optional per-key overrides (applied to the resolved cfgs, so INCLUDE-resolved
    base values CAN be overridden by the per-N file)."""
    src = _PAPER_CONFIGS[n_tasks]
    src_abs = os.path.join(PROJECT_ROOT, src)
    with open(src_abs) as f:
        cfg = json.load(f)
    cfg["RANDOM_SEED"] = base_seed
    cfg["INCLUDE"] = _BASE_TEMPLATE
    if deadline_mode is not None:
        cfg["DEADLINE_MODE"] = deadline_mode
    # Overrides are applied to the per-N config; the base template's value for
    # the same key (if any) is shadowed because load_generation_config merges
    # INCLUDE first then the top-level keys override.
    if overrides:
        cfg.update(overrides)
    out_path = os.path.join(tmp_dir, f"taskset_cfg_paper_{n_tasks}.json")
    with open(out_path, "w") as f:
        json.dump(cfg, f, indent=2)
    return out_path


def _wcet_of(task: dict) -> tuple[float, bool]:
    """Return (wcet, is_perf) exactly as the gate's D2 computes it.

    perf = et_mean; non-perf = execution_time_max. A task is perf iff
    ``time_limit_task`` (the generator's perf flag, which the exporter reads to
    decide whether to force bounds to the TL-grid range).

    NOTE: this operates on the IN-MEMORY task dict from
    ``generate_taskset_parameters``, where the field is ``Et_mean``. The gate
    reads the EMITTED-YAML representation, where the exporter writes the SAME
    value as ``execution_time_mu`` (``yaml_exporter.py:58``:
    ``t['execution_time_mu'] = task_data['Et_mean']`` when ``Et_actual`` is
    absent — the generation-time case). So reading ``Et_mean`` here is
    faithful to the gate's ``execution_time_mu`` read; the two compute the same
    perf WCET.
    """
    is_perf = bool(task.get("time_limit_task", False))
    if is_perf:
        return float(task["Et_mean"]), True
    return float(task["execution_time_max"]), False


def _run_one(n_tasks: int, sample_idx: int, deadline_mode: str,
             scratch_root: str, overrides: dict = None) -> dict:
    base_seed = _BASE_SEED_OFFSET + sample_idx * _BASE_SEED_STEP
    cfg_tmp = os.path.join(scratch_root, f"cfg_n{n_tasks}_s{sample_idx}")
    os.makedirs(cfg_tmp, exist_ok=True)
    cfg_path = _write_sample_config(n_tasks, base_seed, deadline_mode, cfg_tmp,
                                    overrides=overrides)
    cfgs = orch._load_and_validate_cfgs(cfg_path)

    params = generate_taskset_parameters(cfgs, dump_dir=None, n_sec=100)
    tasks = params["tasks"]

    by_core: dict[int, list] = {}
    important_tasks = []
    for idx, t in enumerate(tasks):
        if not t.get("is_important", False):
            continue
        wcet, is_perf = _wcet_of(t)
        period = float(t["period"])
        deadline = float(t.get("deadline", period))
        rec = {
            "idx": idx,
            "core": int(t.get("processorId", 0)),
            "period": period,
            "deadline": deadline,
            "wcet": round(wcet, 2),
            "wcet_over_period": round(wcet / period, 4) if period else None,
            "is_perf": is_perf,
            "et_mean": round(float(t.get("Et_mean", 0.0)), 2),
        }
        important_tasks.append(rec)
        by_core.setdefault(rec["core"], []).append(rec)

    per_core = {}
    for core, recs in sorted(by_core.items()):
        util_sum = sum(r["wcet_over_period"] for r in recs)
        per_core[core] = {
            "n_important": len(recs),
            "wcet_util_sum": round(util_sum, 4),
            "overload": util_sum >= 1.0 - 1e-9,
            "n_perf": sum(1 for r in recs if r["is_perf"]),
            "n_non_perf": sum(1 for r in recs if not r["is_perf"]),
            "tasks": recs,
        }
    max_core_util = max((c["wcet_util_sum"] for c in per_core.values()), default=0.0)
    return {
        "n_tasks": n_tasks,
        "sample_idx": sample_idx,
        "base_seed": base_seed,
        "deadline_mode": cfgs.get("DEADLINE_MODE", "constrained"),
        "n_important": len(important_tasks),
        "per_core": per_core,
        "max_core_wcet_util": round(max_core_util, 4),
        "any_core_overload": any(c["overload"] for c in per_core.values()),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", type=int, nargs="+", default=[4, 8, 16])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--deadline", default="implicit",
                    choices=["implicit", "constrained"],
                    help="DEADLINE_MODE to set (default: implicit = D=T).")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--util-cap", type=float, default=None,
                    help="Override MAX_UTIL_PER_TASK (non-env base-ET cap).")
    ap.add_argument("--env-cap", type=float, default=None,
                    help="Override MAX_UTIL_PER_ENV_TASK (env base-ET cap).")
    ap.add_argument("--sigma-over-et", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="Override SIGMA_OVER_Et_RANGE (non-perf variance).")
    ap.add_argument("--cpu-util", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="Override CPU_UTIL_RANDOM_RANGE (per-core load).")
    ap.add_argument("--cores", type=int, default=None,
                    help="Override N_CORES (default: 2 for all per-N paper configs).")
    args = ap.parse_args()

    overrides = {}
    if args.util_cap is not None:
        overrides["MAX_UTIL_PER_TASK"] = args.util_cap
    if args.env_cap is not None:
        overrides["MAX_UTIL_PER_ENV_TASK"] = args.env_cap
    if args.sigma_over_et is not None:
        overrides["SIGMA_OVER_Et_RANGE"] = list(args.sigma_over_et)
    if args.cpu_util is not None:
        overrides["CPU_UTIL_RANDOM_RANGE"] = list(args.cpu_util)
    if args.cores is not None:
        overrides["N_CORES"] = args.cores

    for n in args.ns:
        if n not in _PAPER_CONFIGS:
            print(f"ERROR: no paper config for N={n}.", file=sys.stderr)
            sys.exit(1)

    ovr_tag = ("_ovr" + "".join(f"_{k}={v}" for k, v in overrides.items())) if overrides else ""
    tag = args.tag or (f"ns{'-'.join(str(n) for n in args.ns)}_s{args.seeds}"
                       f"_{args.deadline}{ovr_tag}")
    out_root = os.path.join(PROJECT_ROOT, "simulation_experiments",
                            "important_utilization", tag)
    scratch_root = os.path.join(out_root, "_scratch")
    os.makedirs(scratch_root, exist_ok=True)

    print(f"\n=== Important-task WCET-utilization measurement ===")
    print(f"N values      : {args.ns}")
    print(f"seeds / N     : {args.seeds}")
    print(f"DEADLINE_MODE : {args.deadline}")
    if overrides:
        print(f"overrides     : {json.dumps(overrides)}")
    print(f"perf WCET     : Et_mean (in-memory; = execution_time_mu on emitted YAML); non-perf WCET = execution_time_max")
    print(f"output        : {out_root}\n")

    all_results = {}
    for n in args.ns:
        print(f"--- N={n} ({args.seeds} seeds) ---")
        samples = []
        for i in range(args.seeds):
            rec = _run_one(n, i, args.deadline, scratch_root, overrides=overrides)
            samples.append(rec)
            flag = "OVERLOAD" if rec["any_core_overload"] else "ok"
            print(f"  N={n} seed={rec['base_seed']:>5}: max_core_wcet_util="
                  f"{rec['max_core_wcet_util']:.3f}  n_imp={rec['n_important']}  "
                  f"[{flag}]")
            for core, c in rec["per_core"].items():
                if c["overload"]:
                    print(f"      core{core}: util={c['wcet_util_sum']:.3f} "
                          f"(perf={c['n_perf']}, nonperf={c['n_non_perf']}) OVERLOAD")
        all_results[n] = samples

    print(f"\n{'='*64}")
    print(f"{'N':>4} {'seeds':>6} {'overload%':>10} {'mean_maxUtil':>13} "
          f"{'max_maxUtil':>12}")
    print(f"{'-'*64}")
    summary = {}
    for n in args.ns:
        samples = all_results[n]
        n_over = sum(1 for s in samples if s["any_core_overload"])
        max_utils = [s["max_core_wcet_util"] for s in samples]
        summary[n] = {
            "n_samples": len(samples),
            "n_overload": n_over,
            "overload_rate": round(n_over / len(samples), 4) if samples else None,
            "mean_max_core_wcet_util": round(sum(max_utils) / len(max_utils), 4) if max_utils else None,
            "max_max_core_wcet_util": round(max(max_utils), 4) if max_utils else None,
        }
        s = summary[n]
        print(f"{n:>4} {s['n_samples']:>6} "
              f"{(s['overload_rate']*100 if s['overload_rate'] is not None else 0):>9}% "
              f"{s['mean_max_core_wcet_util']:>13} "
              f"{s['max_max_core_wcet_util']:>12}")

    record = {
        "config": {
            "ns": args.ns,
            "seeds_per_n": args.seeds,
            "deadline_mode": args.deadline,
            "base_seed_offset": _BASE_SEED_OFFSET,
            "base_seed_step": _BASE_SEED_STEP,
            "overrides": overrides,
        },
        "per_n_summary": {str(n): summary[n] for n in args.ns},
        "per_sample": {str(n): all_results[n] for n in args.ns},
    }
    rec_path = os.path.join(out_root, "important_utilization.json")
    with open(rec_path, "w") as f:
        json.dump(record, f, indent=2)
    print(f"\nSaved: {rec_path}")


if __name__ == "__main__":
    main()
