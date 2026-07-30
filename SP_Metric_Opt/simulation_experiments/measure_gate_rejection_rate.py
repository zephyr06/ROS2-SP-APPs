#!/usr/bin/env python3
"""Measure the P0.8 important-task gate's rejection rate across N, cheaply.

Context
-------
The important-task gate (:func:`run_full_generation_pipeline_with_important_task_gate`)
certifies that the emitted taskset is schedulable for the important subset under
DM-with-top-priority-lock at the seed point. On a failed draw it advances the
seed by +1 and retries (budget 20), then raises LOUDLY if the budget is
exhausted (NEVER silently emits an unschedulable taskset — re-creates the P1.8
substrate). This script answers the open question for P0.8 Step 3 (the deferred
rejection-rate measurement): *how often does the gate actually reject?*

Why n_sec=100 is a faithful proxy
---------------------------------
The gate's verdict is n_sec-stable: identical ``response_time`` (83.6) and
``deadline`` (67) at n_sec=100/200/1000 (verified before this script). The
rejection culprits are perf tasks whose WCET = ``period *
FINAL_Et_OVER_PERIOD_RANGE[1]`` (= ``period * 0.9``) — a TL-grid upper bound
that is *independent of n_sec* (it is read from the per-interval
characteristics, not the simulation horizon). So a short n_sec reproduces the
seed-point verdict a long run would, at ~2-3s/attempt instead of minutes.
The 2×-hyper-period guard (periods max 1000ms → 2s floor) is satisfied at
n_sec=100 (= 100s) for every paper config. (See the n_sec-stability note in the
P0.8 memory topic.)

Independent sampling
--------------------
The gate advances ``RANDOM_SEED`` by +1 *internally* per retry (its +0..19
window). To draw INDEPENDENT tasksets across samples (not collide with that
window), this script varies the BASE seed per sample:
``base_seed = 1000 + sample_idx * 100``. The gate then runs its own
``base_seed + 0..19`` retry window against an independently drawn taskset for
each sample. This isolates "the gate rejects this draw" (a per-sample outcome)
from "the gate's retry eventually recovers" (its internal +0..19 budget).

What it records per sample
--------------------------
  - ``attempts_used`` (1..max_attempts) when the gate PASSED (possibly after
    internal retries), OR
  - ``raised`` when the gate exhausted its budget (a true rejection).
A ``RuntimeError`` from one sample is CAUGHT so it does not abort the sweep —
rejection IS the signal we are measuring.

Output
------
A per-N table (first-try pass rate, gate-rejection rate, mean/max attempts, the
full ``attempts_used`` histogram) + a JSON record under
``simulation_experiments/important_task_gate_rejection/<tag>/`` (matching the
``repro_et_grows_with_period.py`` convention) so the measurement is reproducible.

Usage
-----
    python3 -m simulation_experiments.measure_gate_rejection_rate
    python3 -m simulation_experiments.measure_gate_rejection_rate --samples 10 --ns 4 8 16
    python3 -m simulation_experiments.measure_gate_rejection_rate --samples 5 --ns 4 --n_sec 100

Notes
-----
- This is a MEASUREMENT harness, not a fix. It calls the REAL gate + REAL RTA;
  only the expensive trace generation is bounded via n_sec.
- Distinct base seeds ⇒ independent tasksets; the gate's internal +1..+19 retry
  window is preserved within each sample (verified by the seed-advance gate
  test, ``test_gate_advances_seed_so_retries_re_sample``).
"""
import argparse
import json
import os
import shutil
import sys
import tempfile
import time
import traceback

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib import orchestrator as orch  # noqa: E402

# Per-N paper configs (INCLUDE the shared base template — resolved by
# load_generation_config relative to the config file's dir). N_TASKS is the
# only field the per-N file sets beyond INCLUDE + the seed.
_PAPER_CONFIGS = {
    4: "Gen_Taskset/task_sets_config/taskset_cfg_paper_4.json",
    8: "Gen_Taskset/task_sets_config/taskset_cfg_paper_8.json",
    16: "Gen_Taskset/task_sets_config/taskset_cfg_paper_16.json",
}

# Base seeds for independent samples. Step of 100 leaves the gate's internal
# +0..19 retry window clear of every other sample's draw.
_BASE_SEED_OFFSET = 1000
_BASE_SEED_STEP = 100


# Absolute path to the shared base template the per-N paper configs INCLUDE.
# The per-N configs store INCLUDE as a RELATIVE name ("taskset_cfg_paper_base.json")
# resolved against the config file's own dir + its ``templates/`` subdir — so a
# copy written to a temp dir cannot resolve it. Rewriting INCLUDE to this
# absolute path makes the temp config self-contained (the same trick the
# synthesizer in generation_config_parser uses when it writes per-N configs).
_BASE_TEMPLATE = os.path.join(
    PROJECT_ROOT, "Gen_Taskset", "task_sets_config",
    "templates", "taskset_cfg_paper_base.json",
)


def _write_sample_config(n_tasks: int, base_seed: int, tmp_dir: str) -> str:
    """Write a per-sample temp config with the given base seed.

    Copies the paper config for ``n_tasks`` and overwrites ``RANDOM_SEED`` to
    ``base_seed`` and ``INCLUDE`` to an ABSOLUTE path to the real base template
    (the source INCLUDE is a relative name that only resolves in the paper
    config's own dir — rewritten absolute so the temp copy is self-contained).
    """
    src = _PAPER_CONFIGS[n_tasks]
    src_abs = os.path.join(PROJECT_ROOT, src) if not os.path.isabs(src) else src
    with open(src_abs) as f:
        cfg = json.load(f)
    cfg["RANDOM_SEED"] = base_seed
    cfg["INCLUDE"] = _BASE_TEMPLATE
    out_path = os.path.join(tmp_dir, f"taskset_cfg_paper_{n_tasks}.json")
    with open(out_path, "w") as f:
        json.dump(cfg, f, indent=2)
    return out_path


def _run_one_sample(n_tasks: int, sample_idx: int, n_sec: int,
                    max_attempts: int, scratch_root: str) -> dict:
    """Run one independent draw through the REAL gate; return the per-sample record.

    Returns a dict with ``base_seed``, ``outcome`` ("passed"|"raised"),
    ``attempts_used`` (int if passed, else None), ``elapsed_s``, and (on raise)
    the gate's error message excerpt. NEVER raises — a rejection is recorded,
    not propagated, so the sweep continues.
    """
    base_seed = _BASE_SEED_OFFSET + sample_idx * _BASE_SEED_STEP
    cfg_tmp = tempfile.mkdtemp(prefix=f"gatecfg_n{n_tasks}_s{sample_idx}_",
                               dir=scratch_root)
    out_dir = tempfile.mkdtemp(prefix=f"gateout_n{n_tasks}_s{sample_idx}_",
                               dir=scratch_root)
    cfg_path = _write_sample_config(n_tasks, base_seed, cfg_tmp)

    record = {
        "n_tasks": n_tasks,
        "sample_idx": sample_idx,
        "base_seed": base_seed,
        "outcome": None,
        "attempts_used": None,
        "elapsed_s": None,
        "error_excerpt": None,
    }
    t0 = time.time()
    try:
        report = orch.run_full_generation_pipeline_with_important_task_gate(
            cfg_file=cfg_path,
            n_sec=n_sec,
            dir_path=out_dir,
            add_perf_records=True,
            interact=False,
            max_attempts=max_attempts,
        )
        record["outcome"] = "passed"
        record["attempts_used"] = report["attempts_used"]
    except RuntimeError as e:
        # The gate's loud raise = a TRUE rejection (budget exhausted). This is
        # the signal we are measuring — record it, do not re-raise.
        record["outcome"] = "raised"
        record["attempts_used"] = max_attempts
        record["error_excerpt"] = str(e)[:300]
    except Exception as e:  # noqa: BLE001 — sweep must not abort on a surprise
        record["outcome"] = "error"
        record["attempts_used"] = None
        record["error_excerpt"] = (
            f"{type(e).__name__}: {e}\n{traceback.format_exc()[:600]}"
        )
    finally:
        record["elapsed_s"] = round(time.time() - t0, 2)
        # Clean up the per-attempt generation dir (it can be large; the record
        # captures the verdict, not the taskset). Keep the cfg dir too.
        shutil.rmtree(out_dir, ignore_errors=True)
        shutil.rmtree(cfg_tmp, ignore_errors=True)
    return record


def _aggregate(n_tasks: int, samples: list) -> dict:
    """Reduce per-sample records into a per-N summary."""
    n = len(samples)
    passed = [s for s in samples if s["outcome"] == "passed"]
    raised = [s for s in samples if s["outcome"] == "raised"]
    errored = [s for s in samples if s["outcome"] == "error"]
    first_try = [s for s in passed if s["attempts_used"] == 1]
    attempts = [s["attempts_used"] for s in passed]
    hist: dict[int, int] = {}
    for a in attempts:
        hist[a] = hist.get(a, 0) + 1
    return {
        "n_tasks": n_tasks,
        "n_samples": n,
        "n_passed": len(passed),
        "n_raised": len(raised),
        "n_errored": len(errored),
        "first_try_pass_rate": round(len(first_try) / n, 4) if n else None,
        "gate_rejection_rate": round(len(raised) / n, 4) if n else None,
        "mean_attempts_used": round(sum(attempts) / len(attempts), 3) if attempts else None,
        "max_attempts_used": max(attempts) if attempts else None,
        "attempts_used_histogram": {str(k): v for k, v in sorted(hist.items())},
        "total_elapsed_s": round(sum(s["elapsed_s"] for s in samples), 1),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", type=int, nargs="+", default=[4, 8, 16],
                    help="Task counts to sweep (default: 4 8 16).")
    ap.add_argument("--samples", type=int, default=5,
                    help="Independent draws per N (default: 5).")
    ap.add_argument("--n_sec", type=int, default=100,
                    help="Sim horizon per attempt (default: 100 — faithful proxy, "
                         "see module docstring).")
    ap.add_argument("--max_attempts", type=int,
                    default=orch.IMPORTANT_TASK_GATE_MAX_ATTEMPTS,
                    help="Gate internal retry budget per sample (default: the "
                         "gate's own max, %d)." % orch.IMPORTANT_TASK_GATE_MAX_ATTEMPTS)
    ap.add_argument("--tag", default=None,
                    help="Subdir under important_task_gate_rejection/ (default: auto).")
    args = ap.parse_args()

    for n in args.ns:
        if n not in _PAPER_CONFIGS:
            print(f"ERROR: no paper config for N={n}. Known: {sorted(_PAPER_CONFIGS)}",
                  file=sys.stderr)
            sys.exit(1)

    tag = args.tag or (f"ns{'-'.join(str(n) for n in args.ns)}"
                       f"_s{args.samples}_dur{args.n_sec}")
    out_root = os.path.join(
        PROJECT_ROOT, "simulation_experiments",
        "important_task_gate_rejection", tag,
    )
    # Scratch dirs for temp configs + generation output (cleaned per-sample).
    scratch_root = os.path.join(out_root, "_scratch")
    os.makedirs(scratch_root, exist_ok=True)

    print(f"\n=== Important-task gate rejection-rate sweep ===")
    print(f"N values    : {args.ns}")
    print(f"samples / N : {args.samples}")
    print(f"n_sec       : {args.n_sec}  (faithful proxy — verdict n_sec-stable)")
    print(f"max_attempts: {args.max_attempts}  (gate internal retry budget)")
    print(f"base seeds  : {_BASE_SEED_OFFSET} + idx*{_BASE_SEED_STEP} "
          f"(gate keeps its +0..{args.max_attempts - 1} window)")
    print(f"output      : {out_root}\n")

    all_results = {}
    all_samples = {}
    for n in args.ns:
        print(f"--- N={n} ({args.samples} samples) ---")
        samples = []
        for i in range(args.samples):
            rec = _run_one_sample(n, i, args.n_sec, args.max_attempts, scratch_root)
            samples.append(rec)
            att = rec["attempts_used"]
            att_s = str(att) if att is not None else "-"
            print(f"  N={n} sample{i} seed={rec['base_seed']:>5}: "
                  f"{rec['outcome']:<7} attempts={att_s:<3} "
                  f"({rec['elapsed_s']}s)")
        all_samples[n] = samples
        all_results[n] = _aggregate(n, samples)
        # Drop scratch now that this N is done.
        shutil.rmtree(scratch_root, ignore_errors=True)
        os.makedirs(scratch_root, exist_ok=True)

    print(f"\n{'='*64}")
    print(f"{'N':>4} {'samples':>7} {'passed':>7} {'raised':>7} {'errored':>7} "
          f"{'1stTry%':>8} {'reject%':>8} {'meanAtt':>8} {'maxAtt':>7}")
    print(f"{'-'*64}")
    for n in args.ns:
        r = all_results[n]
        def _f(x):
            return f"{x}" if x is not None else "-"
        print(f"{n:>4} {r['n_samples']:>7} {r['n_passed']:>7} {r['n_raised']:>7} "
              f"{r['n_errored']:>7} "
              f"{_f((r['first_try_pass_rate']*100 if r['first_try_pass_rate'] is not None else None)):>7}% "
              f"{_f((r['gate_rejection_rate']*100 if r['gate_rejection_rate'] is not None else None)):>7}% "
              f"{_f(r['mean_attempts_used']):>8} "
              f"{_f(r['max_attempts_used']):>7}")
    print(f"{'-'*64}")
    for n in args.ns:
        r = all_results[n]
        print(f"  N={n} attempts_used histogram: {r['attempts_used_histogram']}")

    record = {
        "config": {
            "ns": args.ns,
            "samples_per_n": args.samples,
            "n_sec": args.n_sec,
            "max_attempts": args.max_attempts,
            "base_seed_offset": _BASE_SEED_OFFSET,
            "base_seed_step": _BASE_SEED_STEP,
        },
        "per_n_summary": {str(n): all_results[n] for n in args.ns},
        "per_sample": {str(n): all_samples[n] for n in args.ns},
    }
    rec_path = os.path.join(out_root, "gate_rejection_rate.json")
    with open(rec_path, "w") as f:
        json.dump(record, f, indent=2)
    print(f"\nSaved: {rec_path}")


if __name__ == "__main__":
    main()
