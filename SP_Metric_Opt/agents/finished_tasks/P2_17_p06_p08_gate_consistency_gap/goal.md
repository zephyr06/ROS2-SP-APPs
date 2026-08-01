# P2.17 — `compare_optimizers.py` Bypasses P0.8's Important-Task Gate

> RE-FRAMED 2026-08-01 after investigation. The original "P0.8 (mean-ET) vs P0.6
> (worst-case-ET) gate-consistency gap" framing was WRONG — see dev_log. P0.8's
> gate is NOT mean-ET; it is a deterministic worst-case-WCET RTA, and it
> correctly REJECTS the crashed tasksets. The real gap is a gate BYPASS in the
> harness both P0.7 measurement runs used.

## The Goal

`compare_optimizers.py` generates tasksets via the **UNGATED**
`run_full_generation_pipeline` (lines 591/604), never the gated
`run_full_generation_pipeline_with_important_task_gate`. It has no
`--important_tasks_schedulability_check` flag at all (only `run_sim_experiments.py`
wires P0.8's gate). So tasksets that are unschedulable for the important tasks at
the worst case are emitted freely, reach the sim, and trip P0.6's
`ComputeSafeFallback` loud-fail ("No safe fallback exists for this task set —
regenerate a new task set") → P1.15 layer-A aborts the whole run on the crash.

This blocked P0.7's Step-3 SP-penalty A/B at N=6 (`measure_p07_penalty.json`,
taskset_3) AND the compare_against_bf N=4 run (taskset_1) — both launched via
`compare_optimizers.py` (artifacts under `optimizer_comparison/runs/`), both with
legacy +1 per-taskset seeds (1000,1001,...) instead of the gate-widened +20 step.

### Verified evidence (compare_against_bf N=4 taskset_1)

- Both gates read the SAME worst-case WCETs from the interval characteristics
  YAMLs: task_1 max_et=12.91 (period=dl=33, proc 1, important), task_3 max_et=11.24
  (period=dl=20, proc 1, important). DM: task_3 > task_1.
- P0.8 Python gate, run directly on the emitted taskset_1: `schedulable=False`,
  culprit task_1, `R = 12.91 + ceil(R/20)·11.24 → 35.39 > 33`.
- `ComputeSafeFallback` (C++) computed the IDENTICAL result → threw. The two
  gates AGREE; there is no consistency gap between them.

## Why it is NOT a P0.7 bug (unchanged)

P0.7 only gates the online **use** of `safe_fallback_`. The safe-fallback
**compute** is unconditional (P0.6) and runs in BOTH the prod arm (`INCR_Reopt_10`)
and the measurement arm (`INCR_NO_FALLBACK`). Both arms crash with the identical
throw at `OptimizeSP_TL_Incre.cpp:1030`, proving the crash is independent of the
P0.7 flag. P0.7's own code is sound (N=4 shows the mechanism firing).

## Also NOT a P0.8 gate-logic bug (the correction)

P0.8's gate (`important_task_rta.py`) is a DETERMINISTIC worst-case-WCET RTA: non-perf
WCET = `execution_time_max` (perf = `execution_time_mu`), hard `R <= deadline`, DM
priority. It is the HARDER gate. Running it on the crashed taskset_1 returns
`schedulable=False` with the same `R=35.39 > 33` the C++ gate computed. P0.8's gate
would have rejected this taskset — it was simply never asked to (the bypass).

## Open decisions (D1–D2, re-framed — settle with user before any code)

- **D1 — Fix path:** route `compare_optimizers.py`'s generation through the gated
  pipeline (mirror `run_sim_experiments.py`'s `--important_tasks_schedulability_check`
  wiring + the gate-widened +20 per-taskset seed step). This is the minimal fix
  that makes both crashed harnesses self-certifying and likely unblocks the full
  P0.7 A/B re-run with NO P0.6/P0.8 gate-logic change. Open sub-question: should
  the gate default ON in `compare_optimizers.py` too (parity with
  `run_sim_experiments.py`), or stay opt-in there?
- **D2 — Loud-fail scope (only if D1 is insufficient):** if a gated
  `compare_optimizers.py` still hits a taskset P0.8 admits but `ComputeSafeFallback`
  rejects (a TRUE gate-consistency gap, not yet observed), then decide: tighten
  P0.8 to also check the worst-case DAG, OR make P0.6's loud-fail a
  skip-and-continue. Not needed unless such a case appears post-D1.

(D3 — graceful-degrade of the loud-fail — dropped; the loud-fail is the correct
P1.15-layer-A safety behavior and should stay loud.)

## Non-goals

- Changing P0.7's triggers or flag (out of scope; P0.7 is sound).
- Changing P0.8's WCET rule or RTA (sound; correctly rejects the crashed taskset).
- Changing the worst-case-DAG construction (P0.6 §8 — the sound certificate).
