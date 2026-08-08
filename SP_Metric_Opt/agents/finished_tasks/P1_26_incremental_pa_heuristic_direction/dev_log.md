# P1.26 — Incremental PA Re-Search Direction Heuristic — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-08-06

- Task folder created (previous session implemented the flag but ran out of
  tokens before creating the `agents/active_tasks/` entry).
- **Verified the flag is already implemented in the working tree (uncommitted):**
  - `sources/Optimization/OptimizeSP_Incre.cpp:299` `AnalyzePriorityChangeStatus`
    — the ET↓ + highest-weight sub-case (`:310` switch) returns
    `Decrease` (variant 0, baseline) / `Increase` (variant 1, rule removed) /
    `OpenToAll` (variant 2). All other sub-cases unchanged.
  - `sources/Utils/Parameters.h:51` (declared), `Parameters.cpp:36` (defined +
    try/catch yaml load, default 0), `sources/parameters.yaml:42` (default 0).
  - The standing TODO at `:288` ("re-evaluate this heuristic") is preserved.
- **A/B artifacts already present:** `_ab_run.sh` (repo-root driver: phases
  `baseline` = clear_all, `variant` = clear_results) and
  `simulation_experiments/configs/pa_heuristic_ab.json` (INCR_Reopt_10 only,
  N=8/10, 5 tasksets, 300s, interval 10s, run_name_prefix `pa_heuristic_ab`).
- **Snapshot caveat noted:** baseline and variant write the SAME run dir (config
  is identical; the flag lives in `parameters.yaml`, not the JSON), so the
  baseline `comparison_summary.csv` + per-taskset `interval_sp_metrics.txt` MUST
  be copied aside before the variant `clear_results` run.
- Next: Step 0 — build test + `check.SP_OPT` to confirm variant 0 is
  byte-identical / green, then Step 1 baseline run.
