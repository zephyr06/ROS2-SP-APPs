# P1.2 — Reopt Incumbent Degradation (Structural Corruption) — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task created from the Gemini/Kimi multi-round debate
  (`agents/agent_communication/gemini.md` R1–R5, `kimi.md` R1–R4). The debate
  converged on:
  - **Diagnosis:** the SP-value guard (`UpdateRecords`,
    `OptimizeSP_TL_Incre.cpp:105-139`) is sound but structurally blind — it
    protects SP monotonicity, not permutation structural quality. The reopt
    bootstrap `OptimizeFromScratch(K=2)` (`OptimizeSP_Incre.cpp:74`) is
    memoryless w.r.t. π_incumbent and can commit a structurally-worse
    permutation that squeaks past the SP guard; the incremental `OptimizeIncre`
    then cannot restructure it.
  - **Synthesis (if the hazard fires):** Option B (seed the bootstrap beam with
    π_incumbent) + Option C (chain the TL walk via `OptimizeIncre`,
    from_scratch=false after the bootstrap). B gets structural memory, C gets
    the ET cut (current code re-runs `OptimizeFromScratch` per TL step via
    `EvaluateTimeLimitConfig_ScratchOrIncre` at `:152`).
  - **Sequencing:** empirical check FIRST, no code change until the hazard is
    shown to fire. Both agents agreed.
- Verified the existing P25 A/B data is on disk for the empirical check (no new
  run needed for step 1):
  `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks6_dur600_interval10_seed1000/taskset_<t>/<ARM>/<ARM>/interval_sp_metrics.txt`.
  Aggregate `comparison_summary.csv` (tasks6) already shows
  `BF 0.522 > INCR_P1 0.5049 > P10 0.5024 > P30 0.5000 > P60 0.4989 >
  INCR_SCRATCH 0.4968` — consistent with "memory helps, infrequent re-search
  costs a little" but not by itself evidence of corruption (could be benign
  drift). The per-interval trace is what distinguishes the two.
- Verified source line numbers are current (working tree has both optimizer
  files modified, but the cited lines — `UpdateRecords:105`,
  `OptimizeFromScratch:74`, `PerformCoordinateDescentForTaskConfigOpt:241`,
  `ReOptimizePeriodic:453`, `BuildChallengerFromIncumbent:402`,
  `ApproxEqualSP` at `.h:11`, shared `K=2` at `parameters.yaml:7` — all hold).
- Not yet started. Next: Step 1 — mine the tasks6 per-interval SP traces for
  the post-reopt non-recovering-dip signature.
