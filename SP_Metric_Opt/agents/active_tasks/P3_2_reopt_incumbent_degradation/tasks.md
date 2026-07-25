# P1.2 — Tasks (working checklist)

> See `goal.md` for scope. **Investigation first** — do NOT implement B+C until
> the empirical check shows the structural-corruption hazard actually fires.

## Step 1 — Mine existing P25 A/B per-interval SP traces
- [ ] Pick the most informative run root (tasks6, N=6, 60 intervals — more
      reopt events than tasks4). Path:
      `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks6_dur600_interval10_seed1000/`
- [ ] For each arm `INCR_P1`, `INCR_P10`, `INCR_P30`, `INCR_P60`, `INCR_SCRATCH`,
      across all tasksets, read `taskset_<t>/<ARM>/<ARM>/interval_sp_metrics.txt`
      (format `interval,sp`).
- [ ] At each arm's reopt intervals (every n-th interval: P1=1, P10=10, P30=30,
      P60=60), check for the corruption signature: a sharp SP drop at the reopt
      interval that does **not** recover to the pre-reopt trajectory in the
      subsequent intervals. Contrast INCR_P1 (frequent recovery) vs
      INCR_SCRATCH (amnesiac lower bound).
- [ ] Record the verdict (per arm, per taskset) in `dev_log.md`. Distinguish
      corruption (non-recovering dip) from benign drift (smooth decay).

## Step 2 — If traces are ambiguous, instrument structural distance
- [ ] Add a debug dump in `ReOptimizePeriodic` (`OptimizeSP_TL_Incre.cpp:453`)
      logging the Kendall-tau distance between `res_opt_.priority_vec` before
      vs after the reopt, plus the SP delta. Template: the P0.5
      `SeedStateFromIncumbent` dump at `:~430`.
- [ ] Rebuild with `debugMode:1`; run one INCR_P10 N=6 taskset; grep the dump.
- [ ] A high-Kendall-tau commit for a near-zero SP gain = smoking gun. Record.

## Step 3 — Decision (only after Step 1 [+ Step 2 if run] converges)
- [ ] **If hazard does NOT fire:** record the negative result, close the task,
      milestone to top-level `dev_log.md`. The SP-value guard is sufficient in
      practice.
- [ ] **If hazard DOES fire:** TDD-first for B+C —
      - [ ] Failing test: `OptimizeFromScratch` returns a structurally-worse
            permutation that squeaks past the SP guard (via `ApproxEqualSP`
            tie-break or marginal SP gain); assert `res_opt_.priority_vec`
            unchanged after `ReOptimizePeriodic`.
      - [ ] Option B: seed the bootstrap beam with π_incumbent in
            `OptimizeFromScratch` (`OptimizeSP_Incre.cpp:74`).
      - [ ] Option C: chain the TL walk via `OptimizeIncre` (from_scratch=false
            after the bootstrap) in `PerformCoordinateDescentForTaskConfigOpt`
            (`OptimizeSP_TL_Incre.cpp:241`).
      - [ ] `make check.SP_OPT -j5` green; user runs the A/B to confirm.

## Standing constraints
- No `git commit` (user's task; `git add` only).
- No running the full A/B suite myself (user runs `run_simulation_and_plot_figures.sh`).
- Step 1 reads **existing** output; a fresh A/B run is the user's call.
