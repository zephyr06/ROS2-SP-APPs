# P1.23 — Tasks (working checklist)

> Goal: A/B whether the RTA cache speeds up the per-interval scheduler
> optimization. OLD = `d67aaa65` (no cache) + timer-fix `cc9aa0ce`; NEW = HEAD
> `7c47b93b` (cache + timer-fix). Same tasksets, same `INCR_Reopt_10` mode.

## Phase 0 — Baseline worktree + timer-fix port

- [ ] **0a. Create no-cache baseline worktree**
  - `git worktree add _perf_nocache_d67aaa65 d67aaa65` (detached at the
    pre-cache commit; distinct from the existing `_perf_old_ecbed896` which is
    the transaction A/B's baseline and is NOT used here).
- [ ] **0b. Cherry-pick the scheduler-only timer fix onto the baseline**
  - In the worktree: `git cherry-pick cc9aa0ce`.
  - Verify anchors landed: `GetSchedulerExecutionTime()` in
    `SimulationOrchestrator.h`, the chrono bracket in
    `DeterminePrioritiesAndBudgets`, and `RunOrchestrator.cpp` writing
    `GetSchedulerExecutionTime()` to `scheduler_execution_time.txt`.
  - Do NOT commit the cherry-pick into the repo history — leave it as a local
    worktree-only change (the worktree is disposable). The point is a binary,
    not a branch.
- [ ] **0c. Confirm the main working tree is untouched**
  - The main tree's uncommitted `run_sim_experiments.py` / `utils.py` edits and
    the staged figure fix are NOT in the worktree path and must remain intact.

## Phase 1 — Build both binaries (Release / perf path)

- [ ] **1a. Build OLD binary (no cache + timer-fix)**
  - In `_perf_nocache_d67aaa65/SP_Metric_Opt/release/`:
    `cmake --build release --target RunOrchestrator -j6` (Release `-O3 -DNDEBUG`,
    the perf-measurement path).
- [ ] **1b. Build NEW binary (HEAD cache + timer-fix)**
  - In the main tree `SP_Metric_Opt/release/`:
    `cmake --build release --target RunOrchestrator -j6`.
  - HEAD already carries `cc9aa0ce`; no cherry-pick needed.
- [ ] **1c. Smoke both binaries**
  - One short `INCR_Reopt_10` run each at N=4 to confirm the binary launches,
    writes `scheduler_execution_time.txt` as a single float, and does not crash.
    (Reuses an existing N=4 taskset; no full run yet.)

## Phase 2 — A/B run (shared tasksets, N=10)

- [ ] **2a. Reuse ONE shared taskset set across both arms**
  - Use the existing N=10 tasksets under the eval run dir
    (`tasks10_dur600_interval10_seed1000/...`) so OLD and NEW run the **exact
    same** tasksets → the only variable is the binary. Regenerate only if the
    generator config drifted.
- [ ] **2b. Run OLD arm (no cache)**
  - `INCR_Reopt_10`, N=10, dur=600, interval=10, seed=1000, 1 taskset (extend
    to more if N=10 is ambiguous). Output under a `_nocache` run root so it does
    not clobber NEW.
- [ ] **2c. Run NEW arm (cache)**
  - Same config, same tasksets, output under the existing run root (or a
    `_cache` twin for a clean side-by-side).
- [ ] **2d. (Conditional) Extend to N=6 and N=16**
  - Only if N=10 is within noise. The cache signal should grow with N; N=16 is
    the strongest test. Do NOT run the full N=[4,6,8,10,12,14,16] suite.

## Phase 3 — Compare + verdict

- [x] **3a. Read `Mean_Scheduler_Execution_Time_s` from both arms**
  - DONE. OLD mean = 1.31841 s, NEW mean = 0.84172 s (mean over 10 tasksets,
    matched N=10, matched scheduler `INCR_Reopt_10`). NEW/OLD = 0.638.
- [x] **3b. Correctness gate: SP bit-identical OLD vs NEW**
  - DONE. SP bit-identical on all 10/10 tasksets (per-taskset values in
    `dev_log.md`). Cache is a pure memoization — gate holds.
- [x] **3c. Verdict**
  - **SPEEDS UP.** NEW ~36.2% faster than OLD (NEW/OLD = 0.638). Every taskset
    faster (11.6%–53.1%). Decisive at N=10.
- [x] **3d. Log result to dev_log.md + MEMORY.md pointer**
  - DONE (dev_log verdict entry + MEMORY.md pointer added). Closes P1.12's last
    open item (the N=6/10/16 scalability measurement).

## Phase 4 — Cleanup

- [ ] **4a. Remove the no-cache worktree**
  - `git worktree remove _perf_nocache_d67aaa65` (disposable; the cherry-pick
    was never committed). Also still-pending from prior session: remove
    `_perf_old_ecbed896` (the transaction A/B's worktree — unrelated, but stale).
- [ ] **4b. Confirm main tree clean**
  - No stray files; the staged figure fix + uncommitted Python edits intact.
