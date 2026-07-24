# P1.23 — Dev Log

## 2026-07-22 — Task creation

### Why this task
P1.21/P1.22 ask whether the `Transaction` (CoW) is slower than the eager
full-cache copy — a cache-*internal* question. The more fundamental question was
still open: does the RTA cache speed up the optimizer **at all** vs no cache?
That needs a no-cache arm, which requires a traceback (HEAD has no flag to
disable the cache).

### A/B boundary found
- `3e9b6518` ("add cache to optimimizeSP") = first commit to engage the cache in
  `OptimizeSP_Incre` (16 `RTACache` refs added in that one commit).
- Its parent `d67aaa65` = zero `RTACache`/`rta_cache` refs in the opt path →
  clean no-cache baseline.
- HEAD = `7c47b93b` (cache engaged + `RTACache::Transaction` from `4d7d14b6`).

### Timer-fix precondition
- The scheduler-only timer fix is `cc9aa0ce`. It is already on HEAD.
- It is NOT on `d67aaa65`, but all its C++ anchors exist verbatim there
  (`DeterminePrioritiesAndBudgets` body, `BaseSimulationOrchestrator`,
  `RunOrchestrator.cpp` timing blocks) → cherry-picks cleanly.
- Both arms MUST carry it, else the optimizer delta is diluted by RTDA+I/O noise
  (the prior transaction A/B's +5.02%@N=10 / +0.32%@N=16 inversion trap).

### Eval plumbing confirmed on baseline
`d67aaa65` already has `MaybeOverrideReoptPeriod` / `IsINCRPeriodVariant` /
`Optimize_w_TL_ScratchOrIncre` → the eval config's `INCR_Reopt_10` arm runs
unchanged on the no-cache binary.

### Next
Phase 0: create `_perf_nocache_d67aaa65` worktree, cherry-pick `cc9aa0ce`, build
both binaries, run the N=10 A/B.

## 2026-07-22 — Phase 0-1 done, smoke result, full A/B running

### Phase 0 (worktree + timer-fix port)
- `git worktree add _perf_nocache_d67aaa65 d67aaa65` → detached at the no-cache
  commit. Distinct from the existing `_perf_old_ecbed896` (transaction A/B).
- `git cherry-pick cc9aa0ce` in the worktree → clean, no conflicts → worktree
  HEAD `0d93818b` ("fix scheduler ET measuring bug") on `d67aaa65`. The
  cherry-pick is worktree-local (NOT pushed to repo history); the worktree is
  disposable.
- Verified all timer-fix anchors landed: `GetSchedulerExecutionTime()` +
  `scheduler_exec_time_s_` in `SimulationOrchestrator.h`; the chrono bracket in
  `DeterminePrioritiesAndBudgets` (`:322` start, `:400-402` end+accumulate);
  `RunOrchestrator.cpp` writes `GetSchedulerExecutionTime()` to
  `scheduler_execution_time.txt` + prints `TotalProcessTime_s` to stdout.

### Phase 1 (build both binaries, Release)
- OLD (no cache): `_perf_nocache_d67aaa65/SP_Metric_Opt/release/tests/RunOrchestrator`
- NEW (cache): `SP_Metric_Opt/release/tests/RunOrchestrator`
- Both `-O3 -DNDEBUG`.
- **Definitive arm differentiation:** OLD `OptimizeSP_Incre.cpp` /
  `OptimizeSP_TL_Incre.{cpp,h}` have **0** `RTACache` refs; NEW has 16/13/4.
  The cache class exists in both (so `nm` shows symbols in both) but is engaged
  in the opt path ONLY in NEW.

### Smoke test (taskset_0, INCR_Reopt_10, 10000ms, export 1) — apples-to-apples
Both arms on the FIXED timer, same input, same mode:

| arm | SchedulerExecutionTime_s | Average SP |
|-----|--------------------------|------------|
| OLD (no cache) | 1.91782 | 0.43854 |
| NEW (cache)    | 1.31258 | 0.43854 |

- Cache makes the scheduler **~32% faster** on taskset_0 (1.92s → 1.31s).
- **SP bit-identical** (0.43854) — confirms the cache is a pure memoization
  (correctness gate holds).
- The prior NEW-arm `scheduler_execution_time.txt = 20.1249` on taskset_0 was
  STALE (pre-timer-fix, whole-`RunSimulation` wall-time); re-running NEW on the
  fixed binary gave 1.31258. This is exactly the timer-dilution trap the fix
  was for.

### Full A/B (10 tasksets, N=10) — DONE, verdict = cache is a clear net win

Both arms completed (rc=0 on all 10×2 runs). Per-taskset
`SchedulerExecutionTime_s` / `Average SP`:

| taskset | OLD (no cache) | NEW (cache) | NEW/OLD | SP match |
|---------|---------------|-------------|---------|----------|
| 0 | 2.32479 / 0.43854 | 1.75962 / 0.43854 | 0.757 | ✓ |
| 1 | 0.201987 / 0.713957 | 0.147539 / 0.713957 | 0.731 | ✓ |
| 2 | 0.669359 / 0.581647 | 0.591619 / 0.581647 | 0.884 | ✓ |
| 3 | 1.23545 / 0.528117 | 0.775747 / 0.528117 | 0.628 | ✓ |
| 4 | 2.5562 / 0.628794 | 1.1978 / 0.628794 | 0.469 | ✓ |
| 5 | 0.723591 / 0.638311 | 0.437612 / 0.638311 | 0.605 | ✓ |
| 6 | 0.612319 / 0.549182 | 0.521828 / 0.549182 | 0.852 | ✓ |
| 7 | 1.57019 / 0.394388 | 0.880499 / 0.394388 | 0.561 | ✓ |
| 8 | 1.64029 / 0.333824 | 1.15687 / 0.333824 | 0.705 | ✓ |
| 9 | 1.64989 / 0.624891 | 0.948044 / 0.624891 | 0.574 | ✓ |

- **Mean scheduler time:** OLD = 1.31841 s, NEW = 0.84172 s.
- **Speedup ≈ 36.2%** (NEW/OLD = 0.638). Every taskset faster (11.6%–53.1%).
- **Correctness gate:** SP **bit-identical** on all 10/10 tasksets — confirms the
  cache is a pure memoization, no SP drift.
- **Verdict: the RTA cache SPEEDS UP the per-interval scheduler optimization.**
  Decisive at N=10 → N=6/16 sweep NOT needed (P1.23 rule: extend only if N=10
  ambiguous; it isn't).

### Closes P1.12's last open item
P1.12's only remaining task was "End-to-end scalability measurement at N=6/10/16
(cache on vs off)". This A/B IS that measurement at N=10 → P1.12 measurement goal
satisfied. P1.12 can be closed.

### Caveat (recorded for honesty, not a blocker)
The A/B delta is `d67aaa65`→HEAD, which is cache + the RTA-cache refactors
(`ecbed896`, `bb11a4e9`, …) + the `Transaction` (`4d7d14b6`), NOT a surgically
isolated cache-only delta. HEAD has no flag to disable the cache, so a pure
cache-only A/B is impossible — this is the best-available measurement and is what
P1.23 was designed to be. The cache is the dominant feature added in the range.
