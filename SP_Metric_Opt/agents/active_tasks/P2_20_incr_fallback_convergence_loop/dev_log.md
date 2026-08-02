# P2.20 — Dev Log

## 2026-08-01 — Implemented (staged, NOT committed)

**Goal:** `ComputeSafeFallback` ran `OptimizeIncre_w_TL` **single-pass**. P2.19
point (2) (split off as this task) adds a convergence loop: re-run the pass until
a full pass cannot improve `opt_sp_`. Behavior-change ENHANCEMENT to the offline
safe-fallback SP (not a crash fix — P2.19 `58264b20` was the crash fix).

### Design (resolved with user)

**Loop location:** new `virtual OptimizeIncre_w_TL_UntilConvergence(dag, beam)`
wraps `OptimizeIncre_w_TL`. `ComputeSafeFallback:1000` calls the wrapper instead
of the raw pass. Chosen over an inline loop at the call site so the loop's control
flow is unit-testable via a stub (the "virtual wrapper + stub TDD" option).

**Stub seam:** `OptimizeIncre_w_TL` was non-virtual; made it `virtual` so a test
subclass can override it to inject a controlled SP-per-pass sequence. (Other
`virtual` walk methods already exist — `PerformSerializedTaskQueueOptimization`,
`CallOptimizerGivenTimeLimits`, `OptimizeIncreSingleTask` — so this matches the
established testability pattern.)

**Convergence predicate:** `ApproxEqualSP(opt_sp_, sp_before)` (`OptimizeSP_TL_Incre.h:11`,
`rel_tol=1e-9`) → break. `opt_sp_` is `public` (`OptimizeSP_Base.h:130`) so the
wrapper reads it directly before/after each pass.

**Iteration cap: NONE (revised with user, 2026-08-01).** Originally `N + 1`
("N improving passes + 1 confirming"). That rationale was unproven — coordinate
descent on a discrete (PA × TL) space can need *more* than N sweeps (moving one
task can re-open a better move for an already-visited task), so `N+1` could
prematurely cut a genuinely-improving case. The real termination guarantee is
**monotonicity over a finite config space**: every committed pass strictly
increases `opt_sp_` (`WouldBeatIncumbent` commits strictly-greater only), and
`opt_sp_` is drawn from a finite set (finite tasks × finite TL grid
`timePerformancePairs` × finite PA permutations, deterministic RTA) → finitely
many strict improvements → the loop provably terminates with no cap. The runtime
guard is `BFDLSharedBudget` (a cancelled mid-pass commits nothing → no-improvement
→ break), not a pass-count cap. Stop predicate: `opt_sp_ <= sp_before ||
ApproxEqualSP(opt_sp_, sp_before, 1e-3)` — break unless the new SP is strictly
higher beyond 1e-3 RTA noise. The explicit `<=` and 1e-3 tolerance also guard a
monotonicity regression from spinning the loop.

**Scope:** loop applies to `ComputeSafeFallback` (offline safe-fallback compute)
ONLY — not the online per-interval `OptimizeIncre_w_TL`. Matches P2.20's split
rationale (enhancement to the fallback's SP, not required for correctness; avoids
multiplying the per-interval walk's runtime).

### Safety preconditions (verified before implementing)

- **`opt_sp_` public + readable** — `OptimizeSP_Base.h:130`.
- **Monotonic across passes** — each pass re-seeds the carried incumbent as baseline
  (`ResetIncumbentBaseline(false)` → re-commit), then `UpdateRecords`/`WouldBeatIncumbent`
  commits only strictly-greater SP. Non-decreasing pass-to-pass; a pass that fails to
  strictly improve = converged.
- **`AbsorbUpdatedDAG` idempotent** on the same DAG (self-assign + option refresh) →
  re-calling the pass on `fallback_solver.dag_tasks_` is safe; each pass warm-starts
  from the previous pass's `res_opt_`.
- **`BFDLSharedBudget` at `:974`** wraps the whole compute → all passes share one
  budget; `BFSharedBudgetCancelled()` (polled in-walk at `:217,:322,:340,:355,:475,:652`
  in TL_Incre) naturally breaks the loop if the budget expires mid-pass (no commit →
  `sp_after==sp_before` → stop).
- **RTA cache lifecycle (P2.18 lesson):** each pass re-arms the cache via
  `SeedBaselineAndArmCache` and disarms at `RunIntervalDescent:576`; `ResetIncumbentBaseline`
  default-constructs `rta_cache_`. Re-looping is safe — each pass resets + re-arms cleanly.

### TDD

**Red:** `OptimizeIncre_w_TL_UntilConvergence_StopsOnNonImprovingPass`
(`testIncreOpt_w_TL.cpp`, `CompareAndKeepSynthetic` fixture). `ConvergenceStubOpt`
overrides `OptimizeIncre_w_TL` to inject `{0.5, 0.8, 0.8}`: pass 1 sets baseline
(improves over the pre-seeded -1), pass 2 strictly improves, pass 3 fails to improve
→ converged. Asserts `passes_run==3` + `opt_sp_==0.8`. RED with the single-pass stub
(ran 1 pass, `opt_sp_==0.5`).

**Green:** loop implemented in `OptimizeIncre_w_TL_UntilConvergence`. 127/127
`testIncreOpt_w_TL` (was 126; +1 new). 16/17 ctest (sole failure = pre-existing
`testScheduleSimulate`/`CFS_RunOrchestrator_Binary`, unchanged from baseline).

### Changes (3 source files + 1 test + records)

- `sources/Optimization/OptimizeSP_TL_Incre.h` — `OptimizeIncre_w_TL` → `virtual`;
  + `virtual OptimizeIncre_w_TL_UntilConvergence` decl.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — `OptimizeIncre_w_TL_UntilConvergence`
  impl (loop, cap=N+1, `ApproxEqualSP` stop); `ComputeSafeFallback:1000` routed through it.
- `tests/testIncreOpt_w_TL.cpp` — `ConvergenceStubOpt` + the red→green test.
- `agents/active_tasks/P2_20_*/{tasks,dev_log}.md` + memory.

### TODO before review handoff
- P2.19 repro (`taskset_3 INCR_Reopt_10`) still exits 0 (no regression to the crash fix).
- `git add` the staged changes; user reviews + commits.
