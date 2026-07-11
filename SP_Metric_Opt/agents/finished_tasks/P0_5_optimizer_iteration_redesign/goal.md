# P0.5 — Redesign the Optimizer Iteration Process (incumbent state) — RESOLVED 2026-07-10

> **RESOLVED 2026-07-10.** All five phases complete; code committed (commits
> `a8dba07f`→`7fa2e9d2`); 46 `testIncreOpt_w_TL` + 16/16 ctest green (DEBUG
> build, re-verified at closeout). The functional TL-init bug (formerly P0.1) is
> subsumed by construction — `BuildChallengerFromIncumbent` rebuilds the
> adopted-TL DAG from `res_opt_` so both diff sides of `FindTaskWithDifferentEt`
> carry the adopted TL; runtime-confirmed `ndiff` 5→0. Phase-5 review issues:
> 5a unified `ResetIncumbentBaseline`, 5c dropped the stale-TL guard, 5d removed
> `has_incumbent_` (provably redundant — `IfInitialized()` gate), 5e renamed
> `starting_time_limits`, 5f removed dead `any_eval_ran` fallback, 5g total-budget
> patience; 5b kept rebuild-from-champion; 5h (efficiency) moved to P3.1.
> **Authoritative record:** `agents/finished_tasks/summary.md`; rationale in
> memory `p05-subsumes-tl-init-bug.md`. Scope/design text below is the original,
> kept for the rationale trail.

**Priority:** P0 (algorithm-correctness + structural; user's #2 priority)
**Status:** RESOLVED 2026-07-10 — all phases complete & committed
(`a8dba07f`→`7fa2e9d2`); 46 `testIncreOpt_w_TL` + 16/16 ctest green.
**Predecessor:** ~~P0.1~~ — **reversed (2026-07-08).** The incumbent-state redesign
subsumes the *functional* TL-initialization bug by construction
(`BuildChallengerFromIncumbent` reconstructs the adopted-TL DAG from `res_opt_`,
so both sides of `FindTaskWithDifferentEt` carry the adopted TL → the `ndiff`
false positives vanish, no YAML write needed). P0.1 (YAML persistence) is
demoted to **inspectability-only** (the on-disk file reflecting the running
prior; the orchestrator's `setExecutionTime(GetAvgValue())` read at
`SimulationOrchestrator.cpp:395/684` — but jobs are clamped to
`res.id2time_limit` at `:461-463`, so that read is debuggability, not runtime
correctness). **P0.5 is worked first;** P0.1 is a smaller follow-up or an
optional write site in Phase 3d. P0.5 *absorbs* the former P0.1b (data-member
refactor).

> **The design is decided.** See [`design.md`](design.md) for the full spec —
> this file is the scope/summary. The earlier "propose the target shape, do not
> decide" framing is superseded; the user chose the incumbent-state approach
> (res_opt_ as single durable incumbent, `has_incumbent_` gate,
> `CommitIncumbent`/`BuildChallengerFromIncumbent` helpers, transient
> challenger, diff-invariant through reconstruction).

## The problem

`OptimizePA_Incre_with_TimeLimits` carries the running system's "best-so-far"
solution — the **incumbent** 4-tuple `{DAG-with-adopted-TL, SP, PA, TL-config}`
— across intervals. Today that 4-tuple is **split across four data members that
can desync**: `prev_optimizer_` (a full `OptimizePA_Incre` that DUPLICATES
`dag_tasks_` / `sp_parameters_` / `opt_pa_` / `opt_sp_` already held by the
derived class), `res_opt_` (overlaps `opt_pa_`/`opt_sp_`), the base-class
`opt_pa_`/`opt_sp_`, and `time_limit_option_for_each_task_`.

Three concrete symptoms this split causes (all currently masked by defensive
copies; all would re-emerge under any refactor that drops one copy):

1. **`sp_parameters_` desync** — `IfInitialized()` only checks `!opt_pa_.empty()`,
   so `prev_optimizer_` can be "initialized" with an empty `sp_parameters_` →
   `OptimizeIncre`'s SP-eval throws `_Map_base::at` on `thresholds_node`. Masked
   only because `UpdateRecords`'s full copy usually fires between reopt and the
   next incremental call; the manual `prev_optimizer_.sp_parameters_ =
   sp_parameters_` line (`OptimizeSP_TL_Incre.cpp:476`) is load-bearing.
2. **Frozen-baseline DAG** — `OptimizeIncre`'s `dag_tasks_ = dag_tasks_update`
   side-effect (`OptimizeSP_Incre.cpp:296`) only propagates to the incumbent via
   `UpdateRecords`'s copy. A dedicated regression test
   (`OptimizeIncre_AdvancesPrevOptimizerDagTasks`) exists *only* to guard this
   fragile path.
3. **Diff-side cold-start** — the incremental descent must start from the
   carried adopted TL (else `FindTaskWithDifferentEt` flags unchanged perf-pair
   tasks). Today enforced by a per-site `ReconstructTimeLimitVecFromResOpt()`
   override at `OptimizeIncre_w_TL:387` (P1.1 workaround); the reopt path
   (`ReOptimizePeriodic:535`) still cold-starts from the Gaussian-mean TL — the
   same bug on the other path.

Per user (2026-07-08): redesign the **state representation** so the incumbent is
owned once and the cold-start prior is read from the correct source by
construction. (The *flow* re-derivation is downstream, gated on a trustworthy
prior from P0.1 — see `design.md` §5.)

## Goal

Produce a clean, single-source-of-truth incumbent state where:
- The incumbent 4-tuple is owned **once** in `res_opt_` (no parallel
  `prev_optimizer_` cache); the `sp_parameters_` desync class of bug is
  structurally impossible (one writer: `CommitIncumbent`).
- The cold-start prior is read from `res_opt_` at exactly one site
  (`BuildChallengerFromIncumbent`), so both sides of the `FindTaskWithDifferentEt`
  diff carry the adopted TL by construction — the P1.1 per-site override becomes
  the structural norm.
- The DAG-advance side-effect of `OptimizeIncre` is harmless (challenger is a
  throwaway local; the adopted TL in `res_opt_` is the real invariant,
  reconstructed next interval).
- `eval_count_` and `from_scratch_flags` stay as documented test seams (read by
  `testIncreOpt_w_TL.cpp:1057+` / `:770+`); do not silently remove them.
- **Zero public-API change.** Blast radius = two source files + one test file +
  one comment-only touch (see `design.md` §2).

## Approach (TDD, per `agent_coding_rules.md`)

The design is decided (see [`design.md`](design.md)). The approach is the
additive → dual-write → flip-readers → remove migration specified in
`design.md` §4 and mirrored as a per-step checklist in `tasks.md`:

1. **Phase 1 — additively introduce** `has_incumbent_` +
   `CommitIncumbent` / `BuildChallengerFromIncumbent` (no call-site change; +
   focused unit tests for each helper).
2. **Phase 2 — dual-write**: flip `SeedStateFromIncumbent` and `UpdateRecords`
   to call `CommitIncumbent` while still writing `prev_optimizer_`; then flip
   the `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch to build the
   challenger via `BuildChallengerFromIncumbent` (challenger becomes transient).
3. **Phase 3 — migrate tests** off `prev_optimizer_.*` (the 3 DAG-ET
   assertions rewrite to observe the carried TL via `res_opt_`), then **remove**
   `prev_optimizer_` + the `IfInitialized()` gate, then the comment-only fix in
   `OptimizeSP_Incre.cpp:293-295`, then re-derive the P1.1 workaround
   (`OptimizeIncre_w_TL:387` / `ReOptimizePeriodic:535`).
4. **Phase 4 — verify + close**: `testIncreOpt_w_TL` (43) + `ctest` (16) green;
   re-run the P1.1 probe (`ndiff` → ~2); milestone to top-level `dev_log.md`.

**Behavior-preserving by construction.** The existing `testIncreOpt_w_TL` (43) +
`testOptimizeIncrePA` + `ctest` (16) must stay green after each step. The 3
frozen-baseline DAG-ET assertions are rewritten (Phase 3a) to observe the
carried TL — they prove the diff-baseline invariant survives the reconstruction
(see `design.md` §3).

## Files (expected to touch)

- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` — remove `prev_optimizer_`;
  add `has_incumbent_` + `CommitIncumbent` / `BuildChallengerFromIncumbent`;
  rewrite `SeedStateFromIncumbent`, `UpdateRecords` write-block,
  `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch,
  `SeedIncumbentBaseline` gate.
- `sources/Optimization/OptimizeSP_Incre.cpp` — **comment-only** (`:293-295`):
  rewrite the "copies into `prev_optimizer_`" comment to the
  throwaway-challenger / `res_opt_`-carries-the-adopted-TL model.
- `sources/Optimization/OptimizeSP_Base.h` — **read-only reference** for
  `ResourceOptResult` / `OptimimizePA_Base`; no change.
- `tests/testIncreOpt_w_TL.cpp` — migrate the 6 `prev_optimizer_.*`-reading
  tests; rewrite the 3 DAG-ET assertions to observe carried TL via `res_opt_`;
  keep `eval_count_` / `from_scratch_flags` seams.

## Done when

- Single incumbent-state owner; `prev_optimizer_` removed; `has_incumbent_` is
  the interval-0 gate.
- Cold-start prior read from `res_opt_` at one site
  (`BuildChallengerFromIncumbent`); the P1.1 per-site
  `ReconstructTimeLimitVecFromResOpt()` override is the structural norm (folded
  into the helper) or explicitly justified.
- The 3 frozen-baseline DAG-ET assertions rewritten and passing via `res_opt_`
  — the diff-baseline invariant survived the reconstruction.
- `testIncreOpt_w_TL` (43) + `ctest` (16) green after each step.
- P1.1 probe re-run: `ndiff` reconciled with ground truth (~2).
- Re-derivation recorded in `dev_log.md`: the P1.1 workaround kept-as-norm or
  simplified (Phase 3d).
- Milestone appended to top-level `agents/dev_log.md`.

## Out of scope

- **Flow-shape change** — the INCR/REOPT/scratch dispatch, compare-and-keep
  guard, and cold-start bootstrap keep their current logic. Whether the flow
  *simplifies* once the incumbent is owned once is a Phase-3d re-derivation,
  gated on a trustworthy prior from P0.1 — not a precondition of this design.
- **`ResourceOptResult` overlap collapse** — `id2priority` + `id2time_limit` +
  `priority_vec` + `sp_opt` stay as a transport type; `res_opt_` is the store,
  base-class `opt_pa_`/`opt_sp_` stay as thin mirrors (zero public-API change).
- **`PriorityPartialPath` per-path copies** (`OptimizeSP_Incre.h:41-42`) —
  separate perf concern, deferred to P3.1.
- Fix C (per-variation `ObtainSP_DAG` scoring asymmetry) — wrong lever per P1.1;
  not revisited here.
- Fix D (`GetAvgValue` band / `FiniteDist::approx_equal` tolerance) — inert
  today; tracked separately as P2.3 (dead-code cleanup), not a behavior fix.
- The TL-bug *behavior* fix itself (YAML overwrite) — that is P0.1, the
  prerequisite.
