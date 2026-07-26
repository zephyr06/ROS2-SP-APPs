# P2.11 — Dev Log

## 2026-07-25 — Task filed; design recorded + evaluated

### Why this task
The user rejected the prior (A)/(B) design framing for merging incremental +
reopt as "too complicated with many changes that cannot be easily interpreted"
and proposed their own simpler design: **re-opt = the standard incremental path
+ (1) RM-Fast interval-0 init + (2) one upfront re-opt step at each interval
start adopting last interval's TLs → champion, then standard incremental walk
with patience+1.** The user asked to "first evaluate this new design" before any
implementation; this task records that evaluation and the resulting design.

### Evaluation of the user's design (the verdict)
**Headline: the design is ~90% already implemented — it is essentially the P2.9
flag-on path.** Mapping the user's 3 points onto the live code:

1. **Interval 0 = RM-Fast bootstrap → ALREADY EXISTS.** The first call (count==0)
   goes to `OptimizeIntervalFromScratch` → `ResetIncumbentBaseline(true)` with no
   incumbent → falls back to `RateMonotonicPriorityVec()` + min/Gaussian TL. So
   interval-0 RM bootstrap is current behavior, not a new delta. (Minor open
   question: current seeds Gaussian-mean TL via `InitializeTimeLimitsFromETConfig`;
   "RM-Fast" may mean strict min-TL via `SmallestTimeLimitVec` — flagged in
   goal.md open questions.)
2. **"One upfront re-opt on new DAG with adopted TLs → champion, then incremental
   walk, patience+1" → ALREADY EXISTS (P2.9 flag-on).** `RunReoptTLDescent` already
   does, verbatim:
   - `ResetIncumbentBaseline(true)` — re-score carried {pa,tl} under new DAG, commit
     as compare-guard baseline. (= "adopt TLs from last interval")
   - `EvaluateTimeLimitConfig_PAReopt(from_scratch=true)` — one from-scratch PA
     re-opt on the adopted TL → first candidate. `UpdateRecords`' strict-SP guard
     keeps the carried champion if the re-opt is worse. (= "results from this
     single re-optimization will be the champion")
   - Then (flag ON) `OptimizeOneTaskTimeLimit`, patience 1. (= "incremental
     walk, patience+1")

   So the user's #2 is the P2.9 flag-on path, exactly.

3. **The ONE genuine delta: Type-E in the reopt queue.** The user's phrase "do the
   same as the standard incremental optimization" has two readings:
   - **(a) Full merge** — reopt adopts the Type-E + Type-L serialized queue
     (`BuildSerializedTaskQueue`), like incremental. Currently reopt walks Type-L
     only (`CollectTLFlexibleTaskIds`). This adds targeted 1D patches on env-changed
     tasks. Genuinely new.
   - **(b) Loose merge** — reopt keeps Type-L only, uses SubIncremental arm +
     patience 1. Exactly P2.9 flag-on — zero new code.

   Chosen: **(a)** (maximal merge; the more interesting experiment). Fallback (b)
   if Type-E plumbing breaks the cache contract.

### Behavior-change assessment (NOT bit-identical)
Both readings change SP vs current default reopt (flag OFF = FullBeam walk):
- (b) replaces FullBeam (re-opts PA per trial TL) with SubIncremental (patches 1
  task per trial TL). Loses PA exploration; patience-1 partially compensates. =
  the P2.9 A/B already staged.
- (a) = (b) + Type-E — strictly more targeted work. Should be ≥ (b) in SP at
  modest extra cost.

This needs its own A/B (user already plans this). It is NOT a P2.10 bit-identity
gate — it is a behavior change.

### The "frequent reopt performs worse than incremental" hypothesis
Three candidate causes:
1. **Incumbent degradation** — reopt's from-scratch PA re-opt commits a worse
   permutation. **Mitigated in this design**: carried {pa,tl} is the compare-guard
   baseline; `UpdateRecords` adopts only strictly-better. The upfront GlobalBeam
   cannot regress past the carried champion. ✓ safe.
2. **TL thrashing** — reopt re-searches TLs every period; patience-1 allows a
   non-improving step that doesn't pay off next interval. **Not eliminated**
   (patience still 1). Likely culprit. This design inherits it.
3. **Type-E blindness (current reopt only)** — current reopt ignores env-changed
   tasks. Reading (a) ADDS Type-E → strictly better targeted response. This
   dimension should make this design BETTER than current reopt, narrowing the gap
   to incremental.

So the user's hunch ("similar to current frequent reopt") is most likely right
for cause 2, but cause 3 may improve on current reopt. Net effect is empirical —
the A/B will tell. The design is sound to test.

### Simplicity win (the user's stated motivation) — confirmed
End state under this design:
- **1 descent body** (`RunIntervalDescent(mode)`) replaces `RunIncrementalTLDescent`
  + `RunReoptTLDescent`.
- **1 walk arm** (`OptimizeOneTaskTimeLimit`); `WalkOneTaskTimeLimit_FullBeam`
  deleted (the `GlobalBeam` *eval* stays for the one upfront baseline call only).
- **P2.9 flag dies** (reopt always uses SubIncremental walk).

Under reading (a), the reopt-vs-incremental delta shrinks to ~2 things: (i)
patience 0 vs 1, (ii) one upfront GlobalBeam PA re-opt (reopt) vs dedicated
re-score (incremental). That is the maximal merge the user wants.

### Relationship to P2.10 + P2.9
- **P2.10 is the prerequisite** — it merged the duplicated Type-L walk into the
  shared `OptimizeOneTaskTimeLimit` and renamed the confusable symbols.
  This task builds on that shared arm.
- **P2.9 is subsumed** — its flag-on path IS this design's walk. This task deletes
  the flag (reopt always uses the sub-incremental walk) and the legacy full-beam
  arm. P2.9's A/B (flag 0 vs 1) becomes this task's A/B (current-default-reopt vs
  new-merged-reopt).

### Status
Awaiting Phase 0 confirmations (reading (a) vs (b); RM-Fast TL semantics; Type-E
cache-contract verification against the `.cpp`) before implementation. No code
edits yet.

## 2026-07-25 — Phase 0 verified against the `.cpp`; Phase 0.5 rename landed

### Phase 0 closed (read `OptimizeSP_TL_Incre.cpp`)
- **0b (reading (a)/(b)) — RESOLVED.** User approved the updated call path
  (which includes Type-E on the reopt queue). Reading (a) confirmed.
- **0c (RM-Fast TL semantics) — RESOLVED.** `ResetIncumbentBaseline(true)`
  (line 723-754) at interval 0 already seeds `SmallestTimeLimitVec()` +
  `RateMonotonicPriorityVec()` = RM-Fast (strict smallest-TL). The Gaussian-
  mean `InitializeTimeLimitsFromETConfig` is ONLY the seed vector passed into
  `OptimizeIntervalFromScratch` (line 774); the reset overrides it with the
  RM-Fast champion. So interval-0 = RM-Fast champion with **no** upfront re-opt
  step (that step runs only when `IfInitialized()` is true — line 732-740).
- **0d (Type-E cache contract) — VERIFIED.** `EvaluateTimeLimitConfig_SingleTaskPatch`
  asserts `|diff|<=1` (line 208); the comment at line 206-207 states Type-E =
  `|diff|==0` ("the env move was absorbed into `dag_tasks_` before the champion
  was built, so it cancels") → the lighter FullReuse path. So Type-E on the
  reopt path holds the contract, PROVIDED `dag_tasks_prev_pre_tl` is captured
  before the absorb (the Phase 3a plumbing).

### Phase 0.5 — rename `EvaluateTimeLimitConfig_PAReopt` → `_PAReopt` (landed)
**Why:** the name `GlobalBeam` is wrong — the function does both
`OptimizeFromScratch` (from_scratch=true) AND `OptimizeIncre` warm-start
(from_scratch=false), confirmed at line 148-160. It evaluates one TL vector by
RE-OPTIMIZING the priority assignment; "GlobalBeam" describes only the
from_scratch half. The header's own docstring already says "RE-OPTIMIZING the
priority assignment" (line 103).

**Name chosen:** `EvaluateTimeLimitConfig_PAReopt` — parallels the existing
`_SingleTaskPatch` (PA = priority assignment, an established term; Reopt =
re-optimize the PA). Avoids inventing new terminology.

**Scope:** behavior-neutral symbol rename across 5 code files
(`OptimizeSP_TL_Incre.{h,cpp}`, `Parameters.h`, `tests/testIncreOpt_w_TL.cpp`,
`tests/testScheduleSimulate.cpp`) via `sed`. 0 old-name hits remain.

**Verify:** `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
→ 17/17 ctest green (incl. `testIncreOpt_w_TL` 4.35s, `testScheduleSimulate`
0.08s). Header change → used `--clean-first` (stale-`.o` lesson from P1.21).

**Status:** awaiting user review + commit as a standalone modular step BEFORE
the behavior-changing merge. Per agent_coding_rules.md "work by module, commit
by module." The merge (Phases 1-3) builds on this renamed symbol.

## 2026-07-25 — Phase 3a+3b landed (Type-E in the reopt queue, flag-gated)

### State correction first (do NOT trust the design docs' symbol names)
HEAD is `e986f132`, not `6f7ed838`. The P2.10 13-fn rename + the Phase 0.5
`_PAReopt` rename were BOTH reverted. The code retains OLD names:
`ReOptimizePeriodic`, `PerformCoordinateDescentForTaskConfigOpt`,
`PerformSerializedTaskQueueOptimization`, `EvaluateTimeLimitConfig_ScratchOrIncre`,
`EvaluateTimeLimitConfig_SubIncremental`, `OptimizeSingleTaskTimeLimit(_Impl)`.
Only the helper extraction rename survived: `OptimizeOneTaskTimeLimit` (was
`WalkOneTaskTimeLimit_SubIncremental`). The P2.11 design docs (goal.md/tasks.md)
are written against the dropped rename table (`RunIntervalDescent`,
`_PAReopt`, `_SingleTaskPatch`, `WalkOneTaskTimeLimit_FullBeam`) — these symbols
do NOT exist. All implementation work uses the real names.

### What landed
The ONE genuine delta of reading (a): the reopt sub-incremental arm (P2.9
flag-on) now walks the SAME E+L serialized queue the incremental path uses, so
env-changed tasks with no perf pair (Type-E) are reached.

- **3a (plumb):** `ReOptimizePeriodic` captures `dag_tasks_prev_pre_tl` before
  absorbing `dag_tasks_update` (it previously did not — the Type-E diff source
  was missing on the reopt path) and passes it as a REQUIRED arg to
  `PerformCoordinateDescentForTaskConfigOpt` (no default, per the optional-arg
  coding rule — a forgotten default would silently skip Type-E).
- **3b (TDD):** `ReoptWalk_LeverA_On_ReachesEnvChangedTaskViaSerializedQueue`.
  Fixture `CounterDispatcherSynthetic` (pins `ReoptimizationPeriod=10`); env
  change = T_noise (task 1, no perf pair → `{-1}`-only) ET 50→1234. Asserts a
  flag-ON reopt drives a `SubIncremental` call with `task_idx==1`. RED first
  (`subincremental_task_idx: {0,0}` — T_noise skipped by `sorted_indices`),
  GREEN after the fix. Extended the `RecordingDispatcherOpt` seam to record
  `subincremental_task_idx`.
- **Implementation:** the flag-on arm of `PerformCoordinateDescentForTaskConfigOpt`
  now builds `serialized_queue = BuildSerializedTaskQueue(dag_tasks_prev_pre_tl)`
  and walks it with the SAME Type-E/Type-L dispatch as
  `PerformSerializedTaskQueueOptimization` (Type-E → `SubIncremental` at the
  committed TL; Type-L → `OptimizeOneTaskTimeLimit`). The legacy (flag-OFF) arm
  is unchanged — still `sorted_indices` + `OptimizeSingleTaskTimeLimit`.

### Scope / safety
- **Flag-gated, default OFF → prod path bit-identical.** Only the P2.9 flag-on
  path changed. This is the P2.11 merge's first concrete step, staged behind
  the existing flag so the default reopt algorithm is untouched until the A/B.
- **Cache contract holds:** the flag-on arm already re-arms `rta_cache_active_`
  + adopts the baseline champion (P2.9); the Type-E handler's `|diff|==0`
  (Phase 0d) and the Type-L `OptimizeOneTaskTimeLimit`'s `|diff|==1` are
  unchanged. `dag_tasks_prev_pre_tl` captured before the absorb (Phase 3a) is
  exactly the precondition Phase 0d named.
- 17/17 ctest green.

### NOT done (remaining P2.11 phases)
- Phase 0.5 (`_PAReopt` rename) — re-decide; code still has
  `EvaluateTimeLimitConfig_ScratchOrIncre`. NOT a blocker for the merge.
- Phase 1 (unified descent body `RunIntervalDescent(mode)`) — the dedup of the
  two descent bodies. The flag-on arm now shares the Type-E/Type-L dispatch
  with the incremental path (factored into a local lambda); a follow-up can
  extract that into the shared `WalkSerializedTaskQueue` helper (Phase 1/2).
- Phase 2 (delete legacy full-beam arm `OptimizeSingleTaskTimeLimit` +
  `ReoptimizationUseSubIncrementalWalk` flag) — only AFTER the A/B accepts the
  merge.
- Phase 4-6 (build+verify done; A/B experiment at N=16 `INCR_Reopt_1`/`_10`
  flag 0 vs 1; closeout).

### Status
Awaiting user review + commit of this flag-gated Type-E step as a standalone
modular commit. Per "work by module, commit by module."

## 2026-07-25 — Phase 1a landed (shared `WalkSerializedTaskQueue` helper)

### What landed
The per-entry Type-E/Type-L dispatch was duplicated byte-for-byte between the
two descent bodies:
- `PerformSerializedTaskQueueOptimization` (incremental) — inline loop;
- `PerformCoordinateDescentForTaskConfigOpt` flag-on arm — a local
  `walk_serialized_entry` lambda (added in Phase 3b).

Both now call ONE shared helper `WalkSerializedTaskQueue(queue, K, tl, sp,
patience)`, defined right after `BuildSerializedTaskQueue`. Each call site is
one line. The helper body is literally the inline loop (no logic change).

### Scope / safety
- **Behavior-neutral dedup.** No new logic, no flag change, no signature change
  to existing functions. The helper is a pure extraction of identical code.
- **Bit-identity gate:** flag OFF (default) → only the incremental path uses
  the helper in prod, and the extracted body is byte-identical to the inline
  loop it replaced → prod SP unchanged. `testIncreOpt_w_TL` pins both the
  incremental serialized walk AND the Phase 3b Type-E reopt reach test
  (flag-on path), so both call sites of the helper are exercised.
- 17/17 ctest green (`--clean-first`, header changed).

### NOT done (remaining Phase 1+)
- Phase 1b-1e: unify the two full descent bodies into one
  `RunIntervalDescent(K, tl, mode, dag_prev_pre_tl)` parameterized over
  `mode ∈ {Incremental, Reopt}`, then route both entries through it and delete
  the originals. The walk is now shared (1a); the remaining delta is the setup
  preamble (baseline seed: dedicated re-score vs upfront `ScratchOrIncre`
  re-opt; cache arming) + the legacy-arm fallback. This is the larger
  structural unification.
- Phase 0.5 (`_PAReopt` rename) — re-decide; NOT a blocker.
- Phase 2 (delete `OptimizeSingleTaskTimeLimit` + P2.9 flag) — post-A/B only.
- Phase 5 A/B at N=16 (`INCR_Reopt_1`/`_10`, flag 0 vs 1) — the gate.

### Status
Awaiting user review + commit of the Phase 1a dedup as a standalone modular
commit. Per "work by module, commit by module."

## 2026-07-25 — Phase 5 A/B CRASHED: reopt flag-on path throws at interval 0

### Symptom
The flag=1 run of the Phase 5 A/B (`p211_reopt_ab_config.json`, N=16, 10
tasksets) aborted at the FIRST arm of the FIRST taskset:
`taskset_0 / INCR_Reopt_1 instance 0` → `SIGABRT` (exit 134). P1.15 layer A
harness stopped the run, wrote `crash_report.txt` + `taskset_arm_status.csv`,
no summary/plots. `run.log` was empty (the abort message went to stderr which
the harness did not capture). Flag=0 (legacy reopt) ran fine on the same
tasksets — the crash is specific to the flag-on (P2.11 merge) path.

### Reproduction + root cause (gdb backtrace)
Reproduced directly: `build_test/tests/RunOrchestrator <taskset_0> <arm_dir>
INCR_Reopt_1 10000 1` → exit 134, "Aborted (core dumped)". Ran under `gdb
-batch` to get the backtrace. The crash is an **uncaught `std::runtime_error`**
(thrown → `std::terminate` → SIGABRT), NOT an assert. The throw site is
`RTACache::ComputeTaskSetDifference` (`RTA_Cache.cpp:358-362`), the cache's
own unconditional guard:
```
throw std::runtime_error(
    "RTACache::ComputeTaskSetDifference: candidate differs from champion "
    "by more than one task — violates the single-change invariant. ...");
```
The full call chain is exactly the P2.11 flag-on reopt path:
```
ReOptimizePeriodic (OptimizeSP_TL_Incre.cpp:797)
  → PerformCoordinateDescentForTaskConfigOpt (566, from_scratch=true)
    → WalkSerializedTaskQueue (359)
      → OptimizeOneTaskTimeLimit (498, task_idx=1, patience=1)
        → [eval lambda 494] → EvaluateTimeLimitConfig_SubIncremental (222, task_idx=1)
          → RTACache::Evaluate (RTA_Cache.cpp:465)
            → ClassifyReusePerTask (376)
              → ComputeTaskSetDifference (362) ← THROWS
```
The crash fires at **interval 0** (`SimulateInterval(interval_idx=0)`), on the
first backward TL step of the first walked task (task_idx=1).

### The bug: `starting_time_limits` not re-synced to the champion TL before the walk
Phase 0d's contract proof (Type-E = `|diff|==0`, Type-L = `|diff|==1`) assumed
the champion TL and the walk's `starting_time_limits` are the SAME vector at
walk entry. They are NOT, on the reopt flag-on arm:

1. `ReOptimizePeriodic:791-793` — at interval 0 (no incumbent), seeds
   `time_limits = InitializeTimeLimitsFromETConfig()` = the **Gaussian-mean**
   TL. Passes it to `PerformCoordinateDescentForTaskConfigOpt` as
   `starting_time_limits`.
2. `PerformCoordinateDescentForTaskConfigOpt:540` `ResetIncumbentBaseline(true)`
   → interval-0 branch → RM-Fast seed: champion TL = `SmallestTimeLimitVec()`
   (`tl_min`). `res_opt_` now holds `tl_min`.
3. `:541` `EvaluateTimeLimitConfig_ScratchOrIncre(K, starting_time_limits,
   from_scratch=true)` → runs `OptimizeFromScratch` against the Gaussian
   `starting_time_limits`, then `UpdateRecords` adopts the from-scratch
   reopt's result → `res_opt_` TL = the from-scratch reopt's TL (NOT Gaussian,
   NOT `tl_min`).
4. `:553` `champion_tl = ReconstructTimeLimitVecFromResOpt()` = the from-scratch
   reopt's TL.
5. `:556` `AdoptChampion(dag_tasks_, opt_pa_, champion_tl, ...)` — cache
   champion is now `{dag_tasks_, opt_pa_, champion_tl}`.
6. `:566-568` `WalkSerializedTaskQueue(queue, K, starting_time_limits, ...)` —
   walks `starting_time_limits` (STILL the Gaussian from step 1), NOT
   `champion_tl`.
7. First `EvaluateTimeLimitConfig_SubIncremental` call (task_idx=1, one TL step
   from Gaussian): rebuilds `dag_tasks_cur` from `dag_tasks_` + (Gaussian with
   task 1 stepped). The champion DAG was built from `dag_tasks_` + `champion_tl`
   (the from-scratch reopt's TL). **If `champion_tl` ≠ Gaussian in >1 task →
   `IsSingleTaskChange` returns false → `|diff|>1` → THROW.**

On a real N=16 taskset the from-scratch reopt moves multiple TLs off the
Gaussian seed → the divergence is >1 → throw. The Phase 3b TDD fixture
(`ReoptWalk_LeverA_On_ReachesEnvChangedTaskViaSerializedQueue`) did NOT catch
this because its synthetic fixture has the from-scratch reopt leave the TL
equal to the seed (or only 1 task diverges), so `|diff|<=1` held by accident.

### Contrast with the incremental path (why it doesn't throw)
`PerformSerializedTaskQueueOptimization` (incremental, :367-403) commits the
champion FROM the SAME `starting_time_limits` the walk uses (:390
`CommitIncumbent(opt_pa_, current_config_sp, starting_time_limits)`), so at
walk entry `champion_tl == starting_time_limits` → first step `|diff|==0`
(only the walked task differs) → safe. AND `OptimizeOneTaskTimeLimit:506`
re-syncs `starting_time_limits = ReconstructTimeLimitVecFromResOpt()` after
EACH task's walk, keeping the working TL tracking the committed best. The
reopt flag-on arm does NEITHER: it adopts the champion from the from-scratch
reopt's TL (:556) but walks the un-re-synced Gaussian `starting_time_limits`
(:566), and never re-syncs before the walk.

### The fix (one line, mirrors the incremental path)
Before the `WalkSerializedTaskQueue` call at :566, re-sync the working TL to
the champion TL:
```cpp
if (use_subincremental_walk) {
    // Re-sync the working TL to the champion TL just adopted above, so the
    // first walk step diffs champion-TL vs champion-TL-with-one-task-stepped
    // (|diff|==1), not champion-TL vs the Gaussian seed (|diff|>1 → throw).
    // Mirrors PerformSerializedTaskQueueOptimization:390 + OptimizeOneTaskTimeLimit:506.
    starting_time_limits = ReconstructTimeLimitVecFromResOpt();
    std::vector<SerializedTaskQueueEntry> serialized_queue =
        BuildSerializedTaskQueue(dag_tasks_prev_pre_tl);
    current_config_sp = WalkSerializedTaskQueue(
        serialized_queue, K, starting_time_limits, current_config_sp,
        patience);
}
```
This is flag-gated (default OFF) → prod path bit-identical; only the flag-on
path changes. The TDD regression test must drive a real multi-task TL
divergence between the from-scratch reopt and the Gaussian seed (the fixture
gap that let this ship).

### Status
**A/B BLOCKED until this is fixed.** Flag is still 1 in the working tree
(parameters.yaml:46) — the user's flag=0 baseline run completed fine; only the
flag=1 run crashed. NEXT: write the TDD regression test (RED), apply the
one-line fix (GREEN), rebuild + 17/17 ctest, then re-run the flag=1 arm.

## 2026-07-25 (2) — Fix APPLIED (TDD skipped per user); A/B flag=1 re-run launched

### Decision: skip TDD temporarily
User directive (to unblock): "let's speed up dev, and skip TDD temporarily."
The synthetic RED test (Phase 5.5a) was the slow part — constructing a fixture
where the from-scratch reopt provably diverges from the Gaussian seed in >1
task is fiddly (exactly why the Phase 3b fixture missed the crash: it diverged
by accident ≤1). The real `p211_reopt_ab_config.json` flag=1 run IS the
reproducer (it crashed at `taskset_0/INCR_Reopt_1` before the fix), so it is
also the gate: completion of that arm = GREEN. A unit-test pin can be revisited
later if regressions recur. **5.5a marked [~] (skipped, not done).**

### Fix applied (5.5b DONE)
One line at `sources/Optimization/OptimizeSP_TL_Incre.cpp`, inside the
`if (use_subincremental_walk)` block, immediately before the
`WalkSerializedTaskQueue` call:

```cpp
// Re-sync the walk vector to the adopted champion TL. The upfront
// from-scratch reopt (EvaluateTimeLimitConfig_ScratchOrIncre above) +
// AdoptChampion committed a TL that diverges from the incoming
// starting_time_limits (Gaussian seed) by >1 task on a real taskset.
// Without this re-sync the first walk step would diff champion-TL vs
// seed-TL in >1 task → RTACache::ComputeTaskSetDifference throws
// |diff|>1 → SIGABRT. Mirrors the incremental path, which establishes
// champion-TL == walk-start TL via CommitIncumbent. The walk then
// refines the champion, as intended. Flag-gated, default OFF.
starting_time_limits = ReconstructTimeLimitVecFromResOpt();
```

(Note: the dev_log entry above this one sketched the fix *before*
`BuildSerializedTaskQueue`; the applied fix places it *after* the queue build
and *before* `WalkSerializedTaskQueue` — same effect, slightly tighter scope.
`BuildSerializedTaskQueue` does not consume `starting_time_limits`, so the
placement difference is immaterial.)

### Rebuild + ctest (5.5c DONE)
`cmake --build build_test --target check.SP_OPT -j5 --clean-first` →
17/17 ctest green (incl. `testIncreOpt_w_TL`).

### A/B flag=1 re-run launched (5.5d IN PROGRESS)
YAML flag flipped `0 → 1` (`sources/parameters.yaml:46`). Launched:
```
RERUN_MODE=clear_results SKIP_EVAL=1 BIN_DIR=release \
  CONFIG_JSON=simulation_experiments/configs/p211_reopt_ab_config.json \
  bash scripts/run_simulation_plot_eval_ns.sh
```
*(NOTE: the original launch used `BIN_DIR=build_test` — the DEBUG build,
~8.5× slower, which inflated scheduler ET and seeded the "optimization got
much slower" investigation. Corrected to `release` 2026-07-25; all future
A/B runs use `BIN_DIR=release`. The historical command is preserved above
with the fix applied, since the original DEBUG numbers were a measurement
artifact, not a real regression.)*
`clear_results` reuses the identical seeded tasksets from the flag=0 baseline
run and re-simulates every arm with the flag on. `enable_resume_from_existing_
results: false` (per the config) so no arm is skipped. Monitor watching the run
log for the crash signature (`ComputeTaskSetDifference` / `|diff|>1` / SIGABRT
/ `CalledProcessError`) vs progress markers (`taskset_N/INCR_Reopt_X`,
`comparison_summary`, `EXIT_CODE`). The crash historically fired at the FIRST
arm of the FIRST taskset, so passing `taskset_0/INCR_Reopt_1` instance 0 =
fix confirmed.

### NEXT
On run completion: diff the two saved `comparison_summary.csv` (flag=0 baseline
saved earlier vs flag=1 just-run) — `Mean_SP_Metric` + `Mean_Scheduler_
Execution_Time_s` per arm. Then Phase 5b (compare vs pure incremental) + 5c
(accept/revert verdict). YAML flag MUST be flipped back to `0` on exit
regardless of outcome (prod-safe default).

---

## 2026-07-25 — Phase 5 A/B flag=1 arm: SLOWER, not faster + TIME_LIMIT escape

### Symptom (user report, manual timer)
Flag=1 was meant to **speed up** reoptimization (route the reopt TL walk through
the cheap cache-routed sub-incremental eval instead of a full
`OptimizeFromScratch` beam per candidate). It does the OPPOSITE: **much slower**
for BOTH `INCR_Reopt_1` and `INCR_Reopt_10`. User: "there must be something
wrong."

### The sharper clue — TIME_LIMIT budget escape
User did the budget math: `simulation_duration_seconds=300` /
`scheduler_trigger_interval_seconds=10` = 30 intervals; `TIME_LIMIT=1s` per
interval → **max ~30s of optimization per taskset**. But a taskset ran **5 min
and wasn't done**. So `TIME_LIMIT=1` is **NOT effective** under flag=1 — the
cooperative per-interval budget is being escaped ~10×. This is THE bug, not
merely "cache path slower than beam."

### Hypothesis (root cause, from code reading — NOT yet confirmed/fixed)
The flag=1 reopt walk's hot loops contain **NO `BFSharedBudgetCancelled()` poll**:
- `WalkSerializedTaskQueue` (`OptimizeSP_TL_Incre.cpp:341-365`) — the per-entry
  dispatch loop, no poll.
- `OptimizeSingleTaskTimeLimit_Impl` (`:439-481`) — the TL walk loop, no poll.
- `OptimizeIncre_SingleTask` (`OptimizeSP_Incre.cpp:310-358`) — the O(N) PA
  re-search loop over `pa_vec_variations`, no poll.

The ONLY budget check in the flag=1 path is a single coarse entry-poll in
`EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp:218`), which
fires once per eval lambda. But each eval then calls
`OptimizeIncre_SingleTask`, which runs O(N) PA-variation `RTACache::Evaluate`
calls **without re-checking** the cancel. So once the budget expires mid-walk,
the cooperative `BFDLSharedBudget` cancel cannot interrupt the per-interval
work — it runs to completion of the full queue × TL options × PA variations.

### Compounding per-eval overhead (the "slower" part)
Even WITHIN budget, flag=1 does heavy work per candidate TL:
- `RTACache cache_backup = rta_cache_;` (`:185`) — **full cache copy per eval**
  (P1.25 restored the eager save/restore; P1.21's lazy CoW Transaction was
  removed in P1.25). This is O(cache) × every TL candidate × every task × 2
  passes.
- Double DAG rebuild per eval: `UpdateExtDistBasedOnTimeLimit` in the eval
  (`:187`) AND again inside `BuildChallengerFromIncumbent` (`:731-740`,
  `tl_prev = ReconstructTimeLimitVecFromResOpt()` →
  `dag_with_tl_prev = UpdateExtDistBasedOnTimeLimit(...)`).
- O(N) PA-variation `Evaluate`s per TL option via `OptimizeIncre_SingleTask`
  (`FindPriorityVec1D_Variations` up to N positions).

### Why the legacy (flag=0) path stays bounded
`OptimizeFromScratch` (`OptimizeSP_Incre.cpp:100-164`) is K-width-limited (beam
width K, not N) and ends with `opt_sp_ = EvaluateSPWithPriorityVec(...)`, which
respects the cancel → INT_MIN. So the per-interval work is bounded by both the
beam width AND the cooperative cancel. Flag=1 replaced this bounded beam with
an unbounded walk (queue × TL options × N PA variations) whose only cancel
poll is too coarse to fire.

### CONFIRMED 2026-07-25 — root cause: the flag=1 path's only cancel poll is
bypassed; `OptimizeIncre_SingleTask` runs unconditionally after it.

**The mechanism (all confirmed by reading the code):**
- `BFDLSharedBudget shared_budget(now)` is set up ONCE per interval at
  `OptimizeSP_TL_Incre.cpp:617` (`Optimize_w_TL_ScratchOrIncre`), wrapping BOTH
  `ReOptimizePeriodic` (`:626`) and `OptimizeIncre_w_TL` (`:628`). So the budget
  IS active during the reopt walk. `BFSharedBudgetCancelled()` returns true once
  `now - start >= TIME_LIMIT*1000ms` (`OptimizeSP_Base.cpp:38-49`). Cooperative —
  only effective where polled.
- Polls that EXIST: `RTA.cpp:102` (per-task loop in `ProbabilisticRTA_TaskSet`),
  `SP_Metric.cpp:63/107/122` (`ObtainSP_TaskSet`/`ObtainSP_DAG` — the
  `EvaluateSPWithPriorityVec` path), `OptimizeSP_Base.cpp:192/206` (BF beam),
  `OptimizeSP_Incre.cpp:71` (per beam node), `OptimizeSP_TL_Incre.cpp:218`
  (single entry poll in `EvaluateTimeLimitConfig_SubIncremental`).
- Polls ABSENT in the flag=1 hot loops: `WalkSerializedTaskQueue` (`:349`, no
  poll between queue entries), `OptimizeSingleTaskTimeLimit_Impl` (`:461`, no
  poll between TL options), `OptimizeIncre_SingleTask` (`OptimizeSP_Incre.cpp:330`,
  no poll between PA variations). AND the cache path's own scoring has none:
  `RTACache::Evaluate`'s recompute loop (`RTA_Cache.cpp:498-518`) calls
  `GetRTA_OneTask` directly (NOT through `RTA.cpp:102`'s poll), the FullReuse
  `|diff|==0` path returns at `:492` with no recompute at all, and
  `ObtainSP_Full_From_NodeRTAs` → `ObtainSP_DAG_From_Dists` (`SP_Metric.cpp:151-172`)
  is two plain loops with NO poll (incl. the expensive `GetRTDA_Dist_AllChains`
  chain convolution).

**The decisive defect** — `EvaluateTimeLimitConfig_SubIncremental` (`:218-230`):
```cpp
if (BFSharedBudgetCancelled()) {
    challenger.opt_sp_ = INT_MIN;          // baseline Evaluate skipped
} else {
    baseline_rtas = rta_cache_.Evaluate(...);
    challenger.opt_sp_ = ObtainSP_Full_From_NodeRTAs(...);
}
challenger.OptimizeIncre_SingleTask(...);  // :228 — runs UNCONDITIONALLY
```
The `:218` poll guards ONLY the baseline `Evaluate`+`ObtainSP_Full`.
`OptimizeIncre_SingleTask` at `:228` runs regardless of cancel — and it does
O(N) PA-variation `Evaluate` + `ObtainSP_Full_From_NodeRTAs` calls, none polled.
So:
1. Once the 1s budget expires, every subsequent eval skips the (cheap) baseline
   but STILL runs the full N-variation PA re-search (expensive, unpollled). The
   cancel is almost completely ineffective.
2. Even mid-budget, if the budget expires DURING one `OptimizeIncre_SingleTask`
   call, its remaining variations all complete — no poll inside the loop.

**Why flag=0 stays bounded but flag=1 blows the budget:**
- flag=0 reopt intervals: `OptimizeSingleTaskTimeLimit` over `sorted_indices` →
  `EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=true)` → `OptimizeFromScratch`
  (K-beam). The K-beam polls per node (`OptimizeSP_Incre.cpp:71`) AND its
  `EvaluateSPWithPriorityVec`→`ObtainSP_DAG` polls (`SP_Metric.cpp:107/122`).
  Bounded by the cooperative cancel.
- flag=1 reopt intervals: the K-beam runs ONCE upfront (`:541`, bounded), then
  `WalkSerializedTaskQueue` runs the E+L queue with the unpollled
  `OptimizeIncre_SingleTask` per candidate. The upfront beam can consume most of
  the 1s; the walk then runs the rest UNBOUNDED.
- INCR_Reopt_1 (period=1) = every one of 30 intervals is an unbounded reopt walk
  → maximal escape → ~5 min. INCR_Reopt_10 (period=10) = 3 reopt intervals →
  less but still slower than flag=0. Matches the user's manual timer.

**NOTE on the shared walk + bit-identity:** `WalkSerializedTaskQueue`,
`OptimizeSingleTaskTimeLimit_Impl`, and `OptimizeIncre_SingleTask` are SHARED
with the incremental path (`PerformSerializedTaskQueueOptimization`, flag=0 prod).
But the incremental walk is warm-started + patience=0 (strict, ~unimodal → stops
fast) and finishes WITHIN the 1s budget, so a cancel poll there is inert for prod
→ bit-identical. The poll only fires for the over-budget reopt walk (flag=1).

### Fix (proposed, NOT yet applied)
1. **`EvaluateTimeLimitConfig_SubIncremental:228`** — SKIP
   `OptimizeIncre_SingleTask` when the `:218` entry poll fired (return early /
   guard the call). Highest leverage: stops the bulk waste once cancelled.
2. **`OptimizeIncre_SingleTask` PA loop (`OptimizeSP_Incre.cpp:330`)** —
   `if (BFSharedBudgetCancelled()) break;` per variation. Bounds the in-budget
   mid-loop expiry.
3. (defense-in-depth) `WalkSerializedTaskQueue:349` + `OptimizeSingleTaskTimeLimit_Impl:461`
   — break on cancel.
All three are inert outside a `BFDLSharedBudget` scope and inert for the within-
budget incremental path → prod bit-identical. Validate: one taskset /
`INCR_Reopt_1` / flag=1 that ran 5 min → ~30s = fixed. No A/B, no TDD arc.

## 2026-07-25 — SEPARATE ISSUE: flag=0 (prod) per-interval opt also slow at N=16

> Reported by the user AFTER the flag=1 diagnosis. This is a DISTINCT issue from
> the flag=1 TIME_LIMIT escape above — it affects the DEFAULT (flag=0) path, i.e.
> prod. Recorded before applying the flag=1 fix so the two are not conflated.

**Symptom (user):**
- Re-ran simulation at N=16 with `ReoptimizationUseSubIncrementalWalk=0` (the
  prod default). It is ALSO very slow.
- The user has previously run N=16 cases that finished per-interval opt in
  **under 0.1s**. Now the same kind of run takes **~0.5s** per interval — a
  ~5× regression with NO config/flag change in the prod path.
- The user does NOT know which code commit broke it.

**What this is NOT:**
- NOT the flag=1 TIME_LIMIT escape (that needs flag=1; this is flag=0).
- NOT a TIME_LIMIT escape per se — 0.5s is still well under the 1s budget, so
  the cooperative cancel is not the lever here. It's a raw per-eval cost
  regression in the flag=0 prod path.

**Flag=0 prod path (what runs at N=16, flag=0):**
- Incremental intervals: `OptimizeIncre_w_TL` → `PerformSerializedTaskQueueOptimization`
  → `WalkSerializedTaskQueue` → `OptimizeOneTaskTimeLimit` →
  `EvaluateTimeLimitConfig_SubIncremental` (per TL step) → `OptimizeIncre_SingleTask`
  (per eval). This is the SHARED incremental walk.
- Reopt intervals (period=10): `ReOptimizePeriodic` →
  `PerformCoordinateDescentForTaskConfigOpt` (flag-off arm = `:580-597`) →
  `OptimizeSingleTaskTimeLimit` over `sorted_indices` →
  `EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=true)` → `OptimizeFromScratch`
  (K-beam). The legacy full-beam path.

**Suspects (to bisect, NOT yet investigated):**
1. **P1.25's eager cache save/restore** (`RTACache cache_backup = rta_cache_;` at
   `EvaluateTimeLimitConfig_SubIncremental:185`, restored on reject at `:234`).
   A FULL `RTACache` copy per eval. If the cache grew (more entries / larger RTA
   vectors) or the per-eval call count rose, this copy is now costlier. P1.25
   replaced the lazy CoW Transaction with this eager copy.
2. **Per-eval DAG rebuild** (`UpdateExtDistBasedOnTimeLimit` at `:191`) +
   `BuildChallengerFromIncumbent` (`:203`, rebuilds the DAG again inside). Two
   DAG builds per eval. If DAG build cost rose (e.g. a structural change in
   `DAG_Model` / `UpdateExtDistBasedOnTimeLimit`), this compounds per eval.
3. **P1.18 Rule B `ClassifyReusePerTask`** — the window [p_min,p_max] NoReuse,
   top+bottom FullReuse classification. If the classification became more
   conservative (more NoReuse → more recompute in `RTACache::Evaluate`'s
   `:498-518` loop calling `GetRTA_OneTask`), each eval does more RTA work.
4. **P1.20 processor-partition vectorization** — flat `vector<T>` indexed by
   `processorId`. Should be FASTER, not slower — but a regression in the
   indexing or a missed `-1`→`0` normalization could add per-task overhead.
5. **A recent commit on `clean_simulation` branch** (HEAD `52e29e90` "add
   WalkSerializedTaskQueue" / `d132a40d` "change PerformCoordinateDescent... to
   be more similar to incremental opt"). The P2.11 work touched the shared walk
   — even though flag=0 should be bit-identical, the Phase 1a extraction
   (`WalkSerializedTaskQueue`) or Phase 3a (`dag_tasks_prev_pre_tl` capture)
   could have added per-call overhead (e.g. an extra DAG copy for the diff).

**Investigation plan (after the flag=1 fix lands):**
- **Bisect** is the cleanest tool: `git bisect` from a known-fast commit (the
  user's "under 0.1s" baseline) to HEAD, running one N=16 flag=0 interval and
  timing it. The user has not pinned the fast commit, so step 1 = identify a
  known-fast ref (likely pre-P2.9/P2.10/P2.11, i.e. before `5dfd146e`).
- **Profile** the per-interval cost: add a scoped timer around
  `PerformSerializedTaskQueueOptimization` + around
  `EvaluateTimeLimitConfig_SubIncremental` (eval count × per-eval time) to see
  whether the regression is MORE evals or SLOWER per-eval.
- Do NOT conflate with the flag=1 fix — apply the flag=1 TIME_LIMIT fix first
  (it's the recorded, ready fix), THEN bisect the flag=0 regression separately.

## 2026-07-25 (3) — 5.5 + 5.6 fix COMMITTED (`a6922ff5`); TDD pin landed too

### What shipped
Both the 5.5 interval-0 crash fix AND the 5.6 TIME_LIMIT budget-escape fix
landed in one commit on `clean_simulation`:
- **commit `a6922ff5`** — "fix flag=1 reopt arm: walk-start TL re-sync +
  TIME_LIMIT budget polls", 3 files changed, 198 insertions(+), 3 deletions(-).
- Files: `sources/Optimization/OptimizeSP_TL_Incre.cpp`,
  `sources/Optimization/OptimizeSP_Incre.cpp`, `tests/testIncreOpt_w_TL.cpp`.

The two defects are interleaved in the same file region
(`PerformCoordinateDescentForTaskConfigOpt`'s `if (use_subincremental_walk)`
block + the shared eval/walk helpers it calls), so they could not be cleanly
split without interactive staging (unavailable) → one commit covering both,
with a message that names both 5.5 and 5.6 explicitly.

### TDD pin landed despite the "skip TDD" decision
The 2026-07-25 (2) entry recorded "skip TDD temporarily" for 5.5a. In the
event, a regression test WAS written and committed — `ReoptFlagOnMultiTLFlexibleSynthetic`
fixture + `ReoptFlagOn_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs` —
because the real `p211_reopt_ab_config.json` run is too coarse a gate to keep
around as a regression pin. It is a **no-regression guard, not a strict
RED-first TDD arc** (cf. the P1.25 precedent): it pins (a) `EXPECT_NO_THROW` on
a flag-on `ReOptimizePeriodic` whose from-scratch reopt provably diverges from
the Gaussian seed in ≥2 TLs (the fixture gap that let the crash ship — the
Phase 3b single-TL-flexible fixture diverged by accident ≤1), (b) `EXPECT_GE
(divergent, 2)` that the fixture reproduces the divergence, (c) `EXPECT_GT
(subincremental_calls, 0)` that the walk is reached. 17/17 ctest green.

**Cleanup needed:** the test still carries a TEMP DIAGNOSTIC `std::cerr` block
(`[DIAG] gaussian_seed = ... / champion_tl = ... / divergent = ... /
subincremental_calls = ...`) left over from the divergence investigation.
Remove it before the next commit that touches this test — it spams ctest
output. NOT a correctness issue.

### The 4 budget polls (5.6b) — all applied + committed
1. `EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp`, ~`:228`)
   — `OptimizeIncre_SingleTask` call now guarded by `if (!BFSharedBudgetCancelled())`
   (highest leverage: stops the bulk waste once cancelled). `[P2.11 5.6b]` tag.
2. `OptimizeIncre_SingleTask` PA loop (`OptimizeSP_Incre.cpp`, ~`:328`) —
   `if (BFSharedBudgetCancelled()) { break; }` per PA variation.
3. `WalkSerializedTaskQueue` (`OptimizeSP_TL_Incre.cpp`, ~`:349`) —
   `if (BFSharedBudgetCancelled()) { break; }` at top of the per-entry loop.
4. `OptimizeSingleTaskTimeLimit_Impl` (`OptimizeSP_TL_Incre.cpp`, ~`:461`) —
   `if (BFSharedBudgetCancelled()) { break; }` at top of the TL-option loop.
All 4 are inert outside a `BFDLSharedBudget` scope AND inert for the within-
budget incremental path (finishes inside 1s) → prod bit-identical (flag OFF).

### The 5.5 re-sync (5.5b) — applied + committed
`starting_time_limits = ReconstructTimeLimitVecFromResOpt();` inside the
`if (use_subincremental_walk)` block, before `WalkSerializedTaskQueue`. Same
one-line fix sketched in the 2026-07-25 crash entry; flag-gated default OFF.

### What did NOT ship in this commit (still in working tree, unstaged)
- **`sources/parameters.yaml`** — `TIME_LIMIT: 10 → 1` only. Treated as a
  separate config-value concern (the budget the polls enforce); HEAD still has
  `TIME_LIMIT: 10`. Working tree has `1`. The YAML flag
  `ReoptimizationUseSubIncrementalWalk` is at `0` (prod-safe default — verified).
- **`agents/active_tasks/P2_11_merge_reopt_into_incremental/{dev_log,tasks}.md`**
  — these records (this entry + the task checkbox updates). Excluded from the
  fix commit so the commit is purely code+test.
- Untracked: `simulation_experiments/configs/p211_reopt_ab_config.json`,
  `agents/active_tasks/P2_9_speed_up_reoptimization/`,
  `Gen_Taskset/task_sets_config/taskset_cfg_paper_16.json`, the `.parameters.yaml.*.bak`
  editor backups.

### NEXT (unchanged from the standing plan)
- **5.6c validate** — single fast reproducer: one taskset / `INCR_Reopt_1` /
  flag=1 that previously ran 5 min → ~30s = budget respected = fixed. Transient
  flag flip `0→1`, run, RESTORE to `0`. No A/B, no TDD arc.
- **5.7** (DEFERRED until after 5.6c) — bisect the flag=0 prod per-interval
  ~5× slowdown (5.7a known-fast ref pre-`5dfd146e`; 5.7b bisect; 5.7c profile
  MORE-evals vs SLOWER-per-eval; 5.7d fix+validate ~0.1s).
- **Phase 5 A/B** (deferred): flag=0 vs flag=1 at N=16, `INCR_Reopt_1`/`_10`.
- **Phase 1b-1e**: unify the two descent bodies into one
  `RunIntervalDescent(K, tl, mode, dag_prev_pre_tl)`.
- **Phase 2** (post-A/B): delete `OptimizeSingleTaskTimeLimit` + the P2.9 flag.
- **Phase 6**: closeout (overall_tasks.md, memory pointer, P2.9+P2.10 records).

---

## 2026-07-26 — Phase 2 landed (working tree, uncommitted); Phase 4 verified GREEN

### Where we are
HEAD `2250ebfd`. The Phase 2 behavior change (delete the legacy reopt full-beam
arm + the P2.9 flag → merged reopt is the unconditional path) is **landed in the
working tree but NOT committed**. Phase 4 (build + 17/17 ctest) just verified
GREEN. This is the gate before handing to the user for review + commit.

### Phase 2 summary (what's in the working tree, 8 files)
- **2a — delete `OptimizeSingleTaskTimeLimit` (the 7-arg reopt-only wrapper).**
  The 7 `TrialAndErrorTLWalkSynthetic` walk-core tests now call
  `OptimizeSingleTaskTimeLimit_Impl` directly with an eval lambda built by a new
  `StubTLWalkOptimizer::MakeScratchOrIncreEval(K, from_scratch)` helper
  (eliminates 7× lambda repetition; behavior-preserving — the stub's
  `EvaluateTimeLimitConfig_ScratchOrIncre` override ignores `from_scratch`, so
  `evaluated_tls` recording + all assertions unchanged). The `_Impl` seam is the
  documented unit-test surface (header: "The walk core is unit-tested directly
  with an injected TL→SP stub").
- **2b — delete the `ReoptimizationUseSubIncrementalWalk` flag** from
  `Parameters.{h,cpp}` + `parameters.yaml`. Rewrote the 2 P2.9 flag-dispatch
  tests: deleted `ReoptWalk_Legacy_Off_RoutesTrialsThroughScratchOrIncre`
  (premise flag=0⇒legacy gone); renamed `ReoptWalk_LeverA_On_*` →
  `ReoptWalk_Routes*` / `ReoptWalk_ReachesEnvChangedTaskViaSerializedQueue`
  (drop the flag set, keep `subincremental_calls>0` now unconditional);
  `ReoptFlagOn_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs` →
  `Reopt_AtInterval0_DoesNotThrowWhenReoptMovesMultipleTLs`. Stripped the
  `saved_subincremental_walk_` save/restore from both fixtures.
  `grep -rn ReoptimizationUseSubIncrementalWalk sources/ tests/` → empty.
- **2c — comments cleaned.** Removed "flag-on arm" / "lever-A" / "sub-incremental
  walk flag" phrasing in `OptimizeSP_TL_Incre.{h,cpp}` (5 sites) + the test-file
  `subincremental_calls` field comment + the `ReoptFlagOnMultiTLFlexibleSynthetic`
  fixture class comment. The `.cpp` `PerformCoordinateDescentForTaskConfigOpt`
  body is now ONE unconditional path (no `if (use_subincremental_walk)` branch):
  reset → baseline beam → arm cache → re-sync TL → `WalkSerializedTaskQueue` →
  disarm. The legacy `sorted_indices` + `OptimizeSingleTaskTimeLimit` arm is
  deleted. The config `p211_reopt_ab_config.json` `_comment` now leads with
  "A/B PASSED; flag DELETED in Phase 2; procedure NO LONGER RE-RUNNABLE".
- **`TIME_LIMIT` in `parameters.yaml`** stays at `10` (the global default); the
  A/B used `time_limit_seconds: 1` via config override, which is a separate
  config-value concern, not part of this commit's core purpose.

### Phase 4 — build + verify (the gate)
- `cmake --build build_test --target check.SP_OPT -j5 --clean-first` →
  **17/17 ctest passed in 17.38s**, incl. `testIncreOpt_w_TL` (1.66s). Used
  `--clean-first` (Phase 2 deleted a wrapper + changed the `.h`; stale-`.o`
  lesson from P1.21).
- All 17 test binaries green. No new failures, no regressions vs HEAD.

### Scope of the proposed commit (Phase 2 + Phase 4 records)
- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` — the unconditional merged
  reopt path + comment cleanup.
- `sources/Utils/Parameters.{h,cpp}` + `sources/parameters.yaml` — flag deletion.
- `tests/testIncreOpt_w_TL.cpp` — test rewrites + `MakeScratchOrIncreEval`.
- `simulation_experiments/configs/p211_reopt_ab_config.json` — `_comment` update.
- `agents/active_tasks/P2_11_merge_reopt_into_incremental/{tasks,dev_log}.md` —
  Phase 2 + 4 record (this entry + the checkbox updates).
Excluded (unrelated / untracked): `Gen_Taskset/task_sets_config/taskset_cfg_paper_16.json`,
`agents/active_tasks/P2_13_important_task_ddl_vs_sp_metric/`, `../_perf_old_ecbed896`.

### NOT done (remaining P2.11)
- **Phase 1b-1e** — unify the two full descent bodies
  (`PerformSerializedTaskQueueOptimization` + `PerformCoordinateDescentForTaskConfigOpt`)
  into one `RunIntervalDescent(K, tl, mode, dag_prev_pre_tl)` parameterized over
  `mode ∈ {Incremental, Reopt}`, then route both entries through it + delete the
  originals. The walk core is already shared (`WalkSerializedTaskQueue`); the
  remaining delta is the setup preamble (baseline seed: dedicated re-score vs
  upfront `ScratchOrIncre` re-opt) + the patience knob. **Structural-only,
  behavior-neutral if done right** — the two bodies now differ only in the
  preamble + patience, both already factored. Can be a follow-up commit; NOT a
  blocker for landing Phase 2 (the merge is functionally complete).
- **Phase 5b** — compare vs pure incremental (`INCR_Reopt_∞`/no reopt). DEFERRED
  (not the gate; 5a was the gate).
- **Phase 6** — closeout: `overall_tasks.md`, memory pointer, P2.9+P2.10 records.

### Status
Awaiting user review + commit of Phase 2 (+ Phase 4 record) as a standalone
modular commit. Per agent_coding_rules.md "work by module, commit by module."
The merged reopt (sub-incremental walk + Type-E in the serialized queue,
patience+1) becomes the unconditional reopt path; the legacy full-beam arm +
the P2.9 flag are gone.
