# P1.12 — Integrate RTA Cache into the Incremental Optimizer — Dev Log

> Detailed working log for this task. Append chronological entries below.
> Split out of P1.11 on 2026-07-18 (P1.11's scope narrowed to cache design +
> build, Phase 0 DONE).

---

## 2026-07-18 — Task created (split from P1.11)

- User decided to split the cache-integration work into its own task folder
  (`P1_12_integrate_rta_cache`) rather than keep it as P1.11 Phase 1+2. P1.11's
  scope is now the cache design + build (Phase 0, DONE, working tree
  uncommitted); P1.12 owns the wiring of that cache into the live incremental
  optimizer eval path (P1.11's former Phase 1 + Phase 2).
- Created `goal.md` / `tasks.md` / `dev_log.md`. Scope, hazards (A: `RTACache&`
  by reference; B: `perf_coefficient`), and the grounded integration points
  (`CommitIncumbent` @ `OptimizeSP_TL_Incre.cpp:699`,
  `BuildChallengerFromIncumbent` @ `:716`,
  `EvaluateTimeLimitConfig_SubIncremental` @ `:178`, `ResetIncumbentBaseline`,
  `ReconstructTimeLimitVecFromResOpt` @ `:657`, `EvaluateSPWithPriorityVec` as
  the differential oracle) carried over from P1.11's records.
- **No code changes yet.** Working tree unchanged from P1.11 Phase 0 closeout
  (16/16 ctest + 46/46 testRTA green, uncommitted).

### Next: Phase 1 — infrastructure wiring (first small sub-task)

The first bounded sub-task is the `RTACache rta_cache_` member + the
`RTACache&` threading seam (resolves Hazard A), starting with a read of
`OptimizeSP_TL_Incre.h` / `OptimizeSP_TL_Incre.cpp` / `OptimizeSP_Incre.cpp` to
locate `OptimizePA_Incre_with_TimeLimits`, `ResetIncumbentBaseline`,
`CommitIncumbent`, `BuildChallengerFromIncumbent`,
`EvaluateTimeLimitConfig_SubIncremental`, `OptimizeIncre_SingleTask`, and
`EvaluateSPWithPriorityVec`. Awaits explicit user go before code changes.

## 2026-07-18 — Phase 1 scaffolding increment (1/N): `rta_cache_` member

User gave explicit go ("start p1_12"). First bounded increment = the
behavior-preserving scaffolding: declare the cache where it'll live, confirm
the include + construction compile, leave the eval path on the oracle. This
de-risks the header-cycle/include question (RTA_Cache.h takes `PriorityVec` →
`OptimizeSP_Base.h` → `SP_Metric.h` → `RTA.h`; `OptimizeSP_TL_Incre.h` already
transitively includes all of these via `OptimizeSP_Incre.h`, so adding
`RTA_Cache.h` is a true leaf add — no new cycle) and sets up the next
(meatier) increment: the read-side swap + `AdoptChampion` wiring.

**Verification before edit:** cheap grep confirmed all grounded line numbers
are stable (`CommitIncumbent` @ `OptimizeSP_TL_Incre.cpp:699`,
`BuildChallengerFromIncumbent` @ `:716`, `EvaluateTimeLimitConfig_SubIncremental`
@ `:178`, `ReconstructTimeLimitVecFromResOpt` @ `:657`).
**Path correction:** the optimizer sources live under `sources/Optimization/`
(`OptimizeSP_TL_Incre.{h,cpp}`, `OptimizeSP_Incre.{h,cpp}`), NOT
`sources/Safety_Performance_Metric/` as the grounded-integration-points list
had recorded. `RTA_Cache.h` is in the latter. Recorded in `tasks.md`.

**Edits (header only, `sources/Optimization/OptimizeSP_TL_Incre.h`):**
1. Added `#include "sources/Safety_Performance_Metric/RTA_Cache.h"` (after
   `OptimizeSP_TL_BF.h`, before `SP_Metric.h` — alphabetical w/in the
   `sources/...` group).
2. Added `RTACache rta_cache_;` member to `OptimizePA_Incre_with_TimeLimits`
   right after `res_opt_` (the champion store it mirrors), with a doc comment
   marking it Phase-1-scaffolding (declared + constructed, NOT yet read by the
   live eval path; the oracle `EvaluateSPWithPriorityVec` still answers all SP
   evals; the read-side swap + `AdoptChampion` call land in the next increment
   so this change stays behavior-preserving).

**Build/test:** `cmake --build build --target check.SP_OPT -j5` →
**16/16 ctest green** (incl. testIncreOpt_w_TL, testOptimizeIncrePA, testRTA).
No source behavior changed (member is dead state until the read-side swap).

### Next: Phase 1 increment 2/N — the read side + commit point

The next bounded increment wires the WRITE side first (cheapest, fully
deterministic): call `rta_cache_.AdoptChampion(...)` inside `CommitIncumbent`
(`OptimizeSP_TL_Incre.cpp:699`) + reset in `ResetIncumbentBaseline`. Still
behavior-preserving (cache is written but not yet read → no SP output moves).
Then increment 3/N = the READ side (replace `EvaluateSPWithPriorityVec` with
`rta_cache_.Evaluate(...)` in `OptimizeIncre_SingleTask` + the named
SP-assembly helper w/ `perf_coefficient`, Hazard B) — that's the one gated by
differential-TDD bit-identity. The `RTACache&` threading seam (Hazard A) is
deferred until the read side needs it (the member already lives on
`OptimizePA_Incre_with_TimeLimits`; threading is only needed if
`OptimizeIncre_SingleTask` on the *base* `OptimizePA_Incre` must reach it — to
be confirmed when reading `OptimizeSP_Incre.cpp:275-355`).

## 2026-07-18 — Increment 2a: WRITE-side (behavior-preserving)

User gave explicit go ("implement"). Per the finalized `implementation_plan.md`,
this is the 2a increment of the staged landing: write the cache at the commit
point + reset at the baseline point, gate it with a flag so the reopt path
(which shares `CommitIncumbent`) neither throws nor regresses. Cache is NOT yet
read — oracle `EvaluateSPWithPriorityVec` still live → behavior-preserving.

**Grounding before edit (RTA_Cache.h frozen contract re-verified):**
`AdoptChampion(dag, pa, tl, rtas)` + `Evaluate(dag, pa, tl)` take NO
`sp_parameters`; callers pass RAW `dag_tasks_` + `tl` (cache bakes TLs internally
via `ApplyTimeLimitsToTasksExecutionTime`). `Evaluate` does NOT advance the
champion (only `AdoptChampion`/`Initialize` do) → if the champion isn't
advanced to `res_opt_` at each adoption it stays frozen at an earlier triple →
later serialized eval drifts to |diff|>1 → `ComputeTaskSetDifference` throws
(called unguarded by `ClassifyReusePerTask` at RTA_Cache.cpp:383). So
`AdoptChampion` at `CommitIncumbent` is REQUIRED, coupling write + read sides.

**Call-site grounding (OptimizeSP_TL_Incre.cpp):**
- `PerformSerializedTaskQueueOptimization` (:381) is reached ONLY from
  `OptimizeIncre_w_TL` (:656, the incremental path) — confirmed via grep; the
  reopt/legacy-descent paths never reach it.
- The serialized body calls `ResetIncumbentBaseline(false)` (:395), then
  `CommitIncumbent(opt_pa_, current_config_sp, starting_time_limits)` (:415,
  baseline re-score), then per-step `UpdateRecords`→`CommitIncumbent` (:131,
  gated by `should_update`). The Type-L walk binds `eval` to
  `EvaluateTimeLimitConfig_SubIncremental` (:456, NOT `ScratchOrIncre`).
- `CommitIncumbent` (:706) writes `res_opt_` + `opt_pa_`/`opt_sp_` mirrors; the
  `dag_with_tl` is NOT stored (champion rebuilt from `res_opt_.id2time_limit`
  next interval). So passing RAW `dag_tasks_` + committed `tl` to the cache is
  signature-correct.

**Edits:**
1. `OptimizeSP_TL_Incre.h` — added `bool rta_cache_active_ = false;` right after
   `rta_cache_`, with a doc comment (true only in `PerformSerializedTaskQueueOptimization`;
   cleared in `ResetIncumbentBaseline`; gates `CommitIncumbent`'s cache write so
   the reopt path neither throws nor regresses). Refreshed the `rta_cache_` doc
   to reflect 2a (written at `CommitIncumbent` gated by the flag, reset at
   `ResetIncumbentBaseline`, NOT yet read).
2. `OptimizeSP_TL_Incre.cpp::ResetIncumbentBaseline` (:738) — at the TOP of the
   body (before the `if (from_scratch)` branch): `rta_cache_ = RTACache();` +
   `rta_cache_active_ = false;`. Unconditional so both branches + the `else`
   fall-through all start cold; rationale documented (new interval → new env →
   `dag_tasks_` re-seeded → stale cache-champion would make the first serialized
   eval diff >1 → throw).
3. `OptimizeSP_TL_Incre.cpp::PerformSerializedTaskQueueOptimization` (:395) —
   set `rta_cache_active_ = true;` AFTER the `ResetIncumbentBaseline` call (the
   reset clears it, this re-arms it for the walk body).
4. `OptimizeSP_TL_Incre.cpp::CommitIncumbent` (:706) — after the existing
   `res_opt_` writes, `if (rta_cache_active_)`: `Evaluate(dag_tasks_, pa, tl)`
   → rtas then `AdoptChampion(dag_tasks_, pa, tl, rtas)`. The triple being
   committed == the candidate the adopting eval just scored → `Evaluate`
   short-circuits to `FullReuse` and returns the cached `candidate_rta_`
   (near-zero cost, no extra RTA, no SP regression).

**Cost note (honest):** in 2a the adopting eval is still the ORACLE, so the
first `CommitIncumbent` per interval hits `Evaluate` on an empty cache →
`Initialize` (one full RTA). Subsequent commits in the same interval are the
cheap `FullReuse` path. The "no extra RTA" claim fully materializes only once
2b makes the read-side `Evaluate` warm `candidate_rta_` at `:247` first. This
is acceptable for 2a (behavior-preserving; the extra RTA is dwarfed by the
oracle's own RTA at `:247`).

**Gating airtightness (verified):** `ResetIncumbentBaseline` now clears the flag
unconditionally at the top; only `PerformSerializedTaskQueueOptimization`
re-arms it. The reopt path (`:574`/`:620` → `ResetIncumbentBaseline(true)` →
`ScratchOrIncre` → `OptimizeFromScratch`, can commit >1) and the legacy
incremental descent never set it true → their `CommitIncumbent` calls skip the
cache work. No path reaches `Evaluate` with a >1 diff while the gate is on.

**Build/test:** `cmake --build build --target check.SP_OPT -j5` (DEBUG required,
per `sp-opt-test-build-debug-config.md` memory) → **16/16 ctest green**
(`testIncreOpt_w_TL` 4.47s incl. the invariant-asserted serialized walk).

**Real-path coverage:** `testIncreOpt_w_TL::SerializedIncremental_SingleChangeInvariant`
(testIncreOpt_w_TL.cpp:305) uses the CONCRETE `OptimizePA_Incre_with_TimeLimits`
(no stub override) with `debugMode=1` (invariant assertions armed) and runs
`opt.OptimizeIncre_w_TL` → `PerformSerializedTaskQueueOptimization` → re-arms
`rta_cache_active_=true` → every `CommitIncumbent` in the walk now runs
`Evaluate`+`AdoptChampion` under the invariant assertion. Passed, `sp_opt>0`.
So the cache write-side executed under the single-change assertion and stayed
consistent (no throw, no SP regression). The stub-override tests at :937/:1191
override `PerformSerializedTaskQueueOptimization` itself → they DON'T exercise
the real cache write-side (their override calls the base via `this->...` only if
they choose to; the recorded-state ones don't) — but they also passed, so the
header signature change (new member) didn't break the override vtable.

### Next: increment 2b — the READ-side :247 swap

2b is the meatier, differential-TDD-gated increment: add
`ObtainSP_DAG_From_Dists_With_Perf_Coeff` to `SP_Metric.h/.cpp` (Hazard B —
verified `ObtainSP` SP_Metric.cpp:11-15 omits `perf_coefficient`, so the
existing `ObtainSP_DAG_From_Dists` is NOT bit-identical to the oracle
`EvaluateSPWithPriorityVec`→`ObtainSP_DAG`→`ObtainSP_TaskSet:61,65` which DOES
multiply `perf_coefficient`; the existing helper also has a live caller at
`ObtainSPFromRTAFiles:221` so it must NOT be modified), then swap the `:247`
baseline re-score in `EvaluateTimeLimitConfig_SubIncremental` to
`rta_cache_.Evaluate` + the perf-coeff assembly, and add a differential test
asserting cache-eval SP == oracle SP bit-identical. `:249`/`:287`
(`OptimizeIncre_SingleTask`) stay on the oracle this increment (base class,
Hazard A — deferred to 3/N).

### 2b read-side TL-walk swap — ATTEMPTED + REVERTED 2026-07-18 (blocker found)

Attempted the Phase 2 "TL patch dispatch (Loop B)" read-side swap in
`EvaluateTimeLimitConfig_SubIncremental` (OptimizeSP_TL_Incre.cpp:247-250):
replaced the oracle `EvaluateSPWithPriorityVec` baseline re-score + the
`OptimizeIncre_SingleTask` (nullopt→oracle) call with a LOCAL `RTACache`:
`Initialize` the champion on the carried PA + `ObtainSP_Full_From_NodeRTAs`
for the baseline SP, then forward `std::ref(local_cache)` into
`OptimizeIncre_SingleTask` (which takes its per-variation cache branch:
`Evaluate` + `ObtainSP_Full_From_NodeRTAs` + `AdoptChampion`-on-strict-improve).

**BLOCKER: the cache's `Evaluate` NoReuse-recompute path is NOT bit-identical
to the oracle `ProbabilisticRTA_TaskSet`.** Surfaced by `testIncreOpt_w_TL`'s
`OptimizeWithOptimizationSpace` (FAILED: incremental SP 10.2046 > scratch
9.67111, TL[0]=600 not 400). A temporary debugMode probe
(`[2B-VAR-PROBE]` in OptimizeSP_Incre.cpp's cache arm) isolated it:
- **Baseline re-score (|diff|==0, FullReuse, `Initialize`): bit-identical**
  (`[2B-BASELINE-PROBE] delta=0` always). Initialize uses
  `ProbabilisticRTA_TaskSet` — the same fn the oracle uses, so it matches.
- **Per-variation NoReuse recompute (`Evaluate`, RTA_Cache.cpp:452-474):
DIVERGES** — e.g. `cache=12.4631 oracle=12.5296 delta=-0.0665`,
`cache=12.6174 oracle=12.5296 delta=+0.088` (sign varies). Up to 0.53 SP off.

**Root-cause hypothesis (NOT yet fixed):** `Evaluate`'s NoReuse recompute uses
the **2-arg** `GetRTA_OneTask(task_curr, hp_tasks)` (RTA_Cache.cpp:467), which
does `Compress`+`Convolve` **per HP task on the running RTA** (RTA.cpp:31-43).
The oracle `ProbabilisticRTA_TaskSet_SingleCore` (RTA.cpp:90) uses the **3-arg**
`GetRTA_OneTask(task, hp_tasks, hp_tasks_et_conv)` — ONE `Compress` then
`Convolve` against a precomputed HP-ET convolution. `CompressDistributionWithOnlySize`
is LOSSY (bucket-merge, Probability.cpp:382), so compressing at different points
diverges once the support grows past `Granularity`. The priority-path
differentials (P1.13 sub-step 3/3b/3c) MISSED this because their synthetic
`FiniteDist(GaussianDist,min,max,5)` ETs stayed under `Granularity`; the TL-walk's
single-point TL dists (`GetUnitExecutionTimeDist`) convolved with Gaussian HPs
grow past it.

**Attempted fix (REVERTED):** swapped `Evaluate`'s recompute to the 3-arg form
+ inline HP-prefix roll (mirroring RTA.cpp:86-98). Did NOT fix it — delta
remained (sign flipped to negative). The divergence is subtler than the
2-arg-vs-3-arg hypothesis; needs deeper RTA-convolution tracing.

**Reverted ALL three changes** (TL-walk read-side swap + both probes + the
RTA_Cache.cpp 3-arg attempt). Working tree restored to the pre-2b state: 16/16
ctest green, write-side cache only (P1.12 2a), read-side still on the oracle.
No source diff beyond the pre-existing 2a write-side code.

**NEXT (2b, unblocked-when-fixed):** trace the exact RTA-convolution divergence
in `Evaluate`'s NoReuse path vs `ProbabilisticRTA_TaskSet_SingleCore` — likely
`ResolvePreemptionsAndCompress`'s `if_new_preempt` flag (2-arg computes it from
the running RTA's `max_time`; 3-arg from the precomputed convolution — these
can differ, changing which preemptions get added). The cache is NOT a valid
drop-in until this is bit-identical. Priority path (P1.13) is unaffected (its
tests pass) but is built on the same buggy `Evaluate` — a latent correctness
hazard there too, masked by small-ET test fixtures.

## 2026-07-19 — Re-anchoring: state reconciliation after P1.14 Phase 2c split

User: "continue work on task p1_12, integrate rta cache into incremental+TL
optimizer." Re-read the full task record + memory + the committed tree before
touching code. Found the working tree state DIVERGES from what the older dev-log
entries describe:

- HEAD is `d7de4ad1` ("add optimizer to incremental optimization"). The three
  cache-related commits since the 2a log entry are `843f9603` (add rta_cache,
  the frozen P1.11 Phase 0 cache class), `3e9b6518` ("add cache to optimimizeSP"
  — this is P1.13: `RTACacheOpt rta_cache` param threaded into
  `OptimizeIncre` / `OptimizeIncre_SingleTask`, ALWAYS-engaged via a same-scope
  local-cache fallback, Hazard B fixed IN PLACE in `ObtainSP_DAG_From_Dists`),
  and `d7de4ad1` (P1.14-mirror shared time-limit budget guard in
  `Optimize_w_TL_ScratchOrIncre`).
- `git status` shows ZERO source diffs vs HEAD. `grep rta_cache_` in
  `OptimizeSP_TL_Incre.{h,cpp}` → NONE. So the P1.12 2a write-side
  (`rta_cache_` member on `OptimizePA_Incre_with_TimeLimits` + the
  `rta_cache_active_` gate + the gated `AdoptChampion` in `CommitIncumbent`)
  recorded in the 2026-07-18 2a entry is NOT in the committed tree. The P1.14
  Phase 2c "split clean" reverted ALL P1.12 RTA-cache hunks from
  `OptimizeSP_TL_Incre.{h,cpp}` (header back to HEAD; cpp hunks reverted),
  backed them up at `p1_12_increment_2a_backup.patch`, and committed only the
  P1.14-mirror time-limit hunks. So P1.12 is effectively RE-OPENED at HEAD: no
  TL-path cache integration is live.
- What IS live at HEAD: P1.13 (priority path cache) is committed + always-on in
  `OptimizeIncre`. The TL/serialized path (`EvaluateTimeLimitConfig_SubIncremental`
  @ `OptimizeSP_TL_Incre.cpp:247-250`) still calls the ORACLE
  `EvaluateSPWithPriorityVec` for the baseline re-score AND calls
  `OptimizeIncre_SingleTask` with NO cache arg (defaults to nullopt → oracle
  branch, since `OptimizeIncre_SingleTask` itself does NOT auto-create a local
  cache — only `OptimizeIncre` does). So the TL walk pays a full RTA per
  candidate.

**The 2b BLOCKER stands unresolved.** The 2026-07-18 2b entry's root-cause
hypothesis: `Evaluate`'s NoReuse recompute uses the 2-arg
`GetRTA_OneTask(task_curr, hp_tasks)` (RTA_Cache.cpp:458), which does
`Compress`+`Convolve` PER HP task on the running RTA (RTA.cpp:32-44). The oracle
`ProbabilisticRTA_TaskSet_SingleCore` (RTA.cpp:88-113) uses the 3-arg
`GetRTA_OneTask(task, hp_tasks, hp_tasks_et_conv)` — ONE `Compress` on the
running RTA then ONE `Convolve` against a precomputed rolling HP-ET convolution.
`CompressDistributionWithOnlySize` is LOSSY (bucket-merge), so the running-RTA
compress count differs (oracle=1, 2-arg=i) → divergence once the convolved
support grows past `Granularity`. The existing testRTA cache fixtures use tiny
ET dists (max≈7, Granularity=10) that NEVER cross the bucket boundary → all 7
`Evaluate_*` bit-identity tests pass, masking the bug. The attempted 3-arg swap
was REVERTED (delta persisted, sign flipped).

**Baseline confirmed:** `cmake --build build --target check.SP_OPT -j5` (DEBUG)
→ 17/17 ctest green (HEAD, no source diffs).

### Plan: TDD-reproduce the 2b divergence first, then fix `Evaluate`

Per agent_coding_rules (TDD: failing test first → fix → refactor). The right
first increment is a NEW differential test in `tests/testRTA.cpp` that
constructs a fixture with WIDE ET distributions (convolved support >>
`Granularity`) and asserts `RTACache::Evaluate` on a single-task-TL-change
candidate (NoReuse path) is bit-identical to `OracleRtas`. This should FAIL on
HEAD, reproducing the 2b blocker in isolation (cheaper than the full
`testIncreOpt_w_TL` integration that surfaced it). Once it fails, trace the
exact compress/convolve divergence between the 2-arg NoReuse path and the
oracle's 3-arg path and fix `Evaluate` so it reproduces the oracle's
compress-count + convolution-rolling order EXACTLY. The P1.13 priority-path
differential (`OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange`)
stays green throughout (it uses small-ET fixtures unaffected by the fix — the
fix makes the cache MORE correct, never less).

## 2026-07-19 — Docs + memory reconciled to the verified HEAD state

User (after a `/compact` + a "is p1_13 done yet?" aside): "first update p 1_12
related doc and memory files. measurement from p1_12 is fine."

Re-verified the committed state this session (not just carried from the prior
summary): `git log -1` → `d7de4ad1`; `git status --short` on
`sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` → EMPTY (zero diffs); the 2a
backup patch is on disk at `p1_12_increment_2a_backup.patch` (9377 bytes,
`Jul 19 09:40`). So the 2026-07-19 re-anchoring entry's claims hold: no TL-path
cache integration is live at HEAD; the 2a work was reverted by P1.14 Phase 2c.

Doc updates made this entry:
- `goal.md` — rewrote the header note (P1.11 Phase 0 is committed `843f9603`,
  NOT "working tree uncommitted" as the old line said) + added a "Current state
  (2026-07-19)" section up top recording: no live integration at HEAD, blocked
  on the 2b bit-identity defect, NEXT = TDD-reproduce then fix `Evaluate`.
- `tasks.md` — restructured to lead with the verified HEAD state + the 2b
  blocker as Phase 1 step 0 (the gate for all read-side work); marked the 2a
  scaffolding items as DONE-then-REVERTED (with the backup-patch pointer);
  corrected that Hazard B is already fixed in place by P1.13 at HEAD (the
  `implementation_plan.md` "needs a new `*_With_Perf_Coeff` helper" note is
  SUPERSEDED — reuse the corrected `ObtainSP_DAG_From_Dists`); moved the
  base-class `RTACache&` threading (Hazard A) into Phase 2.
- `dev_log.md` — this entry.

Memory updates made this entry:
- `p19-cache-redesign-single-champion.md` (P1.9/P1.11/P1.12 index) — corrected
  the stale "P1.12 2a DONE ... working tree NOT committed" framing to "2a
  REVERTED by P1.14 Phase 2c, backed up at `p1_12_increment_2a_backup.patch`,
  no TL-path cache integration live at HEAD; blocked on the 2b `Evaluate`
  NoReuse bit-identity defect"; updated NEXT to the TDD-reproduce-then-fix plan.
- `p113-rta-cache-priority-opt-standalone.md` (P1.13) — corrected the stale
  "NOT committed" repeats to "committed `3e9b6518`"; recorded the latent
  correctness hazard (P1.13 built on the same buggy `Evaluate`, masked by
  small-ET fixtures) so it's not lost.
- `MEMORY.md` index lines for P1.9/P1.11, P1.12, P1.13 — refreshed to match.

**Measurement (Phase 2 end-to-end scalability) is FINE as-is per user direction
this entry** — not pursued now; it remains a Phase 2 task to run AFTER the 2b
blocker is fixed and the read-side swap + base-class threading land (profiling
a cache whose `Evaluate` is known-wrong would measure the wrong thing).

### NEXT (unchanged from the 2026-07-19 plan above)

TDD-reproduce the 2b divergence: new `tests/testRTA.cpp` differential test with
WIDE ET distributions (convolved support >> `Granularity`) asserting
`RTACache::Evaluate` NoReuse == oracle bit-identical → expect FAIL on HEAD →
then trace + fix `Evaluate`. Awaits explicit user go before code changes.
