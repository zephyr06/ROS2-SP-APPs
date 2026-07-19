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

## 2026-07-19 — 2b BLOCKER RESOLVED (TDD: root-cause reproduced → Evaluate fixed → 17/17 ctest green)

User go received ("start do that"). Executed the TDD plan from the prior NEXT
entry. Working tree, NOT committed (agents only `git add`).

### Root cause — confirmed + isolated at the primitive level

The 2b blocker is a REAL divergence between the two `GetRTA_OneTask` overloads
(`sources/Safety_Performance_Metric/RTA.cpp`):

- **2-arg** `GetRTA_OneTask(task, hp_tasks)` (RTA.cpp:32): for each HP task,
  `Compress(rta_cur); Convolve(hp_et)` on the running RTA → `n_hp` compresses.
- **3-arg** `GetRTA_OneTask(task, hp_tasks, hp_tasks_et_conv)` (RTA.cpp:46):
  `Compress(rta_cur)` ONCE, then `Convolve(hp_tasks_et_conv)` against a
  PRE-CONVOLVED rolling HP-ET prefix (itself built by per-task
  `Compress; Convolve` in `ProbabilisticRTA_TaskSet_SingleCore`, RTA.cpp:110-112)
  → 1 compress of the running RTA.

`CompressDistributionWithOnlySize` (Probability.cpp:382 → `CompressDistribution`,
Probability.cpp:335) is **LOSSY** once `distribution.size() > max_size`
(= `Granularity`); it returns immediately when `size <= max_size`. With `>=2` HP
tasks whose convolved support exceeds `Granularity`, the differing compress count
(`n_hp` vs `1`) yields a different `FiniteDist`. The **3-arg form is the oracle's
definition of correct** (`ProbabilisticRTA_TaskSet_SingleCore` calls it at
RTA.cpp:105); the 2-arg form is the divergent one.

`RTACache::Evaluate`'s NoReuse path called the **2-arg** form
(RTA_Cache.cpp:458 pre-fix) → diverged from the oracle.

### TDD step 1 — failing reproduction test FIRST

First attempted two `Evaluate`-vs-oracle repro tests on a wide-ET fixture
(`TaskSetForTest_3tasks_1core_wideET`, 3 wide Gaussians truncated to [20,80] at
granularity 5, all on core 0) — they PASSED on HEAD (not load-bearing: the
fixture's task 2, when its TL or position changed, never ended up as a wide-ET
NoReuse task with `>=2` wide HP tasks in a divergent shape). Then wrote a DIRECT
primitive differential `GetRTA_OneTaskDifferential.TwoArgVsThreeArg_*` calling
both overloads on identical wide-ET 2-HP input and asserting equality → **FAILED
on HEAD**, definitively reproducing the mechanism (cheaper than the full
`testIncreOpt_w_TL` integration, and isolated to the two overloads with no cache
machinery).

### TDD step 2 — the fix

`RTACache::Evaluate`'s NoReuse walk (RTA_Cache.cpp:452-464) rewritten to mirror
the oracle's `ProbabilisticRTA_TaskSet_SingleCore` loop (RTA.cpp:88-113)
exactly: per core, in candidate priority order, maintain a rolling
`hp_tasks_et_conv` (seeded `IdentityPrefix()`, rolled forward via the existing
anon-namespace `RollPrefix` = `Compress; Convolve` per task) and call the **3-arg**
`GetRTA_OneTask(task_curr, hp_tasks, hp_tasks_et_conv)`. The prefix rolls forward
for EVERY task on the core (FullReuse + NoReuse alike) so a NoReuse task's HP-ET
convolution includes all higher-priority tasks regardless of their verdict —
matching the oracle. `hp_tasks` is still accumulated (the 3-arg form consumes it
for the preemption loop in `ResolvePreemptionsAndCompress`). The champion `rta_`
is untouched (seeds into `candidate_rta_` as before).

### TDD step 3 — permanent root-cause guard

The direct primitive differential is the load-bearing reproduction (it FAILS
asserting equality on HEAD). Its CORRECT permanent form asserts the two overloads
**DIFFER** on wide-ET 2-HP input (`GetRTA_OneTaskDifferential.TwoArgDivergesFromThreeArgOnWideEt`,
EXPECT_FALSE `rta_two_arg == rta_three_arg`): this pins the 2b mechanism so a
future "optimization" that collapses the 3-arg form back into the 2-arg form (or
makes `Compress` lossless) is caught. If they ever become equal, the guard flips
and Evaluate's NoReuse overload choice is no longer load-bearing — until then,
Evaluate MUST call the 3-arg form.

### Verification

- `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` + `cmake --build . --target check.SP_OPT -j5`
  → **17/17 ctest green** (19.39s). All 7 `Evaluate_*` + the P1.13 differential
  tests + the 3 wide-ET repro tests + the new root-cause guard pass.
- `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` PASSES (oracle-only at HEAD;
  the `10.2046 > 9.67111` divergence was only visible with 2a's write-side live,
  which is reverted at HEAD — the fix here is on the READ-side `Evaluate` path
  that 2a's write-side would feed).

### Diffs (working tree, NOT committed)

- `sources/Safety_Performance_Metric/RTA_Cache.cpp` — Evaluate NoReuse walk
  rewritten (15 lines, +13/-2).
- `tests/testRTA.cpp` — 1 new root-cause guard test (+~70 lines); the 2 wide-ET
  repro tests + `TaskSetForTest_3tasks_1core_wideET` fixture were already in the
  working tree from a prior session (the +114 in `git diff --stat` includes them).

### What this unblocks

- **P1.12 Phase 1 step 2** (the `:247` read-side swap) — `Evaluate` is now
  bit-identical to the oracle on the NoReuse path, so the `EvaluateSPWithPriorityVec`
  → `rta_cache_.Evaluate` swap at `OptimizeSP_TL_Incre.cpp:247` is safe to land
  (still needs the 2a write-side scaffolding re-landed first — order per tasks.md:
  re-land 2a from `p1_12_increment_2a_backup.patch`, THEN do the `:247` swap).
- **P1.13 latent hazard CLOSED** — P1.13's priority-path integration is built on
  the same `Evaluate`; this fix makes it correct on wide-ET inputs too (its
  small-ET fixtures no longer mask anything because the path is now correct by
  construction).

### NEXT

Re-land the 2a write-side scaffolding from `p1_12_increment_2a_backup.patch`
(`rta_cache_` member + `rta_cache_active_` gate + gated `AdoptChampion` in
`CommitIncumbent` + reset in `ResetIncumbentBaseline`), then do the `:247`
read-side swap (`EvaluateSPWithPriorityVec` → `rta_cache_.Evaluate` +
`ObtainSP_DAG_From_Dists` perf-coeff assembly against the sorted TL-baked DAG).
Differential TDD bit-identity gate: `testIncreOpt_w_TL::OptimizeWithOptimizationSpace`
must stay green. Awaits user review of THIS increment (the 2b fix) first.

## 2026-07-19 — Increment 2a RE-LANDED (write-side scaffolding; behavior-preserving)

User go received ("move forward"). Re-landed the 2a write-side scaffolding that
P1.14 Phase 2c had reverted (backup at `p1_12_increment_2a_backup.patch`). The
backup patch was the FULL pre-split patch (P1.14-mirror BF hunks + P1.12 2a cache
hunks); the P1.14-mirror BF hunks are ALREADY committed at HEAD (`ecf0c597` +
`d7de4ad1` — `BFDLSharedBudget` at `OptimizeSP_TL_Incre.cpp:609`,
`BFSharedBudgetCancelled()` poll at `OptimizeSP_Incre.cpp:71`), so re-applying
them would duplicate. Landed ONLY the 2a cache hunks, by hand, against the current
HEAD context (which matches the patch's context exactly).

**4 logical hunks, +53/-0, 2 files:**
- `sources/Optimization/OptimizeSP_TL_Incre.h`: `#include "RTA_Cache.h"` +
  `RTACache rta_cache_;` member + `bool rta_cache_active_ = false;` gate (both
  documented as write-only-as-of-2a, behavior-preserving).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` `PerformSerializedTaskQueueOptimization`:
  `rta_cache_active_ = true;` after `ResetIncumbentBaseline(false)` (re-arm for
  the walk body; the reset cleared it).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` `CommitIncumbent`: gated
  `rta_cache_.Evaluate(dag_tasks_, pa, tl)` → `rta_cache_.AdoptChampion(...)` to
  advance the cache-champion to track `res_opt_` (Evaluate does NOT advance the
  champion; only AdoptChampion/Initialize do — else later eval drifts to >1 →
  throw). The adopting Evaluate is against the SAME triple just committed →
  short-circuits to FullReuse → near-zero cost, no extra RTA, no SP regression.
  Gated by `rta_cache_active_` so the reopt path (shares this writer, can commit
  >1) neither throws nor regresses.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` `ResetIncumbentBaseline`:
  `rta_cache_ = RTACache(); rta_cache_active_ = false;` at the top of the body
  (cold cache per interval; default-construct, no `RTA_Cache.h::Clear()` needed;
  reopt path also reaches here with `from_scratch=true` → keeps cache off).

**Verification:** `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` (already configured) +
`cmake --build . --target check.SP_OPT -j5` → **17/17 ctest green** (19.74s).
`testIncreOpt_w_TL` passes (includes `SerializedIncremental_SingleChangeInvariant`,
`debugMode=1`, which runs the full serialized walk with the cache write-side
active under the invariant assertion — `sp_opt>0`). Behavior-preserving by
construction: the cache is WRITTEN (at `CommitIncumbent`) but NOT yet READ —
the oracle `EvaluateSPWithPriorityVec` still answers every SP eval at `:247`/
`:249`/`:287`. So all SP outputs are bit-identical to HEAD.

**NEXT = 2b read-side swap:** replace `EvaluateSPWithPriorityVec` with
`rta_cache_.Evaluate(...)` at the `:247` baseline re-score in
`EvaluateTimeLimitConfig_SubIncremental` (+ perf-coeff assembly via
`ObtainSP_DAG_From_Dists` — Hazard B already fixed in place by P1.13; do NOT add
`*_With_Perf_Coeff`). The `Evaluate` NoReuse path is already bit-identical to the
oracle (2b blocker fixed), so the swap should be bit-identical too — differential
TDD gate: `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` stays green, and a
direct cache-eval-vs-oracle differential on the TL walk asserts bit-identity.
`:249`/`:287` (`OptimizeIncre_SingleTask` full swap) stay on the oracle this
increment (base-class threading = Hazard A, deferred to Phase 2). Awaits user
review of THIS increment (the 2a re-land) first.

## 2026-07-19 — Increment 2b READ-SIDE SWAP landed (working tree, NOT committed)

User: "continue work on task p1_12." Re-anchored against the tree before touching
anything: HEAD advanced to `8e18c39b` ("add rta cache to optimizeSP TL"). The two
prior increments are now COMMITTED — `71da8a45` ("add hp_tasks_et_conv to
RTA_Cache") = the 2b blocker fix (RTA_Cache.cpp + testRTA.cpp), `8e18c39b` =
the 2a write-side re-land (OptimizeSP_TL_Incre.{h,cpp} + task docs). So the
"working tree, NOT committed" framing in the prior NEXT is stale for 2a/2b-fix;
only ONE source diff remains uncommitted vs HEAD.

**That one diff IS the 2b read-side swap** (OptimizeSP_TL_Incre.cpp:247-287,
+41/-2) — it was written in a prior session but never recorded in this log nor
committed. Verified its shape + correctness this session (no edit made):

- Replaces the oracle `EvaluateSPWithPriorityVec(dag_tasks_cur, sp_parameters_,
  challenger.opt_pa_)` baseline re-score with
  `rta_cache_.Evaluate(dag_tasks_cur, challenger.opt_pa_, time_limits)` →
  `ObtainSP_Full_From_NodeRTAs(dag_tasks_cur, sp_parameters_,
  challenger.opt_pa_, time_limits, baseline_rtas)`.
- Uses `ObtainSP_Full_From_NodeRTAs` (SP_Metric.cpp:181, P1.13's helper), NOT
  the bare `ObtainSP_DAG_From_Dists` the prior NEXT suggested. This is the
  BETTER choice: `ObtainSP_Full_From_NodeRTAs` mirrors the oracle
  `EvaluateSPWithPriorityVec` body EXACTLY (bake TL → `UpdateTaskSetPriorities`
  → `ObtainSP_DAG_From_Dists` with the caller-supplied node RTAs + chain
  latencies). Hazard B (perf_coefficient) is handled inside
  `ObtainSP_DAG_From_Dists` (P1.13 fixed it in place). Reusing P1.13's helper
  also avoids a parallel code path — same fn the priority cache path already
  calls at OptimizeSP_Incre.cpp:328,384.

**Double-bake is idempotent (verified):** the candidate DAG passed to `Evaluate`
is `dag_tasks_cur`, already TL-baked at :194 (`UpdateExtDistBasedOnTimeLimit`).
`Evaluate` re-bakes internally (RTA_Cache.cpp:443
`ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl)`).
`ApplyTimeLimitsToTasksExecutionTime` (SP_Metric.cpp:76) replaces
`execution_time_dist` with `GetUnitExecutionTimeDist(tl[i])` wherever
`tl[i] != -1` — a deterministic replacement, so baking an already-baked DAG with
the SAME `tl` is a no-op. The P1.13 priority path uses the OTHER convention
(pass `no_tl` = all -1.0 against a baked DAG); both conventions yield the same
baked result. No correctness issue.

**Invariant comparison is apples-to-apples (verified):** `TryComputeSingleChange`
(RTA_Cache.cpp:252) bakes BOTH the champion (`dag_champion_`+`tl_champion_`,
:259) AND the candidate (`dag_tasks`+`tl`, :261) before `FindTaskWithDifferentEt`
— so whether the candidate DAG was pre-baked is irrelevant; the diff is on the
re-baked forms. Champion was adopted at `CommitIncumbent` on the committed
triple (`dag_tasks_`, `opt_pa_`, committed `tl`); candidate is
(`dag_tasks_cur`=`dag_tasks_` baked with trial `time_limits`, `opt_pa_`,
`time_limits`). Type-L (trial TL != committed) → 1 ET diff → |diff|==1, Evaluate
patches that one task; Type-E (trial TL == committed, env absorbed on both
sides) → 0 ET diff → |diff|==0, Evaluate short-circuits to FullReuse. Matches
the `AssertSingleChangeInvariant` the path already asserts at :233.

**P1.14 cancel contract preserved (verified):** the swap wraps the Evaluate +
assembly in `BFSharedBudgetCancelled()` checks at entry and post-Evaluate,
mirroring the oracle's `INT_MIN`-on-cancel contract (the oracle returns INT_MIN
on entry-cancel + post-ObtainSP_DAG cancel). A cancelled eval returns INT_MIN →
discarded by `UpdateRecords`' strict-> adopt guard → no P1.14 time-limit
regression. The adopt path (`CommitIncumbent`'s gated Evaluate+AdoptChampion)
runs its OWN Evaluate on the committed triple and is unaffected (a cancelled
candidate never reaches CommitIncumbent).

**Verification:** `cmake --build build --target check.SP_OPT -j5` (DEBUG
configured) → **17/17 ctest green** (19.79s), zero warnings on the touched TUs.
Differential gate `testIncreOpt_w_TL::OptimizeWithOptimizationSpace` (the test
that surfaced the 2b divergence at 10.2046 > 9.67111 / TL=600-not-400) PASSES —
`res_incre.id2time_limit[0]==400` and `res_incre.sp_opt <= res_scratch.sp_opt`.
That test runs the FULL serialized walk with the cache READ-SIDE active (every
`:247` re-score now goes through `rta_cache_.Evaluate`), so a non-bit-identical
Evaluate would move incremental SP off scratch SP and fail the `<=` guard. It
holds → the cache read-side is bit-identical to the oracle on this path.

**Phase 1 step 2 is DONE** (in the working tree). What remains for Phase 1:
step 3 (a dedicated direct cache-eval-vs-oracle differential test on the TL walk
— currently covered only indirectly via `OptimizeWithOptimizationSpace`; a
focused unit test would pin bit-identity at the `Evaluate`-vs-`EvaluateSPWithPriorityVec`
level on a TL-walk fixture, isolating it from the full optimizer). Phase 2
(base-class `RTACache&` threading into `OptimizeIncre_SingleTask` at `:289` =
Hazard A, then Loop A/B dispatch + scalability measurement) is NOT started.

### NEXT

Phase 1 step 3 = a focused differential test asserting
`rta_cache_.Evaluate`+`ObtainSP_Full_From_NodeRTAs` ==
`EvaluateSPWithPriorityVec` bit-identical on a TL-walk fixture (TDD: pin the
bit-identity the `OptimizeWithOptimizationSpace` gate covers only end-to-end).
Then Phase 2: base-class threading (Hazard A) + hot-loop dispatch + measurement.

---

## 2026-07-19 — 2b read-side swap is COMMITTED (records correction)

The 2b read-side swap described above was committed at **`5a172973` "enable more
rta cache"** (+65/-31 on `OptimizeSP_TL_Incre.{h,cpp}`), NOT left in the working
tree as the prior entry stated. The prior "Awaits user review" NEXT block was
overtaken by the commit landing; this entry corrects the record.

**Re-verified from a clean DEBUG build** (`cmake --build build --target
check.SP_OPT -j5`, `CMAKE_BUILD_TYPE=DEBUG` uppercase per the build rule):
**17/17 ctest green** (18.51s), incl. `testIncreOpt_w_TL` (the differential gate
that surfaced the 2b divergence — now passes with the cache read-side live) and
`testRTA` (the cache-vs-oracle bit-identity tests). Zero warnings on touched TUs.

**What is LIVE at HEAD now:** the `:247` baseline re-score in
`EvaluateTimeLimitConfig_SubIncremental` calls `rta_cache_.Evaluate`+
`ObtainSP_Full_From_NodeRTAs` (gated by `BFSharedBudgetCancelled()` for the P1.14
contract) instead of the oracle `EvaluateSPWithPriorityVec`. `CommitIncumbent`'s
gated `Evaluate`+`AdoptChampion` (2a write-side) keeps the cache-champion
tracking `res_opt_`. The serialized TL walk is the only live cache-READ site;
`:249`/`:289` (`OptimizeIncre_SingleTask`) still call the oracle (Hazard A —
base-class `RTACache&` threading, deferred to Phase 2).

**Note on the working tree:** the P1.12 task dir was committed into
`finished_tasks/` (premature — Phase 1 step 3 + all of Phase 2 remain) and moved
back to `active_tasks/` in the working tree (uncommitted move). The records now
reflect that P1.12 is ACTIVE, not finished.

---

## 2026-07-19 — Phase 1 step 3 DONE (SP-assembly differential tests)

Added 2 focused direct differential tests pinning the bit-identity the
end-to-end `OptimizeWithOptimizationSpace` gate covers only indirectly, at the
exact `:247` seam the 2b swap touched.

**Files:** `tests/testRTA.cpp` (+3 includes, +1 helper, +2 tests).

- `OracleSP(dag, sp, pa, tl)` helper = `UpdateExtDistBasedOnTimeLimit(dag, tl)`
  → `EvaluateSPWithPriorityVec(dag_with_tl, sp, pa)` — exactly what the
  `:247` baseline re-score did BEFORE the 2b swap (`dag_tasks_cur` bake +
  oracle). The cache arm (`ObtainSP_Full_From_NodeRTAs(dag, sp, pa, tl,
  cache.Evaluate(dag, pa, tl))`) does NOT pre-bake; both must land on the same
  double.
- `SP_Assembly_TypeLChange_BitIdenticalToOracle`: champion = no TLs, candidate =
  task 2's TL moved to 6. Task 2 has `timePerformancePairs {(2,1.0),(6,0.5)}`
  → the TL moves BOTH its ET (Hazard B surface: perf_coefficient != 1.0) AND its
  RTA. |diff|==1 patch branch. `EXPECT_DOUBLE_EQ` oracle vs cache.
- `SP_Assembly_TypeE_NoChange_BitIdenticalToOracle`: candidate == champion (task
  2 TL'd on both sides). |diff|==0 → Evaluate short-circuits to FullReuse; the
  assembly over the champion's rtas must match a fresh oracle recompute.
  `EXPECT_DOUBLE_EQ`.

Both PASS. **17/17 ctest green** (18.59s), zero warnings on the touched TU.

**Why `EXPECT_DOUBLE_EQ` (exact), not a tolerance:** the cache arm and the oracle
arm execute the SAME arithmetic (same `ApplyTimeLimitsToTasksExecutionTime` →
`UpdateTaskSetPriorities` → `ProbabilisticRTA_TaskSet` → `ObtainSP_DAG`-shaped
assembly); the 2b blocker fix made `Evaluate`'s NoReuse walk mirror the oracle's
loop exactly (rolling `hp_tasks_et_conv` + 3-arg `GetRTA_OneTask`). So the two
SPs are bit-identical, not merely close. A tolerance would mask a regression of
the very property these tests exist to pin.

**Phase 1 now COMPLETE.** 2a (write-side) + 2b blocker fix + 2b read-side swap
all committed (`71da8a45`, `8e18c39b`, `5a172973`); step-3 differential pins the
seam. `:249`/`:289` (`OptimizeIncre_SingleTask`) still call the oracle —
base-class `RTACache&` threading = Hazard A, deferred to Phase 2. Phase 2 NOT
started; awaits user go + the measurement run (per task plan, measurement runs
AFTER integration lands, so after Phase 2's `:289` swap).

### NEXT

`git add` the test + record changes (agents don't commit); await user review.
Then Phase 2: thread `RTACache&` from `EvaluateTimeLimitConfig_SubIncremental`
into base `OptimizePA_Incre::OptimizeIncre_SingleTask` (Hazard A) so the
`:249`/`:289` per-variation walk uses the cache; then Loop A (priority-move
patch dispatch) + Loop B (TL patch dispatch) + end-to-end scalability
measurement at N=6/10/16.
