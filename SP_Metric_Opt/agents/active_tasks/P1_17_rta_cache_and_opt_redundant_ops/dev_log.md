# P1.17 — Dev Log

## 2026-07-19 — Filed (no code yet)

User request: "there are many copy paste operations that are not really necessary
in rta cache, and main optimization code… in p1_17, i think code can be further
re-factored and reduce some unnecessary operations."

**Scope set:** refactor + perf on the RTA cache + the TL/INCR optimizer eval
path. NOT a correctness task. Single gate = bit-identical SP to the current
committed code (differential TDD, same contract as P1.12). C++ only.

**Why this is its own task and not folded into P1.12:** P1.12's remaining work
(Loop A/B dispatch + the N=6/10/16 scalability measurement) is about USING the
cache in more places; P1.17 is about making the cache's per-call cost honest
BEFORE that measurement, so the speedup number isn't dragged down by defensive
copies layered in during P1.12's read-side swap, P1.12 item-1b's task-id reindex,
and P1.16's backup/revert. Order: P1.17 should land before the P1.12 Phase 2
scalability measurement (task 3b), else the measurement understates the win.

**Initial redundancy inventory (from reading the committed `Evaluate` body,
`RTA_Cache.cpp:410-502` + `EvaluateTimeLimitConfig_SubIncremental`,
`OptimizeSP_TL_Incre.cpp:~270-300`):**
1. `Evaluate` re-bakes the CHAMPION DAG every call (`:448-451`) even though the
   champion is invariant across calls within one champion lifetime. Cache it on
   `AdoptChampion`/`Initialize`.
2. `PerCoreOrderFromPa` (candidate side) is rebuilt by both
   `ComputeTaskSetDifference` and `ClassifyReusePerTask` on the same `(dag, pa)`
   within one `Evaluate`. One compute suffices.
3. `ExtractTaskSetPerProcessor` (`:476-477`) re-partitions
   `tasks_prioritized` that `PerCoreOrderFromPa` already partitioned. Collapse.
4. `RTACache cache_backup = rta_cache_;` (`OptimizeSP_TL_Incre.cpp:~187`) copies
   the whole cache (champion triple + `rta_` + `hp_prefix_per_core_` +
   `candidate_rta_`) on every `EvaluateTimeLimitConfig_SubIncremental` entry,
   even on the adopt path. Narrow the scope or move to the reject branch.
5. `candidate_rta_.assign(rta_.size(), FiniteDist({Value_Proba(0, 1.0)}))`
   (`:444`) zero-inits every slot then overwrites each — redundant if every slot
   is written by the reindex + recompute loops (prove, then drop; keep if any
   path leaves a slot unwritten).

These are the Phase 1/2 items in `tasks.md`; Phase 0 asks for a profile to rank
them by actual cost before refactoring (validate, not discover).

**Sibling tasks:**
- **P1.18** (`agents/active_tasks/P1_18_classify_reuse_per_task_more_types/`)
  extends `ClassifyReusePerTask` to NEW reuse types (the reserved
  `ReuseHpTasksEt` same-core-suffix slot). P1.17's task 1d (factor the shared
  bake/prioritize/index body into a helper) is the seam P1.18 will plug into —
  keep the helper's return shape stable so P1.18 can add a verdict branch without
  re-touching the dispatch loop.
- **P1.12** Phase 2 item 5 (scalability measurement at N=6/10/16) is unblocked
  by P1.17 task 3b.

NEXT: Phase 0 — confirm the differential gate GREEN on the unmodified tree, then
profile one interval at N=6 to rank the 5 hotspots.

## 2026-07-19 — Phase 0c baseline GREEN; first-increment plan

**Phase 0c (differential gate on unmodified tree):** `cmake --build . --target
check.SP_OPT -j5` from `build/` (`-DCMAKE_BUILD_TYPE=DEBUG`) → **17/17 ctest
PASS** (17.71s). Baseline pinned before any change. The differentials that gate
this task: `testRTA.cpp::Evaluate_*_BitIdenticalToOracle`,
`SP_Assembly_*_BitIdenticalToOracle`, `testOptimizeIncrePA::*`,
`testINCRTimeout::*`.

**Grounded re-read of the `Evaluate` hot path (`RTA_Cache.cpp:410-502`) +
`TryComputeSingleChange` (`:252-343`):** the champion DAG is baked MORE than
once per `Evaluate` call, not just at the `:448-451` site the inventory listed:
- `Evaluate` → `ClassifyReusePerTask` (`:423`) → `ComputeTaskSetDifference`
  (`:383`) → `TryComputeSingleChange` (`:357`). `TryComputeSingleChange` bakes
  the champion at `:258-259` (`ApplyTimeLimitsToTasksExecutionTime(
  dag_champion_.tasks, tl_champion_)`) AND builds
  `PerCoreOrderFromPa(dag_champion_, pa_champion_)` at `:276`.
- THEN `Evaluate`'s own reindex block bakes the champion AGAIN at `:448-451`.
So per `Evaluate` with a live champion: champion baked ≥2×, champion per-core
order built ≥1×. All of these are invariant across one champion lifetime
(`AdoptChampion`/`Initialize` are the only champion writers).

**First increment (task 1a, narrowly scoped):** cache the champion's baked +
pa-sorted `TaskSet` as `champ_prioritized_`, set in `Initialize` +
`AdoptChampion` (both already compute `tasks_prioritized` locally for
`RebuildPrefixes` — move it into the member after `RebuildPrefixes` consumes
the const-ref), and read it in `Evaluate`'s reindex block instead of re-baking.
This removes ONE champion bake + ONE `UpdateTaskSetPriorities` per `Evaluate`.
`TryComputeSingleChange`'s champion bake is left for a later increment (it
needs `champ_dag_baked` as a `DAG_Model` for `FindTaskWithDifferentEt`, a
different shape — don't conflate).

Gate: existing `Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle` +
`SP_Assembly_*` differentials; expect 17/17 unchanged.

### 1a increment 1 — DONE (champion bake hoisted out of Evaluate's reindex)

**Change:** added `TaskSet champ_prioritized_` member to `RTACache`
(`RTA_Cache.h:165`). `Initialize` (`RTA_Cache.cpp:188`) and `AdoptChampion`
(`:209`) now store `tasks_prioritized` into the member (they already built it
locally for `RebuildPrefixes`). `Evaluate`'s reindex block (`:444-456`) reads
`champ_prioritized_` directly instead of re-running
`ApplyTimeLimitsToTasksExecutionTime(dag_champion_.tasks, tl_champion_)` +
`UpdateTaskSetPriorities(champ_baked, pa_champion_)` on every call.

**Net per `Evaluate` (with a live champion):** −1
`ApplyTimeLimitsToTasksExecutionTime` + −1 `UpdateTaskSetPriorities` on the
champion side. The candidate-side bake (`:437-439`) stays — the candidate
genuinely changes per call (Type-L trial TL).

**Bit-identity gate:** 17/17 ctest DEBUG PASS (20.22s) after a forced
`touch RTA_Cache.cpp` rebuild. The differentials that would catch a champion-
order scramble (`Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle`,
`SP_Assembly_TypeLChange_BitIdenticalToOracle`, `SP_Assembly_TypeE_NoChange_…`)
are all green.

**Next increment candidate:** 1a is "cache champion bake in Evaluate's reindex"
only. The champion is STILL re-baked inside `TryComputeSingleChange`
(`:258-259`, as `champ_dag_baked` for `FindTaskWithDifferentEt`) on every
`Evaluate` via `ClassifyReusePerTask → ComputeTaskSetDifference →
TryComputeSingleChange`. That bake needs a `DAG_Model` shape (not a `TaskSet`),
so it is a separate, later increment — do NOT conflate.

### 1b increment 1 — DONE (collapse candidate per-core partition to one build)

**Redundancy removed:** `Evaluate` called `ClassifyReusePerTask(dag_tasks, pa,
tl)` (`:425-426`), which internally calls `PerCoreOrderFromPa(dag_tasks, pa)`
(`:392-393`) — a SECOND `UpdateTaskSetPriorities` (full task-set sort) + partition
on the candidate, on top of the `tasks_prioritized` sort `Evaluate` itself does
at `:441` and the `ExtractTaskSetPerProcessor` partition at `:477`.

**Change:** `Evaluate` now builds `per_core = ExtractTaskSetPerProcessor(
tasks_prioritized)` ONCE (right after the `task_id2index` loop), then derives
the `verdict` vector INLINE from `ComputeTaskSetDifference`'s `diff` + `per_core`
(mirrors `ClassifyReusePerTask` exactly: |diff|==0 → all FullReuse; |diff|==1 →
tasks on `diff.core` are NoReuse). The recompute loop reuses the same `per_core`.

**Key correctness invariant:** `PerCoreOrderFromPa(dag_tasks, pa)` (un-baked
candidate) and `ExtractTaskSetPerProcessor(tasks_prioritized)` (baked candidate)
produce the SAME task-id partition and per-core order — baking only changes ET,
never `processorId` or priority order. So the `diff.core` locator from
`ComputeTaskSetDifference` (which uses the un-baked `PerCoreOrderFromPa`) keys
correctly into the baked `per_core`. Pinned by
`Evaluate_PriorityMove_CrossCoreScramble_BitIdenticalToOracle` + the
`Evaluate_*_OneTaskPatch` / `Evaluate_NoReuseWideEtTaskWithTwoWideHpTasks_*`
differentials.

**Net per `Evaluate` (with a live champion):** −1 `UpdateTaskSetPriorities`
(the `ClassifyReusePerTask`-internal sort) + −1 `PerCoreOrderFromPa` partition
build. `ComputeTaskSetDifference`'s OWN two `PerCoreOrderFromPa` calls
(candidate + champion, inside `TryComputeSingleChange`) remain — those are a
deeper increment (need to thread a pre-built partition through the
`const` `TryComputeSingleChange` API).

**`ClassifyReusePerTask` status:** now test-only (no production callers). KEPT
as the public self-contained query that documents the verdict semantics + pins
them via `testRTA.cpp::ClassifyReusePerTask_*`; it is the P1.18 extension seam.
Stale "Evaluate calls ClassifyReusePerTask" header comment updated.

**Bit-identity gate:** 17/17 ctest DEBUG PASS (20.89s) after a forced rebuild.

### 1c — DONE (drop redundant `candidate_rta_` zero-init)

**Redundancy removed:** `Evaluate` zero-initialized every candidate buffer slot
with `candidate_rta_.assign(rta_.size(), FiniteDist({Value_Proba(0, 1.0)}))`
(`RTA_Cache.cpp:472`) — N FiniteDist constructions per call, every one
immediately overwritten by the reindex or recompute loops.

**Proof every slot is written before read (so the zero-init is dead):**
- The reindex loop writes `candidate_rta_[task_id2index[tid]] = rta_[k]` for
  every champion task `champ_prioritized_[k]` (FullReuse tasks keep this value).
- The recompute loop overwrites the NoReuse slots (`candidate_rta_[task_id2index.at(id)]`).
- A slot is unwritten only if its task id is neither a champion task id (reindex
  misses it) NOR NoReuse (recompute misses it). NoReuse tasks are covered by the
  recompute loop, so the only hazard is a FullReuse candidate task whose id is
  absent from `champ_prioritized_`.
- Under the single-change invariant — the ONLY path `Evaluate` serves, since
  `ComputeTaskSetDifference` THROWS on |diff|>1 — `TryComputeSingleChange`'s
  per-core size check (`AnalyzePrioritySwitch` step 1, `RTA_Cache.cpp:113`)
  rejects any candidate whose task set differs from the champion's (size
  mismatch ⇒ NotSingle ⇒ throw). So candidate task set == champion task set ⇒
  every candidate task id IS a champion task id ⇒ present in `champ_prioritized_`
  ⇒ written by the reindex loop. ∎

**Change:** `assign(N, FiniteDist({Value_Proba(0,1.0)}))` → `resize(N)`. Sizes
the buffer without constructing N identity-zero FiniteDists; a no-op when
Initialize/AdoptChampion already sized the member to N (the common case — both
set `candidate_rta_ = rta_/rtas`). `resize` (not `assign`) makes the "filled
below" intent explicit. Bound check: `it->second ∈ [0, N)` because
`task_id2index` maps to candidate priority-positions `0..N-1` and `rta_.size()`
== N under the invariant.

**TDD pin:** `Evaluate_IdentityCandidate_EverySlotFilled_NoZeroInit` — the
|diff|==0 early-return path (line 488) skips the recompute loop ENTIRELY, so
every slot must be filled by the reindex loop alone. This is the exact case a
"dropped the assign but a FullReuse slot went unwritten" regression would trip:
the test asserts both non-emptiness (a default-constructed FiniteDist has an
empty `distribution`) AND bit-identity to the oracle. The identity-zero FiniteDist
the prior `assign` used would also fail the value check against a non-degenerate
oracle entry, so the test catches the redundancy itself, not just the bug.

**Bit-identity gate:** 17/17 ctest DEBUG PASS (19.96s); 54/54 testRTA (incl. the
new pin + the `Evaluate_PriorityMove_CrossCoreScramble_*` / `SP_Assembly_*`
differentials).

### 1a remainder — DONE (hoist champion TL-bake out of TryComputeSingleChange)

**User prompt 2026-07-20:** "code such as `TaskSet champ_baked =
ApplyTimeLimitsToTasksExecutionTime(dag_champion_.tasks, tl_champion_)` ...
is not necessary as cham-related info can be read from [the cached champion
members] ... champion config is relatively stable and don't change, function
inputs are typically new dag_tasks."

**Redundancy removed:** `TryComputeSingleChange` re-baked the champion
(`ApplyTimeLimitsToTasksExecutionTime(dag_champion_.tasks, tl_champion_)`,
`RTA_Cache.cpp:260-261`) on EVERY call — once per `Evaluate` via
`ClassifyReusePerTask → ComputeTaskSetDifference → TryComputeSingleChange`
(now via the inline verdict derivation, still routed through
`ComputeTaskSetDifference → TryComputeSingleChange`). The champion TL-bake is
invariant across one champion lifetime (only `Initialize`/`AdoptChampion`
mutate the champion triple), so it is cacheable.

**Why a NEW member, not `champ_prioritized_`:** `FindTaskWithDifferentEt`
(`OptimizeSP_Incre.cpp:197-212`) walks `dag.tasks[i]` by INDEX `i` and compares
`execution_time_dist` — it needs CANONICAL (task-id) order on both sides so
index `i` means the same task. `champ_prioritized_` is PA-SORTED
(`UpdateTaskSetPriorities`-reordered), so feeding it to
`FindTaskWithDifferentEt` would break the index correspondence with the
candidate's canonical `dag_tasks.tasks`. The needed artifact is the champion's
TL-baked tasks in canonical order = the exact `tasks_baked` local
`Initialize`/`AdoptChampion` already build at `:181`/`:205` BEFORE pa-sorting
it into `champ_prioritized_`. Cached that as `champ_tasks_baked_` (new member,
`RTA_Cache.h:172`).

**Change:**
- `RTA_Cache.h`: added `TaskSet champ_tasks_baked_;` member (+ invariant doc).
- `RTA_Cache.cpp::Initialize` (`:188`) + `AdoptChampion` (`:210`): store
  `tasks_baked` into `champ_tasks_baked_` right after `champ_prioritized_` (no
  extra work — `tasks_baked` already existed as a local).
- `RTA_Cache.cpp::TryComputeSingleChange` (`:258-269`): dropped the champion
  `ApplyTimeLimitsToTasksExecutionTime` call; `champ_dag_baked.tasks` now reads
  `champ_tasks_baked_`. The `DAG_Model champ_dag_baked = dag_champion_;`
  struct-copy + `FindTaskWithDifferentEt`'s `const DAG_Model&` signature are
  KEPT (eliminating them cleanly means changing `FindTaskWithDifferentEt`'s
  signature in another module — scope creep for this commit; it reads only
  `.tasks[i].execution_time_dist` so the copy is a no-op semantically). The
  candidate bake stays — candidate `tl` genuinely changes per call.

**Net per `TryComputeSingleChange` (with a live champion):** −1
`ApplyTimeLimitsToTasksExecutionTime` on the champion side (N per-task ET
clones removed). Candidate-side bake + the two `PerCoreOrderFromPa` calls
(candidate + champion, `:277-280`) remain — the latter is the 1b-remainder,
deferred (needs const-API threading).

**TDD pin:** `Evaluate_TLChange_AfterAdoptChampion_StaleBakeGuard` — adopts a
2ND champion whose TL differs from the 1st, then Evals a candidate that
differs from the 2nd by one task's ET + asserts the diff lands on task 1. This
is the exact hazard the cache introduces: if `champ_tasks_baked_` were left
holding the 1st champion's bake after the 2nd `AdoptChampion`,
`FindTaskWithDifferentEt` would compare against the stale bake → mis-located
diff or a false |diff|>1 throw. The existing `Evaluate_TLChange_OneTaskPatch` +
`Evaluate_TLAndPriorityMove_CombinedPatch` + `ComputeTaskSetDifference_ClassesThreeCases`
already exercise the ET-diff path with a single champion; this pin adds the
multi-champion staleness case.

**Bit-identity gate:** 17/17 ctest DEBUG PASS (21.80s); 55/55 testRTA.

### Throwaway-local cleanup — DONE (write baked artifacts straight into members)

**User prompt 2026-07-20:** "there are still unnecessary copy paste, such as
`TaskSet tasks_baked = ApplyTimeLimitsToTasksExecutionTime(...)`; `TaskSet
tasks_prioritized = UpdateTaskSetPriorities(tasks_baked, pa)`; ... no need to
create `tasks_baked`, just use `champ_tasks_baked`. similarly, in `AdoptChampion`
and `TryComputeSingleChange`."

**Redundancy removed:** `Initialize`/`AdoptChampion`/`TryComputeSingleChange`
each built a throwaway local (`tasks_baked` / `cand_baked`) and then copied it
into a member — the local served no purpose because the consuming functions all
take `const TaskSet&` and return-by-value (verified:
`ApplyTimeLimitsToTasksExecutionTime(const std::vector<Task>&, const
std::vector<double>&) → TaskSet` at `SP_Metric.cpp:76-87`;
`UpdateTaskSetPriorities(const TaskSet&, ...) → TaskSet` at
`OptimizeSP_Base.cpp:61`; `RebuildPrefixes(const TaskSet&)` at
`RTA_Cache.h:206`). They copy internally + never mutate their input, so handing
them a member directly is safe.

**Change (3 sites, pure refactor — same calls, same order, no new members):**
- `Initialize` (`:181-190`): dropped `tasks_baked` + `tasks_prioritized` locals;
  writes `ApplyTimeLimitsToTasksExecutionTime(...)` straight into
  `champ_tasks_baked_`, then `UpdateTaskSetPriorities(champ_tasks_baked_, pa)`
  straight into `champ_prioritized_`, then `ProbabilisticRTA_TaskSet(
  champ_prioritized_)` straight into `rta_`, then `RebuildPrefixes(
  champ_prioritized_)`. −2 local `TaskSet` constructions (each is a vector<Task>
  clone of N tasks).
- `AdoptChampion` (`:206-211`): same pattern (skips `ProbabilisticRTA_TaskSet`
  — takes caller-supplied `rtas`). −2 local constructions.
- `TryComputeSingleChange` (`:266-269`): dropped `cand_baked` local; writes
  `ApplyTimeLimitsToTasksExecutionTime(dag_tasks.tasks, tl)` straight into
  `cand_dag_baked.tasks`. −1 local construction per call.

**No new TDD pin:** this is a no-behavior-change local-elimination — the same
functions are called in the same order with the same arguments, only the
destination of the return value changes (member vs. local-then-member). The
existing differentials cover it: `Evaluate_TLChange_AfterAdoptChampion_StaleBakeGuard`
(multi-champion, the cache-staleness hazard) + `Evaluate_TLChange_OneTaskPatch` +
`ComputeTaskSetDifference_ClassesThreeCases` + the `SP_Assembly_*` bit-identity
suites. If the direct-to-member write had introduced an aliasing bug (e.g. a
consumer mutating its input), these would trip.

**Bit-identity gate:** 17/17 ctest DEBUG PASS (22.59s); 55/55 testRTA.

