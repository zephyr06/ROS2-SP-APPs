# P1.9 — Tasks (working checklist)

> See `goal.md` for scope. **Perf, not correctness** — every patch must be
> bit-identical to the full recompute (TDD vs the oracle). Target regime:
> larger N (N=10/N=16). Per `agent_coding_rules.md`: one small sub-task at a
> time, ask the user to review + commit after each.

## Locked design decisions (2026-07-13)

1. **`hp_tasks_et_conv_vec[i]`** = convolution of the **higher-priority
   tasks' ET distributions** `[0, i)` on a core — i.e. the rolling
   `hp_tasks_et_conv` (`RTA.cpp:73`) snapshotted at priority index `i`.
   `hp_tasks_et_conv_vec[0]` = `FiniteDist({Value_Proba(0, 1.0)})` (empty HP
   set). The 3-arg `GetRTA_OneTask` (`RTA.cpp:44`) already consumes exactly
   this value, so a patched suffix gets `ResolvePreemptionsAndCompress` for
   free.
2. **Cache compute/patch are free functions** in `RTA.h`/`RTA.cpp`, NOT class
   methods. Each takes the cache by reference and mutates it in place.
3. **One cache per interval optimization, never crosses intervals.** The cache
   is a member of `OptimizePA_Incre_with_TimeLimits` (the TL-walk owner), reset
   at the start of each interval. **Implement the no-PA-change (TL) path
   first**; the priority-move (PA-change) path comes second.
4. **The cache replaces the existing RTA path outright** — no knob, no fallback.
   `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`) is replaced by
   `ComputeRTA_FullAndCache` on the baseline path and by the patchers on the
   candidate paths. Correctness is gated by the differential unit tests, not by
   keeping a slow path alive.

---

## API revision — 2026-07-15 (user-driven, supersedes the role-2 surface)

> Revises the **role-2 reuse query** recorded above and in `RTA_Cache.h`. The
> staged step-3c code (uncommitted) carries the old surface; the rewrite below
> replaces it before 3c is committed. The role-1 build (`ComputeRTA_FullAndCache`)
> and role-3 patcher signatures (`PatchRTA_OneTaskTL` / `PatchRTA_PriorityMove`)
> are UNCHANGED.

Four decisions from the user on 2026-07-15, overriding the staged 3c role-2 API:

### (R1) `ClassifyReuse` returns a **per-task enum vector**, not `CacheReuseInfo`

The staged 3c role-2 surface is a per-core `struct CacheReuseInfo { bool
fully_reusable; bool same_core_partition; int common_prefix_length; }` returned
as `std::unordered_map<int, CacheReuseInfo>` from `AnalyzeCacheReuse`. The user
objects: the three fields are **conditionally interdependent** (the staged
`common_prefix_length` is only actionable when `same_core_partition==true` — see
the partition-change caveat pinned by the staged test `AnalyzeCacheReuse_
PartitionChange_NotReusable`), which is "not clear and a bit messy" against the
end-goal. Replace it with a **per-task** classification: one entry per task id,
indexed by id (NOT per-core), giving a direct reusability verdict for that
task's RTA.

```cpp
// One classification per task, indexed by task id (aligned with the flat
// `rtas` vector ComputeRTA_FullAndCache returns — exactly what the optimizer
// consumes). Richer than a bool because reuse is not binary: a task whose own
// ET changed needs its RTA recomputed but can still reuse the HP-prefix
// checkpoint; a task downstream of a changed task reuses neither.
enum class RTAReuseClass {
    RtaReuse,                 // cached rta valid verbatim — return it, zero work
    RecomputeWithHpPrefix,    // own rta stale, but hp_tasks_et_conv_vec[p] reusable
    Recompute                 // must recompute from scratch (prefix invalidated)
};
std::vector<RTAReuseClass> ClassifyReuse(
    const std::unordered_map<int, PerCoreRTACache>& cache,
    const DAG_Model& dag_tasks,
    const std::vector<int>& changed_task_ids);
```

- **Three values (not bool)** because the user anticipates ≥3 reuse categories:
  full rta reuse, recompute-with-HP-prefix-reuse, and plain recompute (and
  "other types of reuse" later — e.g. prefix-reuse without the suffix). The
  enum leaves room for those without an API change.
- **Per-task by id** (not per-core): the optimizer builds a flat
  `std::vector<FiniteDist> rta(n_tasks)`, so a flat per-task verdict is exactly
  its consumption shape. `fully_reusable`-on-a-core becomes "every task on that
  core is `RtaReuse`"; the cross-core skip is just "tasks on the untouched core
  are all `RtaReuse`".
- **Reusability is prefix-monotone per core** — RTA at sorted position i
  depends only on ET dists + order of `[0..i]`, so `j<i` reusable ⇒ `j`
  reusable. The monotone `true…true false…false` prefix maps cleanly onto the
  enum (leading `RtaReuse`, then `RecomputeWithHpPrefix`/`Recompute` for the
  suffix). The old `common_prefix_length` is just the count of leading
  `RtaReuse` — derivable, no longer a field.

### (R2) `PerCoreRTACache` is a **class**, not a struct — add member helpers

The staged 3c type is a plain `struct` with public fields. Promote it to a
`class` with private members and accessor + helper methods as needed (the
locked data members are unchanged): `ProcessorId()`, `SortedTaskIds()`,
`TlVec()`, `Rta()`, `HpTasksEtConvVec()` (const refs), plus per-core helpers
that the build + patchers + classifier share — e.g. `Size()`,
`PositionOfTask(int task_id) const`, `ContainsTask(int task_id) const`. The
mutators (`SetRtaForTask`, `UpdateTlAt`, etc.) arrive with the patchers
(steps 4/5); only the read accessors the classifier + build need land now.
Encapsulation centralizes the `sorted_task_ids`↔`rta`↔`tl_vec`↔
`hp_tasks_et_conv_vec` **alignment invariant** (all four are length-n,
parallel arrays indexed by sorted position) in one type instead of trusting
every free function to keep them in lockstep.

### (R3) Drop `CacheConsistentWith` — derivable from the reuse vector

The staged whole-cache predicate `bool CacheConsistentWith(cache, dag, pa_vec,
tl_vec)` (every core `fully_reusable`) is redundant once a per-task reuse
vector exists: "cache exactly matches candidate" = "every task is `RtaReuse`".
`CacheConsistentWith` is removed; callers that need the exact-match short-circuit
call `ClassifyReuse` and check `all == RtaReuse`. This is point (3) of the
user's feedback ("i don't see what's `CacheConsistentWith`, i feel we can
achieve it with the cache re-use vector").

### (R4) `ClassifyReuse` v0 is **simple** — same-processor check only

The FIRST implementation of `ClassifyReuse` is deliberately simple (user: "i
want to keep the first implementation simple, and we'll only check whether the
changed tasks are in the same processor as the task under analysis"):

> For each task `t` under analysis: if ANY changed task shares `t`'s
> `processorId` → `Recompute` (be conservative — assume the worst within the
> same core); else `RtaReuse` (changed tasks are on a different core → `t`'s
> RTA is untouched). Empty cache → all `Recompute`.

This is **coarser than the prefix logic** the staged 3c implemented (it does
not exploit the HP-prefix reuse point within a core yet — every task on a
changed core becomes `Recompute`, even the prefix above the change). That's
intentional: v0 establishes the per-task enum API + the class shape with a
trivially-correct classifier, and the prefix refinement (`RecomputeWithHpPrefix`
for the suffix above a changed position, `RtaReuse` for the unchanged prefix)
lands as a later step once the patchers (4/5) actually consume it. The
`RecomputeWithHpPrefix` value is **declared but not produced by v0** — reserved
for that refinement. v0's correctness is easy to TDD: any task whose core has no
changed task is byte-identical to the cached rta (RtaReuse); the changed core's
tasks recompute (against `ComputeRTA_FullAndCache`'s full recompute).

**Note on v0 conservativeness**: v0 marks the ENTIRE changed core `Recompute`
(loses the within-core prefix reuse). That's fine for v0's purpose (API + shape
+ trivial-correctness TDD) but is NOT the perf payoff — the payoff comes when
the classifier distinguishes prefix (`RtaReuse`) from suffix
(`RecomputeWithHpPrefix`) on the changed core, which is a later step paired
with the patchers. v0 is staged as a correctness scaffold, not a speedup.

### What this changes in the staged 3c surface

- `RTA_Cache.h`: `struct PerCoreRTACache` → `class PerCoreRTACache` (private
  members, read accessors + helpers). `struct CacheReuseInfo` + the
  `AnalyzeCacheReuse`/`CacheConsistentWith` decls → the `enum class
  RTAReuseClass` + `ClassifyReuse` decl. `ComputeRTA_FullAndCache` and the two
  patcher declarations are UNCHANGED.
- `RTA_Cache.cpp`: `AnalyzeCacheReuse`/`CacheConsistentWith` bodies →
  `ClassifyReuse` v0 (same-processor check). `ComputeRTA_FullAndCache` adapted
  to build via the class's setters/Populate. The dead `DeriveCandidatePerCore
  Order`/`CoreOrder` helpers (which served the prefix walk) are removed.
- `testRTA.cpp`: the 4 staged `AnalyzeCacheReuse_*` tests +
  `CacheConsistentWith_*` tests → `ClassifyReuse` v0 tests (identity candidate
  → all `RtaReuse`; TL change on core 0 → core-0 tasks `Recompute`, core-1
  tasks `RtaReuse`; priority move on core 0 → same; empty cache → all
  `Recompute`). The partition-change test is DROPPED from v0 (same-processor
  check has no notion of partition migration; the prefix logic that motivated
  it is deferred — documented above). The build/self-consistency/TL-
  differential tests move to the class accessors.
- `goal.md` + `dev_log.md` + memory: the CacheReuseInfo/AnalyzeCacheReuse/
  CacheConsistentWith references are updated to the enum + class + ClassifyReuse
  v0 form.

---

## API revision 2 — 2026-07-15 (self-supplied cache; HOLD-OFF on RTA_Cache)

> **STATUS (2026-07-15): ON HOLD.** The user found a more-important issue to
> resolve first and wants to hold off on `RTA_Cache`. This section records the
> API-design decisions already made (so they're not lost) + the **remaining
> questions for the user** at the bottom. **No code written** for rev 2 yet —
> design-only. When work resumes, answer the open questions first, then write
> `RTA_Cache.h` to the locked surface, THEN implement 3b.

Five decisions from the user on 2026-07-15 (rev 2), reshaping the cache to be
**self-supplied**. These **supersede** the rev-1 §"API revision" surface above
where they conflict (rev-1's `Populate`/`ClassifyReuse`/`CacheReuseInfo`-free
form is itself superseded by the self-supplied class below). The staged 3c code
on disk still carries the rev-1 surface; the rev-2 rewrite replaces it when
work resumes.

### (Rev2-1) `PerCoreRTACache` is whole-taskset, self-supplied, codebase-consistent

The cache is **one object for the WHOLE task set across ALL cores** (NOT the
rev-1 `std::unordered_map<int, PerCoreRTACache>` — one per core). It mirrors
`ProbabilisticRTA_TaskSet` (RTA.cpp:115), which partitions by `processorId`
internally. Member convention must match the rest of the codebase:

- **`rta_` is FLAT, indexed by TASK ID** — the same order
  `ProbabilisticRTA_TaskSet` returns (it builds `task_id2index[id]=i` from its
  input and scatters by it; under the `dag.tasks[i].id==i` invariant that's
  task-id/dag-position order). NOT the per-core sorted order.
- The cache **stores the priority vector `pa_`** (NOT `sorted_task_ids`).
  `sorted_task_ids` may be DERIVED inside an implementation function from
  `(dag_tasks_, pa_)`, but it is NOT a class member — members stay consistent
  with how the rest of the codebase describes a candidate (dag + pa + tl).
- The cache **stores a copy of the champion `dag_tasks_`** (→ `std::shared_ptr`
  later for efficiency; copy is the v0 stand-in). Benefit: the cache becomes
  **fully self-supplied** — pass a new `(dag_tasks, tl, pa)` triple and it
  returns which tasks' RTA can be reused (as a vector), without the caller
  having to track what changed.

So the locked members are: `dag_tasks_` (copy), `pa_`, `tl_`, `rta_` (flat by
task id). **No `sorted_task_ids_` member.** `hp_tasks_et_conv_vec_` is NOT a
member in v0 — the within-core HP-prefix checkpoint store lands as an INTERNAL
member paired with the `RecomputeWithHpPrefix` refinement (a later step); v0's
cross-core skip needs only `rta_`. (Name: `PerCoreRTACache` is now historical —
it's whole-taskset. Renaming to `RTACache` is OPEN Q1.)

### (Rev2-2) `Initialize(dag_tasks, pa, tl)` replaces `Populate` (cold-start full RTA)

Rename the rev-1 `Populate(processor_id, core_tasks, time_limits)` to a
whole-taskset **`Initialize(dag_tasks, pa, tl)`** — stores the champion triple
and computes the full N-task RTA (applies tl + pa internally, drives the
per-core `ProbabilisticRTA_TaskSet_SingleCore`), writing `rta_`. Overwrites all
prior state. Expensive (full RTA) — call once per interval/champion, NOT per
candidate. (Replaces both rev-1 `Populate` and the per-core map build.)

### (Rev2-3) `UpdateFullCache(dag_tasks, pa, tl, rtas)` — cheap adopt-commit

The user agrees the cache is **bundled with a champion** (issue #1 in the rev-2
critique): the hot loop adopts a candidate as champion on every improvement
(`sp_eval > opt_sp_`), and re-`Initialize`-ing per adopt would do a full RTA
recompute and **defeat the cache**. So add a cheap commit: **`UpdateFullCache
(dag_tasks, pa, tl, rtas)`** stores a pre-computed `rtas` (the ones
`EvaluateRTA_WithCache` just returned for this candidate) + the triple, **no
RTA**. Caller pattern:

```cpp
auto rtas = EvaluateRTA_WithCache(cand_dag, cand_pa, cand_tl, cache);  // read-only
double sp  = AssembleSP_HazardB(cand_dag, rtas, sp_parameters);        // perf_coeff-corrected
if (sp > opt_sp_) { opt_sp_ = sp; opt_pa_ = cand_pa;
                    cache.UpdateFullCache(cand_dag, cand_pa, cand_tl, rtas); }  // cheap
```

Name chosen by the user: unrelated to "champion" (the cache itself is not about
champions). This is the rev-1 critique's `AdoptChampion`, renamed.

### (Rev2-4) `CheckTaskSetRTAReuse(dag_tasks, pa, tl) → vector<RTAReuseClass>`

Member, READ-ONLY (no RTA, no mutation). Classifies every task's RTA reuse
against the stored champion, for the candidate triple. Returns one
`RTAReuseClass` per task, indexed by task id. **Replaces** the rev-1 free
function `ClassifyReuse(cache, dag, changed_task_ids)` — now a member, and the
caller no longer passes `changed_task_ids` (the cache diffs the candidate
triple against its stored champion internally; it derives what changed from
`(dag_tasks_, pa_, tl_)` vs the passed triple).

The user also adds an implementation requirement (point 3): **add a function to
check how many tasks' ET changed** between the stored champion and the
candidate — the cache derives the changed-task set from `(dag, pa, tl)` (ET
changes via tl; pa changes via the priority vector). This is the internal diff
`CheckTaskSetRTAReuse` runs; it doubles as a guard (multi-task diff → broader
`Recompute`). OPEN Q3: is this a separate public member (e.g.
`CountChangedETTasks(dag, tl)`) or purely internal?

v0 classification stays the rev-1 R4 rule: a task is `Recompute` if any changed
task shares its `processorId`, else `RtaReuse`; empty cache → all `Recompute`.
`RecomputeWithHpPrefix` declared, not produced (paired with the prefix
refinement / patchers, a later step).

### (Rev2-5) `GetRTA_OneTask(dag, pa, tl, task_id, reuse_level) → FiniteDist`

Member. Returns task `task_id`'s RTA for the candidate triple, dispatching on
`reuse_level` (which the caller obtained from `CheckTaskSetRTAReuse` on the
SAME triple): `RtaReuse → rta_[task_id]`; `Recompute → full recompute from the
candidate triple`. The selective/inspection API — NOT the hot-loop path (its
`Recompute` branch rebuilds the per-core sorted structure per call,
O(N log N)/task; the hot loop must use the batch `EvaluateRTA_WithCache` with
one partition+sort per candidate). OPEN Q2: the `reuse_level` arg is TRUSTED
(a fabricated/stale level returns a wrong rta silently) — keep the trusted
contract, or have it re-derive internally (safer, slower)? OPEN Q4: the member
name `GetRTA_OneTask` shadows the free `GetRTA_OneTask(task, hp_tasks)` in
RTA.h — rename to `RTAForTask`?

### Outside the class: `EvaluateRTA_WithCache(dag, pa, tl, cache) → vector<FiniteDist>`

Free function, READ-ONLY w.r.t. the cache. The optimizer hot-loop call.
Computes the candidate's full flat RTA — reusing `rta_[task_id]` for `RtaReuse`
tasks, batch-recomputing the `Recompute` tasks with a SHARED per-core sorted
structure (one partition+sort per candidate, not per task) — and returns it.
Does NOT mutate the cache; the caller commits an adopted candidate via
`UpdateFullCache`. Bit-identical to a full recompute by construction; TDD pins
it. (Consolidates rev-1 `ComputeRTA_FullAndCache` + the two patchers into one
entry point — the rev-1 "Unified Cache-Aware API" extra idea, now first-class.)

### `tl` threading at the seam — DECIDED: pass time limits directly (option a)

The API takes `(dag, pa, tl)`. At `OptimizeIncre`'s call site the `dag` arrives
with TLs already baked (`UpdateExtDistBasedOnTimeLimit` at
OptimizeSP_TL_Incre.cpp:146), and `OptimizeIncre` does NOT currently receive a
separate `tl`. The user chose **option (a): pass `time_limits` directly from
outside** (option b, cache-derives-tl-from-baked-dists, REJECTED). Verified
against source 2026-07-15: `EvaluateTimeLimitConfig_ScratchOrIncre`
(OptimizeSP_TL_Incre.cpp:142-146) **takes `time_limits` as a parameter** and
the vector is **in scope at the `:161` call site** (`optimizer.OptimizeIncre
(dag_tasks_cur)`), so threading `tl` into `OptimizeIncre` is a clean pass-down.
So the seam becomes `OptimizeIncre(dag, tl, cache)` (required `cache&` + `tl`,
in addition to `dag`). Confirms the user's point: "during searching for time
limits, we have a time limit vector ready to use" — yes.

> **Note:** the seam's `cache&` is the rev-2 whole-taskset `PerCoreRTACache`
> (one object), NOT the rev-1 per-core map. The §"Step 3b design" wiring shape
> below (which still says `std::unordered_map<int, PerCoreRTACache>
> per_core_rta_cache_`) is STALE w.r.t. rev 2 — it must be rewritten to
> `PerCoreRTACache per_core_rta_cache_;` (one member) when 3b is implemented.

### What rev 2 changes in the staged 3c surface (when work resumes)

- `RTA_Cache.h`: the rev-1 `class PerCoreRTACache` (per-core, `Populate`,
  `SortedTaskIds` accessor, per-core query helpers) → rev-2 whole-taskset
  self-supplied class (`Initialize`, `UpdateFullCache`,
  `CheckTaskSetRTAReuse`, `GetRTA_OneTask`; members `dag_tasks_`/`pa_`/`tl_`/
  `rta_`; NO `sorted_task_ids_` member). Rev-1 free `ClassifyReuse` → member
  `CheckTaskSetRTAReuse`. Rev-1 `ComputeRTA_FullAndCache` + the two patchers →
  free `EvaluateRTA_WithCache` (+ the member pull APIs). The two patcher decls
  (`PatchRTA_OneTaskTL`/`PatchRTA_PriorityMove`) are FOLDED INTO
  `EvaluateRTA_WithCache`'s internal dispatch, not separate free functions.
- `RTA_Cache.cpp`: rewritten to the rev-2 surface.
- `testRTA.cpp`: rev-1 `ClassifyReuse_*` tests → `CheckTaskSetRTAReuse_*` on
  the self-supplied class; the build/differential tests move to `Initialize` +
  `EvaluateRTA_WithCache`.

### Remaining questions for the user (answer before writing RTA_Cache.h)

- **Q1 (name):** `PerCoreRTACache` is now whole-taskset (historical name). Keep
  it, or rename to `RTACache`?
- **Q2 (trusted arg):** `GetRTA_OneTask(..., reuse_level)` — keep the TRUSTED
  contract (level must come from `CheckTaskSetRTAReuse` on the same triple; a
  fabricated/stale level returns a wrong rta silently, fine for a perf cache),
  or re-derive internally (safer, slower)?
- **Q3 (changed-ET count API):** the "check how many tasks' ET changed" helper
  the user asked for — separate public member (e.g.
  `CountChangedETTasks(dag, tl) → int` / `ChangedETTaskIds(...) → vector<int>`),
  or purely internal to `CheckTaskSetRTAReuse`?
- **Q4 (shadow name):** member `GetRTA_OneTask` shadows the free
  `GetRTA_OneTask(task, hp_tasks)` in RTA.h — rename to `RTAForTask` to
  disambiguate, or keep (C++ member/free overload resolution handles it)?
- **Q5 (resume gating):** the "more-important issue to resolve first" — is it
  logged anywhere (an agent folder / a task id), so P1.9 knows what it's
  blocked behind? (If it's a separate task, I'll note the blocker id here.)

---

## The cache value type (pure data, no PA-search state)

> Lives in `RTA_Cache.h` (NOT `RTA.h`) — see "Header layout" below for why the
> relocation breaks the header cycle that forced the step-3a as-built signature.

```cpp
// RTA_Cache.h, alongside the ComputeRTA_FullAndCache declaration.
struct PerCoreRTACache {
    int processor_id = -1;
    // sorted_task_ids[i] = task id at priority index i on this core
    // (sorted by pa_vec priority, descending HP-first). This is the SUFFICIENT
    // proxy for pa_vec: RTA cares about per-core ORDER, not priority VALUES —
    // a cross-core priority shift that preserves each core's internal order
    // changes no core's RTA. So we do NOT store pa_vec (storing the raw values
    // on top of sorted_task_ids would be redundant for validity; it'd only add
    // inspectability, which we're declining here).
    std::vector<int> sorted_task_ids;
    // tl_vec[i] = time limit of sorted_task_ids[i] at cache-build time; -1 = no
    // TL (the immutable base Gaussian dist). ET-DIST VALIDITY PROXY: within an
    // interval, ET dists mutate ONLY via TL application
    // (ApplyTimeLimitsToTasksExecutionTime → GetUnitExecutionTimeDist(tl), which
    // is FiniteDist({{tl, 1.0}}) — a deterministic point mass), so
    //   ET-dist[i] unchanged  <=>  stored tl_vec[i] == current tl_vec[i]
    // (tl==-1 means "still the immutable base dist", which never moves mid-walk).
    // The ONE exception is the one-time WCET ablation (ApplyWCETAblationIfRequired,
    // OptimizeSP_TL_Incre.cpp:487, gated on use_wcet_execution_time) — a setup
    // boundary, not mid-walk; the interval reset (decision 3) covers it.
    std::vector<double> tl_vec;
    // rta[i] = RTA distribution of sorted_task_ids[i]. Length == sorted_task_ids.size().
    std::vector<FiniteDist> rta;
    // hp_tasks_et_conv_vec[i] = HP-ET convolution of sorted_task_ids[0..i).
    //   hp_tasks_et_conv_vec[0] == FiniteDist({Value_Proba(0, 1.0)}).
    //   hp_tasks_et_conv_vec[i] is exactly what the 3-arg GetRTA_OneTask consumes.
    std::vector<FiniteDist> hp_tasks_et_conv_vec;
};
```

Memoized **output**, not search state — that's the property that keeps it off
the P0.5 rejected-challenger drift hazard (`goal.md` §"Cache lifetime"). It's a
pure function of `(per-core taskset, per-core priority order, per-core ET
dists)`, and within an interval the last two reduce to `(sorted_task_ids,
tl_vec)` — the two things the cache now stores.

## Header layout (RTA_Cache.h / RTA_Cache.cpp — NEW)

Cache-related RTA functions live in a **dedicated header/source pair**,
`sources/Safety_Performance_Metric/RTA_Cache.h` + `RTA_Cache.cpp`, NOT in
`RTA.h`/`RTA.cpp`. `RTA_Cache.h` `#include`s `RTA.h` (for `FiniteDist`,
`TaskSet`, the `GetRTA_OneTask`/`SingleCore` primitives) + `DAG_Model.h` (for
`DAG_Model`) + `OptimizeSP_Base.h` (for `PriorityVec`).

This is what un-breaks the header cycle: in step 3a the cache decls sat in
`RTA.h`, but the locked signature needs `PriorityVec` (declared in
`OptimizeSP_Base.h`, which includes `SP_Metric.h`, which includes `RTA.h`) → a
back-edge. Moving the decls to `RTA_Cache.h` (which NOTHING upstream includes)
removes the back-edge: `RTA.h` stays included by `SP_Metric.h` as before, and
`RTA_Cache.h` is a downstream leaf that the optimizer layer pulls in directly.
CMake picks the new `.cpp` up automatically at the next `cmake ..` (top-level
`CMakeLists.txt:22` globs `sources/*.cpp`).

## The free functions (RTA_Cache.h / RTA_Cache.cpp)

Three API roles, kept separate so the optimizer controls commit discipline:

### (1) Full compute + populate — the cache BUILD entry point

```cpp
// Full compute, populates `cache` (one PerCoreRTACache per processorId) AND
// returns the flat rta vector (same shape ProbabilisticRTA_TaskSet returned).
// Replaces the bare ProbabilisticRTA_TaskSet call on the baseline/descent path.
//
// LOCKED SIGNATURE (2026-07-14, step 3c — restores the original design).
// Takes the FULL input tuple (dag, pa_vec, tl_vec) and applies both
// UpdateTaskSetPriorities + ApplyTimeLimitsToTasksExecutionTime internally,
// then drives the 2-arg SingleCore. This needs `PriorityVec`, which is why the
// decls moved to RTA_Cache.h (header-cycle note above).
//
// Why re-apply pa_vec/tl_vec inside instead of taking a pre-prepared TaskSet:
// the caller (EvaluateSPWithPriorityVec) already applies pa_vec (and TLs are
// applied even further upstream), so re-applying here is technically redundant
// — but the cost is a trivial O(N) priority write + dist clone, negligible
// next to the RTA it gates, and it buys (a) the cache fn owning the full input
// tuple natively (so it can store tl_vec without the caller threading it down
// the 4-deep ObtainSP_* chain) and (b) NOT having to thread tl_vec through
// ObtainSP_DAG/ObtainSP_TaskSet signatures (those stay pure SP-scoring fns).
// The TLs must be applied to the BY-ID dag.tasks before priorities are sorted,
// matching the live path's order (ObtainSP_DAG 3-arg → ApplyTimeLimits on the
// by-id TaskSet → ObtainSP_DAG 2-arg → ... → SingleCore sorts by priority).
std::vector<FiniteDist> ComputeRTA_FullAndCache(
    const DAG_Model& dag_tasks,
    const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits,  // -1 = no TL (keep base dist)
    std::unordered_map<int, PerCoreRTACache>& cache);
```

### (2) Reuse-validity query — "can I reuse, and from what prefix?" (READ-ONLY)

> **SUPERSEDED 2026-07-15** by the API revision above (§"API revision"): the
> per-core `CacheReuseInfo`/`AnalyzeCacheReuse`/`CacheConsistentWith` surface
> described here is replaced by the per-task `enum class RTAReuseClass` +
> `ClassifyReuse` (v0 = same-processor check). This section is kept as the
> design history for the prefix logic that the later refinement will reintroduce
> on top of the enum.

The key challenge the user flagged: identifying **when a cache component can be
reused and when it cannot**. This is a first-class, read-only API — it does NO
RTA, just a cheap structural diff of the cache against a candidate
`(dag, pa_vec, tl_vec)`. It returns, **per core**, how much of the cached
prefix is reusable:

```cpp
// Per-core reuse classification of `cache` against a candidate (pa_vec, tl_vec).
// Read-only — never mutates the cache, never runs RTA. Cheap O(M) per core.
struct CacheReuseInfo {
    // true iff this core's cached state exactly matches the candidate
    // (same sorted order AND same tl_vec) → the cached rta is returned verbatim,
    // zero work. Covers the "candidate == champion" short-circuit AND the
    // "change landed on a DIFFERENT core" case (this core untouched).
    bool fully_reusable = false;
    // true iff the SAME set of task ids lives on this core in cache and candidate
    // (same core partition). If false, the cache can't serve this core at all —
    // a task migrated cores, which the single-core patchers can't handle → the
    // caller must full-recompute this core.
    bool same_core_partition = false;
    // Length of the common HP-prefix [0, p) that is reusable verbatim:
    //   hp_tasks_et_conv_vec[0..p) + rta[0..p) are valid for the candidate.
    // p = first position where sorted_task_ids OR tl_vec diverges from the
    // candidate. The suffix [p, n) must be recomputed. For fully_reusable, p==n.
    // This is the reuse point the patchers (role 3) consume as their starting
    // checkpoint. 0 means no reusable prefix (full recompute of this core).
    int common_prefix_length = 0;
};
std::unordered_map<int, CacheReuseInfo> AnalyzeCacheReuse(
    const std::unordered_map<int, PerCoreRTACache>& cache,
    const DAG_Model& dag_tasks,
    const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits);

// Convenience: whole-cache exact-match predicate (every core fully_reusable).
// The "is this candidate identical to what the cache holds?" short-circuit.
bool CacheConsistentWith(
    const std::unordered_map<int, PerCoreRTACache>& cache,
    const DAG_Model& dag_tasks,
    const PriorityVec& priority_assignment,
    const std::vector<double>& time_limits);
```

What the classifier does NOT decide: whether the divergent suffix is a
**single-task** change (patchable) vs a multi-task reorg (not patchable). That
is the **patcher's** job — the patcher knows the intended change (one TL / one
move) and validates the diff matches; if it doesn't, the patcher falls back to
`ComputeRTA_FullAndCache` for that core. So the classifier gives the reuse
**point**; the patcher gives the reuse **verdict** for its specific change type.

### (3) Cache-update patchers — the UPDATE API (Loop A + Loop B)

```cpp
// TL patch (Loop B, no PA change): one task's TL changed to `new_tl` at
// priority position `priority_position` on `core`. Reuses
// hp_tasks_et_conv_vec[priority_position], recomputes rta[p..n) via the 3-arg
// GetRTA_OneTask, returns other cores' cached rta untouched. MUTATES
// cache[core] in place to the candidate's state (suffix rta + tl_vec + the
// rolled-forward hp_tasks_et_conv_vec[p+1..n] are written back).
std::vector<FiniteDist> PatchRTA_OneTaskTL(
    int changed_task_id, int core, int priority_position, double new_tl,
    const DAG_Model& dag_tasks,
    std::unordered_map<int, PerCoreRTACache>& cache);

// Priority-move patch (Loop A, PA change): one task moved old_pos→new_pos on
// `core`. Reuses hp_tasks_et_conv_vec[min(old_pos,new_pos)], replays the suffix
// in the new sorted order. The O(N²)→O(k) payoff. MUTATES cache[core] in place.
std::vector<FiniteDist> PatchRTA_PriorityMove(
    int moved_task_id, int core, int old_pos, int new_pos,
    const DAG_Model& dag_tasks,
    std::unordered_map<int, PerCoreRTACache>& cache);
```

### Cache-update / commit discipline (the user's "locked with champion in pairs")

The patchers are the **update-cache** API — they bring the cache to the
candidate's state as a side effect of computing the candidate's RTA. The
crucial rule (user, 2026-07-14): **the cache is kept in lockstep with the
champion solution — it's a (champion `(pa_vec, tl_vec)`, cache) pair.** So the
cache is NOT auto-updated after every RTA-with-cache call; the update happens
**only when a candidate is promoted to champion**. Consequences:

- The optimizer must NOT let a *rejected* candidate's patch clobber the
  champion cache. Two acceptable patterns (decided at integration, step 3b/4/5):
  (a) patch into a **scratch cache** copy for speculative candidates, swap-in
  only on accept; or (b) evaluate-then-patch — compute the candidate RTA without
  committing, and patch the champion cache only on accept. The cache API does
  NOT enforce this; the optimizer owns the pair discipline.
- `ComputeRTA_FullAndCache` is the build/refresh path (new interval, or a
  non-patchable change → full recompute). It overwrites `cache`, so it is its
  own commit (call it only for the new champion).
- `AnalyzeCacheReuse` (role 2) is how the optimizer decides **per core** whether
  to patch (suffix-reusable), skip (fully-reusable), or full-recompute
  (not-reusable / partition changed) — before any RTA runs.

## Cache storage + validity

```cpp
class OptimizePA_Incre_with_TimeLimits {
    // ...existing members...
    // One per interval. Reset (cleared) at the start of each interval's
    // optimization — never carries across intervals.
    std::unordered_map<int, PerCoreRTACache> per_core_rta_cache_;
};
```

Validity reduces to the `AnalyzeCacheReuse` classification above (no separate
hash). Within an interval the cache is a pure function of
`(sorted_task_ids, tl_vec)` — the two things it stores — so the per-core diff
of those against the candidate IS the validity check:
- `same_core_partition == false` → not reusable (task migrated cores; the
  single-core patchers can't handle it; full-recompute this core). **and**
- `tl_vec[i] == current tl_vec` per position (the ET-dist proxy: within an
  interval ET dists mutate only via TL, so tl-equality ⟺ ET-dist-equality). A
  TL change to a *different* core leaves this core's `tl_vec` untouched → this
  core stays `fully_reusable`. A TL change to *this* core at position `p` makes
  `common_prefix_length == p` (suffix-reusable, not fully). **and**
- the cache was built this interval (interval reset ⇒ full flush; this is also
  what covers the one-time WCET-ablation setup boundary across intervals).

---

## Phase 0 — measure before claiming

- [ ] **Micro-benchmark**: `ProbabilisticRTA_TaskSet_SingleCore` cost vs
      tasks-per-core M at M=3,5,8 (N=6/10/16 with 2 balanced cores), and the
      split of convolve-cost vs `ResolvePreemptionsAndCompress` while-loop
      cost. Built DEBUG (`cmake -DCMAKE_BUILD_TYPE=DEBUG ..` +
      `cmake --build . --target check.SP_OPT -j5`; lib is
      `libSP_OPTDebug.so`). Confirms convolve-count is the dominant term
      before investing in the cache. Creates `tests/benchConvolve.cpp` (the
      ghost entry in git status — file does not yet exist on disk).

## Phase 1 — the cache infrastructure (two sub-tasks)

- [x] **HP-prefix checkpoint store** (the patching primitive) — **LANDED
      2026-07-13**. In `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp`),
      instead of letting `hp_tasks_et_conv` be a single rolling value that
      gets discarded, snapshot it into `hp_tasks_et_conv_vec[i]` = HP-prefix
      convolution of tasks `[0, i)` at the top of each iteration `i`. The
      3-arg `GetRTA_OneTask` already consumes this value. TDD-verified:
      `HpTasksEtConvVec_MatchesRollingValue` (independently replays the rolling
      value, bit-identical) + `TwoArgOverload_SameRtasAsOneArg` (2-arg
      returns bit-identical `rtas` to 1-arg); the 6 pre-existing pinned-`rtas`
      tests are the behavior-preservation oracle (1-arg now delegates to
      2-arg). `check.SP_OPT` 16/16 green. Pure refactor — no behavior change
      to any returned `rtas`. Implementation: 2-arg overload
      `(tasks, hp_tasks_et_conv_vec&)` holds the logic; 1-arg is a thin
      wrapper. See `dev_log.md` §2026-07-13 (step 2).
- [~] **Per-core RTA cache** (the storage + the full-compute-and-cache entry
      point) — **step 3a LANDED 2026-07-14 (staged, NOT committed)** with the
      *as-built* signature `ComputeRTA_FullAndCache(const TaskSet&, cache&)` in
      `RTA.h`/`RTA.cpp` + 2 differential tests. **SUPERSEDED by step 3c** (the
      relocation + signature restore) — see below. Step 3a's value was proving
      the cache fn mirrors `ProbabilisticRTA_TaskSet` bit-identically; step 3c
      re-hosts that same logic at the locked signature in `RTA_Cache.h`/`.cpp`.
- [x] **Step 3c LANDED 2026-07-14 (staged, NOT committed)** — relocate to
      `RTA_Cache.h`/`.cpp`, restore the locked 4-arg sig
      `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` (applies pa + tl
      internally, option 1), add the `tl_vec` field to `PerCoreRTACache`, add
      the read-only `AnalyzeCacheReuse`/`CacheConsistentWith` reuse-query API,
      add the two header decls (`ApplyTimeLimitsToTasksExecutionTime` →
      `SP_Metric.h`, `ExtractTaskSetPerProcessor` → `RTA.h`). Patchers declared
      (locked sigs) but defined in steps 4/5. 8 TDD tests (differential vs
      oracle + self-consistency incl. `tl_vec` alignment + TL differential +
      4 `AnalyzeCacheReuse` scenarios + `CacheConsistentWith`). 16/16 ctest
      green (DEBUG). See `dev_log.md` §2026-07-14 (step 3c).
      **REVISED 2026-07-15 (before commit): the role-2 API was redesigned per
      the §"API revision" above — `CacheReuseInfo`/`AnalyzeCacheReuse`/
      `CacheConsistentWith` are REPLACED by the per-task `enum class
      RTAReuseClass` + `ClassifyReuse` (v0 = same-processor check), and
      `PerCoreRTACache` is promoted from struct to class with accessors. The
      3c tests move to the new API. See `dev_log.md` §2026-07-15 (API revision).**

## Phase 1.5 — wire the cache into the live eval path (step 3b, baseline-only)

- [ ] **Step 3b — baseline-only cache wiring + retire oracle from the
      incremental path.** Per the 2026-07-14 scope decision: add the
      `per_core_rta_cache_` member to `OptimizePA_Incre_with_TimeLimits` (reset
      per interval in `ResetIncumbentBaseline`); add the cache-aware eval seam
      (**DECIDED 2026-07-15 user: a REQUIRED `PerCoreRTACache&` param on
      `OptimizeIncre` — always active, no separate fn, no optional pointer**;
      primary target is `OptimizeIncre` `:239`/`:279`, the per-candidate O(N²)
      sites) that drives `ComputeRTA_FullAndCache` for the node RTAs + the
      **perf_coefficient-corrected** chain assembly (Hazard B); pass the cache
      down at the `:161` incremental call site in
      `EvaluateTimeLimitConfig_ScratchOrIncre` so `OptimizeIncre` takes the
      cache-aware route. **Scope refinement (2026-07-15): the cache benefits the
      incremental branch's per-candidate evals (`:239`/`:279`); the from-scratch
      branch's beam search (`UpdateSP`→`GetRTA_OneTask`) bypasses
      `ProbabilisticRTA_TaskSet` and is NOT cache-replaceable — only its single
      `:136` final eval is, which is secondary (signature unchanged at 3b).**
      Retire `ProbabilisticRTA_TaskSet` / `ObtainSP_TaskSet` from the incremental
      optimizer's path ONLY (BF/TL_BF keep the free-fn path for now). No patching
      yet — the cache is built per candidate (full compute + populate), reuse is
      steps 4/5. **Two integration hazards the wiring must resolve** (see
      "Step 3b design" §Hazard A/B below): (A) the cache owner (derived class)
      and the PA-loop call sites (base class, sliced inner optimizer) are on
      different classes → the seam threads the cache in by parameter, NOT via a
      derived-class method or virtual override; (B) `ObtainSP_DAG_From_Dists`
      omits `perf_coefficient` from the node term → the assembly inlines it
      (NOT a bare `ObtainSP_DAG_From_Dists` call). Full design + call-chain map
      + hazard analysis in the build-order 3b entry above.
      TDD: differential — cache-aware eval SP bit-identical to
      `EvaluateSPWithPriorityVec` on the same `(dag_with_TLs, pa_vec)` across a
      sweep; fixture MUST include a perf-pair task + a chain (else Hazard B
      hides). DEBUG build, ctest 16/16 + new differential green.

## Phase 2 — dispatch the cache in the two hot loops

- [ ] **TL patch dispatch** (Loop B, the no-PA-change path — **implement
      first**). Wire `PatchRTA_OneTaskTL` into the TL walk
      (`OptimizeSingleTaskTimeLimit`, `OptimizeSP_TL_Incre.cpp:194-239`,
      driven by `PerformCoordinateDescentForTaskConfigOpt` `:241-281`). On a
      TL change to the task at priority position `p` on core A: reuse
      `cache[A].hp_tasks_et_conv_vec[p]`, recompute `rta[p..n)` via the 3-arg
      `GetRTA_OneTask`, return cached `rta` for all other cores. Low risk —
      no reordering, HP-prefix structure unchanged. TDD: differential vs
      `ComputeRTA_FullAndCache` (full recompute) across a sweep of TL moves
      (every task, every TL option, both directions); bit-identical.
- [ ] **Priority-move patch dispatch** (Loop A, the PA-change path — the
      O(N²) payoff). Wire `PatchRTA_PriorityMove` into the 1D priority loop
      (`OptimizeIncre`, `OptimizeSP_Incre.cpp:269-287`; variations from
      `FindPriorityVec1D_Variations` `:180-212`). On a priority move of one
      task from `old_pos` to `new_pos` on core A: reuse
      `cache[A].hp_tasks_et_conv_vec[min(old_pos, new_pos)]`, replay the suffix
      `[min(old_pos,new_pos), n)` in the new sorted order. Higher risk —
      reordering changes which tasks are HP for which; the TDD oracle (full
      recompute) is the safety net. TDD: differential vs
      `ComputeRTA_FullAndCache` across a sweep of priority moves (every
      task, every target position in the search half); bit-identical.
- [ ] **End-to-end scalability measurement**. Re-run the Phase-0 micro-bench
      with patching wired in, at N=6/10/16. Confirm the ~3-4× candidate-cost
      reduction predicted in `dev_log.md` §2026-07-13 on the dominant O(N²)
      term. Record actual numbers (revise the estimate if
      `ResolvePreemptionsAndCompress` dominates instead).

## Build order (descriptive, no opaque labels)

1. Phase 0 micro-bench — measure first, settles convolve vs
   `ResolvePreemptionsAndCompress` as the dominant term.
2. HP-prefix checkpoint store — smallest, pure refactor of `RTA.cpp:58-85`;
   the primitive both patchers reuse.
3. Per-core RTA cache — `PerCoreRTACache` + `ComputeRTA_FullAndCache`,
   replaces `ProbabilisticRTA_TaskSet` on the baseline path.
   - **3a** (DONE, staged): struct + free function + differential test at the
     *as-built* sig `(const TaskSet&, cache&)` in `RTA.h`/`RTA.cpp`. Proved
     bit-identical to the oracle. No eval-path wiring.
   - **3c** (DONE 2026-07-14, staged — supersedes 3a): relocated
     `PerCoreRTACache` + the cache API to a **new** `RTA_Cache.h`/`RTA_Cache.cpp`,
     **restored the locked signature** `ComputeRTA_FullAndCache(dag, pa_vec,
     tl_vec, cache)` (applies pa + tl internally — option 1), **added `tl_vec`
     to `PerCoreRTACache`** (ET-dist validity proxy), added the **read-only
     `AnalyzeCacheReuse` / `CacheConsistentWith` reuse-query API** (returns
     per-core reuse info, never runs RTA), and added header declarations for
     `ApplyTimeLimitsToTasksExecutionTime` (in `SP_Metric.h`) +
     `ExtractTaskSetPerProcessor` (in `RTA.h`). The patchers
     (`PatchRTA_OneTaskTL`/`PatchRTA_PriorityMove`) are **declared** (locked
     sigs) but **defined** in steps 4/5 (Loop B/A). 8 TDD tests; 16/16 ctest
     green. Separate review-and-commit cycle.
   - **3b** (after 3c): wire `ComputeRTA_FullAndCache` into the baseline/descent
     eval path. **Per the 2026-07-14 scope decision, 3b is BASELINE-ONLY:**
     wire the cache into the incremental optimizer's eval path (the
     `OptimizePA_Incre`-via-`EvaluateTimeLimitConfig_ScratchOrIncre` path),
     retire `ProbabilisticRTA_TaskSet` from **that path only**; leave the
     shared free-fn path (`OptimizePA_BF` / `OptimizeSP_TL_BF`) on the old
     oracle for now. No patching yet — the cache is BUILT per candidate (full
     compute + populate, role 1), not patched (patching is steps 4/5). **The
     2026-07-14 second pass grounded two integration hazards the wiring must
     resolve** (Hazard A: cache owner on the derived class vs PA loop on the
     base class with a sliced inner optimizer → seam threads the cache by
     parameter; Hazard B: `ObtainSP_DAG_From_Dists` omits `perf_coefficient`
     → assembly inlines it). **2026-07-15 refinement: the seam's primary
     target is `OptimizeIncre` `:239`/`:279` (the per-candidate O(N²) sites);
     `OptimizeFromScratch` `:136` is a single final eval (secondary), and its
     beam-search internals bypass `ProbabilisticRTA_TaskSet` entirely so are
     out of scope. **Seam shape DECIDED 2026-07-15 (user): cache is ALWAYS
     ACTIVE in the incremental optimizer via a REQUIRED `PerCoreRTACache&` param
     on `OptimizeIncre` — no separate free fn (S2 rejected), no optional pointer
     (S1-as-optional rejected); the required ref satisfies `agent_coding_rules.md`
     L3/L10. Storage on the derived owner (NOT base — Hazard A: the `:160`
     challenger is sliced every candidate, a base-member cache would die each
     candidate and kill step 4 reuse), threaded by parameter.** Full design +
     the call-chain map + hazard analysis below. Separate review-and-commit cycle.

### Step 3b design — the call chain 3b rewires (code-grounded 2026-07-14)

The live eval path that 3b rewires, traced end-to-end:

```
OptimizePA_Incre_with_TimeLimits::EvaluateTimeLimitConfig_ScratchOrIncre   (TL-walk per-candidate eval; OptimizeSP_TL_Incre.cpp:142)
  ├─ dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)  // bakes TLs into dag.tasks[].execution_time_dist (OptimizeSP_TL_BF.cpp:6 — IDENTICAL to ApplyTimeLimitsToTasksExecutionTime)
  ├─ from_scratch:  OptimizePA_Incre optimizer(dag_tasks_cur, ...); optimizer.OptimizeFromScratch(K)   // fresh beam search
  └─ incremental:   OptimizePA_Incre optimizer = BuildChallengerFromIncumbent(); optimizer.OptimizeIncre(dag_tasks_cur)  // THROWAWAY challenger (P0.5), rebuilt every candidate
       │
       ├─ INCREMENTAL branch — OptimizeIncre(dag_tasks_cur)  [OptimizeSP_Incre.cpp:233]  ← THE CACHE TARGET (Loop A hot path)
       │    ├─ baseline:   EvaluateSPWithPriorityVec(dag_tasks_cur, sp_parameters_, opt_pa_)        // :239
       │    └─ PER VARIATION (FindPriorityVec1D_Variations — up to N per changed task = the O(N²) term):
       │         EvaluateSPWithPriorityVec(dag_tasks_cur, sp_parameters_, priority_assignment)      // :279  — shared free fn (OptimizeSP_Base.cpp:148)
       │           ├─ tasks_eval = UpdateTaskSetPriorities(dag.tasks, pa_vec)   // applies pa_vec ONLY (TLs already baked upstream at :146)
       │           └─ ObtainSP_DAG(dag_eval, sp)                                // SP_Metric.cpp:89
       │                ├─ ObtainSP_TaskSet(dag.tasks, sp) → ProbabilisticRTA_TaskSet(tasks)   // SP_Metric.cpp:53 → RTA.cpp:115  ← THE ORACLE TO RETIRE (this path only)
       │                └─ GetRTDA_Dist_AllChains<ObjReactionTime>(dag)        // chain RT dists — SEPARATE dist source, NOT RTA, untouched by the cache
       │
       └─ FROM-SCRATCH branch — OptimizeFromScratch(K)  [OptimizeSP_Incre.cpp:74]  (the reopt path; from_scratch=true at TL_Incre.cpp:152)
            ├─ BEAM SEARCH (per partial-path expansion, :83-124): PriorityPartialPath::UpdateSP(task_id)   // OptimizeSP_Incre.cpp:46-62
            │    └─ GetRTA_OneTask(tasks[task_id], hp_tasks)   // 2-arg, RTA.cpp:31 — hp_tasks filtered by processorId in-place (:50-52)
            │       └─ perf_coefficient ALREADY inlined as effective_weight = weight*perf_coeff (:55-60)
            │          → BYPASSES ProbabilisticRTA_TaskSet entirely → NOT cache-replaceable at 3b
            │             (the cache mirrors ProbabilisticRTA_TaskSet_SingleCore; this is a different code path)
            └─ FINAL EVAL (ONCE per call): EvaluateSPWithPriorityVec(dag_tasks_, sp_parameters_, opt_pa_)   // :136 — shared free fn → ObtainSP_DAG → ProbabilisticRTA_TaskSet
               → cache-replaceable, but it's a SINGLE call per OptimizeFromScratch, NOT per-candidate;
                  the beam-search cost above is what dominates from-scratch, and it is UNTOUCHED by the cache at 3b
```

**Scope consequence (code-grounded 2026-07-15):** the cache's 3b benefit is
concentrated in the **incremental** branch (`OptimizeIncre` `:239`/`:279` — exactly
where the O(N²) Loop-A term lives, so it's the right place). The **from-scratch/reopt**
branch gets cache benefit ONLY at its single final eval (`:136`); its beam-search
internals (`UpdateSP` → `GetRTA_OneTask`) bypass `ProbabilisticRTA_TaskSet` and are
out of scope for this cache. Idea 11 targets the per-candidate eval cost; the from-
scratch beam search is a separate cost not addressed here (and not the target).
1. **TLs are baked upstream, pa_vec applied at the eval fn.**
   `UpdateExtDistBasedOnTimeLimit` (OptimizeSP_TL_BF.cpp:6) IS
   `ApplyTimeLimitsToTasksExecutionTime` (SP_Metric.cpp:70) — both set
   `execution_time_dist = GetUnitExecutionTimeDist(tl[i])` for `tl[i]!=-1`.
   It runs in `EvaluateTimeLimitConfig_ScratchOrIncre` (OptimizeSP_TL_Incre.cpp:146)
   BEFORE the challenger is built. `EvaluateSPWithPriorityVec` (OptimizeSP_Base.cpp:148)
   applies pa_vec only. So at the RTA plug-in point (`ObtainSP_TaskSet`) the tl_vec
   is already gone (baked into ET dists) — exactly why step 3c's option-1
   `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` re-applies BOTH internally
   (it owns the full input tuple natively, recovering tl_vec for storage without
   threading it down the 4-deep `ObtainSP_*` chain).
2. **The cache can't live on the challenger — and the PA loop is on the BASE class.**
   The challenger (`OptimizePA_Incre`, the BASE class) is rebuilt from `res_opt_`
   every `EvaluateTimeLimitConfig_ScratchOrIncre` call (P0.5
   `BuildChallengerFromIncumbent`, OptimizeSP_TL_Incre.cpp:394 — constructs
   `OptimizePA_Incre challenger(...)`, sliced to base). A cache member on it would be
   discarded every candidate. The cache MUST live on the outer
   `OptimizePA_Incre_with_TimeLimits` (the DERIVED class, decision 3) — BUT the
   `EvaluateSPWithPriorityVec` call sites are in the BASE class's
   `OptimizeIncre` (OptimizeSP_Incre.cpp:239 baseline, :279 per-variation — the
   per-candidate hot sites, the O(N²) Loop-A term) and `OptimizeFromScratch`
   (OptimizeSP_Incre.cpp:136 — a SINGLE final eval, NOT per-candidate; see the
   call-chain map's FROM-SCRATCH branch). So the cache owner (derived) and the call
   sites (base, sliced inner optimizer) are on different classes — the crux Hazard A
   resolves. Threading the cache in by parameter (the decided seam — a required
   `PerCoreRTACache&` on `OptimizeIncre`) reaches the call sites without inheritance
   surgery; a derived-class method or virtual override does NOT
   (the inner object is sliced to base, so the vptr is the base vtable, and the loop
   calls the free fn directly). Note: `OptimizeFromScratch`'s beam-search internals
   (`PriorityPartialPath::UpdateSP` → `GetRTA_OneTask` 2-arg) do NOT call
   `EvaluateSPWithPriorityVec` at all — they bypass `ProbabilisticRTA_TaskSet`, so the
   cache (which mirrors it) cannot replace them; only the `:136` final eval is in
   scope.
3. **`EvaluateSPWithPriorityVec` is shared infrastructure.** It's a free fn called by
   `OptimizePA_Incre` (OptimizeSP_Incre.cpp:136,239,279), `OptimizePA_BF`
   (OptimizeSP_BF.cpp:19), and `OptimizePA_Incre_with_TimeLimits::ResetIncumbentBaseline`
   (OptimizeSP_TL_Incre.cpp:424,436). Baseline-only scope (the 2026-07-14 decision)
   means 3b does NOT force BF/TL_BF through the cache; the shared free fn stays, and
   the incremental path gets a cache-aware route beside it.
4. **Chain RTDA is orthogonal to the RTA cache.** `ObtainSP_DAG` (SP_Metric.cpp:96-107)
   adds the path-latency SP term from `GetRTDA_Dist_AllChains`, a separate distribution
   source. `ObtainSP_DAG_From_Dists` (SP_Metric.cpp:129) already assembles the full SP
   from PRECOMPUTED node-RT dists + chain dists. **BUT it is NOT bit-identical to the
   oracle (Hazard B below) — do NOT reuse it verbatim; the corrected assembly inlines
   `perf_coefficient`.** Compute per-node RTAs via `ComputeRTA_FullAndCache` (replacing
   `ProbabilisticRTA_TaskSet`), then combine with chain RTDA via the **perf_coefficient-
   corrected** assembly (Hazard B), NOT a bare `ObtainSP_DAG_From_Dists` call.

**Two integration hazards the wiring must resolve (code-grounded 2026-07-14,
second pass):**

**Hazard A — the cache owner and the PA loop are on different classes.** The cache
member lives on the **derived** `OptimizePA_Incre_with_TimeLimits` (decision 3). But
the per-candidate `EvaluateSPWithPriorityVec` call sites live in the **base**
`OptimizePA_Incre`: `OptimizeFromScratch` (OptimizeSP_Incre.cpp:136) and
`OptimizeIncre`'s 1D variation loop (OptimizeSP_Incre.cpp:239,279). The inner
optimizer constructed in `EvaluateTimeLimitConfig_ScratchOrIncre` is a **base**
`OptimizePA_Incre` — `BuildChallengerFromIncumbent` (OptimizeSP_TL_Incre.cpp:398)
constructs `OptimizePA_Incre challenger(...)` (sliced to base), and the from-scratch
branch (OptimizeSP_TL_Incre.cpp:151) constructs `OptimizePA_Incre optimizer(...)`
too. So:
- A cache-aware method on the **derived** class is **unreachable** from the base-class
  PA loop (the inner object is sliced to base).
- A `virtual` override of the eval won't fire either: the inner objects are
  constructed as base `OptimizePA_Incre`, so the vptr points at the base vtable, and
  `EvaluateSPWithPriorityVec` is a **free function** the loop calls directly, not a
  member the loop dispatches through.
- The cache member is therefore **not visible** at the PA-loop call sites as written.

This is the real crux 3b solves. **Seam shape DECIDED 2026-07-15 (user): the cache is
ALWAYS ACTIVE in the incremental optimizer — no separate function, no optional knob.**
Realized as a **required `PerCoreRTACache&` parameter on `OptimizePA_Incre::OptimizeIncre`**
(by reference, not a pointer). Storage stays on the durable derived owner
`OptimizePA_Incre_with_TimeLimits` (`per_core_rta_cache_`), threaded down at the `:161`
production call site. This is neither S1 (optional pointer) nor S2 (separate free fn)
as originally framed — it's the required-param form that satisfies all three of the
user's constraint + the codebase structure + `agent_coding_rules.md` L3/L10 at once:

  - **No separate function** → S2's `EvaluateSPWithCache` free fn is REJECTED. The
    cache-aware routing lives inside `OptimizeIncre` itself (replacing its two
    `EvaluateSPWithPriorityVec` calls at `:239`/`:279` with `ComputeRTA_FullAndCache`
    + the perf_coefficient-corrected assembly, Hazard B).
  - **Always active** → there is NO non-cache code path within `OptimizeIncre`. Every
    call routes node RTAs through `ComputeRTA_FullAndCache`. (The shared free fn
    `EvaluateSPWithPriorityVec` is UNCHANGED and still used by `OptimizePA_BF` /
    `OptimizeSP_TL_BF` / `ResetIncumbentBaseline` — those classes never call
    `OptimizeIncre`, so they're untouched; verified `OptimizePA_BF` is a separate
    `OptimimizePA_Base` subclass with no `OptimizeIncre`.)
  - **Required, not optional** → a `PerCoreRTACache&` (ref, not `PerCoreRTACache*`)
    satisfies L3/L10 ("reduce optional args... raise an error if something important
    is not passed"; "do not make things optional if not needed"). There is no nullptr
    fallback and no default — the caller MUST supply a cache.

**Two mechanical consequences the codebase structure forces (NOT new design choices):**

  - **Storage MUST be on the derived owner, threaded by parameter — NOT a base-class
    member.** The `:160` challenger is `OptimizePA_Incre optimizer = BuildChallengerFromIncumbent();`
    where `BuildChallengerFromIncumbent` (`:394`) returns base `OptimizePA_Incre` BY
    VALUE → the challenger is sliced + rebuilt every candidate. A base-class
    `per_core_rta_cache_` member would die every candidate and **kill step 4's
    cross-candidate/cross-core reuse** (the entire point of the cache). So the durable
    cache lives on `OptimizePA_Incre_with_TimeLimits` and is threaded into the sliced
    challenger's `OptimizeIncre` by reference. This is exactly Hazard A.
  - **The direct base-class test must pass a local cache.** `testOptimizeIncrePA.cpp:258`
    constructs a bare `OptimizePA_Incre opt(dag_tasks, sp_parameters)` (no derived owner)
    and calls `opt.OptimizeIncre(dag_tasks_update)`. Under the required-param signature
    this test must construct a local `std::unordered_map<int, PerCoreRTACache>` and pass
    it. At 3b's build-per-candidate semantics (role 1, no reuse yet) a per-call local
    cache is bit-identical to the oracle, so the test stays a valid differential oracle;
    the cross-candidate reuse (step 4) is what requires the durable owner, exercised by
    the TL-walk tests not this unit test.

**Seam scope (2026-07-15): the per-candidate hot sites are `OptimizeIncre` `:239`/`:279`
only; `OptimizeFromScratch` `:136` is a single final eval (not per-candidate), so
threading the cache there is low-value — the from-scratch path's cost is its beam search,
which the cache can't touch. So the seam's PRIMARY target is `OptimizeIncre`;
`OptimizeFromScratch:136` is a secondary, optional target (its signature is UNCHANGED
at 3b).**

  - **(S3) Move the PA loop onto the derived class** (override `OptimizeIncre`/
    `OptimizeFromScratch` to call the cache-aware eval). Rejected as too invasive for
    3b: it duplicates the PA-loop logic on the derived class and forks the two paths;
    the decided required-param seam achieves the same routing with a single change.

**Hazard B — `ObtainSP_DAG_From_Dists` omits `perf_coefficient` (NOT bit-identical
to the oracle).** The oracle path is `ObtainSP_DAG` (SP_Metric.cpp:89) →
`ObtainSP_TaskSet` (SP_Metric.cpp:53), whose node term (SP_Metric.cpp:61-65)
multiplies by `tasks[i].GetPerfCoefficient()`. `ObtainSP_DAG_From_Dists`
(SP_Metric.cpp:129-149) instead calls `ObtainSP` (SP_Metric.cpp:11) for the node
term, which has **no** `perf_coefficient` factor. `GetPerfCoefficient`
(RegularTasks.h:83) returns `GetPerfTerm(timePerformancePairs, avg_et)` for any task
WITH perf pairs (the TL-optimizable tasks — exactly the ones in the cache's scope),
and `1.0` only when pairs are empty. So routing the cache-aware node RTAs through a
bare `ObtainSP_DAG_From_Dists` would **diverge** from the oracle for every perf-pair
task. The corrected assembly must inline the node term with `perf_coefficient`:
```
sp_overall = Σ_i ObtainSP(rtas[i], tasks[i].deadline, thresholds_node[i],
                          weights_node[i] * tasks[i].GetPerfCoefficient())
           + Σ_chain ObtainSP(path_latency_dists[c], chains_deadlines_[c],
                              thresholds_path[c], weights_path[c])
```
— i.e. a new helper (or an overloaded `ObtainSP_DAG_From_Dists` variant) that matches
`ObtainSP_TaskSet`'s node term exactly. The chain term is unchanged (the oracle's
chain term has no perf coefficient — verify `ObtainSP_DAG` SP_Metric.cpp:100-107).
This is the **correctness gate** for 3b: the differential test MUST use perf-pair
tasks, or Hazard B hides.

**Wiring shape (3b, baseline-only; seam DECIDED 2026-07-15 = required `PerCoreRTACache&`
param on `OptimizeIncre`, always active, no separate fn):**
1. Add `std::unordered_map<int, PerCoreRTACache> per_core_rta_cache_;` to
   `OptimizePA_Incre_with_TimeLimits` (the durable owner — NOT the base class; see
   Hazard A: the `:160` challenger is sliced + rebuilt every candidate, so a
   base-member cache would die each candidate and kill step 4 reuse). Reset
   (`.clear()`) at the start of each interval's search — `ResetIncumbentBaseline` is
   the natural site (runs before the descent; both branches). At 3b the cache is
   **built fresh per candidate** (full compute + populate, role 1); reuse/patching is
   steps 4/5.
2. Change `OptimizePA_Incre::OptimizeIncre`'s signature to take a **required**
   `PerCoreRTACache&` (ref, not pointer, not optional) in addition to
   `dag_tasks_update`. Internally its two `EvaluateSPWithPriorityVec` calls (`:239`
   baseline, `:279` per-variation) are replaced by the cache-aware route: drive
   `ComputeRTA_FullAndCache` for the node RTAs + the **perf_coefficient-corrected**
   assembly (Hazard B) for the chain term → returns SP. It does NOT call
   `ObtainSP_DAG`/`ObtainSP_TaskSet`/`ProbabilisticRTA_TaskSet`. Always active — no
   nullptr branch, no separate free fn.
3. At the inner-optimizer call site in `EvaluateTimeLimitConfig_ScratchOrIncre`
   (OptimizeSP_TL_Incre.cpp:161 incremental — the primary site), pass
   `per_core_rta_cache_` down: `optimizer.OptimizeIncre(dag_tasks_cur, per_core_rta_cache_)`.
   The `:151` from-scratch site is UNCHANGED at 3b (`OptimizeFromScratch` keeps its
   signature; only its single `:136` final eval could later benefit, secondary). The
   `dag_tasks_cur` already has TLs baked (:146); `ComputeRTA_FullAndCache` re-applies
   them internally (step 3c option 1) — the double-apply is idempotent (point mass at
   the same TL) and harmless, already the case for `ComputeRTA_FullAndCache`'s
   contract.
4. The shared free fn `EvaluateSPWithPriorityVec` is UNCHANGED — `OptimizePA_BF`,
   `OptimizeSP_TL_BF`, and `ResetIncumbentBaseline` (:424,:436) keep it. **Retirement
   is path-local:** `ObtainSP_TaskSet` / `ProbabilisticRTA_TaskSet` stop being called
   from the incremental optimizer's path; the free fn + BF/TL_BF keep them. Decision
   4's "no fallback" applies to the incremental path only at 3b; full retirement is
   deferred until steps 4/5 land the patchers (a dead free-fn path kept for BF is
   acceptable until then — flagged for step 6's end-to-end review).

**TDD for 3b:** differential — the cache-aware eval's SP output is bit-identical to
the existing `EvaluateSPWithPriorityVec` on the same `(dag_with_TLs, pa_vec)` across
a sweep (several pa_vecs incl. the champion + a TL change). **The fixture MUST include
at least one task with non-empty `timePerformancePairs` (so `GetPerfCoefficient() !=
1.0`) AND a non-trivial chain** — else Hazard B (the `perf_coefficient` omission)
hides and the differential passes for the wrong reason. Since
`ComputeRTA_FullAndCache` is already pinned bit-identical to `ProbabilisticRTA_TaskSet`
(step 3c test), the 3b differential pins the *assembly* (node RTAs + chain RTDA +
perf_coefficient → SP) end-to-end — which is exactly where Hazard B lives. Built DEBUG
(`cmake -DCMAKE_BUILD_TYPE=DEBUG ..` + `cmake --build . --target check.SP_OPT -j5`;
lib is `libSP_OPTDebug.so`); ctest 16/16 + the new differential green.
4. Step 3b — baseline-only cache wiring + retire the oracle from the
   incremental path (see Phase 1.5 + the build-order 3b entry above; Hazard A/B
   analysis in "Step 3b design").
5. TL patch dispatch (Loop B, no PA change) — the path the user wants first;
   low risk.
6. Priority-move patch dispatch (Loop A, PA change) — the O(N²) payoff.
7. End-to-end measurement at N=6/10/16.

Each step is one review-and-commit cycle per `agent_coding_rules.md`.

## Done when
- [ ] All of: HP-prefix checkpoint store, per-core RTA cache, baseline-only
      cache wiring (3b), TL patch dispatch, priority-move patch dispatch —
      landed with `cmake --build . --target check.SP_OPT -j5` green + `ctest`
      16/16, each patch TDD-verified bit-identical to the full-recompute
      oracle; **and** the end-to-end measurement records the actual N=6/10/16
      speedup (or documents why it fell short). The old
      `ProbabilisticRTA_TaskSet` / `ProbabilisticRTA_TaskSet_SingleCore`
      direct-call path is removed from the incremental optimizer's path
      (retired there per decision 4); the shared free-fn path stays for BF/
      TL_BF until step 7's end-to-end review decides whether full retirement
      is warranted.

---

## Extra Ideas & Refinements (Proposed 2026-07-15)

To maximize the performance gains and code safety of the caching layer, the following optimizations and API refinements are added to the backlog:

- [ ] **Unified Cache-Aware API (`EvaluateRTA_WithCache`)**:
  - **Concept**: Consolidate `ComputeRTA_FullAndCache`, `PatchRTA_OneTaskTL`, and `PatchRTA_PriorityMove` into a single `EvaluateRTA_WithCache` entry point.
  - **Tasks**:
    - Implement a single function that queries `ClassifyReuse` (the per-task `RTAReuseClass` vector — see §"API revision") to identify each task's reuse status.
    - Automatically choose to reuse (0 cost), replay the core suffix starting at the mismatch index `p`, or perform a full recompute (for task migrations).
    - Simplify the optimizer-facing seam to take just the `(dag, pa_vec, tl_vec, cache)` tuple, removing manual tracking of changed core/priority indices.
- [ ] **Zero-Copy Candidate Order Derivation**:
  - **Concept**: The staged `DeriveCandidatePerCoreOrder` performed multiple copies of full `Task` structures (which contain names and heap-allocated `FiniteDist` arrays). Removed in the 2026-07-15 API revision (v0 `ClassifyReuse` needs no candidate-order derivation — just `processorId` comparison).
  - **Tasks**:
    - When the prefix-refined `ClassifyReuse` (the later step that reintroduces HP-prefix reuse) needs the candidate's per-core sorted order, derive it working entirely with task ID integers and time limit doubles.
    - Prevent copying any `Task` or `FiniteDist` objects during variation evaluations, keeping cache validity checks O(N) and allocation-free.
- [ ] **Allocation-Free Flat RTA Rebuilding**:
  - **Concept**: When flattening the cached core distributions to match the prioritized task set order, avoid hash-map allocations.
  - **Tasks**:
    - Implement rebuilding using a contiguous array indexed by task ID, mapping cached `rta` distributions to the flat output vector in a cache-friendly, O(N) manner.
- [ ] **Speculative Cache Copying**:
  - **Concept**: Ensure speculative evaluations do not pollute the champion's cache by using the "scratch copy on spec, swap on accept" pattern.
  - **Tasks**:
    - Wire a copy constructor copy of the cache map (`~16KB` for `N=16`) at candidate evaluation, swapping it into `champion_cache_` only upon promotion.

