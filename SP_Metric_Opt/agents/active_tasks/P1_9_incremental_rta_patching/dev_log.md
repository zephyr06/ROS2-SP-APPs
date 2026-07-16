# P1.9 — Incremental RTA Patching — Dev Log
> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-13
- **Task filed.** Elevated Idea 11 (incremental RTA patching — the
  priority-prefix half of Idea 2) from `P1_1_efficiency_optimizations/`'s
  deferred bucket to an active P1.9 task, per the user's call that "it will
  have good impact" with the explicit scope target of **scalability at
  larger N (N=10/N=16)**. Two-sub-task Phase 1 (per-core RTA cache 1a +
  HP-prefix checkpoint store 1b), then Phase 2 dispatch (Loop B TL patch 2a
  low-risk, Loop A priority patch 2b = the O(N²) payoff), plus a Phase 0
  micro-bench to measure before claiming.
- **Impact re-evaluation (grounded in source, not guessed)** — the basis for
  elevating. Three cost terms per interval; M ≈ tasks-per-core ≈ N/2,
  Granularity G=10, perf-pair fraction f:
  - **Term 1 (per-convolve)**: already collapsed ~3× by Idea 10 (the
    single-point fast path, `Probability.cpp` `ConvolveSinglePoint`); TL'd-
    task convolves are O(G), no sort. Does not grow with N beyond prefix
    length.
  - **Term 2 (1D priority loop) = the O(N²) term**: `FindPriorityVec1D_
    Variations` (`OptimizeSP_Incre.cpp:180-212`) generates up to **N
    candidates per changed task** (Increase: `lb=0, ub=old_priority_index`;
    Decrease/OpenToAll span up to `N-1`). Each candidate pays a full M-task
    `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:58`). So per changed
    task: **N × M ≈ N²/2 convolve-heavy evals**. At N=6 ~18; N=10 ~50; N=16
    ~128. This is the term that blows up. Patching collapses each candidate
    from O(M) convolves to O(k), k = affected suffix length.
  - **Term 3 (TL walk) = O(N)**: `PerformCoordinateDescentForTaskConfigOpt`
    (`OptimizeSP_TL_Incre.cpp:241-281`) iterates all perf-pair tasks × 2
    directions × patience steps; ~O(f·N) candidates/interval, each a full
    M-task RTA today. Patching → recompute only suffix `[p, M)`.
  - **Asymmetry that matters**: at N=6 Term 2 ≈ Term 3 in raw candidate
    count; at N=16 Term 2 dominates ~25:1. So the earlier "might be
    marginal at N=6" caveat was right for N=6 but **wrong for the
    scalability regime the user cares about**. Phase 2b (Loop A priority
    patch) is where the payoff concentrates.
- **Per-processor skip (item a)** = `ProbabilisticRTA_TaskSet`
  (`RTA.cpp:100-119`) already partitions by `processorId` via
  `ExtractTaskSetPerProcessor` (`:87`) and runs `SingleCore` per core
  independently. A TL/priority change on core A leaves core B's entire RTA
  vector byte-identical → return cached, skip its `SingleCore` entirely.
  With `N_CORES=2` (constant default, `taskset_generator.py:27` `suggest: 2`)
  and greedy balanced assignment (`:546-554`, each task → least-loaded core
  → ~N/2 per core even as N grows — confirmed: N=6 uses {0,1}; N=10 cfg_10
  is 5/5), per-core skip saves ~N/2 tasks' RTA per candidate. Cores do NOT
  scale with N under the current generator, so this lever is available at
  every N.
- **Compound estimate**: per-processor skip (halve the cores) × prefix-suffix
  reuse (halve the suffix) ≈ **~3-4× fewer convolve-heavy evals** on the
  dominant O(N²) term, on top of Idea 10's per-convolve ~3×. Stated as an
  estimate, not a measurement — 2c's bench is what nails it.
- **Architectural placement decided (preliminary)**: the cache lives in
  `OptimizePA_Incre_with_TimeLimits` (the TL-walk owner), **not** in the
  throwaway challenger built by `BuildChallengerFromIncumbent`
  (`OptimizeSP_TL_Incre.cpp:394`) — the P0.5 redesign rebuilds the
  challenger from `res_opt_` each `EvaluateTimeLimitConfig_ScratchOrIncre`
  call, so challenger-held state is discarded. The cache is pure
  distribution memoization (no PA-search state) → does NOT carry the
  rejected-challenger drift hazard (P1.1 `goal.md` §3), provided it's
  invalidated on (1) PA change the patch can't handle, (2) multi-task diff,
  (3) new interval (DAG_ET change). **Concrete cache implementation design
  is the next deliverable — proposed to the user, not yet designed.**
- **No source changes, no tests run this entry.** Idea 11 remains un-
  implemented; the P1.1 `idea_queue.md` entry still says "High Priority
  (new 2026-07-12)" — to be updated to point here once the user confirms
  the task is filed + the cache design is aligned.

## 2026-07-13 (cache design aligned with user)
- **Cache implementation design proposed and locked.** Four decisions,
  confirmed by the user:
  1. `hp_tasks_et_conv_vec[i]` = convolution of the **higher-priority tasks'
     ET dists** `[0, i)` on a core — confirmed as "hp tasks' ET, that's
     fine." `hp_tasks_et_conv_vec[0]` = `FiniteDist({Value_Proba(0, 1.0)})`.
     Consumed verbatim by the existing 3-arg `GetRTA_OneTask` (`RTA.cpp:44`).
  2. **Cache compute/patch are free functions** in `RTA.h`/`RTA.cpp`, NOT
     class methods. Each takes the cache by reference and mutates it in
     place. (User: "make it an independent function rather than class
     member, take rta cache as an input parameter and modify it in-place.")
  3. **One cache per interval optimization, never crosses intervals.** Lives
     as a member of `OptimizePA_Incre_with_TimeLimits`, reset at the start
     of each interval. **Implement the no-PA-change (TL) path first**;
     priority-move (PA-change) path second. (User: "we'll use one cache per
     interval optimization. it doesn't cross interval. for implementation,
     you can start with the one without PA changes.")
  4. **The cache replaces the existing RTA path outright** — no knob, no
     fallback. `ProbabilisticRTA_TaskSet` is replaced by
     `ComputeRTA_FullAndCache` on the baseline path and by the patchers on
     candidate paths. Correctness gated by differential unit tests, not a
     kept slow path. (User: "the cache version will replace existing RTA
     calculation. we'll rely on unit tests to make sure code is correct.")
- **`tasks.md` rewritten** to embed the design: the `PerCoreRTACache` value
  type, the three free functions (`ComputeRTA_FullAndCache`,
  `PatchRTA_OneTaskTL`, `PatchRTA_PriorityMove`), the cache storage +
  validity predicate, and a descriptive build order. Dropped the opaque
  `1a/1b/2a/2b` labels (user: "i don't know what does 1b and 1a 2b refer
  to") in favor of named sub-tasks: HP-prefix checkpoint store, per-core
  RTA cache, TL patch dispatch (Loop B, first), priority-move patch
  dispatch (Loop A, second), end-to-end measurement.
- **Idea-queue cross-link done** (prior entry's loose end): P1.1
  `idea_queue.md` Idea 11 Status now reads "ELEVATED to active task P1.9 on
  2026-07-13" with a link to `../P1_9_incremental_rta_patching/`.
- **No source changes, no tests run this entry either.** Design only. Next
  concrete step (gated on user OK to start coding): Phase 0 micro-bench
  (`tests/benchConvolve.cpp`), to measure-before-claiming.

## 2026-07-13 (step 2 landed: HP-prefix checkpoint store)
- **User skipped the Phase 0 micro-bench** ("skip micro-bench, start your
  first task now") → dove straight into build-order step 2. `tests/
  benchConvolve.cpp` remains un-created (still a ghost entry in git status);
  the measure-before-claiming gate is deferred to the end-to-end measurement
  (build-order step 6) instead.
- **HP-prefix checkpoint store landed** — pure refactor of
  `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:58-85`). The function now
  additionally emits `hp_tasks_et_conv_vec[i]` = the rolling
  `hp_tasks_et_conv` snapshotted at the **top** of iteration `i` (i.e. the
  convolution of the higher-priority tasks' ET dists `[0, i)`), with
  `hp_tasks_et_conv_vec[0]` = `FiniteDist({Value_Proba(0, 1.0)})`. This is
  exactly the value the 3-arg `GetRTA_OneTask` (`RTA.cpp:44`) already
  consumes — so the patchers (steps 4 & 5) can reuse it verbatim.
- **Implementation shape**: added a 2-arg overload
  `ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec&)`; the
  1-arg version is now a thin wrapper that discards the prefix
  (`return ...SingleCore(tasks, ignored)`). Single source of truth — the
  rolling-convolution logic lives in exactly one place. The 6 existing 1-arg
  test callers (and all of `ProbabilisticRTA_TaskSet` / the SP eval path)
  are unchanged.
- **Two mechanical bugs caught during TDD**, both surfaced by the RED build:
  1. `Task`'s 6th ctor param is `std::string name`, **not** `processorId`
     (`processorId` is a default-`-1` member, set post-construction). My
     fixture's `Task(0, d0, 5, 5, 0, 0)` tried to construct `name` from `int
     0` → `basic_string::_M_construct null` in `SetUp()`. Fixed: use the
     5-arg form (`Task(id, exec, period, ddl, priority)`) matching the
     existing fixtures; `SingleCore` ignores `processorId` anyway (sorts by
     `priority`).
  2. My behavior-preservation test **discarded the 2-arg's return value**
     (`ProbabilisticRTA_TaskSet_SingleCore(tasks, hp_tasks_et_conv_vec_ignored);`
     as a statement, not an assignment) → `rtas_two_arg` stayed empty. The
     2-arg returns `rtas` **by value** (only `hp_tasks_et_conv_vec` is an
     out-param). Fixed: `rtas_two_arg = ProbabilisticRTA_TaskSet_SingleCore(...)`.
  - **Build-config gotcha rediscovered**: `RTA.cpp` did **not** include its
    own header `RTA.h` (only `RegularTasks.h`). When the 1-arg overload
    (defined first in the file) was made to delegate to the 2-arg, the 2-arg
    wasn't yet declared at that point in the TU → "too many arguments to
    function" (the call resolved to the 1-arg itself). Fixed: added
    `#include "sources/Safety_Performance_Metric/RTA.h"` to `RTA.cpp`.
    (Both overloads ARE declared in `RTA.h`, in 1-arg-then-2-arg order — the
    include makes that visible.) Worth noting for steps 3–5, which will add
    more free functions to `RTA.h`/`RTA.cpp`.
- **TDD**: 2 new tests in `tests/testRTA.cpp` (`TaskSetForTest_3tasks_prefix`
  fixture, 3 tasks, priorities 0<1<2):
  - `HpTasksEtConvVec_MatchesRollingValue` — independently replays the rolling
    `hp_tasks_et_conv` from `RTA.cpp:73,81-82` and asserts
    `hp_tasks_et_conv_vec[i] == rolling` for every `i`. GREEN (bit-identical
    to the rolling value).
  - `TwoArgOverload_SameRtasAsOneArg` — explicit same-input differential:
    the 2-arg overload returns bit-identical `rtas` to the 1-arg. GREEN.
  - The 6 pre-existing pinned-`rtas` tests are the behavior-preservation
    oracle (1-arg now delegates to 2-arg → they transitively pin the 2-arg's
    `rtas` output too). All GREEN.
- **Verification**: `cmake --build . --target check.SP_OPT -j5` → 16/16 ctest
  green (DEBUG build, `libSP_OPTDebug.so`). No source behavior change to any
  returned `rtas` vector — this step is pure infrastructure (the checkpoint
  store the patchers will reuse).
- **Next step (gated on user review + commit)**: build-order step 3 — the
  per-core RTA cache: `PerCoreRTACache` struct + `ComputeRTA_FullAndCache`
  free function, replacing `ProbabilisticRTA_TaskSet` on the baseline path.

## 2026-07-14 (rename: prefix_conv → hp_tasks_et_conv_vec)
- **User feedback**: "the name `prefix_conv` is less clear than the old name
  `hp_tasks_et_conv`, use the old name, refactor your code changes." Fair —
  `hp_tasks_et_conv` is the established name in this codebase for this exact
  quantity (the 3-arg `GetRTA_OneTask` parameter, `RTA.cpp:44`).
- **Rename applied across `RTA.cpp`/`RTA.h`/`testRTA.cpp` + the P1.9 task
  docs** (`dev_log.md`, `tasks.md`, `goal.md`, `overall_tasks.md`). The 2-arg
  out-param is now `hp_tasks_et_conv_vec` (the `_vec` suffix distinguishes the
  snapshotted-vector out-param from the rolling-scalar `hp_tasks_et_conv`
  local that still lives in the same function body — avoiding shadowing); the
  discarded-prefix local in the 1-arg wrapper is
  `hp_tasks_et_conv_vec_ignored`; the test is
  `HpTasksEtConvVec_MatchesRollingValue`. No behavior change — pure rename.
- **Verification**: `cmake --build . --target check.SP_OPT -j5` → 16/16 ctest
  green (DEBUG build, `libSP_OPTDebug.so`). `git grep prefix_conv` in
  `sources/`+`tests/` returns no matches.

## 2026-07-14 (step 3a landed: per-core RTA cache infra + differential test)
- **Scope split with user OK**: step 3 (per-core RTA cache) done as the smallest
  testable unit first — `PerCoreRTACache` struct + `ComputeRTA_FullAndCache`
  free function + differential test, **without** touching the eval path. The
  eval-path wiring (threading `per_core_rta_cache_` down `EvaluateSPWithPriority
  Vec` → `ObtainSP_DAG` → `ObtainSP_TaskSet`) is invasive (4-deep call chain,
  none currently take a cache param) and is a separate step 3b review-and-commit
  cycle. This keeps step 3a pure infra, fully testable in isolation.
- **Signature divergence from the locked design — flagged for user review.**
  The locked `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` (tasks.md)
  cannot go in `RTA.h` as decision 2 specified: `PriorityVec` is declared in
  `OptimizeSP_Base.h`, which includes `SP_Metric.h`, which includes `RTA.h` → a
  header cycle (RTA.h's own declarations sit after its include line, so
  SP_Metric.h would see them half-defined). Plus folding pa_vec/tl_vec
  application into the function would duplicate `UpdateTaskSetPriorities`
  (`OptimizeSP_Base.cpp:28`) + `ApplyTimeLimitsToTasksExecutionTime`
  (`SP_Metric.cpp:70`). As-built signature is `ComputeRTA_FullAndCache(const
  TaskSet& tasks, std::unordered_map<int, PerCoreRTACache>& cache)` — takes the
  already-prepared `tasks` (priorities + TLs applied by the caller, exactly as
  `ProbabilisticRTA_TaskSet`'s callers do today). 1:1 swap at the call site,
  cycle-free, non-duplicating. `TimeLimitVec` was never a codebase type — TLs
  are `std::vector<double>` throughout (e.g. `ObtainSP_DAG(..., const
  std::vector<double>& time_limits)`). The two patchers
  (`PatchRTA_OneTaskTL`/`PatchRTA_PriorityMove`) still take `const DAG_Model&`
  in the docs — they'll need the same re-examination when step 4/5 lands; for
  now they're doc-only.
- **Implementation** (`RTA.cpp`): `ComputeRTA_FullAndCache` mirrors
  `ProbabilisticRTA_TaskSet` (`RTA.cpp`) exactly — `task_id2index` map,
  `ExtractTaskSetPerProcessor` partition, scatter `rtas_curr` back into flat
  `rtas` by `task_id2index` — but drives the 2-arg `SingleCore` (emits
  `hp_tasks_et_conv_vec`) and, per core, retains `entry.rta = rtas_curr`,
  `entry.hp_tasks_et_conv_vec` (the out-param), and `entry.sorted_task_ids`
  (rebuilt by sorting a local copy by `priority`, matching SingleCore's internal
  sort, so `sorted_task_ids[i]` lines up with `rta[i]`/`hp_tasks_et_conv_vec[i]`).
  `cache.clear()` at entry. `RTA.h` gained `#include <unordered_map>` (the
  struct + fn signature use it; the header was previously non-self-contained for
  it, relying on `RTA.cpp`'s own include).
- **TDD**: 1 new fixture + 2 new tests in `tests/testRTA.cpp`:
  `TaskSetForTest_4tasks_2cores_cache` (4 tasks, 2 cores, 2/core — deliberately
  NOT the v9 fixture, which has only 1 task/core and so cannot exercise the
  per-core partition/cache; `processorId` set post-construction as a default
  member, matching the existing fixtures' style). Tests:
  - `ComputeRTA_FullAndCache_SameRtasAs_ProbabilisticRTA_TaskSet` — the
    decision-4 differential gate: flat `rtas` from the cache path are
    bit-identical to `ProbabilisticRTA_TaskSet` (the oracle) on the same input.
  - `ComputeRTA_FullAndCache_CacheIsSelfConsistent` — both cores present,
    `sorted_task_ids`/`rta`/`hp_tasks_et_conv_vec` lengths equal,
    `hp_tasks_et_conv_vec[0]` == identity, HP-first ordering holds,
    `entry.rta[i]` == flat `rtas[task's index]`, every task in exactly one core.
  - A 3rd TL differential test was drafted then REMOVED:
    `ApplyTimeLimitsToTasksExecutionTime` is defined in `SP_Metric.cpp` but
    declared in NO header → invisible to `testRTA.cpp`. Rather than add a header
    declaration (scope creep beyond 3a) the TL coverage is deferred to step 4
    (TL patch dispatch), where TLs are the actual subject and the application is
    already wired in the live path. The no-TL differential already pins cache
    correctness vs the oracle; TLs only mutate `execution_time_dist` before the
    call — the cache fn is TL-agnostic.
- **Verification**: `cmake --build . --target check.SP_OPT -j5` → 16/16 ctest
  green (DEBUG build, `libSP_OPTDebug.so`); both new tests PASS when run
  directly via `--gtest_filter`. No behavior change to any returned `rtas` —
  `ComputeRTA_FullAndCache`'s output is bit-identical to
  `ProbabilisticRTA_TaskSet` by construction + test.
- **Staged, NOT committed** (per agent_coding_rules: `git add` only, user
  reviews + commits): `sources/Safety_Performance_Metric/RTA.h`,
  `RTA.cpp`, `tests/testRTA.cpp`. `ProbabilisticRTA_TaskSet` / `_SingleCore`
  unchanged — still the live-path + the differential oracle (removal at step 3b,
  decision 4). **Next step (gated on user review + commit)**: step 3b — wire
  `ComputeRTA_FullAndCache` into the eval path + retire
  `ProbabilisticRTA_TaskSet` from the live path.

## 2026-07-14 (cache-validity design refined: tl_vec + RTA_Cache.h + option 1)
- **Trigger**: user asked to discuss cache validity — "the cache depends on
  priority assignment vector, and time limit assignments … the key challenge is
  to identify when / whether a certain component of the cache can be reused …
  saving the priority assignment vector and time limit vectors can help in
  identifying the reusable components." The user is right; code-grounded the
  answer, then the user drove two follow-on decisions.
- **Claim verified against source** (load-bearing): `tl_vec` is a SUFFICIENT
  proxy for ET-dist validity within an interval. Grep of every
  `execution_time_dist` assignment site → only 3 real mutations:
  `SP_Metric.cpp:75` (`ApplyTimeLimitsToTasksExecutionTime` →
  `GetUnitExecutionTimeDist(tl)`), `OptimizeSP_TL_BF.cpp:11` (TL-driven), and
  `OptimizeSP_TL_Incre.cpp:487` (`ApplyWCETAblationIfRequired`, one-time setup,
  gated on `use_wcet_execution_time`). `RTDA_Prob.h:40` is a const-ref binding
  (false positive); `SP_Metric.cpp:120` is commented out.
  `GetUnitExecutionTimeDist(time_limit)` (`Probability.h:177-180`) returns
  `FiniteDist({{time_limit, 1.0}})` — a deterministic point mass at the TL. So
  within an interval's search, ET dist is a pure function of tl_vec
  (`tl_vec[i]` → point mass at that TL; `tl_vec[i]==-1` → unchanged immutable
  base Gaussian), EXCEPT across the one-time WCET-ablation boundary. So
  `stored_tl[i] == current_tl[i]` ⟺ ET-dist[i] unchanged. The WCET-ablation
  boundary is covered by the interval reset (decision 3), not by tl_vec.
- **pa_vec NOT stored** — `sorted_task_ids` (already in the struct) is the
  sufficient proxy: RTA cares about per-core ORDER, not priority VALUES. A
  cross-core priority shift that preserves each core's internal order changes
  no core's RTA, so `sorted_task_ids` detects every reorder that matters;
  storing raw `pa_vec` values on top is redundant for validity (only value:
  inspectability, which we're declining).
- **Component-level reuse (the actual payoff)** — `hp_tasks_et_conv_vec[p]` is
  the reuse primitive, so validity locates a reuse point `p` per core, not a
  binary valid/invalid:
  - TL change at position `p` on core A (priority unchanged): reuse
    `hp_tasks_et_conv_vec[0..p]` + `rta[0..p)` + all other cores untouched;
    invalidate `rta[p..n)` + `hp_tasks_et_conv_vec[p+1..n]` on core A. `p` =
    first index where `stored_tl[i] != current_tl[i]`.
  - Priority move `old_pos→new_pos` on core A (TLs unchanged): reuse the common
    prefix `hp_tasks_et_conv_vec[0..min(old,new)]`; invalidate the suffix
    `[min, n)` replayed in the new order. `min` from comparing `sorted_task_ids`
    old vs new.
- **User decision 1 — option 1 signature**: user asked "why does option 1
  duplicate the functions … even if it duplicates, the cost seems trivial?"
  Verified against source: `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148`)
  applies pa_vec (`UpdateTaskSetPriorities`) but NOT tl_vec — TLs are applied
  upstream (caller builds the TL-baked `dag`, passes it in). So at the RTA
  plug-in point (`ObtainSP_TaskSet`, `SP_Metric.cpp:55`) tl_vec is already gone
  (baked into ET dists). Recovering it for storage needs the caller to pass it.
  Option 1 `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` re-applies
  `UpdateTaskSetPriorities` + `ApplyTimeLimitsToTasksExecutionTime` internally
  (trivial O(N), negligible next to RTA) → owns the full input tuple natively,
  stores `tl_vec` without threading it down the 4-deep `ObtainSP_*` chain, and
  leaves `ObtainSP_DAG`/`ObtainSP_TaskSet` signatures untouched. User: "go with
  option 1." This OVERTURNS the step-3a as-built signature divergence reason —
  the DRY concern was overweighted (RTA dominates the cost).
- **User decision 2 — cache lives in optimizer**: confirmed decision 3 stands
  — the cache member sits in `OptimizePA_Incre_with_TimeLimits`, NOT in
  `EvaluateSPWithPriorityVec` (a per-candidate free fn) or the throwaway
  challenger. Step 3b wiring threads it from the optimizer into a cache-aware
  eval path.
- **Header-cycle resolved**: the locked sig needs `PriorityVec` (in
  `OptimizeSP_Base.h` ← `SP_Metric.h` ← `RTA.h`). Moving the cache decls to a
  NEW `RTA_Cache.h` (which nothing upstream includes) breaks the back-edge;
  `RTA_Cache.h` includes `RTA.h` + `DAG_Model.h` + `OptimizeSP_Base.h`,
  all cycle-free. Verified: `TaskModel/` never includes `RTA.h`;
  `RTA.h`/`Probability.h`/`RegularTasks.h` never include `OptimizeSP_Base.h`/
  `DAG_Model.h`.
- **Prerequisite surfaced**: `ApplyTimeLimitsToTasksExecutionTime` is defined in
  `SP_Metric.cpp:70` but declared in NO header → the cache fn (in `RTA_Cache.cpp`,
  which includes `SP_Metric.h`) can't call it. Step 3c must add the decl to
  `SP_Metric.h`. (Step 3a dodged this by taking pre-prepared `tasks`.)
- **Step 3c plan (next, gated on user OK)**: create `RTA_Cache.h`/`.cpp`;
  move `PerCoreRTACache` + `ComputeRTA_FullAndCache` out of `RTA.h`/`.cpp`;
  restore `(dag, pa_vec, tl_vec, cache)` sig + apply pa/tl internally; add
  `tl_vec` field to `PerCoreRTACache`; add `ApplyTimeLimitsToTasksExecutionTime`
  decl to `SP_Metric.h`; update the 2 step-3a tests for the new sig + assert
  `tl_vec` stored; re-run `cmake ..` (glob picks up new `.cpp`); build DEBUG,
  `check.SP_OPT` 16/16.
- **Docs updated** (`tasks.md` + `goal.md` + this entry) BEFORE any code, per
  the user's "first update tasks folder with this new detailed design, then
  implement it." No source/test changes this entry yet.

## 2026-07-14 (API roles + commit discipline — implementation-path discussion)
- **User reframed the implementation path**: focus on the `PerCoreRTACache` API
  first (the standalone functions), THEN integrate it into the optimizer class.
  Two new design points to capture before coding:
  1. **Reuse-validity as a first-class API** — "the key challenge is
     identifying conditions when we can reuse the cache and when we cannot."
     Added a READ-ONLY query API `AnalyzeCacheReuse(cache, dag, pa_vec, tl_vec)`
     → per-core `CacheReuseInfo{fully_reusable, same_core_partition,
     common_prefix_length}`. It runs NO RTA — just a cheap structural diff of
     `sorted_task_ids` + `tl_vec` against the candidate (those two ARE the
     within-interval validity state). Plus a `CacheConsistentWith` whole-cache
     exact-match convenience. Deliberate separation of concerns: the classifier
     gives the reuse **point** (prefix length p, per core); the **patcher**
     gives the reuse **verdict** for its specific change type (one TL / one
     move) and falls back to `ComputeRTA_FullAndCache` if the diff doesn't
     match its assumption. The classifier doesn't decide single-vs-multi-task
     (that's patcher-specific).
  2. **Cache-update discipline** — "each time after we call RTA with the cache,
     do we update the cache? … cache is locked with the champion solution in
     pairs." Answer: the patchers (`PatchRTA_*`) update-in-place as a side
     effect of computing the candidate RTA, BUT the cache is a
     `(champion (pa_vec, tl_vec), cache)` pair — a *rejected* candidate's patch
     must NOT clobber the champion cache. So commit is **explicit, not
     automatic**: patch only on candidate promotion. Two acceptable patterns
     (decided at integration, not in 3c): (a) scratch-cache copy for
     speculative candidates, swap-in on accept; (b) evaluate-then-patch. The
     cache API does NOT enforce this — the optimizer owns the pair discipline.
     `ComputeRTA_FullAndCache` is its own commit (overwrite; call only for the
     new champion / new interval).
- **Prerequisite discovered during this**: `ExtractTaskSetPerProcessor`
  (`RTA.cpp:102`) is defined-but-undeclared (like
  `ApplyTimeLimitsToTasksExecutionTime`) — `AnalyzeCacheReuse` needs it to
  derive the candidate's per-core partition to diff against the cached
  `sorted_task_ids`. So 3c adds BOTH header decls: `ApplyTimeLimitsToTasks
  ExecutionTime` → `SP_Metric.h`; `ExtractTaskSetPerProcessor` → `RTA.h`.
- **Scope of the code change this round (per user)**: add `RTA_Cache.h` +
  `RTA_Cache.cpp` for review FIRST — `PerCoreRTACache` struct, the 3 API roles
  (build / reuse-query / update-patchers), the two header decls. Patchers are
  declared (locked sigs) but defined in steps 4/5. **No optimizer integration
  this round** — that's a separate review cycle.
- **Docs updated** (`tasks.md`: free-functions section restructured into 3
  roles + commit-discipline subsection; build-order 3c entry updated to list
  the reuse-query API + both header decls; this entry). No source/test
  changes yet — writing `RTA_Cache.h`/`.cpp` next.

## 2026-07-14 (step 3c landed: RTA_Cache.h/.cpp + locked sig + reuse-query API + tests)
- **Step 3c implemented + TDD-verified + built green.** `RTA_Cache.h`/`.cpp`
  created; `PerCoreRTACache` + `ComputeRTA_FullAndCache` + `AnalyzeCacheReuse` +
  `CacheConsistentWith` relocated OUT of `RTA.h`/`.cpp` to the new leaf pair
  (breaks the header cycle the locked sig needs `PriorityVec` for). The
  step-3a as-built sig `ComputeRTA_FullAndCache(const TaskSet&, cache&)` is
  GONE; the **locked** sig
  `ComputeRTA_FullAndCache(const DAG_Model&, const PriorityVec&, const
  std::vector<double>&, cache&)` stands, applying pa_vec + tl_vec internally
  (option 1, per the user). `PerCoreRTACache` gained the `tl_vec` field (ET-dist
  validity proxy). Patchers `PatchRTA_OneTaskTL`/`PatchRTA_PriorityMove`
  declared (locked sigs) but NOT defined — that's steps 4/5.
- **Header decls added (the prerequisites)**: `ApplyTimeLimitsToTasksExecution
  Time` → `SP_Metric.h`; `ExtractTaskSetPerProcessor` → `RTA.h`. Both were
  defined-but-undeclared; the cache TU (which `#include`s `SP_Metric.h` +
  `RTA.h`) needs them to apply TLs + derive the candidate per-core partition.
- **Implementation shape** (`RTA_Cache.cpp`): `ComputeRTA_FullAndCache` mirrors
  the live path order — `ApplyTimeLimitsToTasksExecutionTime` on the by-id
  `dag.tasks` → `UpdateTaskSetPriorities` (sorts HP-first) →
  `ExtractTaskSetPerProcessor` → per core, defensively re-sort + build
  `sorted_task_ids`/`tl_vec` aligned to sorted position `i`, drive the 2-arg
  `SingleCore` (emits `hp_tasks_et_conv_vec`), store `entry.rta`, scatter the
  flat `rtas` back by `task_id2index`. `cache.clear()` at entry (own-commit).
  `AnalyzeCacheReuse` runs NO RTA — it derives the candidate's per-core
  `(sorted_task_ids, tl_vec)` via a file-local `DeriveCandidatePerCoreOrder`
  helper (same apply-TLs → apply-pa → partition → sort pipeline, on local
  copies — input `dag` is const), then per core: `same_core_partition` =
  same SET of ids; `common_prefix_length` = first position where
  sorted_task_ids OR tl_vec diverge; `fully_reusable` = same partition + same
  length + full prefix. `CacheConsistentWith` = whole-cache every-core
  fully_reusable (false on empty).
- **TDD** (`tests/testRTA.cpp`, `TaskSetForTest_4tasks_2cores_cache` fixture
  rewritten to hold `DAG_Model` + `PriorityVec` + `time_limits` — the 4-arg
  inputs; `processorId` set on tasks BEFORE `DAG_Model(tasks, {}, {})` ctor):
  8 tests:
  - `ComputeRTA_FullAndCache_SameRtasAs_Oracle` — the decision-4 differential
    gate; flat `rtas` bit-identical to the oracle (apply TLs → apply pa_vec →
    `ProbabilisticRTA_TaskSet`). GREEN.
  - `ComputeRTA_FullAndCache_CacheIsSelfConsistent` — per core: lengths of
    `sorted_task_ids`/`rta`/`tl_vec`/`hp_tasks_et_conv_vec` all equal,
    `hp_tasks_et_conv_vec[0]` == identity, HP-first ordering holds,
    `entry.rta[i]` == flat `rtas[task's index]`, **`tl_vec[i]` aligns with
    `time_limits[sorted_task_ids[i]]`** (the new ET-dist proxy assertion), every
    task in exactly one core. GREEN.
  - `ComputeRTA_FullAndCache_WithTL_SameRtasAs_Oracle` — the TL coverage
    DEFERRED from 3a: a real TL on task 1 (point-mass ET dist) → cache still
    bit-identical to the oracle; also pins the stored `tl_vec` reflects the TL
    on the affected core. GREEN.
  - `AnalyzeCacheReuse_IdentityCandidate_FullyReusable` — same (pa_vec, tl_vec)
    → every core `fully_reusable` + `CacheConsistentWith` true. GREEN.
  - `AnalyzeCacheReuse_TLChangeOnCore0_PartialPrefixAndCrossCoreUntouched` — TL
    change to task 1 (sorted pos 1) on core 0: core 0 `fully_reusable==false`,
    `same_core_partition==true`, `common_prefix_length==1` (the suffix-reuse
    point the TL patcher consumes); core 1 (different core, untouched)
    `fully_reusable==true` (the cross-core skip payoff); `CacheConsistentWith`
    false. GREEN.
  - `AnalyzeCacheReuse_PriorityMoveOnCore0_NoCommonPrefix` — swap t0/t1 order on
    core 0: `same_core_partition==true`, `common_prefix_length==0` (diverges at
    pos 0); core 1 still fully reusable. GREEN.
  - `AnalyzeCacheReuse_PartitionChange_NotReusable` — task 2 migrates core 1→0:
    `same_core_partition==false` on BOTH cores (the "can't patch" signal; caller
    full-recomputes); `fully_reusable==false`; `CacheConsistentWith` false.
    GREEN. **Caveat captured in the test comment**: `common_prefix_length` is
    NOT asserted here — it's the raw longest element-wise common prefix, which
    is 2 (not 0) when the migration lands in the suffix ({t0,t1} still match).
    That value is intentionally not actionable: the patchers gate on
    `same_core_partition` FIRST. (First draft wrongly asserted ==0; the RED
    failure — actual 2 — was the test being wrong, NOT the impl.)
  - `CacheConsistentWith_EmptyCache_IsFalse`. GREEN.
- **One TDD RED caught + fixed (the partition-change one above)**: the impl
  was right; my first-draft expectation (`common_prefix_length == 0` on a
  suffix-migration) was wrong. Fixed by dropping the raw-prefix assertion in the
  migration case and pinning the actionable `same_core_partition==false`
  signal instead, with a comment explaining why the raw prefix is moot there.
  This is a real design clarification worth the comment: the classifier's
  `common_prefix_length` is only meaningful under `same_core_partition==true`.
- **Verification**: `cmake build` (re-glob picks up `RTA_Cache.cpp`) →
  `cmake --build build --target check.SP_OPT -j5` → **16/16 ctest green**
  (DEBUG build, `libSP_OPTDebug.so`); `testRTA` directly → 19 tests PASS (0
  FAILED). `ProbabilisticRTA_TaskSet` / `_SingleCore` UNCHANGED — still the
  live path + the differential oracle (removal at step 3b, decision 4).
- **Staged, NOT committed** (per agent_coding_rules: `git add` only, user
  reviews + commits): `sources/Safety_Performance_Metric/RTA_Cache.h`,
  `RTA_Cache.cpp`, `RTA.h`, `RTA.cpp`, `SP_Metric.h`, `tests/testRTA.cpp`.
  **Next step (gated on user review + commit)**: step 3b — wire
  `ComputeRTA_FullAndCache` into the baseline/descent eval path (cache member
  in `OptimizePA_Incre_with_TimeLimits`) + retire `ProbabilisticRTA_TaskSet`
  from the live path.

## 2026-07-14 (step 3b scope decision + call-chain map — docs only, pre-code)
- **Resumed P1.9; step 3c still staged-not-committed** (verified: `git status`
  shows RTA_Cache.h/.cpp + RTA.h/.cpp + SP_Metric.h + testRTA.cpp staged;
  last commit is `0c1f846b improve info printed`, NOT the cache — so the user
  has not yet committed 3c). State matches the [[p19-rta-cache-step3c-landed]]
  memory. This entry is DOCS-ONLY — updating the task description for 3b
  before any code, per the established docs-first pattern here.
- **Step 3b scope decided with the user: BASELINE-ONLY.** The framing that
  surfaced the decision: the cache member lives on
  `OptimizePA_Incre_with_TimeLimits` (decision 3, the TL-walk owner, durable
  across a candidate), but the per-candidate RTA happens TWO LAYERS DOWN,
  inside a THROWAWAY `OptimizePA_Incre` challenger that
  `EvaluateTimeLimitConfig_ScratchOrIncre` rebuilds EVERY candidate (P0.5
  `BuildChallengerFromIncumbent`, OptimizeSP_TL_Incre.cpp:394), via the shared
  free fn `EvaluateSPWithPriorityVec` → `ObtainSP_DAG` → `ObtainSP_TaskSet` →
  `ProbabilisticRTA_TaskSet`. So the cache can't live on the challenger
  (discarded every candidate), and `EvaluateSPWithPriorityVec` is shared with
  `OptimizePA_BF` / `OptimizeSP_TL_BF` — retiring the oracle "outright
  everywhere" (decision 4's literal reading) would touch brute force too.
  User picked **baseline-only**: wire the cache into the incremental
  optimizer's eval path, retire `ProbabilisticRTA_TaskSet` from THAT path
  only, leave BF/TL_BF on the oracle. No patching yet — cache is BUILT per
  candidate (full compute + populate, role 1); reuse is steps 4/5.
- **Call chain traced end-to-end and verified against source** (the map now
  lives in `tasks.md` §"Step 3b design"). Four load-bearing facts:
  1. **TLs baked upstream, pa_vec applied at the eval fn.**
     `UpdateExtDistBasedOnTimeLimit` (OptimizeSP_TL_BF.cpp:6) IS
     `ApplyTimeLimitsToTasksExecutionTime` (SP_Metric.cpp:70) — both set
     `execution_time_dist = GetUnitExecutionTimeDist(tl[i])` for `tl[i]!=-1`.
     It runs in `EvaluateTimeLimitConfig_ScratchOrIncre`
     (OptimizeSP_TL_Incre.cpp:146) BEFORE the challenger is built;
     `EvaluateSPWithPriorityVec` (OptimizeSP_Base.cpp:148) applies pa_vec only.
     At the RTA plug-in point tl_vec is already gone — validates step 3c's
     option-1 `ComputeRTA_FullAndCache(dag, pa_vec, tl_vec, cache)` re-applying
     BOTH internally (owns the full input tuple, recovers tl_vec for storage
     without threading it down the 4-deep `ObtainSP_*` chain).
  2. **Cache can't live on the challenger** (rebuilt every candidate, P0.5).
     Must live on the outer `OptimizePA_Incre_with_TimeLimits` and be threaded
     down — OR the cache-aware eval hoisted to a level the outer optimizer
     controls. (The `tasks.md` wiring shape picks the latter: a cache-aware
     eval method on the outer optimizer.)
  3. **`EvaluateSPWithPriorityVec` is shared infra** (called by
     OptimizePA_Incre:136/239/279, OptimizePA_BF:19, and
     ResetIncumbentBaseline:424/436). Baseline-only keeps it for BF/TL_BF; the
     incremental path gets a cache-aware route beside it.
  4. **Chain RTDA is orthogonal to the RTA cache.** `ObtainSP_DAG`
     (SP_Metric.cpp:96-107) adds the path-latency SP from
     `GetRTDA_Dist_AllChains`, a separate dist source.
     `ObtainSP_DAG_From_Dists` (SP_Metric.cpp:129) already assembles the full
     SP from PRECOMPUTED node-RT dists + chain dists → the cache-aware eval
     reuses it: node RTAs via `ComputeRTA_FullAndCache`, then combine via
     `ObtainSP_DAG_From_Dists`.
- **Wiring shape captured in `tasks.md`**: a cache-aware eval entry on
  `OptimizePA_Incre_with_TimeLimits` (durable owner) — given
  `(dag_with_TLs_baked, pa_vec, tl_vec)`, drives `ComputeRTA_FullAndCache` for
  the node RTAs, folds chain RTDA via `ObtainSP_DAG_From_Dists`, returns SP.
  Per-candidate calls in `OptimizeFromScratch`/`OptimizeIncre`'s 1D loop
  (OptimizeSP_Incre.cpp:279 etc.) route through it. `per_core_rta_cache_`
  reset per interval (decision 3), built fresh per candidate at 3b (no reuse
  yet). TDD: differential — cache-aware SP bit-identical to
  `EvaluateSPWithPriorityVec` on the same `(dag_with_TLs, pa_vec)` across a
  sweep; pins the node-RTA + chain-RTDA → SP assembly end-to-end (the
  `ComputeRTA_FullAndCache`-vs-oracle pin is already step 3c's test).
- **Docs updated BEFORE code** (`tasks.md`: new Phase 1.5 step 3b task +
  build-order 3b entry + call-chain map + updated Done-when; this entry).
  `goal.md` next-step pointer refreshed. **No source/test changes this entry.**
  Next concrete step (gated on user review + commit of 3c, then OK to code 3b):
  implement the cache-aware eval entry + route the incremental path's
  per-candidate calls through it + the differential test.

## 2026-07-14 (step 3b wiring concretized — two integration hazards found, docs-only)
- **Resumed P1.9 to concretize the step 3b wiring** (the prior entry left the
  wiring as an abstract "cache-aware eval entry on the outer optimizer"; the user
  asked to update the task description now). This entry is DOCS-ONLY — grounding the
  abstract shape against the actual source before any code, per the docs-first
  pattern here. Step 3c still staged-not-committed (verified: `git status` shows
  RTA_Cache.h/.cpp + RTA.h/.cpp + SP_Metric.h + testRTA.cpp staged; HEAD `0c1f846b`,
  NOT the cache).
- **Read the full call chain against source** — `EvaluateTimeLimitConfig_Scratch
  OrIncre` (OptimizeSP_TL_Incre.cpp:142), `EvaluateSPWithPriorityVec`
  (OptimizeSP_Base.cpp:148), the `ObtainSP_*` chain (SP_Metric.cpp:89/53/129),
  `OptimizePA_Incre::OptimizeFromScratch`/`OptimizeIncre` (OptimizeSP_Incre.cpp:74,
  233 incl. the :136/:239/:279 eval sites), `BuildChallengerFromIncumbent`
  (OptimizeSP_TL_Incre.cpp:394), `GetPerfCoefficient` (RegularTasks.h:83). Two
  integration hazards the abstract shape did NOT resolve — both now pinned in
  `tasks.md` §"Step 3b design":
  - **Hazard A (class mismatch):** the cache member lives on the DERIVED
    `OptimizePA_Incre_with_TimeLimits` (decision 3), but the per-candidate
    `EvaluateSPWithPriorityVec` call sites are in the BASE `OptimizePA_Incre`
    (`OptimizeFromScratch` :136, `OptimizeIncre` :239,:279). The inner optimizer is
    constructed as a BASE `OptimizePA_Incre` in BOTH branches of
    `EvaluateTimeLimitConfig_ScratchOrIncre` (from-scratch :151
    `OptimizePA_Incre optimizer(...)`; incremental :160 =
    `BuildChallengerFromIncumbent()` which constructs `OptimizePA_Incre challenger
    (...)` at :398 — sliced to base). So a cache-aware method on the derived class is
    UNREACHABLE from the PA loop, and a `virtual` override won't fire (sliced inner
    object → base vptr; plus the loop calls the free fn directly, not through `this`).
    Resolution: thread the cache into the PA loop by **parameter** (seam S1:
    optional `PerCoreRTACache*` on `OptimizeFromScratch`/`OptimizeIncre`, default
    nullptr = unchanged free-fn path for BF/TL_BF; the derived class passes
    `&per_core_rta_cache_` at the :151/:160 call sites). S2 (free
    `EvaluateSPWithCache(dag,sp,pa,tl,cache&)`) and S3 (override the PA loop on the
    derived class) noted; S1 preferred (single seam, no inheritance surgery, free fn
    survives for BF). S3 rejected as too invasive for 3b.
  - **Hazard B (`perf_coefficient` omission — correctness):** the oracle path
    `ObtainSP_DAG`→`ObtainSP_TaskSet` (SP_Metric.cpp:53-67) multiplies the node SP
    term by `tasks[i].GetPerfCoefficient()` (:61-65). But
    `ObtainSP_DAG_From_Dists` (SP_Metric.cpp:129-149) calls `ObtainSP` (:11) for the
    node term, which has NO `perf_coefficient` factor. `GetPerfCoefficient`
    (RegularTasks.h:83-89) returns `GetPerfTerm(timePerformancePairs, avg_et)` for
    any task WITH perf pairs (the TL-optimizable tasks — exactly the cache's scope),
    1.0 only when pairs are empty. So a bare `ObtainSP_DAG_From_Dists` call would
    DIVERGE from the oracle for every perf-pair task. Resolution: the cache-aware
    assembly inlines the node term WITH `perf_coefficient` (a new helper or an
    overloaded `ObtainSP_DAG_From_Dists` variant matching `ObtainSP_TaskSet`'s node
    term exactly); the chain term is unchanged (the oracle's chain term
    SP_Metric.cpp:100-107 has no perf coefficient — verified). The prior entry's
    "reuse `ObtainSP_DAG_From_Dists`" inference is OVERTURNED.
  - **TDD consequence captured:** the 3b differential fixture MUST include a
    perf-pair task (so `GetPerfCoefficient() != 1.0`) AND a non-trivial chain, else
    Hazard B hides and the differential passes for the wrong reason. This is the
    correctness gate for 3b.
- **Wiring shape (S1) written into `tasks.md`:** (1) add `per_core_rta_cache_` to
  the derived class, reset in `ResetIncumbentBaseline` (runs before the descent,
  both branches); (2) the cache-aware seam drives `ComputeRTA_FullAndCache` for node
  RTAs + the perf_coefficient-corrected chain assembly → SP (does NOT call
  `ObtainSP_DAG`/`ObtainSP_TaskSet`/`ProbabilisticRTA_TaskSet`); (3) pass the cache
  down at the :151/:160 inner-optimizer call sites; (4) shared free fn
  `EvaluateSPWithPriorityVec` UNCHANGED — BF/TL_BF/`ResetIncumbentBaseline`(:424,
  :436) keep it; retirement is path-local. The double TL-apply (dag_tasks_cur
  already has TLs baked at :146; `ComputeRTA_FullAndCache` re-applies internally per
  step 3c option 1) is idempotent (point mass at the same TL) — already the cache
  fn's contract.
- **Docs updated** (`tasks.md`: fact 2 rewritten to surface the slicing crux; fact 4
  corrected (overturns the `ObtainSP_DAG_From_Dists` reuse); new "Two integration
  hazards" subsection + concrete S1/S2/S3 seam analysis + 4-step wiring shape; TDD
  section tightened with the perf-pair+chain fixture requirement; Phase 1.5 + build-
  order 3b entries updated to reference the hazards; `goal.md` out-of-scope + next-
  step pointer refreshed; this entry). **No source/test changes.** Next concrete step
  (gated on user commit of 3c + OK to code): implement 3b per the S1 wiring —
  `per_core_rta_cache_` member + the cache-aware seam + the perf_coefficient-
  corrected assembly + the differential test.


## 2026-07-15 (step 3b call-chain map corrected — from-scratch beam search bypasses the cache target)

Resumed the 3b docs-first pass. Re-traced the live eval path against source and found
the call-chain map in `tasks.md` §"Step 3b design" materially overstated the cache's
3b reach: it lumped `OptimizeFromScratch` and `OptimizeIncre` together as calling
`EvaluateSPWithPriorityVec` "PER PRIORITY CANDIDATE." That's only true for
`OptimizeIncre`.

**Code-grounded correction (verified against source):**
- `OptimizeIncre` (OptimizeSP_Incre.cpp:233) — baseline eval at :239, per-variation
  eval at :279 (the Loop-A O(N²) term, up to N variations per changed task via
  `FindPriorityVec1D_Variations`). BOTH go through `EvaluateSPWithPriorityVec` →
  `ObtainSP_DAG` → `ObtainSP_TaskSet` → `ProbabilisticRTA_TaskSet`. **These are the
  cache's 3b target.**
- `OptimizeFromScratch` (OptimizeSP_Incre.cpp:74) — its beam search (:83-124) does
  NOT call `EvaluateSPWithPriorityVec` per candidate. Each partial-path expansion
  calls `PriorityPartialPath::UpdateSP(task_id)` (:46-62), which calls
  `GetRTA_OneTask(tasks[task_id], hp_tasks)` — the **2-arg** overload (RTA.cpp:31),
  with hp_tasks filtered by processorId in-place (:50-52), and `perf_coefficient`
  ALREADY inlined as `effective_weight = weight * perf_coeff` (:55-60). This
  **bypasses `ProbabilisticRTA_TaskSet` entirely**, so the cache (which mirrors
  `ProbabilisticRTA_TaskSet_SingleCore`) cannot replace it. `OptimizeFromScratch`
  calls `EvaluateSPWithPriorityVec` exactly ONCE, at its final eval (:136) — that
  single call IS cache-replaceable, but it's not per-candidate.

**Scope consequence for 3b:** the cache's per-candidate benefit is concentrated in
the **incremental** branch (`OptimizeIncre` :239/:279 — exactly where the O(N²)
Loop-A term lives, so it's the right place). The **from-scratch/reopt** branch gets
cache benefit ONLY at its single final eval (:136); its beam-search internals are
out of scope. Idea 11 targets the per-candidate eval cost; the from-scratch beam
search is a separate cost not addressed here (and not the target). This does NOT
change the 3b plan's value — the O(N²) Loop-A term is the dominant cost at larger N,
and it's the incremental branch.

**Second correction surfaced — S1 vs the coding rules.** The S1 seam (optional
`PerCoreRTACache*` on `OptimizeIncre`, default nullptr = old path for BF/TL_BF) was
marked "preferred." Re-checked `agent_coding_rules.md` L3/L10: "reduce usage of
optional arguments... if something important is needed but not passed, raise an
error"; "do not make things optional if not needed." A default-nullptr cache pointer
is exactly that discouraged pattern. Two readings:
- The null path is NOT a degraded fallback — it's the legitimately-different BF/TL_BF
  path that has no cache and should never get one. So S1's optionality is arguably
  "two real modes," not "forgot a config."
- But S2 (a separate `EvaluateSPWithCache(dag, sp, pa_vec, tl_vec, cache&)` free fn
  with the cache as a REQUIRED arg) fits the rules more cleanly — the cache is a
  required arg of the cache-aware fn, not an optional knob on the existing method.

**Flagged for the user** (per `agent_coding_rules.md` L11 "ask users if you're not
certain about design choices"): S1 vs S2 turns on this. S1 is less code (no new free
fn, just an optional arg); S2 fits the coding rules better. No code written either
way — this is a design choice for the user.

**Docs updated (docs-only, no source/test changes):**
- `tasks.md` §"Step 3b design" call-chain map: split the single
  "OptimizeFromScratch / OptimizeIncre call, PER PRIORITY CANDIDATE" block into two
  branches (INCREMENTAL = the cache target, with :239/:279; FROM-SCRATCH = beam
  search via `UpdateSP`→`GetRTA_OneTask` 2-arg bypassing the cache, plus the single
  :136 final eval). Added a "Scope consequence" paragraph.
- `tasks.md` fact 2: sharpened to distinguish `OptimizeIncre` :239/:279 (per-
  candidate) from `OptimizeFromScratch` :136 (single final eval), and noted the beam
  search bypasses `ProbabilisticRTA_TaskSet`.
- `tasks.md` seam analysis (S1/S2/S3): added the seam-scope note (primary target =
  `OptimizeIncre`; `:136` secondary), surfaced the S1-vs-coding-rules tension with
  the exact rule wording, noted S2 fits the rules better.
- `tasks.md` Phase 1.5 + build-order 3b + wiring-shape entries: refreshed to name
  `OptimizeIncre` :239/:279 as the primary target, `:160` as the primary call site,
  `:151`/`:136` as secondary; S1-vs-S2 flagged for the user.
- `goal.md` next-step pointer: refreshed with the scope note + the S1/S2 flag.
- This entry.

Next concrete step (gated on user commit of 3c + OK to code + user's S1-vs-S2 call):
implement 3b per the chosen seam — `per_core_rta_cache_` member + the cache-aware
eval (S1 optional-arg or S2 free-fn) + the perf_coefficient-corrected assembly + the
differential test (perf-pair task + chain fixture). Per the docs-first pattern, NO
source code was written in this step — only the task description update the user
asked for.

## 2026-07-15 (step 3b seam DECIDED — required `PerCoreRTACache&` param, always active)

The user resolved the S1-vs-S2 flag I raised for step 3b's cache-aware eval seam:

> "don't add separate function, cache should always be activated in incremental
> optimizer"

This is **neither S1 nor S2** as originally framed — it's the required-param form:
- **No separate function** → S2's `EvaluateSPWithCache` free fn is REJECTED. The
  cache-aware routing lives *inside* `OptimizeIncre`, replacing its two
  `EvaluateSPWithPriorityVec` calls (`:239` baseline, `:279` per-variation) with
  `ComputeRTA_FullAndCache` + the perf_coefficient-corrected chain assembly (Hazard B).
- **Always active** → there is NO non-cache code path within `OptimizeIncre`. No
  nullptr branch, no fallback.
- **Required, not optional** → `PerCoreRTACache&` (ref, not `PerCoreRTACache*`, no
  default) — satisfies `agent_coding_rules.md` L3/L10 ("reduce optional args... raise
  an error if something important is not passed"; "do not make things optional if not
  needed"). The caller MUST supply a cache.

**Verified against source before recording (not a new design choice — the slicing
forces it):**
- `OptimizeIncre` has exactly TWO call sites: production `OptimizeSP_TL_Incre.cpp:161`
  (`optimizer.OptimizeIncre(dag_tasks_cur)`, where `optimizer` is a SLICED base
  `OptimizePA_Incre` built at `:160` by `BuildChallengerFromIncumbent()`) and test
  `testOptimizeIncrePA.cpp:258` (a bare `OptimizePA_Incre opt(...)` with no derived
  owner). No other callers.
- `BuildChallengerFromIncumbent` (`:394`) returns base `OptimizePA_Incre` BY VALUE →
  the challenger is sliced + rebuilt every candidate. So the cache storage **cannot**
  be a base-class member (it would die every candidate and kill step 4's
  cross-candidate/cross-core reuse — the entire point of the cache). Storage MUST
  stay on the durable derived owner `OptimizePA_Incre_with_TimeLimits`
  (`per_core_rta_cache_`), threaded into the sliced challenger's `OptimizeIncre` by
  reference. This is exactly Hazard A.
- `OptimizePA_BF` is a separate `OptimimizePA_Base` subclass with NO `OptimizeIncre`
  call → the shared free fn `EvaluateSPWithPriorityVec` stays untouched for BF/TL_BF/
  `ResetIncumbentBaseline`. The signature change is cleanly scoped to `OptimizeIncre`.
- Two mechanical consequences (NOT new design): (1) storage on derived owner +
  threaded by required ref (Hazard A, mandatory); (2) the direct base-class test
  (`:258`) must construct + pass a local cache — at 3b's build-per-candidate semantics
  a per-call local cache is bit-identical to the oracle, so the test stays a valid
  differential oracle.

Docs updated (NO source code written — docs-first, gated on user commit of 3c + OK to
code):
- `tasks.md` seam analysis: rewrote S1/S2/S3 → the decided required-`PerCoreRTACache&`
  seam, with the three user-constraint points (no separate fn / always active /
  required-not-optional) and the two forced mechanical consequences; S3 kept as the
  rejected too-invasive alternative; S2 marked REJECTED.
- `tasks.md` fact 2 Hazard-A paragraph: "seam S1" → "the decided seam (required
  `PerCoreRTACache&` on `OptimizeIncre`)".
- `tasks.md` Phase 1.5 step-3b + build-order 3b + wiring-shape entries: S1/S2 flag
  replaced with the decision; wiring-shape step 2 now states the required-ref
  signature change + the `:161` pass-down
  (`optimizer.OptimizeIncre(dag_tasks_cur, per_core_rta_cache_)`); `:151` from-scratch
  site marked UNCHANGED at 3b.
- `goal.md` next-step pointer: S1/S2 flag replaced with the decision + storage-on-
  derived note.
- This entry.

Next concrete step (gated on user commit of 3c + OK to code): implement 3b per the
DECIDED seam — `per_core_rta_cache_` member on `OptimizePA_Incre_with_TimeLimits`
(reset in `ResetIncumbentBaseline`) + required `PerCoreRTACache&` param on
`OptimizeIncre` (replacing `:239`/`:279` with `ComputeRTA_FullAndCache` + the
perf_coefficient-corrected assembly) + the `:161` pass-down + the differential test
(perf-pair task + chain fixture) + update `:258` test caller. NO source code written
this step.

## 2026-07-15 (Extra Ideas & Refinements Backlog updated)

Evaluated the staged implementation of the caching layer and discussed design improvements with the user. The user decided to add these suggestions to the backlog.

Appended four suggestions to the task records in `tasks.md` and `goal.md`:
1. **Unified Caching API (`EvaluateRTA_WithCache`)** to consolidate full computes, TL patching, priority patching, and exact-match reuse into a single entry point with auto-classification of status.
2. **Zero-Copy Order Derivation** to optimize `AnalyzeCacheReuse` by operating on task ID primitives instead of full `Task` structures and `FiniteDist` arrays.
3. **Allocation-Free Flat RTA Rebuilding** to reconstruct the flat prioritized output in `O(N)` using contiguous array indexing.
4. **Speculative Cache Copying** using "scratch copy on spec, swap on accept" pattern (very cheap at ~16KB map size).

Updated `tasks.md` and `goal.md` to document these backlog items as future design refinements.

## 2026-07-15 (API revision — role-2 reuse query redesigned, pre-commit)

The user reviewed the staged step-3c code (still uncommitted, HEAD `0c1f846b`)
and pushed back on the role-2 reuse-query surface with three objections:
1. They recalled asking for a **per-task vector** indicating each task's RTA
   reusability, not the per-core `CacheReuseInfo` struct.
2. `CacheReuseInfo`'s three conditionally-interdependent fields
   (`fully_reusable` / `same_core_partition` / `common_prefix_length`) are "not
   clear and a bit messy" against the end-goal — specifically the documented
   caveat that `common_prefix_length` is only actionable under
   `same_core_partition==true` (the partition-change TDD-RED from 3c).
3. `CacheConsistentWith` looked redundant.

The user then gave four design considerations, recorded verbatim in
`tasks.md` §"API revision" and applied to the staged code this session:

- **(R1)** `ClassifyReuse` returns a **per-task `enum class RTAReuseClass`
  vector`** (indexed by task id), not `CacheReuseInfo`. Three values —
  `RtaReuse` / `RecomputeWithHpPrefix` / `Recompute` — because reuse is not
  binary and the user anticipates ≥3 categories later. Per-task-by-id aligns
  with the flat `rtas` vector the optimizer consumes; the old per-core
  `fully_reusable`/`common_prefix_length` become derivable ("every task on the
  core is `RtaReuse`" / "count of leading `RtaReuse`").
- **(R2)** `PerCoreRTACache` is a **class** (private data + read accessors +
  helpers), not a struct. Centralizes the sorted_task_ids↔rta↔tl_vec↔
  hp_tasks_et_conv_vec alignment invariant. Read accessors (`ProcessorId`/
  `SortedTaskIds`/`TlVec`/`Rta`/`HpTasksEtConvVec`) + query helpers (`Size`/
  `PositionOfTask`/`ContainsTask`) + the `Populate` build path land now;
  patcher mutators arrive with steps 4/5.
- **(R3)** `CacheConsistentWith` is **dropped** — "cache exactly matches
  candidate" = "all tasks `RtaReuse`" via `ClassifyReuse`.
- **(R4)** `ClassifyReuse` **v0 is deliberately simple**: a task is `Recompute`
  if any changed task shares its `processorId` (conservative — whole changed
  core), else `RtaReuse`; empty cache → all `Recompute`. Does NOT exploit the
  within-core HP-prefix reuse point yet (unchanged prefix above a change is
  `Recompute`, not `RtaReuse`/`RecomputeWithHpPrefix`). That prefix refinement
  lands paired with the patchers (4/5); `RecomputeWithHpPrefix` is declared but
  NOT produced by v0. v0 is a correctness scaffold (API + shape + trivial
  TDD), not the perf payoff.

**Code changes (staged, NOT committed):**
- `RTA_Cache.h`: `struct PerCoreRTACache` → `class PerCoreRTACache` (private
  members, read accessors + `Size`/`PositionOfTask`/`ContainsTask` + `Populate`).
  `struct CacheReuseInfo` + `AnalyzeCacheReuse`/`CacheConsistentWith` decls →
  `enum class RTAReuseClass` + `ClassifyReuse(cache, dag, changed_task_ids)`.
  `ComputeRTA_FullAndCache` + the two patcher declarations UNCHANGED.
- `RTA_Cache.cpp`: `AnalyzeCacheReuse`/`CacheConsistentWith` bodies → v0
  `ClassifyReuse` (collect changed-task processorIds → mark same-core tasks
  `Recompute`, else `RtaReuse`; empty cache → all `Recompute`). The build path
  moved into `PerCoreRTACache::Populate` (sorts HP-first defensively so rta/
  hp_tasks_et_conv_vec align; looks up tl by task id directly since
  `time_limits` is id-indexed). The dead `DeriveCandidatePerCoreOrder`/
  `CoreOrder` helpers (served the prefix walk) are REMOVED. `ComputeRTA_
  FullAndCache` now drives `Populate` per core.
- `testRTA.cpp`: the 4 staged `AnalyzeCacheReuse_*` tests +
  `CacheConsistentWith_EmptyCache_IsFalse` → 4 `ClassifyReuse` v0 tests
  (identity → all `RtaReuse`; TL change on core 0 → core-0 tasks `Recompute`,
  core-1 `RtaReuse`; priority move on core 0 → same shape; empty cache → all
  `Recompute`). The partition-change test is DROPPED from v0 (same-processor
  check has no notion of migration; the prefix logic that motivated it is
  deferred — documented in `tasks.md` §"API revision" R4 note). The build/
  self-consistency/TL-differential tests move to the class accessors.

**Key invariant verified against source this session:** `UpdateTaskSetPriorities`
(`OptimizeSP_Base.cpp:28`) returns a fresh priority-sorted TaskSet (does NOT
mutate its input); `ProbabilisticRTA_TaskSet_SingleCore` scatters its returned
`rtas` by INPUT position but fills `hp_tasks_et_conv_vec` by SORTED position —
so `Populate` sorts defensively before calling SingleCore, making `rta_[i]`/
`hp_tasks_et_conv_vec_[i]`/`sorted_task_ids_[i]`/`tl_vec_[i]` all align at
sorted position i. Also re-confirmed the `dag.tasks[i].id == i` invariant
(asserted in `UpdateTaskSetPriorities:32`), which makes a per-task-id vector
well-defined (id == position).

**Verification:** `cmake --build . --target check.SP_OPT -j5` → 16/16 ctest
green (DEBUG, `libSP_OPTDebug.so`); the 7 cache-fixture tests all pass under
their new `ClassifyReuse_*` names (filter-run confirmed the new names execute —
not a stale-binary pass). The role-1 build (`ComputeRTA_FullAndCache`) remains
bit-identical to `ProbabilisticRTA_TaskSet` (the differential test is
unchanged). `ProbabilisticRTA_TaskSet`/`_SingleCore` UNCHANGED — still the live
path + the differential oracle (retired at step 3b).

The step-3c SCOPE (cache infrastructure, baseline-only 3b, Hazard A/B, the
DECIDED required-`PerCoreRTACache&` seam on `OptimizeIncre`) is UNCHANGED by
this revision — only the role-2 API shape moved. 3b implementation is still
gated on user commit of 3c + OK to code.

## 2026-07-15 — API rev 2 (self-supplied cache) + HOLD-OFF

Design-only session (no code written). The user reshaped the cache API a second
time, to a **self-supplied** form, then put `RTA_Cache` on HOLD to resolve a
more-important issue first. Full record in `tasks.md` §"API revision 2".

**User's 5 reshaping decisions (rev 2):**
1. `PerCoreRTACache` is **whole-taskset across all cores** (NOT the rev-1
   per-core map), self-supplied. `rta_` is FLAT indexed by task id (same order
   `ProbabilisticRTA_TaskSet` returns). Stores `pa_` (NOT `sorted_task_ids` —
   derivable in impl). Stores a **copy of the champion `dag_tasks_`** (→
   `shared_ptr` later) so the cache is self-supplied: pass a new
   `(dag, tl, pa)` triple, get back a reuse vector.
2. `Populate` → **`Initialize(dag_tasks, pa, tl)`** (cold-start full RTA).
3. **`UpdateFullCache(dag_tasks, pa, tl, rtas)`** — cheap adopt-commit (the
   cache is bundled with a champion; re-`Initialize` per adopt would defeat the
   cache). Named by the user, unrelated to "champion".
4. **`CheckTaskSetRTAReuse(dag_tasks, pa, tl) → vector<RTAReuseClass>`** member
   (replaces rev-1 free `ClassifyReuse`; caller no longer passes
   `changed_task_ids` — the cache diffs the triple vs its stored champion).
   User also wants a **"how many tasks' ET changed"** helper (derives the
   changed-task set from `(dag, pa, tl)`).
5. **`GetRTA_OneTask(dag, pa, tl, task_id, reuse_level) → FiniteDist`** member
   (selective pull; NOT the hot-loop path). Plus free
   `EvaluateRTA_WithCache(dag, pa, tl, cache)` outside the class (the hot-loop
   batch entry; consolidates rev-1 `ComputeRTA_FullAndCache` + the two patchers;
   read-only, commit via `UpdateFullCache`).

**`tl` seam — DECIDED option (a):** pass `time_limits` directly from outside.
Verified against source: `EvaluateTimeLimitConfig_ScratchOrIncre`
(OptimizeSP_TL_Incre.cpp:142-146) takes `time_limits` as a param, in scope at
the `:161` `OptimizeIncre` call site → seam becomes
`OptimizeIncre(dag, tl, cache)` (required `cache&` + `tl`). Option (b)
(cache-derives-tl-from-baked-dists) REJECTED. Confirms the user's "we have a
time limit vector ready to use" — yes.

**5 open questions left for the user** (in `tasks.md` §"API revision 2" →
"Remaining questions"): Q1 rename `PerCoreRTACache`→`RTACache`?; Q2
`GetRTA_OneTask`'s `reuse_level` trusted or re-derived?; Q3 changed-ET-count
helper public or internal?; Q4 member `GetRTA_OneTask` shadows free
`GetRTA_OneTask` in RTA.h — rename to `RTAForTask`?; Q5 is the "more-important
issue" logged anywhere (blocker id)?

**STATUS: ON HOLD.** No `RTA_Cache.h`/`.cpp` written for rev 2. When work
resumes: answer Q1–Q5 → write `RTA_Cache.h` to the locked surface → implement
3b. The staged 3c code on disk still carries the rev-1 surface (uncommitted).

