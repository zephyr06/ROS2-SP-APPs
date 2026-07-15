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
