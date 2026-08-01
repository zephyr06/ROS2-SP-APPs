# INCR per-activation ET grows with ReoptimizationPeriod — root cause & fix

> **Status: ROOT CAUSE FOUND; FIX A + FIX B + LATENT BUG APPLIED & TDD-VERIFIED
> (2026-07-04); all committed. Runtime A/B re-run DONE 2026-07-04 19:41 —
> PARTIAL PASS.** This file is the final record for the issue originally
> documented as "KNOWN ISSUE (reproduced, NOT yet fixed)" in
> `agents/finished_tasks/P24_task.md` and the memory
> `p25-incr-et-grows-with-period.md` (both now updated to reflect the fix +
> A/B verdict). The earlier record's hypothesis ("incremental path between
> reopts dominates, incumbent drifts") was *directionally* correct but
> *mechanistically* wrong: the cost is NOT proportional to staleness — it
> saturates at 6 diff-tasks the very first incremental interval and stays
> there. The real driver is a diff-baseline that is frozen at the reopt
> interval, plus a per-variation `ObtainSP_DAG` cost the from-scratch path
> avoids. Details + verified fix below.
>
> **Commits:** `986a9cfe` (Fix A — `OptimizeIncre` advances `dag_tasks_`),
> `de4e9636` (Fix B `{-1}`-only skip + zero-work fallback + latent
> `SeedStateFromIncumbent` `sp_parameters_` carry). HEAD = `de4e9636`.
> **16/16 ctest green** in DEBUG (`testIncreOpt_w_TL` 29/29).
>
> **A/B verdict (2026-07-04 19:41, rebuilt release binary 19:34):** Fix A+B
> **eliminated the pathological 3× growth** — P10/P30/P60 collapsed from
> 301–343 → 88–91 ms/act (ts0) and 185–194 → 72–75 (ts2), toward the
> P1≈INCR_SCRATCH floor; P60/P1 ratio 3.4×→1.47× (ts0), 2.1×→1.13× (ts2);
> numerical pass bar (P30/P60 vs P10 ≤ 1.1–1.3×) met on both (ts0 1.035×,
> ts2 0.96×). BUT the **literal directional flip** `INCR_P1 ≥ P10 ≥ P30 ≈ P60`
> was **NOT** met — P1 is still the cheapest INCR arm (62 < 88–91 ts0;
> 65 < 72–75 ts2); the direction flattened, not flipped. This is the §4a/§6
> prediction for a Fix A+B-only deployment: the residual P1 < P60 gap is the
> per-variation `ObtainSP_DAG` asymmetry that **Fix C** (deferred in §6) was
> meant to remove. Fix A+B killed the frozen-baseline pathology (the actual
> bug); Fix C is the lever for a full flip. See `agents/finished_tasks/P24_task.md`
> § "DONE: A/B re-run" for the full before/after tables + procedure.
>
> **Update (2026-07-04, later): Fix A APPLIED & VERIFIED via TDD.** Added
> failing unit test `CompareAndKeepSynthetic.OptimizeIncre_AdvancesPrevOptimizerDagTasks`
> in `tests/testIncreOpt_w_TL.cpp` (asserts `prev_optimizer_.dag_tasks_` advances
> to the current interval's DAG after `OptimizeIncre_w_TL`); it FAILS on unfixed
> code (`prev_optimizer_.dag_tasks_.tasks[1]` stays at bootstrap ET ~50.0 instead
> of the mutated 1234.0) and PASSES after Fix A. Full ctest 16/16 green in DEBUG
> build. Fix A is the single-line `dag_tasks_ = dag_tasks_update;` before
> `return opt_pa_` in `OptimizeIncre` (§6).
>
> **Update (2026-07-04, latest: Fix B APPLIED & VERIFIED via TDD + a latent
> `SeedStateFromIncumbent` bug fixed as a prerequisite.** Three changes this pass:
> (1) **Latent bug:** `SeedStateFromIncumbent` seeded `prev_optimizer_`'s
> dag/opt_pa_/opt_sp_ but NOT `sp_parameters_` → `IfInitialized()` (which only
> checks `!opt_pa_.empty()`) let the incremental branch run with an EMPTY
> `sp_parameters_` → `OptimizeIncre`'s `thresholds_node.at(id)` threw
> `_Map_base::at`. Masked in production (UpdateRecords' full-copy usually fires
> between a reopt and the next incremental call, populating sp_parameters_ as a
> side effect) but it blocks Fix B's fallback in the all-`{-1}` no-improvement
> case. Fix: one line, `prev_optimizer_.sp_parameters_ = sp_parameters_;` in
> `SeedStateFromIncumbent`. Verified safe in isolation (full ctest green except
> the two Fix B red tests, which fail on assertions not crashes).
> (2) **Fix B `{-1}`-only skip:** in `PerformCoordinateDescentForTaskConfigOpt`,
> skip a task whose only TL option is `-1` (`opts.size()==1 && opts[0]==-1.0` —
> the exact no-`timePerformancePairs` predicate). The main cost win on the reused
> P25 tasksets (every task is `{-1}`-only).
> (3) **Fix B zero-work fallback:** when the descent runs zero evals (all tasks
> `{-1}`-only), run ONE `EvaluateTimeLimitConfig_ScratchOrIncre` with the
> incumbent `time_limits` so UpdateRecords fires and Fix A's `dag_tasks_`
> advance propagates into `prev_optimizer_` (otherwise the frozen-baseline
> pathology Fix A fixes silently returns). Guarded on `!dag_tasks_.tasks.empty()`
> and `!any_eval_ran`.
> Two TDD tests added (`PerformCoordinateDescent_AllMinusOneOnly_…`,
> `PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`) — both FAIL on
> latent-bug-fixed-but-Fix-B-not-yet code (deltas 2 and +1) and PASS after Fix B.
> **16/16 ctest green in DEBUG build** (`testIncreOpt_w_TL` 29/29). The runtime
> A/B re-run (§1 table flip) is still PENDING — not part of this TDD cycle.

---

## 1. Symptom (reproduced with the current binary, 2026-07-04 10:52 build)

Serial, contention-free repro (`simulation_experiments/repro_et_grows_with_period.py`,
ts0, 2 reps, min):

| arm          | per-act (ms) | wall (s) |
|--------------|--------------|----------|
| BF           | 1095.5       | 32.9     |
| **INCR_P1**  | **101.9**    | 3.06     |
| INCR_P10     | 294.0        | 8.82     |
| INCR_P30     | 341.8        | 10.25    |
| INCR_P60     | 316.7        | 9.50     |
| INCR_SCRATCH | 103.1        | 3.09     |

Pattern: `INCR_P1 ≈ INCR_SCRATCH` (fastest, ~102 ms) `< INCR_P10 (~294) < INCR_P30 ≈ INCR_P60 (~320–342, plateau)`.

The theoretically-impossible part: `INCR_P1` reoptimizes **every** interval (the
supposedly-expensive wide-radius from-scratch search) yet is the *fastest* INCR
arm, tied with the amnesiac `INCR_SCRATCH`. So the wide from-scratch search is
**cheap** (~40 ms/interval); the **incremental** path between reopts is what's
expensive (~210–265 ms/interval). This file explains why.

---

## 2. Method: per-interval instrumentation

Added lightweight, `debugMode`-gated instrumentation (inert when
`debugMode==0`, the production default — confirmed 16/16 ctest green in the
DEBUG build with it in place):

- `sources/Safety_Performance_Metric/SP_Metric.cpp` / `.h`: a global counter
  `g_incr_et_debug_sp_dag_calls` incremented on every `ObtainSP_DAG` call.
- `sources/Optimization/OptimizeSP_Incre.cpp`: `OptimizeIncre` prints
  `ndiff=<n tasks whose ET dist changed vs the carried baseline>`,
  `nvar=<priority variations generated>`, `sp_dag_calls=<ObtainSP_DAG calls
  during this one OptimizeIncre invocation>`.
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`: the INCR
  branch prints, per interval, `path=REOPT|INCRE`, total `sp_dag_calls` for the
  interval, and `opt_ms` (wall of `Optimize_w_TL_ScratchOrIncre`).

Reproduce the trace (after `cmake --build release`, with `debugMode: 1` in
`sources/parameters.yaml`):
```
release/tests/RunOrchestrator <ts_dir> <out> INCR_P60 10000 1   # stderr has the [INCR-ET-DBG] lines
release/tests/RunOrchestrator <ts_dir> <out> INCR_P1  10000 1
```
Trace artifacts saved under
`simulation_experiments/optimizer_comparison/et_repro/dbg_trace_ts0/{P1,P10,P30,P60}/stderr.txt`.

---

## 3. The numbers that nail it

### INCR_P1 (every interval = REOPT)
```
interval=0..29  path=REOPT  sp_dag_calls=7  opt_ms≈40   (every interval identical)
OptimizeIncre calls: 0
```
Per REOPT interval = **7** `ObtainSP_DAG` calls ≈ **40 ms**.

### INCR_P60 (interval 0 = REOPT, intervals 1–29 = INCRE)
```
interval=0  path=REOPT  sp_dag_calls=7    opt_ms≈37
interval=1  path=INCRE  sp_dag_calls=138  opt_ms≈209
interval=2  path=INCRE  sp_dag_calls=132  opt_ms≈175
...
interval=29 path=INCRE  sp_dag_calls=132  opt_ms≈279
OptimizeIncre calls: 174  (= 29 incremental intervals × 6 calls/interval)
EVERY OptimizeIncre call: ndiff=6  nvar=21  sp_dag_calls=21
```
Per INCRE interval = **132** `ObtainSP_DAG` calls ≈ **210–265 ms**.

### Same for P10 / P30 (saturation check)
| arm | REOPT intervals | INCRE intervals | `ndiff` on every INCRE | avg INCRE opt_ms | avg REOPT opt_ms |
|-----|-----------------|-----------------|------------------------|------------------|------------------|
| P1  | 30 | 0  | —         | —       | ~40  |
| P10 | 3  | 27 | always 6  | 258.7   | 42.7 |
| P30 | 1  | 29 | always 6  | 263.5   | 42.3 |
| P60 | 1  | 29 | always 6  | ~210    | 37   |

**Three facts, all verified from the traces:**
1. **`ndiff = 6` on every single `OptimizeIncre` call** (174/174 for P60,
   162/162 for P10, 174/174 for P30). It never varies, never grows with
   staleness — it saturates at 6 (= all tasks) the *first* incremental interval.
2. **Per-interval INCRE cost is ~constant (~210–265 ms)** regardless of how long
   the incumbent has aged. There is no "grows-with-staleness" term.
3. **The plateau** (P30 ≈ P60) is just "fraction of INCRE intervals → 1": total
   wall ≈ `n_reopt·40ms + n_incre·250ms`. As the period grows, `n_incre/30 → 1`,
   so total → `30·250ms ≈ 7.5 s` (+ sim overhead → ~9–10 s observed). P10 is
   slightly *cheaper* per-act than P30/P60 only because its 3 REOPT intervals
   (40 ms each) dilute the mean. More reopts → *lower* average — the exact
   inverse of "reopt is the expensive step."

---

## 4. Root cause (the mechanism, end to end)

### 4a. The two paths use different SP-evaluation kernels

- **From-scratch path** (`OptimizeFromScratch`, the REOPT/wide-search kernel,
  `sources/Optimization/OptimizeSP_Incre.cpp:74`): Audsley-style beam search,
  K=`Layer_Node_During_Incremental_Optimization`=2 partial paths. Each partial
  path, at each of the N priority levels, extends by each not-yet-assigned task
  via `AssignAndUpdateSP` → `UpdateSP` → **`GetRTA_OneTask`** — a *single-task*
  RTA (cheap). The full `ObtainSP_DAG` is called **once**, at the very end
  (`EvaluateSPWithPriorityVec`, line 136), to score the final PA. So a
  from-scratch eval = O(K·N²) cheap `GetRTA_OneTask` calls + 1 `ObtainSP_DAG`.

- **Incremental path** (`OptimizeIncre`, `OptimizeSP_Incre.cpp:233`): diffs the
  carried baseline DAG against the new DAG via `FindTaskWithDifferentEt`
  (line 243), and for **each** diff task generates up to **N** priority
  variations (`FindPriorityVec1D_Variations`), scoring **each** variation with
  `EvaluateSPWithPriorityVec` → **`ObtainSP_DAG`** (line 254). So an incremental
  eval = `ndiff` × ~N × cost(`ObtainSP_DAG`), and `ObtainSP_DAG` is the
  expensive kernel (full `ProbabilisticRTA_TaskSet` over all tasks + all
  reaction-time chains — `SP_Metric.cpp:89`).

**The asymmetry is the whole story.** `OptimizeFromScratch` dodges `ObtainSP_DAG`
during its search; `OptimizeIncre` pays it per variation.

### 4b. Why `ndiff = 6` on every incremental call (the frozen baseline)

`OptimizeIncre` diffs `this->dag_tasks_` against the passed-in `dag_tasks_update`.
In the incremental path (`EvaluateTimeLimitConfig_ScratchOrIncre`,
`from_scratch=false`, `OptimizeSP_TL_Incre.cpp:137-142`):
```
OptimizePA_Incre optimizer = prev_optimizer_;          // copy carries prev_optimizer_.dag_tasks_
optimizer.OptimizeIncre(dag_tasks_cur);                // diffs prev_optimizer_.dag_tasks_ vs dag_tasks_cur
```
So the diff baseline is **`prev_optimizer_.dag_tasks_`**. When is that updated?
- At a **reopt**: `SeedStateFromIncumbent` → `prev_optimizer_.UpdateDAG(dag_with_tl)`
  (`OptimizeSP_TL_Incre.cpp:311`).
- In `UpdateRecords`: `prev_optimizer_ = optimizer` (line 111) — but
  `OptimizeIncre` **never assigns `this->dag_tasks_`** (grep confirms: the only
  writes to `dag_tasks_` on an `OptimizePA_Incre` are the constructor and
  `UpdateDAG`). So the copy's `dag_tasks_` stays equal to `prev_optimizer_.dag_tasks_`,
  and `prev_optimizer_ = optimizer` writes back the *same* `dag_tasks_` it had.
  ⇒ **`prev_optimizer_.dag_tasks_` is frozen between reopts.**

What is it frozen *to*? At the reopt, `dag_with_tl = UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev)`
— the reopt interval's DAG with the reopt-interval TL applied. So the baseline
is "the reopt interval's (TL-applied) DAG," and every subsequent incremental
interval diffs against that one frozen snapshot.

### 4c. Why all 6 tasks always differ (no TL options → raw drift is visible)

The reused P25 tasksets have **no `timePerformancePairs`** on any task
(`generator_config.json` has no TL pairs; verified:
`taskset_characteristics_interval_*.yaml` lists `execution_time_mu/sigma/min/max`
but zero `timePerformancePairs`). Consequences:
- `RecordCloseTimeLimitOptions` returns `{-1}` for every task
  (`OptimizeSP_TL_BF.cpp:75-77`), so `time_limit_option_for_each_task_[i] = {-1}`
  for all i. Every task's TL is **always -1**.
- `UpdateExtDistBasedOnTimeLimit` with all-`-1` returns the DAG **unchanged**
  (it only swaps `execution_time_dist` when `time_limit[i] != -1`). So
  `dag_tasks_cur` = the raw current DAG, and `prev_optimizer_.dag_tasks_` = the
  raw reopt-interval DAG.
- The raw ET params **drift every interval** for every task (verified from the
  YAMLs: `execution_time_mu`, `_min`, `_max` all change interval-to-interval,
  by far more than the `FiniteDist::operator==` tolerance of `1e-1`).

⇒ `FindTaskWithDifferentEt` returns all 6 tasks on **every** incremental call,
from the very first one. `ndiff=6` always, independent of how stale the
incumbent is. That is why cost **saturates immediately** rather than growing
with the period — the earlier "staleness grows with period" intuition was wrong
about the mechanism even though it pointed at the right path.

### 4d. The per-interval arithmetic (matches the traces exactly)

- **REOPT interval** = `ReOptimizePeriodic`:
  `SeedIncumbentBaseline` (1 `ObtainSP_DAG` for the RM/incumbent baseline eval)
  + coordinate descent over 6 tasks × 1 TL option (`-1`) = 6
  `EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=true)` calls, each doing
  `OptimizeFromScratch` (0 `ObtainSP_DAG` during the search) + 1 final
  `EvaluateSPWithPriorityVec` = 1 `ObtainSP_DAG`.
  **Total = 1 + 6 = 7 `ObtainSP_DAG` ≈ 40 ms.** ✓ (matches P1 trace: 7 calls)

- **INCRE interval** = `OptimizeIncre_w_TL`:
  coordinate descent over 6 tasks × 1 TL option (`-1`) = 6
  `EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=false)` calls, each
  running `OptimizeIncre` → `ndiff=6`, 21 variations × 1 `ObtainSP_DAG` + 1
  initial `EvaluateSPWithPriorityVec` = 22 `ObtainSP_DAG` per call.
  **Total = 6 × 22 = 132 `ObtainSP_DAG` ≈ 210–265 ms.** ✓ (matches P60 trace)

So INCRE/REOPT per-interval cost ratio ≈ 132/7 ≈ 19× in `ObtainSP_DAG` calls,
≈ 5–6× in wall time (the rest of the interval is sim overhead present in both).

---

## 5. Is it a *logging* issue or an *implementation* issue?

**Both, but the implementation issue is primary; the "logging" framing is a
red herring that the earlier record half-caused.**

- **Not a measurement artifact.** The serial repro (one process at a time, no
  parallel-worker contention) reproduces the pattern; the per-interval
  instrumentation confirms the wall-time difference is real `ObtainSP_DAG` work,
  not wall-clock noise. `RunOrchestrator`'s `ExecutionTime_s` is honest.
- **Not the `duration_ms` arg footgun** (the [[runorchestrator-duration-arg-semantics]]
  bug): the repro passes `10000` (correct per-interval horizon), and the
  instrumentation counts actual kernel calls, not wall time alone.
- **It IS an implementation issue** — two of them, in fact:
  1. **Frozen diff baseline** (§4b): `prev_optimizer_.dag_tasks_` is never
     advanced by the incremental path, so the diff is always "reopt-interval DAG
     vs current DAG" and (with drifting ET) sees all tasks as changed every
     time. The incremental path was designed to diff *consecutive* intervals
     (cheap: ~0–2 tasks change), but the baseline is frozen at the reopt, so it
     diffs *stale* vs *fresh* (expensive: all tasks change).
  2. **Per-variation `ObtainSP_DAG`** (§4a): `OptimizeIncre` re-scores every
     priority variation with the full DAG SP kernel, while `OptimizeFromScratch`
     avoids that during its search. Even with a correct consecutive-interval
     diff (small `ndiff`), each remaining diff task still costs N×`ObtainSP_DAG`.
  3. **No-op search when there are no TL options** (§4c): with zero
     `timePerformancePairs`, the coordinate descent has exactly one TL option
     (`-1`) per task, so there is *nothing to search* — yet each of the 6
     per-task evals still runs the full `OptimizeIncre` priority sweep. The
     incremental path does 6 full priority-variation sweeps per interval for
     zero TL benefit. (This is why `INCR_NO_TL` — which sets
     `disable_time_limit_opt` and short-circuits to a single eval — would be
     fast; the INCR path doesn't take that fast path even when every task's TL
     option set is `{-1}`.)

The earlier record's "logging vs implementation" hedge is resolved: it is an
implementation issue, and the fix targets §4b/§4c.

---

## 6. Fix

### Fix A (primary, correct, minimal): advance `prev_optimizer_.dag_tasks_` on the incremental path so the diff is consecutive-interval, not stale-vs-fresh.

`OptimizeIncre` should diff against the **previous interval's** DAG, not the
reopt interval's. The clean way: after `OptimizeIncre` runs (it already
received `dag_tasks_update` as the new DAG), set `this->dag_tasks_ =
dag_tasks_update` so the copy written back into `prev_optimizer_` by
`UpdateRecords` carries the *current* interval's DAG forward. Then the next
incremental interval diffs consecutive intervals (small `ndiff`) instead of
fresh-vs-frozen-reopt (always 6).

In `sources/Optimization/OptimizeSP_Incre.cpp`, `OptimizeIncre`, after the diff
loop (before `return opt_pa_`):
```cpp
// Advance the carried baseline to THIS interval's DAG so the next incremental
// call diffs consecutive intervals (small ndiff) rather than the frozen reopt
// interval against the current one (always ndiff=N when ET drifts). Without
// this, prev_optimizer_.dag_tasks_ is only updated at reopts (SeedStateFromIncumbent),
// so every intervening incremental step re-diffs all tasks that drifted since
// the reopt — O(N) ObtainSP_DAG calls per interval even when consecutive
// intervals differ in ~0-2 tasks.
dag_tasks_ = dag_tasks_update;
return opt_pa_;
```
(Also keep the existing `EvaluateSPWithPriorityVec(dag_tasks_update, ...)` at the
top — that's the SP reset, unrelated.)

**Expected effect:** `ndiff` drops from 6 to ~0–2 (only tasks that changed ET
*in the last interval*), so each `OptimizeIncre` does ~0–2×N variations instead
of 6×N, and per-INCRE-interval `ObtainSP_DAG` calls drop from ~132 to ~0–44.
INCRE intervals become cheaper than REOPT intervals (the incremental path's
whole reason for existing). ET should then **decrease** as the period grows
(more cheap incremental intervals), matching the original expectation.

**Caveat / must-verify:** `dag_tasks_` is also read by `OptimizeIncre` itself
*before* this assignment (the `FindTaskWithDifferentEt(dag_tasks_, ...)` call).
That read must see the *previous* interval's DAG (which it does: `dag_tasks_`
still holds the prior value until we overwrite it at the end). The assignment
must come *after* the diff loop. Placing it right before `return` is correct.

**Also verify** the from-scratch path is unaffected: `OptimizeFromScratch` does
not use `dag_tasks_` for diffing (it builds from the constructor's DAG), and
`ReOptimizePeriodic` re-seeds `prev_optimizer_.dag_tasks_` via
`SeedStateFromIncumbent`/`UpdateDAG` anyway, so the reopt→incremental handoff
still starts the diff chain at the reopt interval's DAG (correct: the first
incremental interval after a reopt legitimately diffs reopt-vs-current, which
is a real diff and should be searched).

### Fix B (secondary, independent win): short-circuit the coordinate descent when a task has only one TL option (`{-1}`), and skip `OptimizeIncre` when `ndiff == 0`.

Even with Fix A, when a task's `time_limit_option_for_each_task_[i] == {-1}`
there is exactly one TL value and no search to do — the per-task eval should
not run a full priority sweep. And when `ndiff == 0` (no ET changed since the
previous interval, common after Fix A), `OptimizeIncre` should return early
without any `ObtainSP_DAG` variation calls.

In `sources/Optimization/OptimizeSP_Incre.cpp`, `OptimizeIncre`, near the top
(after the SP reset, before the diff loop):
```cpp
std::vector<DiffObj> tasks_with_diff_et =
    FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update);
if (tasks_with_diff_et.empty()) {
    dag_tasks_ = dag_tasks_update;   // Fix A: still advance the baseline
    return opt_pa_;                  // nothing changed → keep the incumbent PA
}
```
And in `PerformCoordinateDescentForTaskConfigOpt`
(`sources/Optimization/OptimizeSP_TL_Incre.cpp:179`), skip a task whose option
set is `{-1}` only (no real search):
```cpp
for (size_t idx : sorted_indices) {
    const auto& opts = time_limit_option_for_each_task_[idx];
    if (opts.size() == 1 && opts[0] == -1.0) continue;   // no TL freedom; PA
                                                         // search is OptimizeIncre's job, not the descent's
    ...
}
```
(Verify this doesn't break the `disable_time_limit_opt` path, which already has
its own short-circuit via `OptimizeWithTimeLimitOptDisabled`.)

### Fix C (optional, structural): make `OptimizeIncre`'s per-variation scoring incremental too.

`OptimizeIncre` re-scores each priority variation with full `ObtainSP_DAG`. A
deeper fix would re-evaluate only the affected task's RTA (the variation moves
one task's priority, so only that task and the tasks it jumps over change).
This is a larger refactor and not needed to resolve the issue; Fix A + B bring
INCRE below REOPT, which is the goal. Listed for completeness.

> **UPDATE 2026-07-04 (N=8/10 ET profiling pass):** the "Fix A + B bring INCRE
> below REOPT" claim above is **NOT borne out by the runtime A/B** (see §7 and
> the N=8/10 profiling below). At N=10 REOPT (`INCR_SCRATCH`) is the *cheapest*
> arm (229 ms ≈ 0.98× P1), not slower than INCR; the structural asymmetry Fix C
> addresses is the real residual gap and **grows with N**. Fix C is therefore
> upgraded from "optional, listed for completeness" to "the lever that would
> make INCR actually beat REOPT at larger N". The directional heuristic the
> user proposed (ET↑ → only try lower priority; ET↓ → only try higher priority)
> is **already implemented** — see `AnalyzePriorityChangeStatus` +
> `FindPriorityVec1D_Variations` (`OptimizeSP_Incre.cpp:213-231, 180-212`); it
> is why `nvar ≈ N/2` not `N²`. The remaining cost is the *full-DAG re-eval per
> variation*, not the variation count. Details + measured call counts in §10.

---

## 10. RTA call-count profiling (2026-07-04, N=10, post-Fix-A+B, HEAD `de4e9636`)

**Motivation.** The N=8/10 ET profiling (§7) showed ET grows monotonically with
period at N=10 (P1 234 < P10 262 < P30 268 < P60 273) and REOPT is the cheapest
arm. To confirm the cause is "INCR makes more RTA calls than scratch" (and not,
e.g., per-call cost growth), the `debugMode`-gated instrumentation added in the
Fix-A/B pass was run on N=10 taskset 0 with `debugMode=1`. This is a
*measurement* pass — no code changed; only `parameters.yaml:30` was flipped 0→1
for the duration of the run and reverted after.

**Setup.** `./release/tests/RunOrchestrator <N10_ts0> <out> <arm> 10000 1` for
`arm ∈ {INCR_P1, INCR_P10, INCR_SCRATCH}`. stderr carries the `[INCR-ET-DBG]`
lines (per-interval path + `sp_dag_calls` + `opt_ms` from
`SimulationOrchestrator.cpp:309-316`, and the `ndiff/nvar/sp_dag_calls` line
from `OptimizeIncre` at `OptimizeSP_Incre.cpp:269-276`); stdout carries the
per-eval `EvaluateSPWithPriorityVec` flood and is discarded.

### 10a. Measured call counts (N=10, taskset 0, 30 intervals)

| arm | path / interval | avg `sp_dag_calls` per interval | avg `opt_ms` per interval | `ms / call` |
|---|---|---|---|---|
| `INCR_P1` | all 30 REOPT | **2.0** | 13.3 | 6.65 |
| `INCR_P10` | 3 REOPT (intervals 0,10,20) | 2.0 | 12.1 | — |
| `INCR_P10` | 27 INCRE | **45.1** | 76.2 | 1.69 |
| `INCR_SCRATCH` | all 30 REOPT (amnesiac) | (not instrumented; ~2) | ~12 | — |

**Totals across the sim:** INCR_P10 = 3×2 (REOPT) + 27×45 (INCRE) ≈ **1221
`ObtainSP_DAG` calls**; INCR_P1 = 30×2 = **60 calls**. So INCR_P10 does
~**20× more** full-DAG SP evaluations than INCR_P1 — exactly the asymmetry §4a
predicts, now measured. REOPT (`OptimizeFromScratch`) uses the cheap
`GetRTA_OneTask` per beam node (K=2 beam, N steps → ~2·N cheap calls) plus
**1** `ObtainSP_DAG` at the end (`OptimizeSP_Incre.cpp:136`) — hence the steady
`sp_dag_calls=2` per REOPT interval (1 from `SeedIncumbentBaseline`'s baseline
re-eval at `OptimizeSP_TL_Incre.cpp:370`, 1 from the `OptimizeFromScratch` final
eval). The INCRE path pays `ObtainSP_DAG` **per variation**.

### 10b. Why `nvar ≈ 45` at N=10 (not N²=100)

`FindPriorityVec1D_Variations` (`OptimizeSP_Incre.cpp:180`) already restricts
the sweep to ONE direction by `AnalyzePriorityChangeStatus` (lines 213-231):
ET↑ → `Decrease` (sweep `[old_idx .. N-1]`, ~N/2 positions) unless the task has
the unique-highest weight; ET↓ → `Increase` (sweep `[0 .. old_idx]`, ~N/2). So
the user's proposed heuristic — "if ET increases, only try decreasing priority;
if ET decreases, only try increasing" — **is already the implementation**.
`ndiff` is ~7-8 (most tasks' ET drifts across an interval), and each diff task
contributes ~N/2 ≈ 5 variations → 8×5.6 ≈ 45. Confirmed by the distribution:
`ndiff=8 nvar=45` is the modal row (3 intervals); `nvar` ranges 27-62.

The **directional pruning is already done.** The remaining cost is NOT the
variation count — it is that **each variation re-runs `ProbabilisticRTA_TaskSet`
on the full task set** (`SP_Metric.cpp:55`, via `EvaluateSPWithPriorityVec` →
`ObtainSP_DAG` → `ObtainSP_TaskSet`). At N=10 with 0 chains (these tasksets
have no `chains` YAML key → `GetRTDA_Dist_AllChains` is a no-op), the entire
`ObtainSP_DAG` cost is the per-task RTA convolution, recomputed from scratch
for all 10 tasks on every variation even though a 1D priority move only changes
the RTA of the moved task and the tasks it jumps over.

### 10c. Levers to cut INCR's per-variation cost (ranked)

1. **Incremental RTA reuse (Fix C proper, largest win).** A 1D priority move of
   task `t` from index `i` to `j` only changes the RTA of `t` and the tasks in
   `[min(i,j) .. max(i,j)]` — the hp-set of everyone outside that range is
   unchanged. `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:58`) already
   computes RTA in priority order with a running `hp_tasks_et_conv`
   convolution; re-evaluating only the affected slice (and reusing the prefix
   convolution up to `min(i,j)`) would cut each variation from O(N) convolutions
   to O(|j-i|). Expected: ~5-10× fewer RTA convolutions per variation at N=10.
   This is the structural fix that makes INCR beat REOPT at large N.

2. **Cache the baseline RTA across variations within one `OptimizeIncre` call.**
   Currently each of the ~45 variations calls `EvaluateSPWithPriorityVec`
   independently, and each rebuilds `TaskSet tasks_eval =
   UpdateTaskSetPriorities(...)` + a fresh `ProbabilisticRTA_TaskSet`. The
   baseline (`opt_pa_`, evaluated once at line 239) is already computed; the
   variations could diff against a cached per-task RTA vector instead of
   recomputing the whole set. Simpler than (1) — a per-call memo, not a
   refactor of the RTA kernel — but only a constant-factor win (~2-3×), not
   asymptotic.

3. **Early-exit the variation sweep when SP stops improving.** The sweep is
   greedy per diff-task; if the first 1-2 variations of a task don't beat
   `opt_sp_`, the rest usually don't either (the SP landscape is smooth in
   priority position for these tasksets). A "no improvement after K tries →
   skip remaining positions for this task" heuristic trades a small SP loss for
   a large call-count cut. Risk: a real optimum further along gets missed.
   Cheapest to implement; least principled.

4. **Reduce `ndiff` itself.** `ndiff≈8` at N=10 means 8/10 tasks' ET
   distributions differ between consecutive intervals — the tasksets are
   high-variance (Gaussian ET, σ wide enough that `FindTaskWithDifferentEt`
   fires on most tasks every interval). If the ET-change predicate were made
   tolerance-banded (ignore sub-σ drift), `ndiff` would drop and `nvar` with
   it. This is a taskset-property lever, not an algorithmic one — it depends on
   what "different ET" should mean semantically.

**Recommended order:** (2) first (low-risk constant-factor win, validates the
caching infra), then (1) (the structural fix; the real answer). (3) and (4) are
dials to tune after (1)/(2) land, not first-line fixes.

### 10d. Artifacts (this profiling pass)

- Traces: `/tmp/etprof_dbg/{INCR_P1,INCR_P10,INCR_SCRATCH}/{stdout,stderr}` (not
  committed; reproducible by flipping `parameters.yaml:30` to 1 and re-running).
- N=10 tasksets (reused from §7): `simulation_experiments/optimizer_comparison/
  et_repro/et_profiling_tasks10_dur300/taskset_{0,1,2}/`.
- Aggregated ET (mean ± stdev across 3 tasksets, `debugMode=0`): see §7 table.
- `parameters.yaml:30` reverted to `debugMode: 0` after the run.

### Recommended scope

Ship **Fix A** (primary, ~2 lines, restores the incremental path's reason to
exist) **+ Fix B** (the `ndiff==0` early-return and the `{-1}`-only skip,
~6 lines, avoids no-op sweeps). Defer Fix C. After the fix, re-run
`repro_et_grows_with_period.py` on ts0/ts2 and confirm the ordering flips to
`INCR_P1 ≥ INCR_P10 ≥ INCR_P30 ≈ INCR_P60` (ET non-increasing in period), and
that 16/16 ctest stays green (especially `testOptimizeIncrePA`,
`testIncreOpt_w_TL` which exercise `OptimizeIncre` and the TL coordinate
descent).

---

## 7. Why the earlier "staleness grows with period" hypothesis was half-right

It correctly blamed the incremental path between reopts (not the reopt step).
But it predicted cost *grows* with staleness → would predict P60 > P30 > P10
monotonically. The data shows P30 ≈ P60 (plateau) and P10 < P30. The
instrumentation resolves this: cost is set by `ndiff`, which saturates at N=6
the *first* incremental interval (because the diff baseline is frozen at the
reopt and ET drifts every interval for every task). Once `ndiff` is saturated,
extra period length adds zero cost → plateau. The hypothesis was right about
*which path* and wrong about *why it scales* — it scales with "is the
incremental path exercised at all," not with "how long has the incumbent aged."

---

## 8. Artifacts

- Instrumentation (this pass, `debugMode`-gated, inert in production):
  - `sources/Safety_Performance_Metric/SP_Metric.{h,cpp}` — `g_incr_et_debug_sp_dag_calls`
  - `sources/Optimization/OptimizeSP_Incre.cpp` — `OptimizeIncre` `ndiff/nvar/sp_dag_calls` line
  - `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` — per-interval `path/sp_dag_calls/opt_ms` line
- Traces: `simulation_experiments/optimizer_comparison/et_repro/dbg_trace_ts0/{P1,P10,P30,P60}/stderr.txt`
- Repro (unmodified): `simulation_experiments/repro_et_grows_with_period.py`
- Reused tasksets: `runs/p25periodAB_run_prod_dur300_interval10_seed1000_tasks4x6x8/sim/tasks6_dur300_interval10_seed1000/taskset_0`

---

## 11. `ndiff` ground truth + the float-noise bug (2026-07-04, post-profiling)

**Trigger.** The §10 profiling reported `ndiff≈8` at N=10, but a direct read of
the taskset interval YAMLs showed only 2-3 tasks' ET actually change per
interval. This section reconciles the two, corrects a speculative "Bug 2" I
(initially, wrongly) floated, and pins down the real lever — a tolerance-banded
diff predicate (Fix D).

### 11a. Terminology (precise, for the record)

| term | meaning | code |
|---|---|---|
| `ndiff` | # tasks whose ET dist `!=` baseline `dag_tasks_`'s ET dist, in one `OptimizeIncre` call | `OptimizeSP_Incre.cpp:242` → `FindTaskWithDifferentEt` |
| `nvar` | # priority-assignment variations swept in that call (each = 1 `EvaluateSPWithPriorityVec` = 1 `ObtainSP_DAG`) | `OptimizeSP_Incre.cpp:259` (`dbg_n_variations++`) |
| "2.0/45.1 calls/interval" | REOPT-path avg / INCRE-path avg `ObtainSP_DAG` calls (INCR_P10: 3 of 30 intervals REOPT, 27 INCRE) | `[INCR-ET-DBG]` `path=REOPT\|INCRE` field |
| "12.1/76.2 ms/interval" | same split, wall time | `opt_ms` field |
| "INCR_P1 = 60 calls" | 60 `ObtainSP_DAG` calls total across 30 intervals (period=1 → every interval is REOPT, 2×30) | `g_incr_et_debug_sp_dag_calls` |

### 11b. Ground truth from the taskset YAMLs (N=10, `tasks10_dur600_interval10_seed1000/taskset_0`)

Direct `diff` of consecutive `taskset_characteristics_interval_*.yaml`:

```
per consecutive interval pair: [noise-only] [real-drift] [total flagged by exact !=]
  0->1: noise=1 real=2 total=3
  1->2: noise=0 real=2 total=2
  2->3..7->8: noise=0 real=2 total=2  (every pair)
```

- **3/10 tasks are soft-RT** (have `performance_records_*`, hence TL freedom);
  7 are static hard-RT (`mu`/`sigma=1.0` fixed, TL always `-1`).
- **2 tasks genuinely drift per interval** — the soft-RT ones whose `mu`/`sigma`
  re-estimate from simulated observations. Occasionally +1 from float round-off.
- The diff predicate flags **2-3, never 8.** So the instrumented `ndiff=8`
  (§10b) is **not** "8 tasks' ET really changed"; it's a measurement artifact.

### 11c. Bug 1 — `FiniteDist::operator!=` is near-exact and fires on round-off

`FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:145`) uses `operator!=` on the
`FiniteDist` ET object. The chain:

- `Value_Proba::operator==` (`Probability.cpp:12-16`): `approx_equal_double(value, …, 1e-1)` AND
  `approx_equal_double(probability, …, 1e-1)` — relative tol 0.1 per sample.
- `FiniteDist::operator==` (`Probability.cpp:355-364`): sizes equal, every
  `Value_Proba` equal under the above, AND `min_time`/`max_time` equal under
  `approx_equal_double(…, 1e-1)`.

This looks tolerant, but it fails on **sub-representable drift after
re-discretization**. Concrete case from the YAML (task 6, interval 0→1):

```
execution_time_mu: 9.09076653007259  ->  9.090766530072594   (15th-sig-digit round-off)
```

The `mu` round-off is ~1e-13, but when each interval's `(mu, sigma, min, max)`
is re-discretized into a `FiniteDist` of `granularity=10` samples spanning
`[min, max]`, the sample **values** are `min + k·step` for `step=(max-min)/9`.
A 1e-13 shift in `mu`/`min` shifts a sample's `value` by ~1e-13×step. On a
*small* sample value (e.g. a near-zero `value`), the **relative** tolerance
`|Δ|/|a| < 0.1` can flip — `approx_equal_double` divides by `|a|`, and if the
sample `value` is itself tiny, a 1e-13 absolute delta is a large *relative*
delta. **Result: a task whose true ET is byte-identical gets flagged
"different,"** because its discrete representation shifted one sample past the
relative threshold.

`FiniteDist` already has the right tool unused:
`approx_not_equal(other, tolerance)` (`Probability.h:148-151`). The diff path
just doesn't call it.

### 11d. "Bug 2" — CORRECTED (I was wrong; recording the correction)

I initially speculated a second bug: "the TL coordinate descent calls
`OptimizeIncre` multiple times per interval (once per task), and between those
calls `dag_tasks_` is the advanced one from the prior call, so the diff is
within-interval and `ndiff` is inflated by TL re-evals, not env drift."

**This is false.** Tracing the code:

1. `PerformCoordinateDescentForTaskConfigOpt` (`OptimizeSP_TL_Incre.cpp:179`)
   calls `EvaluateTimeLimitConfig_ScratchOrIncre(...)` once per TL option per
   task — but on the N=10 taskset **every task is `{-1}`-only** (no
   `timePerformancePairs`), so the `{-1}`-only skip (Fix B, line 197) fires for
   every task and the descent body runs **zero** TL evals. Only Fix B's
   zero-work fallback (line 229-231) fires — **one** eval.
2. `EvaluateTimeLimitConfig_ScratchOrIncre` (line 122-160):
   `OptimizePA_Incre optimizer = prev_optimizer_;` — a **copy**, not a mutation
   of `prev_optimizer_`. `OptimizeIncre` runs on the copy.
3. `UpdateRecords` (line 82-119) writes `prev_optimizer_ = optimizer` **only if
   `should_update`** (SP strictly improved, or SP-equal with smaller TL sum).
   On a no-improvement eval — the common case for the fallback —
   `prev_optimizer_` is **not touched**.
4. `UpdateExtDistBasedOnTimeLimit` (`OptimizeSP_TL_BF.cpp:6-16`) only rewrites
   `execution_time_dist` for tasks with `time_limit[i] != -1`. On this all-`{-1}`
   taskset it's a **no-op for all 10 tasks** — `dag_tasks_cur` == `dag_tasks_`
   exactly. **No TL-mutation signal enters the diff at all.**

So on this taskset the diff is **purely env drift** (the 2-3 from §11b), and the
within-interval re-diff structure I posited doesn't exist. **`ndiff=8` is
entirely Bug 1** (float-noise on the comparison), not a descent artifact.

### 11e. The real residual lever: Fix A's advance doesn't propagate on no-improvement

There *is* a real subtlety the correction exposes, just not the one I claimed.
Fix A (`OptimizeSP_Incre.cpp:287`) advances `dag_tasks_ = dag_tasks_update` —
but on the **copy** `optimizer` (line 139). That advance only reaches
`prev_optimizer_` when `UpdateRecords` does `prev_optimizer_ = optimizer`, which
requires `should_update`. On the all-`{-1}` taskset, the only eval per interval
is Fix B's fallback, and whether it improves SP determines whether the baseline
advances. **When the fallback's eval doesn't improve SP, `prev_optimizer_`'s
`dag_tasks_` stays frozen at the last improving interval** — and the *next*
interval diffs against that stale baseline, accumulating real drift across
multiple intervals until an improving eval finally advances it.

This is a **milder** version of the original frozen-baseline pathology Fix A
targeted: Fix A fixed the "never advances at all" case, but the "advances only
on improvement" case can still lag by several intervals on tasksets where SP has
plateaued. It does not cause the 20× call-count gap (that's Bug 1 inflating
`ndiff` per call); it adds at most a few extra diff-tasks when the baseline lags.
Worth a note, not a separate fix — Fix D (below) subsumes the symptom by making
the predicate tolerance-banded regardless of how stale the baseline is.

### 11f. Fix D — tolerance-banded diff predicate (the cheap, high-leverage fix)

**One-line change** in `FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:145`):
replace `execution_time_dist != …execution_time_dist` with
`execution_time_dist.approx_not_equal(other, tol)` (the method already exists at
`Probability.h:149`). With `tol=1e-2` (1% relative):

- Task 6's 1e-13 round-off vanishes (well under 1%).
- Sub-σ Gaussian wobble that doesn't move the discrete distribution by >1% is
  ignored — and a <1% distribution change genuinely doesn't warrant a priority
  re-sweep (the priority decision rests on `GetAvgValue()`-relative comparisons,
  which `AnalyzePriorityChangeStatus` already uses).

Pair with a **banding rule on the priority-relevant quantity**: even if a task
passes the dist-level `approx_not_equal`, only treat it as "diff" if
`|GetAvgValue() - old.GetAvgValue()| / old.GetAvgValue() > 1e-2`. That's the
quantity the downstream heuristic reasons about, so it's the principled
threshold.

**Expected effect:** `ndiff` drops from ~8 (noise-inflated) to the true ~2-3;
`nvar` and `ObtainSP_DAG` calls drop proportionally — **roughly a 3× win on the
INCRE path**, zero algorithmic change, zero correctness risk (only tightening a
"did it change" predicate). This is the single highest bang-for-buck lever in
this file.

### 11g. Fix E — taskset generator (considered, **not** recommended)

The "8/10 tasks depend on env, the 80% ratio is too high — modify the simulation
task-set logic" instinct is understandable but rests on the `ndiff=8`
mismeasurement. Ground truth is 2/10 soft-RT tasks drift (20%), which is
**already sane** for an online re-optimization benchmark (the whole point of
re-estimating `mu`/`sigma` per interval is that observed ET drifts). The 80%
number was Bug 1. **Do not change the taskset generator** — Fix D removes the
noise that made the ratio look pathological.

### 11h. Fallback-to-scratch when `ndiff ≥ N/2` (safety net, not primary)

The "if too many tasks change, don't bother with incremental" instinct is sound
as a **fallback**, not a primary fix. Right form: if `ndiff ≥ N/2` (more than
half genuinely moved), the incumbent is probably stale — fall back to
`OptimizeFromScratch` (1 `ObtainSP_DAG`), skip the per-variation sweep. Once
Fix D lands this rarely fires (true `ndiff` is ~2-3 at N=10); keep it as a
safety net for genuine churn.

### 11i. Recommended order (supersedes §10c's)

1. **Fix D** (tolerance diff) — ~1 line + banding rule, TDD test with two
   near-identical `FiniteDist`s, measure `ndiff` drop on the N=10 repro.
   **Highest bang-for-buck, lowest risk.** Directly addresses "reduce the
   change criteria."
2. **Fix C** (incremental RTA reuse, §10c.1) — the structural win for the
   *remaining* per-variation cost, after Fix D cuts the variation count.
3. **Fallback-to-scratch when `ndiff ≥ N/2`** — cheap safety net.
4. ~~Fix E (taskset generator)~~ — not needed; drift ratio is sane once Bug 1
   is fixed.

### 11j. Artifacts (this pass)

- N=10 taskset interval YAMLs: `simulation_experiments/optimizer_comparison/
  tasks10_dur600_interval10_seed1000/taskset_0/taskset_characteristics_interval_{0..59}.yaml`
- The `diff interval_0.yaml interval_1.yaml` excerpt (task 4 real drift, task 6
  round-off) is reproducible by running §11b's Python over the above dir.
- Comparison semantics: `Probability.cpp:12-16` (`Value_Proba==`),
  `:355-364` (`FiniteDist==`), `:148-151` (unused `approx_not_equal`).
- Tests: 16/16 ctest green in the DEBUG build (`build/`) with the instrumentation
  compiled in; release test-binary failures are pre-existing stale-link cruft
  (`bf_tie_break_type` undefined symbol; release build doesn't build test
  executables — `tests/CMakeLists.txt:1-3` gates `gtsamAddTestsGlob` on
  `CMAKE_BUILD_TYPE STREQUAL DEBUG`) and unrelated to this change.

## 9. Cross-references

- Original record: `agents/finished_tasks/P24_task.md` "KNOWN ISSUE" section
  (updated 2026-07-04: header now reads "FIX APPLIED & TDD-VERIFIED; runtime A/B
  re-run PENDING"; a "PENDING: A/B re-run" subsection with the full procedure +
  pass criterion was appended at the end of that file).
- Memory: `p25-incr-et-grows-with-period.md` (updated 2026-07-04 to reflect the
  root cause + Fix A/B applied & TDD-verified; its "Runtime A/B re-run (STILL
  PENDING)" line remains open until the re-run confirms the ordering flip).
- Related: `runorchestrator-duration-arg-semantics.md` (ruled out — correct arg
  used), `p24-reoptimization-design.md`.

---

## 12. Follow-up investigation (2026-07-06): "N=6 ET ≈0.09s vs N=8 ET ≈0.04s is impossible"

> **User report:** Ran `MODE=test CONFIG_JSON=.../p25_period_ab_config.json
> ./scripts/run_end_to_end.sh` twice, changing only
> `num_tasks_for_cross_task_comparison` (6 vs 8). Read ET from each run's
> `sim/tasks4_sweep_interval10_seed1000/comparison_summary.csv`. N=6 run shows
> `Mean_Scheduler_Execution_Time_s ≈ 0.096`; N=8 run shows `≈ 0.044`. "N=6
> slower than N=8 is theoretically impossible." Asked for explanation/debug,
> NOT a re-run.
>
> **Verdict: NOT a bug. NOT an impossible result. The two ET numbers are not
> comparable — they come from two different-N, different-duration, different-
> taskset-count runs, read out of a directory whose name (`tasks4_sweep_...`)
> is the SAME in both runs by coincidence. Three confounds, all confirmed from
> the per-run `config.json` snapshots and raw `scheduler_execution_time.txt`
> files. No re-run performed (per user request).**

### 10.1 What the user actually compared

The path the user read in BOTH runs is `sim/tasks4_sweep_interval10_seed1000/`.
That directory is produced by the **sweep stage** (stage 2, `interval_sweep.py`),
whose task count comes from `num_tasks_for_single_task_figures` — **NOT** from
`num_tasks_for_cross_task_comparison`. In both runs the snapshot shows
`num_tasks_for_single_task_figures = 4`, so BOTH `tasks4_sweep_...` folders are
**N_TASKS = 4** (confirmed: each `taskset_0/generator_config.json` has
`N_TASKS=4`, `DESC="Paper parameters for 4 tasks"`, `RANDOM_SEED=1000`).

The N=6 / N=8 the user set in `num_tasks_for_cross_task_comparison` only
controls the **simulate stage** (stage 1), whose output goes to a *different*
sibling dir: `tasks6_dur600_interval10_seed1000/` and `tasks8_dur300_interval10_seed1000/`
respectively. The user did not look at those.

So the comparison "N=6 ET vs N=8 ET" is really **"N=4 sweep (dur600, 20 tasksets)
ET vs N=4 sweep (dur300, 10 tasksets) ET"**. The N is identical; duration and
taskset count differ.

### 10.2 The per-run config snapshots (the ground truth)

Each run root saves a `config.json` snapshot. Reading the `test_mode` block of
each (both runs were `MODE=test`):

| key | tasks8/dur300 run | tasks6/dur600 run |
|---|---|---|
| `num_tasks_for_cross_task_comparison` | `[8]` | `[6]` |
| `num_tasks_for_single_task_figures` | `4` | `4` |
| `num_tasksets_to_generate` | `10` | `20` |
| `simulation_duration_seconds` | `300` | `600` |
| `parallel_worker_processes` | `8` | `8` |

So between the two runs the user changed **three** test_mode keys, not one:
task count 8→6, taskset count 10→20, and duration 300→600. (The working-tree
`git diff` of the config vs HEAD confirms the same: test_mode was edited from
`single=6 / 10 tasksets / 300s` to `single=4 / 20 tasksets / 600s` — i.e. the
config was further edited after the dur300 run and before the dur600 run.)

### 10.3 Why ET(diff) is ~2.2× — three compounding confounds

**Confound A — divisor artifact (dominant).** `Mean_Scheduler_Execution_Time_s`
in `comparison_summary.csv` is computed in
`run_sim_experiments.py:analyze_single_instance` (lines 111-141) as
`total_exec_time / num_intervals`, where:
- `total_exec_time` = the single number in `scheduler_execution_time.txt`,
  which C++ writes as the **whole-process wall-clock** of `RunOrchestrator`
  (`RunOrchestrator.cpp:83-125`: `start_time` before orchestrator construction,
  `end_time` after `RunSimulation()` + all file writes — see line 111 comment:
  *"C++ writes total process duration; we convert to per-scheduler-call average
  by dividing by the number of intervals"*).
- `num_intervals` = count of `taskset_characteristics_interval_*.yaml` =
  `n_sec / scheduler_trigger_interval`.

For the sweep folders: dur600 → **60 intervals**, dur300 → **30 intervals**.
The total process time grows with run length (more intervals to schedule +
more file I/O), and it is divided by a **different** number in each run. Raw
totals (INCR_P1, taskset_0): dur600 total = 7.06 s / 60 = 0.118; dur300 total =
1.15 s / 30 = 0.038. The ratio of the reported means (0.096 / 0.044 ≈ 2.2×) is
essentially "longer run, bigger total, same divisor logic" — not "N=6 slower
than N=8".

**Confound B — wall-clock contention (secondary, known issue).** Both runs used
`parallel_worker_processes = 8` with 5 arms, so up to 5 `RunOrchestrator`
processes ran concurrently on the box. Because ET is whole-process wall time
(not CPU time), each arm's number is inflated by contention from the others,
non-uniformly across arms. This is exactly **Hypothesis 2** documented in
`simulation_experiments/repro_et_grows_with_period.py` (header). The contention
window differs between the two runs (20 vs 10 tasksets, 60 vs 30 intervals), so
the contention inflation differs too. The repro script exists specifically to
isolate this by running arms serially.

**Confound C — non-deterministic tasksets.** Despite both sweep folders being
N=4 / seed=1000, the on-disk SP trajectories of the shared prefix **diverge**
(`diff` of `interval_sp_metrics.txt` first 30 lines of taskset_0/INCR_P1 shows
differences from interval 3 onward). So the two runs are NOT over the same
tasksets — taskset generation is not bit-reproducible across runs (likely RNG
sequencing differences from the different `n_tasksets`/`n_sec`). Different
tasksets ⇒ different per-interval optimizer work ⇒ different ET, independent of
A and B.

### 10.4 The arithmetic check (closes the loop)

For INCR_P1 across all tasksets in each sweep folder:

- dur600 run: Σ total = 115.21 s over 20 tasksets × 60 intervals ⇒ mean total
  5.76 s ⇒ per-interval 5.76/60 = **0.0960** ← matches CSV `0.096012` ✓
- dur300 run: Σ total = 13.31 s over 10 tasksets × 30 intervals ⇒ mean total
  1.33 s ⇒ per-interval 1.33/30 = **0.0444** ← matches CSV `0.044373` ✓

The CSV numbers are arithmetically faithful to the raw files. Nothing is
mis-computed; the inputs just aren't comparable.

### 10.5 Is the per-interval cost itself pathological? (partial yes, separate issue)

The *mean* per-interval cost differs 0.096 vs 0.044 even though both are N=4.
If per-interval cost were constant, doubling the intervals (30→60) should ~double
the total (1.33 → ~2.66), but the dur600 mean total is 5.76 — ~4.3× the dur300
mean total, not 2×. Two effects:
- Different tasksets (Confound C) — dur600's 20 tasksets include some heavy ones
  (taskset_15 total 13.26 s, taskset_11 9.73 s) that have no counterpart in
  dur300's 10.
- The known "ET grows over a run" pathology (§1-§8 + §10-§11 of this file, Fix A/B applied)
  — longer runs give the per-variation `ObtainSP_DAG` cost more room to
  accumulate. Fix A+B killed the 3× frozen-baseline growth but the residual
  per-variation asymmetry (Fix C, deferred) remains, so longer horizons still
  cost more per interval on average.

This is a real but *separate* concern from the user's "N=6 vs N=8" framing.

### 10.6 What to do (recommendations, not executed)

1. **Compare like with like.** To compare N=6 vs N=8 ET, read the **simulate**
   stage dirs (`tasks6_dur600_...` vs `tasks8_dur300_...`), not the sweep dir —
   AND hold `simulation_duration_seconds` and `num_tasksets_to_generate` fixed
   across the two runs. As-is, the two runs differ in duration (600 vs 300) and
   taskset count (20 vs 10), so even the simulate-stage dirs aren't directly
   comparable on ET.
2. **Use CPU time, not wall time, for ET.** The C++ ET is whole-process wall
   clock including orchestrator construction, file I/O, and parallel-worker
   contention (Confound B). For a clean per-activation scheduler cost, time
   only `RunSimulation()`'s scheduling hot path in CPU time, or run arms
   serially via `repro_et_grows_with_period.py`.
3. **Don't read ET out of `tasks4_sweep_...` to make N-cross-task claims.**
   That dir is always N = `num_tasks_for_single_task_figures` (=4 here),
   independent of `num_tasks_for_cross_task_comparison`.
4. **Re-enable bit-reproducible tasksets** if cross-run ET comparison is needed
   (Confound C) — or only compare runs that share `n_sec` + `n_tasksets` + seed.

### 10.7 Evidence files (for the record)

- Run snapshots: `runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks6/config.json`
  and `runs/p25periodAB_run_test_dur300_interval10_seed1000_tasks8/config.json`.
- Raw ET: `<run>/sim/tasks4_sweep_interval10_seed1000/taskset_<i>/<ARM>/<ARM>/scheduler_execution_time.txt`.
- ET→CSV path: `simulation_experiments/run_sim_experiments.py:111-141`
  (`analyze_single_instance`), `simulation_experiments/utils.py:201-208`.
- C++ ET writer: `tests/RunOrchestrator.cpp:83-133` (wall clock around
  construction + `RunSimulation()`).
- Sweep dir naming: `simulation_experiments/interval_sweep.py:174,404`
  (`tasks{num_tasks_for_single_task_figures}_sweep_...`).
- Contention-isolation harness: `simulation_experiments/repro_et_grows_with_period.py`.
