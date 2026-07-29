# P0.6 — Offline Static Solution (Fall-Back Seed)

**Priority:** P0 (blocks the fall-back mechanism, P0.7)
**Status:** design / not started
**Depends on:** P0.8 (task-set schedulability for important tasks under DM — certifies the
SEED point the static solution starts from). P0.7 (fall-back) depends on THIS task.
**Blocks:** P0.7 (the fall-back swaps the static solution in when an online trigger fires).

## Goal (revised algorithm, 2026-07-27)

Produce a **deterministic, offline static scheduling solution** computed ONCE before the
interval-optimization loop begins, stashed on the orchestrator as `static_solution_`, so
the fall-back (P0.7) can swap it in when an online trigger fires. "Static" = a fixed
priority assignment + a fixed time-limit vector, found by an **offline WCET-mode TL walk**
with a **skip-and-continue important-task-schedulability filter**. This is the safe floor:
a conservative WCET-based plan that preserves important-task schedulability (certified at
the seed by P0.8, preserved during the walk by the filter) while reaching the **best-SP
safe** TL configuration the walk can find.

### Budget asymmetry drives the mechanism (user's 2026-07-27 direction)

> "overall, we have high budget for offline analysis, so we can try more 'walk' during
> offline analysis, just find a safe solution with good performance. during online,
> budget is very tight, use this to adjust your design."

So the offline walk (THIS task) **explores fully** — ample budget → **skip-and-continue**
on unsafe candidates, keep walking to find the best-SP safe point. The online guard
(P0.7) is the **tight-budget** counterpart — it **halts** on the first unsafe candidate
and compares against this static solution. The HALT semantics live in P0.7, NOT here.
P0.6's filter is skip-and-continue.

### The algorithm

1. **Seed.**
   - Select the important tasks. **D1 RESOLVED (2026-07-27):** top-50% by `sp_weight`,
     persisted as `bool Task::is_important` (set at generation, read here — see
     "Designing important tasks"). So "select the important tasks" = read
     `task.is_important`; count = `ceil(N/2)` = `(N+1)//2`.
   - Assign priorities **separately** to important vs non-important by **Deadline Monotonic**:
     important tasks occupy the top priority slots (DM-ordered within the group); the rest
     fill the lower slots (DM-ordered within the group); every rest-task below every
     important task. This is `AssignDMRespectingGroupOrder(important_ids)` — the helper
     extracted from the triplicated DM sort (see step 1 of `tasks.md`).
   - Seed every TL-optimizable (perf) task's time limit to its **minimum** option
     (`SmallestTimeLimitVec`, the smallest grid point → smallest ET → least interference →
     most schedulable). Non-perf tasks have no TL freedom (TL = -1).

2. **Walk under WCET mode.**
   - Flip `GlobalVariables::use_wcet_execution_time = true` for the static solution's SP
     evaluation (mirror `INCR_WCET`, `SimulationOrchestrator.cpp:348-355`). Under WCET
     mode, non-perf tasks collapse to a point mass at WCET (no TL freedom); only perf
     tasks retain TL freedom, so the walk adjusts **perf-task TLs only**.
   - **WCET value (D2 RESOLVED 2026-07-27):** WCET per task = the **maximum ET that task
     exhibits across all generated interval tasksets** (the global max, NOT just the
     task's own distribution `max_time`). The user confirmed interval ETs are
     pre-generated, so this global max is computable offline before the interval loop
     (Q2 — code-verification in flight, task #7; the existing `ApplyWCETAblationIfRequired`
     uses `execution_time_dist.max_time` per-task, so P0.6 likely adds a precompute that
     takes the max across `dag_tasks_vecs_` per task id, then collapses to that point
     mass). This global-max WCET is what makes P0.7's ET-jump trigger (a) safe by
     construction: any interval's jumped ET ≤ the global max the static solution was
     computed at.
   - Run the incremental TL walk (`OptimizeIncre_w_TL`-style serialized queue) starting
     from the DM-grouped + min-TL seed. **The existing TL walk is TL-only — it does NOT
     mutate the seeded PA** (`WalkOneTaskWithTimeLimitOptions` scores each candidate TL
     under the current `opt_pa_`; the PA is inherited from the seed and never changed). So
     "fix priority, walk TL" is the walk's natural behavior — no new fix-priority flag is
     needed (the existing `disable_time_limit_opt` is the *inverse*: fix TL, walk PA).

3. **Skip-and-continue schedulability filter (P0.6's novel piece).**
   - During the TL walk, before adopting any candidate TL that improves global SP, check
     important-task schedulability (inline RTA: for each important task τ_i,
     `R_i = WCET_i + Σ_{j important, prio(j)<prio(i)} ceil(R_i/period_j)·WCET_j` over
     higher-priority important-task interference; schedulable ⇔ R_i ≤ deadline_i — the
     same recurrence P0.8 uses).
   - **If a candidate would improve global SP but breaks important-task schedulability →
     SKIP that candidate and CONTINUE the walk** (try other TL options for this task,
     then other tasks). Do NOT halt. The walk keeps exploring the perf/RTA tradeoff from
     the safest point (min-TL) upward, committing every SP-better-and-safe candidate as
     the new incumbent.
   - **D6 (guard quantity, RESOLVED):** under WCET mode the RTA distribution is a point
     mass, so DDL-miss is 0 or 1 (R_i ≤ vs > deadline_i) — the RTA form and the
     DDL-miss-chance form coincide offline. Use the RTA form (R_i ≤ deadline_i) for
     consistency with P0.8. (The online guard in P0.7, which runs OUTSIDE WCET mode, uses
     the real DDL-miss-chance from the SP metric — see P0.7.)
   - The returned static solution = the **best-SP TL configuration that maintained
     important-task schedulability** — the highest-SP safe point the walk reached.

4. **Return.** The committed incumbent `{priority_vec, id2time_limit}` — the SAME shape
   `DeterminePrioritiesAndBudgets` returns (`ResourceOptResult`,
   `SimulationOrchestrator.cpp:313`). The fall-back (P0.7) swaps that pair in place of the
   optimizer's `res_opt_` when a trigger fires. The PA is the DM-grouped seed (unchanged
   by the walk); the TL vector is the walk's best-SP-safe result.

### What changed from the prior (2026-07-27 morning) design

The prior design had the **early-stop HALT guard inside P0.6's offline walk**. The user's
budget-asymmetry direction splits that: the HALT moves ONLINE (P0.7 trigger (b), tight
budget — take the first safe incumbent and compare vs static); the OFFLINE walk (ample
budget) instead **skip-and-continues** to find the best-SP safe point. P0.6's filter is
skip-continue, not halt. Net: P0.6 returns a stronger safe floor (best-SP-safe, not just
the first safe point); P0.7's online guard stays cheap (halt-on-first-unsafe + one SP
compare).

## Why offline + why it doesn't count toward scheduler ET

- **Offline:** computed once, before `for (i = 0; i < dag_tasks_vecs_.size(); i++)
  SimulateInterval(...)` (`SimulationOrchestrator.cpp:304-308`). It depends ONLY on the
  taskset (periods, deadlines, sp_weights, WCETs, TL grids) — NOT on runtime traces — so it
  can be precomputed. It is a constant the fall-back reads, not a per-interval solve.
- **ET accounting:** `DeterminePrioritiesAndBudgets` (`SimulationOrchestrator.cpp:313-322`)
  brackets the scheduler decision and accumulates into `scheduler_exec_time_s_`, which
  `RunOrchestrator` writes to `scheduler_execution_time.txt` — the online-performance
  monitor. The static-solution computation must run OUTSIDE that bracket (before the loop,
  not inside `DeterminePrioritiesAndBudgets`) so it does NOT inflate the reported
  per-interval scheduler ET. You MAY profile its own running time (log it separately as
  `static_solution_compute_time.txt`) for reporting, but it is explicitly excluded from the
  online ET metric. The user stated this explicitly: "the static solution is generated
  offline before running the whole interval optimization loops, you may profile its running
  time but that doesn't count into scheduler's average ET." The revised algorithm runs a
  full TL walk offline, so its compute cost is O(walk), not O(sort) — still offline, still
  ET-excluded, but heavier than a flat-WCET design.

## Where it hooks in (code grounding)

- **Compute site:** after `incr_optimizer_` construction
  (`SimulationOrchestrator.cpp:300-302`) and before the interval loop
  (`SimulationOrchestrator.cpp:304`). Add a `ComputeStaticSolution()` call here for the
  INCR-family modes (the modes that have an online optimizer to fall back FROM). Store the
  result on the orchestrator (a `ResourceOptResult static_solution_` member).
- **DM-grouped PA seed:** `DeadlineMonotonicPriorityVec()` (`OptimizeSP_TL_Incre.cpp:695-707`)
  is plain DM (period asc, ties by avg ET). The static solution needs a grouped variant —
  `AssignDMRespectingGroupOrder(top_group_ids)` — important ids first (DM within group),
  rest after (DM within group). Same helper the DM/DM_FAST/DM_SLOW orchestrator branches
  (`SimulationOrchestrator.cpp:356/368/388`) triplicate and can call with empty top-group
  (behavior-identical).
- **Min-TL seed:** `SmallestTimeLimitVec()` (`OptimizeSP_TL_Incre.h:161`) + the
  `SeedIncumbentFromDMFast` pattern (`OptimizeSP_TL_Incre.cpp:758-766`) — DM PA + min-TL,
  scored and committed as the incumbent baseline. The static solution seeds identically
  except PA = DM-grouped (not plain DM).
- **TL walk (TL-only, PA fixed):** `OptimizeIncre_w_TL` → `PerformSerializedTaskQueueOptimization`
  → `RunIntervalDescent(Incremental)` → `WalkSerializedTaskQueue` →
  `WalkOneTaskWithTimeLimitOptions` (`OptimizeSP_TL_Incre.cpp:500+`). The walk mutates only
  `time_limits[task_idx]`; `opt_pa_` is inherited from the seed and never changed. ✓ matches
  "fix priority, walk TL."
- **WCET semantics:** flip `GlobalVariables::use_wcet_execution_time = true` for the whole
  static-solution compute (seed + walk + filter), exactly as `INCR_WCET` does
  (`SimulationOrchestrator.cpp:349-350`). Restore after. **Plus the global-max-across-intervals
  precompute** (D2) — see "WCET value" above; the existing `ApplyWCETAblationIfRequired`
  (`OptimizeSP_TL_Incre.cpp:830-840`) collapses to `execution_time_dist.max_time` per-task,
  which P0.6 extends to the cross-interval max (mechanism being code-verified, task #7).
- **Skip-and-continue filter (NEW):** there is no existing schedulability filter in the
  walk — `IsBetterTimeLimitOption` adopts any SP-better TL. The filter hooks the
  per-candidate adoption decision: before adopting a SP-better candidate TL, run the inline
  important-task RTA; if any important task's R_i > deadline_i, **skip** the candidate (do
  not adopt) and **continue** the walk. Implementation shape (D5 RESOLVED): a virtual hook
  on the walk (`ShouldAdoptCandidate(candidate_sp, candidate_schedulable)`) — the walk's
  existing virtuals (`CallOptimizerGivenTimeLimits`, `OptimizeIncreSingleTask` are virtual
  for exactly this kind of test-seam) suggest the pattern. Reuses the RTA recurrence
  inline (NOT a C++ round-trip per candidate — cheap).
- **DDL-miss probability (alternative filter quantity):** `GetDDL_MissProbability`
  (`RTA.cpp:154-169`) sums the tail of the RTA distribution beyond the deadline — the
  quantity the `sp_threshold` is defined against (`SP_Func`, `SP_Metric.h:31-41`). Under
  WCET mode the RTA dist is a point mass, so DDL-miss is 0 or 1 (R_i ≤ vs > deadline_i) —
  equivalent to the RTA check offline (D6).

## Designing important tasks (D1 RESOLVED 2026-07-27)

**RESOLVED:** 50% of the tasks in a taskset are important, indicated by SP weights;
the rest are non-important. The label is **persisted** as `bool is_important` on the
C++ `Task` class — set at generation (the generator assigns `sp_weight` at
`taskset_generator.py:551`, then sorts by weight desc and marks the top-50%
`is_important = true`, emitting `important: true/false` to YAML alongside `sp_weight`),
read here by the static solution. This is candidate (a) — top-X% by `sp_weight` — with
**X = 50** and the label persisted rather than recomputed.

**Why persisted (not recomputed per consumer):** promotes "important" from a post-hoc
*analysis* label (the old `compute_important_task_miss_rate`, `utils.py:182-220`, sort
by weight + `ceil(N * 0.10)`) to a **generation-time property**. Unifies the definition
across P0.6 (priority lock + skip-continue filter), P0.7 (two online triggers), P0.8
(generation-time RTA), AND the analysis path — all read `task.is_important` → no drift
between "which tasks the static solution locks" and "which the analysis reports." Also
gives P0.8 (Python) the important set directly from the generated taskset — no duplicate
top-X% logic in Python.

**Grounding:**
- `sp_weight` is NOT on `Task` today — it lives in `ParametersSP::weights_node`
  (`ParametersSP.h:51`), loaded from YAML (`ParametersSP.cpp:25-29`). The new bool goes
  ON `Task` (`RegularTasks.h:35-109`), read at `ReadTaskSet` construction
  (`RegularTasks.h:129`).
- Generator assigns `sp_weight`: `taskset_generator.py:551` (`random.uniform`,
  P2.15). `yaml_exporter.py:68` emits `sp_weight` → add `important` emission here too.
- Count = `ceil(N * 0.5)` = `(N + 1) // 2` (mirrors old `int(N * pct + 0.9999)` ceil).
  `sp_weight` is continuous uniform [0.1, 1.0] → ties measure-zero → clean boundary;
  if a tie lands exactly on the boundary, break by task id (deterministic).
- **Config knob migration:** the old `important_task_top_percentage = 0.10`
  (`paper_simulation_config.json:66`) + `minimum_important_tasks_count = 1` (line 67)
  are ANALYSIS-config knobs for `compute_important_task_miss_rate`. Under the new rule
  the fraction is a GENERATION parameter — default name `IMPORTANT_TASK_RATIO: 0.5` in
  the generator config (`taskset_generator.py` CONFIG_SPECS); the analysis path reads the
  bool and drops its own pct. Exact knob placement is an implementation detail (NOT a
  blocker — the rule is fixed).

**Scope note:** adding `is_important` to `Task` + the generator labeling + the
`ReadTaskSet` parse is a *shared enabling change* that P0.6/P0.7/P0.8 + the analysis path
all need. Land it as the first step (TDD: a taskset of N tasks has exactly `ceil(N/2)`
important; the top-weight half is the important set; ties broken by id), BEFORE the
P0.6 walk. Track this as tasks.md step 0.5 (new).

## Scope (what this task IS / IS NOT)

**IS:**
- A new offline `ComputeStaticSolution()` that emits a `ResourceOptResult`
  (DM-grouped PA + WCET-mode-walked TL vector with skip-and-continue filter), stored on
  the orchestrator.
- A refactor of the triplicated DM deadline-sort (DM/DM_FAST/DM_SLOW branches) into a shared
  `AssignDMRespectingGroupOrder` helper (the static solution's PA seed uses the same
  helper). Small, behavior-identical, TDD-covered.
- The **skip-and-continue schedulability filter**: an inline important-task RTA check
  hooked into the TL walk's per-candidate adoption decision; skips SP-better-but-unsafe
  candidates and continues the walk. This is the novel, non-trivial piece.
- The **global-max-across-intervals WCET precompute** (D2) — the WCET value the offline
  walk collapses each task to.
- A separate profile log for the static-solution compute time (NOT in
  `scheduler_execution_time.txt`).
- TDD: unit-test (1) the grouped PA ordering, (2) the min-TL seed, (3) the filter skips
  an SP-better-but-unsafe candidate and continues to a different safe candidate; (4) the
  returned solution is the best-SP-safe point.

**IS NOT:**
- The fall-back INVOCATION (when to swap the static solution in) — that's P0.7. The HALT
  early-stop guard is P0.7's online trigger (b); P0.6's filter is skip-and-continue, not
  halt.
- The generation-time schedulability GUARANTEE — that's P0.8. P0.8 certifies the taskset is
  schedulable for important tasks under DM at the SEED (DM-grouped + min-TL + WCET); P0.6's
  filter preserves schedulability DURING the walk. They compose (see "Relationship to P0.8"
  below) but are distinct: P0.8 = precondition (re-generate if even the seed is unsafe);
  P0.6 = in-walk preservation.
- A change to the SP metric, the optimizer's online path, or `SP_Func`.
- A new scheduler arm in the A/B list. The static solution is an internal fall-back state,
  not a comparable scheduler. (If the user later wants a `STATIC_ONLY` arm for ablation,
  file separately.)

## Relationship to P0.8 (refined)

P0.8 guarantees, at generation time, that the taskset is schedulable for the important
tasks under DM. With the revised algorithm, "under DM" must mean **at the static solution's
seed point**: DM-grouped PA + min-TL + WCET. The min-TL seed is the MOST schedulable point
(smallest perf-task ET = least interference); if even the seed is unschedulable for
important tasks, the walk has no safe starting point → P0.8's re-generate handles that. So:

- **P0.8's RTA must use the same conservative seed point** (DM-grouped + min-TL + WCET) that
  P0.6 seeds from — consistency requirement. If P0.8 certifies a different point, the seed
  P0.6 starts from might already be unsafe.
- **P0.6's skip-and-continue filter** then keeps the walk schedulable as TLs grow from min
  upward — it never adopts an unsafe candidate, so the returned best-SP-safe solution
  inherits the seed's guarantee. The filter's RTA is the same recurrence as P0.8's, applied
  per candidate.

## Done when

- [x] "Important tasks" selection rule settled (2026-07-27): top-50% by `sp_weight`,
      persisted as `bool Task::is_important` (generator-set, YAML-emitted). See
      "Designing important tasks" + memory `important-tasks-design-decision`.
- [x] D2 settled (2026-07-27): WCET per task = max ET across all generated interval
      tasksets (global max, computable offline — Q2 user-confirmed; code-verification
      task #7 in flight for the exact acquisition mechanism).
- [x] D3 settled (2026-07-27): per-taskset (the orchestrator runs one taskset per
      worker invocation → per-taskset == once per run; no cross-taskset reuse).
- [x] D4 settled (2026-07-27): in-memory `static_solution_` member (the fall-back reads
      it inside `SimulateInterval`); a file dump is inspectability-only, default off.
- [x] D5 settled (2026-07-27): skip-and-continue filter via a virtual hook on the walk
      (`ShouldAdoptCandidate`).
- [x] D6 settled (2026-07-27): RTA form (R_i ≤ deadline_i) offline — equivalent to
      DDL-miss-chance under WCET (point mass → 0/1).
- [x] D7 settled (2026-07-27): **skip-and-continue** offline (ample budget → explore for
      best-SP-safe). The HALT semantics are P0.7's online trigger (b).
- [ ] Add `bool is_important` to `Task` + generator labeling + `ReadTaskSet` parse
      (shared enabling change; TDD: exactly `ceil(N/2)` important, top-weight half,
      ties by id). Land BEFORE the P0.6 walk.
- [ ] `AssignDMRespectingGroupOrder(top_group_ids)` helper extracted; DM/DM_FAST/DM_SLOW +
      the static solution's PA seed all call it. Bit-identical SP for the three existing
      arms (TDD red→green).
- [ ] Global-max-across-intervals WCET precompute (D2) — per-task WCET = max ET across
      `dag_tasks_vecs_`; the offline walk collapses to this point mass.
- [ ] `ComputeStaticSolution()` implemented: seeds DM-grouped PA + min-TL under WCET mode,
      runs the TL-only walk with the skip-and-continue filter, returns the best-SP-safe
      incumbent `ResourceOptResult`.
- [ ] Skip-and-continue filter: inline important-task RTA; skips SP-better-but-unsafe
      candidates, continues the walk; TDD test constructs an unsafe candidate and a
      different safe SP-better candidate, asserts the walk skips the unsafe and adopts
      the safe.
- [ ] Static solution computed before the interval loop, stored on the orchestrator,
      excluded from `scheduler_exec_time_s_` (verified: reported scheduler ET unchanged vs.
      baseline at the same N).
- [ ] Separate `static_solution_compute_time` profile emitted (does NOT enter the online ET
      metric).
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] `git add` staged; user reviews (no commit). `dev_log.md` + memory updated.

## Open decisions (settled 2026-07-27 — recorded for reference)

- **D1 — Important-task selection rule.** ✅ RESOLVED: top-50% by `sp_weight`, persisted as
  `bool Task::is_important`. See "Designing important tasks."
- **D2 — WCET value.** ✅ RESOLVED: max ET across all generated interval tasksets (global
  max per task), computable offline. The existing `ApplyWCETAblationIfRequired`
  (per-task `execution_time_dist.max_time`) is extended by a cross-interval max precompute.
- **D3 — Per-taskset vs per-run.** ✅ RESOLVED: per-taskset (== once per worker invocation).
- **D4 — In-memory vs file.** ✅ RESOLVED: in-memory `static_solution_` member.
- **D5 — Filter implementation shape.** ✅ RESOLVED: virtual hook on the walk
  (`ShouldAdoptCandidate(candidate_sp, candidate_schedulable)`).
- **D6 — Filter quantity.** ✅ RESOLVED: RTA form (R_i ≤ deadline_i) offline; ≡
  DDL-miss-chance under WCET.
- **D7 — HALT vs SKIP-AND-CONTINUE.** ✅ RESOLVED: **skip-and-continue** offline (ample
  budget). HALT is P0.7's online trigger (b).

## Reference docs

- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:297-311` — optimizer
  construction + interval loop (compute site is between these).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:313-367` —
  `DeterminePrioritiesAndBudgets` (the ET-bracketed scheduler decision) + the `DM` branch.
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:348-355` — `INCR_WCET` arm
  (`use_wcet_execution_time = true` WCET-semantics path to reuse).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:695-707` — `DeadlineMonotonicPriorityVec` (plain
  DM; the grouped variant `AssignDMRespectingGroupOrder` wraps this).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:758-766` — `SeedIncumbentFromDMFast` (DM PA +
  min-TL seed pattern; the static solution seeds identically with grouped PA).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:500+` — `WalkOneTaskWithTimeLimitOptions`
  (the 1D TL walk primitive; TL-only, PA fixed — confirms "fix priority, walk TL").
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:659-678` — `OptimizeIncre_w_TL` (the
  interval TL-walk entry; the static solution runs its body once, offline, with the filter).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:830-840` — `ApplyWCETAblationIfRequired`
  (existing per-task WCET collapse; P0.6 extends to cross-interval max per D2).
- `sources/Optimization/OptimizeSP_TL_Incre.h:113` — `BootstrapIncumbentFromDMFast` (the
  existing DM-fast seed; the static solution is its offline, grouped-PA, WCET-mode,
  filtered analog).
- `sources/Safety_Performance_Metric/RTA.cpp:154-169` — `GetDDL_MissProbability` (the
  DDL-miss tail; under WCET ≡ R_i ≤ deadline_i — the filter quantity).
- `sources/Safety_Performance_Metric/SP_Metric.h:31-41` — `SP_Func` (threshold ≥
  violate_probability ⇒ reward; the safety semantics).
- `sources/TaskModel/RegularTasks.h:39-107` — `Task` struct (`period`, `deadline`,
  `priority`, `execution_time_dist`, `timePerformancePairs`, `utilization()`).
- `sources/Utils/Parameters.h:19-20` — `disable_time_limit_opt` (the inverse: fix TL, walk
  PA — NOT what P0.6 wants) / `use_wcet_execution_time`.
- `simulation_experiments/utils.py:182-220` — `compute_important_task_miss_rate` (existing
  top-X%-by-weight important-task selection — the convention to match or revise per D1).
- `simulation_experiments/configs/paper_simulation_config.json:66-67` —
  `important_task_top_percentage` / `minimum_important_tasks_count`.
- `Gen_Taskset/lib/feasibility_clamp.py` — P1.8 clamp (skips perf tasks; the gap P0.8 must
  close for important-task schedulability).
- Memory [`p213-important-task-ddl-vs-sp-metric`](../../../) — SP threshold = DDL-miss
  threshold; the safety semantics the fall-back (P0.7) and the filter (D6) check against.
- Memory [`p18-incr-wcet-outperforms-incr`](../../../) — generator ET-feasibility defect +
  clamp (perf-task gap, the live generator question P0.8 owns).
