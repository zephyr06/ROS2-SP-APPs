# P0.6 — Tasks (working checklist)

> See `goal.md` for the revised algorithm (DM-grouped seed + min-TL → WCET-mode TL walk →
> **skip-and-continue** important-task-schedulability filter → best-SP-safe incumbent) +
> the "important tasks" design decision (D1). Depends on P0.8 (schedulability guarantee at
> the seed point). Blocks P0.7 (fall-back invocation).
> **All design decisions D1–D7 RESOLVED 2026-07-27** (see `goal.md` "Open decisions").

## 0. Design decisions (RESOLVED)
- [x] **D1 RESOLVED (2026-07-27):** top-50% by `sp_weight`, persisted as
      `bool Task::is_important` (generator-set, YAML-emitted, read by all consumers).
      See `goal.md` "Designing important tasks" + memory `important-tasks-design-decision`.
- [x] **D2 RESOLVED (2026-07-27):** WCET per task = max ET across all generated interval
      tasksets (global max, computable offline). Extends `ApplyWCETAblationIfRequired`
      (per-task `execution_time_dist.max_time`) with a cross-interval max precompute.
- [x] **D3 RESOLVED (2026-07-27):** per-taskset (== once per worker invocation).
- [x] **D4 RESOLVED (2026-07-27):** in-memory `static_solution_` member.
- [x] **D5 RESOLVED (2026-07-27):** filter via virtual hook `ShouldAdoptCandidate(...)`.
- [x] **D6 RESOLVED (2026-07-27):** RTA form (R_i ≤ deadline_i) offline (≡ DDL-miss-chance
      under WCET point-mass).
- [x] **D7 RESOLVED (2026-07-27):** **skip-and-continue** offline (ample budget → explore
      for best-SP-safe). The HALT semantics are P0.7's online trigger (b).
- [ ] Record decisions in `dev_log.md` + memory; sync with P0.7/P0.8 owners. **Coordinate
      with P0.8: its RTA must use the SAME seed point (DM-grouped + min-TL + WCET) this
      task seeds from** (see `goal.md` "Relationship to P0.8").

## 0.5. Add `bool is_important` to `Task` + generator labeling (SHARED enabling change)
> D1 landed here. Needed by P0.6/P0.7/P0.8 + the analysis `compute_important_task_miss_rate`.
> Land BEFORE the P0.6 walk. Generator-side work pairs with P0.8 (same Python pass).
- [ ] C++: add `bool is_important` member to `Task` (`RegularTasks.h:35-109`); default
      `false`; expose in the `ReadTaskSet` YAML parse (`RegularTasks.h:129`,
      `ParametersSP.cpp:25-29` shows the `sp_weight` parse to mirror — `important:` key).
- [ ] Generator: after `sp_weight` assignment (`taskset_generator.py:551`), sort the
      taskset by `sp_weight` desc, mark the top-`ceil(N*0.5)` = `(N+1)//2` tasks
      `is_important = true`. Add `IMPORTANT_TASK_RATIO: 0.5` to CONFIG_SPECS
      (`taskset_generator.py:36` area). `yaml_exporter.py:68` emits `important` alongside
      `sp_weight`. Ties (measure-zero under continuous-uniform weight, but for safety):
      break by task id (deterministic).
- [ ] TDD red→green: a generated taskset of N tasks has exactly `ceil(N/2)` important;
      the important set = the top-weight half; ties broken by id (construct a tie case).
- [ ] Analysis migration: `compute_important_task_miss_rate` (`utils.py:182-220`) reads
      `task.is_important` instead of recomputing top-`pct`; the `important_task_top_percentage`
      / `minimum_important_tasks_count` config knobs (`paper_simulation_config.json:66-67`)
      become dead — ruthless-prune (remove) or document as analysis-only fallback. Decide
      in this step.
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (no SP behavior change —
      the bool is additive, no consumer reads it yet).

## 1. Extract the DM deadline-sort helper (behavior-identical refactor)
- [ ] TDD red: test `AssignDMRespectingGroupOrder` on a fixed taskset — (a) empty
      top-group reproduces the current `DM` branch order exactly (period-ascending, ties by
      avg ET); (b) a non-empty top-group puts those ids first (DM within group), rest after
      (DM within group), every top-group id before every rest id.
- [ ] Extract `AssignDMRespectingGroupOrder(top_group_ids)` from the `DM` branch
      (`SimulationOrchestrator.cpp:356-367`) — mirrors `DeadlineMonotonicPriorityVec`
      (`OptimizeSP_TL_Incre.cpp:695-707`) with the group-lock. Returns `priority_vec`.
- [ ] Rewire `DM`, `DM_FAST`, `DM_SLOW` branches (356/368/388) to call it with empty
      top-group (behavior unchanged — no priority lock).
- [ ] TDD green: `cmake --build build_test --target check.SP_OPT -j5` — bit-identical SP
      for DM/DM_FAST/DM_SLOW vs. baseline.

## 2. Global-max-across-intervals WCET precompute (D2)
> The offline walk collapses each task's ET to a point mass at its global-max ET. The
> existing `ApplyWCETAblationIfRequired` uses per-task `execution_time_dist.max_time`;
> P0.6 needs the max across ALL interval tasksets in `dag_tasks_vecs_`.
- [ ] Code-verify (task #7) the acquisition mechanism: is `dag_tasks_vecs_` populated
      before the interval loop? Does each per-interval taskset carry its own ET per task?
      Confirm the global-max precompute is feasible at the offline compute site.
- [ ] TDD red: construct 2 interval tasksets where task X has ET 5 in interval 0 and ET 8
      in interval 1; assert the precomputed global-max WCET for X = 8 (not 5, not the
      task's own `max_time` if that differs).
- [ ] Implement `ComputeGlobalMaxWCETPerTask(dag_tasks_vecs_)` → `vector<double>` per task
      id. The offline walk collapses to these point masses (extend or wrap
      `ApplyWCETAblationIfRequired`).
- [ ] TDD green.

## 3. Static-solution PA + min-TL seed
- [ ] TDD red: test the seed — important tasks occupy the top `n_important` slots in DM
      order; rest fill lower slots in DM order; every important task's priority < every
      rest task's priority; perf tasks seeded at `SmallestTimeLimitVec`, non-perf at -1.
- [ ] Implement the seed step: select important set (per D1), PA via
      `AssignDMRespectingGroupOrder(important_ids)`, TL via `SmallestTimeLimitVec`.
      Score + commit as the incumbent baseline (mirror `SeedIncumbentFromDMFast`,
      `OptimizeSP_TL_Incre.cpp:758-766`, but grouped PA + global-max WCET).
- [ ] TDD green: seed-ordering unit test passes.

## 4. Skip-and-continue schedulability filter (P0.6's novel piece)
> D7 = skip-and-continue offline (NOT halt). The HALT semantics are P0.7's online trigger.
- [ ] TDD red: construct a taskset + two TL candidates for one task — candidate A is
      SP-better but makes an important task's R_i > deadline_i (unsafe); candidate B is
      SP-better-than-current AND safe. Assert the filter (i) skips A (does not adopt),
      (ii) continues the walk, (iii) adopts B, (iv) returns B's TL as the incumbent.
- [ ] Implement the filter (per D5 — virtual hook
      `ShouldAdoptCandidate(candidate_sp, candidate_schedulable)`). Inline important-task
      RTA: for each important τ_i, `R_i = WCET_i + Σ_{j important, prio(j)<prio(i)}
      ceil(R_i/period_j)·WCET_j`; schedulable ⇔ R_i ≤ deadline_i (per D6 — RTA form).
- [ ] Wire the filter into the TL walk's per-candidate adoption decision (before
      `IsBetterTimeLimitOption` adopts a SP-better TL, run the filter; if it returns
      "unsafe," skip the candidate and continue — do NOT halt the walk).
- [ ] TDD green: filter-skips-unsafe-adopts-safe test passes.

## 5. Wire offline + WCET-mode + ET-excluded
- [ ] Implement `ComputeStaticSolution()`: set `use_wcet_execution_time = true` (mirror
      `INCR_WCET`, `SimulationOrchestrator.cpp:349-350`), apply the global-max WCET
      precompute (step 2), run seed (step 3) + filtered walk (step 4), restore the flag,
      return the committed `ResourceOptResult`.
- [ ] Call `ComputeStaticSolution()` after `incr_optimizer_` construction
      (`SimulationOrchestrator.cpp:300-302`), before the interval loop (304-308).
- [ ] Store result in an in-memory `static_solution_` member (per D4).
- [ ] Verify the compute is OUTSIDE `DeterminePrioritiesAndBudgets`'s ET bracket
      (316-322): reported `scheduler_execution_time.txt` unchanged vs. baseline at the same
      N (this is the "doesn't count into scheduler ET" guarantee).
- [ ] Emit a separate `static_solution_compute_time` profile (does NOT enter the online ET
      metric).

## 6. Verification + records
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] Spot-check: at a smoke-test N, `static_solution_` is populated; its PA is
      DM-grouped; its TL vector is the walk's best-SP-safe result (perf tasks ≥ min-TL,
      non-perf = -1); important tasks are schedulable under it (filter invariant holds on
      the returned solution).
- [ ] Confirm the seed point matches P0.8's RTA seed point (DM-grouped + min-TL + WCET).
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).
