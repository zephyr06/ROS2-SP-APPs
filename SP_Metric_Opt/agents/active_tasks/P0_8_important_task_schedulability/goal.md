# P0.8 — Taskset Schedulability for Important Tasks under DM (Seed Certification)

**Priority:** P0 (the guarantee the fall-back chain rests on)
**Status:** design / not started
**Depends on:** ~~"Important tasks" selection rule (D1, shared with P0.6/P0.7)~~ —
**D1 RESOLVED 2026-07-27** (see below).
**Blocks:** P0.6 (the static solution is only a *meaningful* safe floor if the taskset
is DM-schedulable for the important tasks at the seed point) and P0.7 (D4 — the fall-back
trusts the static solution without re-checking, because this guarantee is exact).

## Goal

Ensure every generated simulation taskset is **schedulable for the important tasks under
the static solution's DM-with-top-priority-lock at the SEED point** (P0.6). The static
solution seeds from DM-grouped PA + min-TL + WCET; for it to be a true safe floor, the
important tasks must meet their deadlines at that seed — otherwise the fall-back swaps to
a plan that *also* misses, and the safety guarantee is hollow.

The user's verbatim direction:
> "during random task set generation stage, we'll need to ensure the task set is
> schedulable for important tasks under DM. Otherwise we'll re-generate, or adjust our
> task set generation logic."

So this is a **generation-time guarantee**, enforced in `Gen_Taskset/lib/` (the
taskset generator), NOT a runtime check. Two levers, per the user:

1. **Re-generate** — if a generated taskset fails the important-task DM-schedulability
   test, reject it and regenerate (the generator already has a retry pattern:
   `uunifast_distribution` retries, `taskset_generator.py:151-170`).
2. **Adjust the generation logic** — if the rejection rate is too high, fix the
   generator so tasksets are schedulable by construction (e.g. tighten the ET/period
   draw, enforce `deadline > WCET` for important tasks — the gap P1.8's
   `feasibility_clamp.py` left open because it SKIPS perf tasks, and perf tasks are
   often the high-weight important tasks; P2.13 D1 flagged this).

## What "schedulable for important tasks under DM at the seed" means (precise)

Under the static solution's seed (P0.6): important tasks occupy the top `n_important`
priority slots, DM-ordered within the group, WCET execution at the **min-TL seed point**
(perf-task ET = smallest TL grid option; non-perf ET = WCET). A task τ_i in the
important group is schedulable iff its **worst-case response time** under fixed-priority
interference from ONLY the higher-priority important tasks is ≤ its deadline:

  R_i = WCET_i + Σ_{j ∈ important, prio(j) < prio(i)} ceil(R_i / period_j) · WCET_j
  schedulable_i ⇔ R_i ≤ deadline_i

(The rest-of-tasks group has LOWER priority → they don't interfere with the important
group; that's the whole point of the priority lock. So the schedulability test for the
important group is self-contained: it considers only important-group higher-priority
interference. This is the standard exact fixed-priority RTA, scoped to the important
group.)

**WCET value (D2 RESOLVED 2026-07-27):** WCET per task = the **max ET that task exhibits
across all generated interval tasksets** (the global max, the same value P0.6's offline
walk uses). For P0.8's Python RTA, this means: at generation time, the generator knows
each task's ET distribution parameters; the WCET is the upper bound of that distribution
across all intervals the generator will emit. (For perf tasks, the TL-grid upper bound =
`period * FINAL_Et_OVER_PERIOD_RANGE[1]`; for non-perf, `execution_time_dist.max_time`
= `et_mean + 2*sigma`. P0.8 must use the SAME WCET fields P0.6 uses — consistency
requirement, see "Relationship to P0.6.") Q2 (is the global max computable offline?)
user-confirmed YES; code-verification task #7 in flight.

**Per-core (D3 RESOLVED 2026-07-27):** the generator partitions tasks to cores by
`processorId` (best-fit-decreasing, `taskset_generator.py:555-562`); the orchestrator
runs one RunQueue per core (P1.7). So the RTA is **per-core**: within each core's
important subset, the R_i recurrence over that core's higher-priority important tasks.
The important group is self-contained per core (the priority lock is within-core).

## Where it hooks in (code grounding)

- **Generation side (this task's home):** `Gen_Taskset/lib/taskset_generator.py` —
  the taskset is emitted as `taskset_characteristics*.yaml` (via `yaml_exporter.py`).
  The schedulability test runs post-generation (like `feasibility_clamp.py` does) or
  inline in the generation loop; on failure, retry-regenerate.
- **Existing feasibility logic:** `feasibility_clamp.py` (P1.8) is the precedent — a
  deterministic post-generation pass. It clamps non-perf-task ET inside the period but
  SKIPS perf tasks (`_is_perf_task`, the perf-task gap). This task adds an
  important-task DM-schedulability test that covers BOTH task types (perf tasks
  included — their WCET is the global-max ET per D2).
- **C++ RTA exists** (`sources/Safety_Performance_Metric/RTA.cpp`,
  `RTA_LL.h:25-61`) but is the SCORER's analytic engine, not a generation-time
  feasibility gate. Do NOT couple generation to the C++ RTA — implement the test in
  Python (the generator is Python; a C++ round-trip per taskset is wasteful and
  fragile). The Python RTA for the important group is a small fixed-point iteration
  (the R_i recurrence above); TDD-cover it.
- **The `uunifast_distribution` retry pattern** (`taskset_generator.py:151-170`) is
  the model for the regenerate-on-fail lever (budget 20, per D4).

## Why generation-time (not runtime)

- The static solution is computed offline (P0.6) and is constant across intervals. If
  the taskset isn't DM-schedulable for the important tasks at the seed, NO interval's
  fall-back can save them — so the guarantee must hold at generation time, once per taskset.
- This keeps the runtime fall-back (P0.7) cheap: it can trust the static solution
  (D4 in P0.7 — no per-interval re-check) because the taskset was certified at
  generation.

## Relationship to P0.6 (seed-point consistency)

P0.6's offline walk seeds from DM-grouped PA + min-TL + WCET and skip-and-continues to
find the best-SP safe point. P0.8 certifies the SEED (the most schedulable point —
smallest perf ET = least interference). If even the seed is unschedulable for important
tasks, P0.6's walk has no safe starting point → P0.8's re-generate handles it; P0.6's
filter then preserves schedulability DURING the walk (it never adopts an unsafe
candidate, so the returned best-SP-safe solution inherits the seed's guarantee).

**Consistency requirement:** P0.8's RTA MUST use the SAME seed point (DM-grouped +
min-TL + WCET) and the SAME WCET fields P0.6 uses (the global-max ET per task, D2). If
P0.8 certifies a different point or different WCET, the seed P0.6 starts from might
already be unsafe. P0.8 = precondition; P0.6 = in-walk preservation. The two must agree
on the seed point and the WCET fields.

## Scope (what this task IS / IS NOT)

**IS:**
- A Python fixed-priority RTA test for the important group under DM-with-top-priority-lock
  + WCET at the seed, in `Gen_Taskset/lib/` (e.g. `important_task_rta.py`). Per-core (D3).
- A generation-time gate: every emitted taskset must pass; on failure, regenerate (retry
  pattern, budget 20 per D4); if the rejection rate is excessive, a generator-logic fix
  (second lever).
- Coverage of BOTH non-perf and perf tasks (closes the `feasibility_clamp.py` perf-task
  gap for the important set), using the global-max WCET per D2.
- TDD: the RTA recurrence on constructed tasksets (schedulable / unschedulable /
  boundary R_i = deadline_i), per-core.
- A reported rejection rate (how many regenerations per accepted taskset) so the user
  can decide if the generation logic needs the second lever.

**IS NOT:**
- The static solution itself — P0.6.
- The fall-back invocation — P0.7.
- A change to the C++ scorer's RTA (`RTA.cpp`) — that's the metric's analytic engine;
  this is a generation-time feasibility gate in Python.
- A change to `feasibility_clamp.py` — that clamp stays (it fixes non-perf ET
  feasibility); this task adds the important-task DM guarantee on top. (If the two
  interact — e.g. the clamp's relabeling changes the deadline the RTA tests against —
  document the ordering: clamp first, then important-task RTA, per D5.)
- The "important tasks" selection rule — D1, shared, settled.

## Done when

- [x] **D1 settled (2026-07-27):** important = top-50% by `sp_weight`, persisted as
      `bool Task::is_important` (generator-set — so the Python RTA reads it directly from
      the generated taskset, no duplicate top-X% logic). See P0.6 `goal.md` "Designing
      important tasks" + memory `important-tasks-design-decision`.
- [x] **D2 settled (2026-07-27):** WCET per task = max ET across all generated interval
      tasksets (global max, same as P0.6). Perf WCET = TL-grid upper bound
      (`period * FINAL_Et_OVER_PERIOD_RANGE[1]`); non-perf WCET = `et_mean + 2*sigma` =
      `execution_time_dist.max_time`. P0.8 uses the SAME fields P0.6 uses.
- [x] **D3 settled (2026-07-27):** per-core RTA (generator partitions by `processorId`;
      orchestrator runs one RunQueue per core, P1.7). RTA within each core's important
      subset.
- [x] **D4 settled (2026-07-27):** retry budget = 20, then LOUD raise (NEVER silently
      emit an unschedulable taskset — re-creates the P1.8 substrate). Then propose the
      second lever (generator-logic fix) to the user.
- [x] **D5 settled (2026-07-27):** clamp first, then RTA (the RTA tests the final
      emitted taskset's deadlines).
- [ ] Python `important_task_rta.py`: fixed-priority RTA for the important group
      (R_i recurrence, WCET-based, DM-with-top-lock priority order, per-core). TDD red→green.
- [ ] Generation-time gate: taskset rejected + regenerated on failure; retry pattern
      modeled on `uunifast_distribution`, budget 20. Bounded retries → loud raise on
      exhaustion.
- [ ] Both task types covered (non-perf `execution_time_max`; perf TL-grid upper bound
      per D2) — matching P0.6's WCET fields.
- [ ] Rejection rate reported (log: attempts per accepted taskset, by N).
- [ ] If rejection rate excessive → second lever (generator-logic fix) proposed to the
      user (do NOT implement without user go — it's a generation-behavior change).
- [ ] TDD: `Gen_Taskset/tests/` — RTA recurrence tests (schedulable / unschedulable /
      boundary, per-core) + a generation integration test asserting every emitted taskset
      passes the gate. `pytest Gen_Taskset/tests/` green.
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).

## Open decisions (settled 2026-07-27 — recorded for reference)

- **D1 — Important-task selection rule.** ✅ RESOLVED: top-50% by `sp_weight`, persisted as
  `bool Task::is_important`. The Python RTA's "important group" = the tasks the generator
  labeled `is_important = true` (read directly from the generated taskset object in the
  same Python pass — no duplicate top-X% logic).
- **D2 — Perf-task WCET field.** ✅ RESOLVED: global-max ET across all generated interval
  tasksets (same as P0.6 D2). Perf WCET = `period * FINAL_Et_OVER_PERIOD_RANGE[1]`;
  non-perf WCET = `execution_time_dist.max_time` (= `et_mean + 2*sigma`).
- **D3 — Per-core vs global RTA.** ✅ RESOLVED: per-core (generator partitions by
  `processorId`; orchestrator one RunQueue per core). RTA within each core's important
  subset.
- **D4 — Retry budget + failure mode.** ✅ RESOLVED: 20 retries, then loud raise (NEVER
  silent). Then propose the second lever to the user.
- **D5 — Interaction with `feasibility_clamp.py`.** ✅ RESOLVED: clamp first, then RTA
  (the RTA tests the final emitted taskset's deadlines).

## Reference docs

- `Gen_Taskset/lib/taskset_generator.py:33-34, 145-170, 530-614` — generator config
  knobs (`SP_THRESHOLD_RANGE`, `FINAL_Et_OVER_PERIOD_RANGE`), `uunifast_distribution`
  retry pattern, threshold sampling, core assignment (555-562), perf-task TL grid (588-602).
- `Gen_Taskset/lib/feasibility_clamp.py` — P1.8 post-generation clamp (the precedent;
  the perf-task gap this task closes for the important set).
- `Gen_Taskset/lib/yaml_exporter.py` — taskset emission (`taskset_characteristics*.yaml`).
- `sources/Safety_Performance_Metric/RTA.cpp:154-169` — `GetDDL_MissProbability`
  (the analytic tail; the C++ RTA the Python test mirrors in spirit, NOT in code).
- `sources/Safety_Performance_Metric/RTA_LL.h:25-61` — the C++ fixed-priority RTA
  (reference for the recurrence; do NOT couple generation to it).
- `sources/TaskModel/RegularTasks.h:39-107` — `Task` (`period`, `deadline`,
  `execution_time_dist`, `timePerformancePairs`, `utilization()`).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp` — P1.7 core partitioning
  (`GetProcessorIds`, one RunQueue per core; the per-core semantics D3 matches).
- `simulation_experiments/configs/paper_simulation_config.json:66-67` —
  `important_task_top_percentage` / `minimum_important_tasks_count`.
- Memory [`p18-incr-wcet-outperforms-incr`](../../../) — generator ET-feasibility
  defect + clamp (the perf-task gap this task closes for important tasks).
- Memory [`p213-important-task-ddl-vs-sp-metric`](../../../) — D1 (perf-task DDL
  feasibility for important tasks); the analytic-miss-prob semantics.
- P0.6 `goal.md` — the static solution whose seed schedulability this task guarantees.
- P0.7 `goal.md` — D4 (trust this guarantee vs re-check the static solution at runtime).
