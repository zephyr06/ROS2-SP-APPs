# P0.9 — System-wide Deadline Monotonic + important-first priority lock (seed PA)

> **Scope:** switch the priority-assignment **seed** (the re-opt / interval-0 first
> init) from plain Rate Monotonic (period sort) to **Deadline Monotonic (deadline
> sort) with an important-first group lock**, and make the C++ seed, the Python RTA,
> and the P0.6 plan all agree on it.
>
> **Why a separate task:** this is a lockstep prerequisite for P0.6 + P0.8. P0.8
> certifies that the important tasks are schedulable under the static solution's
> priority order at the SEED. If the Python RTA certifies under one order (DM,
> important-first) while the C++ scheduler runs another (plain RM), the
> certification is hollow — exactly the silent correctness hole P0.8 exists to
> prevent. So the order must be unified **system-wide** before P0.8's gate wires in.
> This task supersedes the `AssignRMRespectingGroupOrder` helper P0.6 planned
> (`P0_6_static_solution/goal.md:42,281`) — same important-first block structure,
> but DM-within-group, and moved into the shared first-init now (not P0.6-only).

## Motivation

1. **Constrained deadlines make RM suboptimal.** The generator sets
   `deadline = period * uniform(0.5, 1.0)` (`taskset_generator.py:534`), so `D < T`
   is common. For constrained deadlines, **Deadline Monotonic is the optimal
   fixed-priority assignment** — DM certifies every taskset RM does, plus more.
   Running RM while certifying under DM (or vice-versa) is a divergence; running
   DM everywhere is strictly better for schedulability.

2. **The important-first group lock is P0.6/P0.8's core idea.** Important tasks
   (top-50% by `sp_weight`, `bool Task::is_important`) occupy the TOP priority
   slots; non-important fill the lower slots. This means non-important tasks
   **never interfere** with the important group → the important-group RTA is
   self-contained (only higher-priority important tasks on the same core
   interfere). The Python RTA (`important_task_rta.py`) already assumes this lock
   implicitly (it filters to important tasks only). The C++ seed must use the SAME
   lock so the certification matches the running scheduler.

3. **User direction (2026-07-28):** the re-optimization's first initialization
   method should match the Python schedulability test — "first sort by importance,
   then assign DM to important and non-important task separately."

## What changes

### C++ — the seed PA (the re-opt / interval-0 first init)

`SeedIncumbentFromRMFast` (`OptimizeSP_TL_Incre.cpp:758-766`) currently calls
`RateMonotonicPriorityVec()` (`:695-707`) — plain RM: sort all tasks by period
ascending, ties by avg ET. This is the seed PA for:
- `ResetIncumbentBaseline`'s interval-0 else-branch (re-opt's first interval), AND
- `BootstrapIncumbentFromRMFast` (the `INCR_NO_REOPT` arm's interval-0 entry).

**Change:** replace the plain-RM sort with an **important-first group-locked DM**
sort:
1. Partition tasks by `is_important` (important first).
2. Within each group, sort by **deadline** ascending (DM), ties by avg ET
   ascending (deterministic — matches the current RM tie-break).
3. Concatenate: [important (DM-ordered)] ++ [non-important (DM-ordered)].
   Every non-important task lands below every important task → the priority lock.

This is `AssignDMRespectingGroupOrder(important_ids)` — the DM analogue of P0.6's
planned `AssignRMRespectingGroupOrder`, promoted into the shared first-init.

### Python — the RTA sort key

`_important_priority_order` (`important_task_rta.py:123-152`) currently sorts the
important subset by **period** (RM-within-important). **Change:** sort by
**deadline** (DM-within-important). The group lock is already implicit (the
function filters to important only). Docstrings + the "RM-ordering invariance"
test (`test_important_task_rta.py`) update from RM framing to DM.

### Records — P0.6 plan refs

`P0_6_static_solution/goal.md` references "RM-grouped PA", "RM-ordered within the
group", and `AssignRMRespectingGroupOrder`. Update to "DM-grouped",
"DM-ordered within the group", `AssignDMRespectingGroupOrder`. The seed point
P0.6 + P0.8 share becomes **DM-grouped + min-TL + WCET**.

### Out of scope

- The **orchestrator** has no own RM/DM path (grounded: empty grep on
  `SimulationOrchestrator.{cpp,h}` for priority-assignment terms) — it consumes
  baked `priority` fields. No orchestrator change.
- The optimizer's **descent** (PA moves during the walk) is untouched — only the
  **seed PA** changes. P0.8 certifies the seed point; the descent is P0.6's
  in-walk preservation (skip-and-continue filter), separate task.

## Design decisions

- [x] **D1 RESOLVED (2026-07-28, user direction):** sort key = **deadline**
      ascending (DM), NOT period (RM). Constrained deadlines (`D = T·U(0.5,1.0)`)
      make DM optimal.
- [x] **D2 RESOLVED (2026-07-28, user direction):** important-first group lock —
      important tasks occupy the top slots, DM-ordered within the group; non-
      important fill the lower slots, DM-ordered within the group. Matches the
      Python RTA's implicit lock exactly.
- [x] **D3 RESOLVED (2026-07-28, grounded):** scope = C++ seed PA
      (`RateMonotonicPriorityVec` → group-locked DM) + Python RTA sort key + P0.6
      plan refs. Orchestrator excluded (no own RM path).
- [x] **D4 RESOLVED (2026-07-28, user direction):** **rename to DM** —
      `RateMonotonicPriorityVec` → `DeadlineMonotonicPriorityVec`,
      `SeedIncumbentFromRMFast` → `SeedIncumbentFromDMFast`,
      `BootstrapIncumbentFromRMFast` → `BootstrapIncumbentFromDMFast`, across
      `.h`/`.cpp`/tests/comments. Full cascade.
- [x] **D5 RESOLVED (2026-07-28, user direction):** DM tie-break = **avg ET
      ascending** (`execution_time_dist.GetAvgValue()`), matching the current RM
      tie-break (smallest behavior delta; deterministic).
- [x] **D6 RESOLVED (2026-07-28, user direction):** **accept the behavior change**.
      Verification gate = "important-task schedulability improves/holds + SP within
      an agreed tolerance", NOT bit-identical. Log the SP delta. Prod A/B re-run
      proposed separately (not run unilaterally).
- [x] **D7 RESOLVED (2026-07-28, user direction):** **scope = FULL system-wide** —
      also switch the orchestrator's standalone baseline arms `"RM"`/`"RM_FAST"`/
      `"RM_SLOW"` (`SimulationOrchestrator.cpp:356-401`) to deadline-sort + rename
      the strings to `"DM"`/`"DM_FAST"`/`"DM_SLOW"`. Cascades into `TaskData/*.json`
      configs + 10+ result/figure Python scripts + tests. Breaks result-
      comparability with prior runs (accepted). Rationale: these baselines have
      their OWN period-sort (`SimulationOrchestrator.cpp:361,373,393`) — leaving
      them RM while the certified path goes DM would re-introduce the divergence
      P0.8 exists to prevent; "system-wide" means system-wide. The fall-back uses
      the P0.6 static solution (separate from these baselines), but the baselines
      must still agree on the priority model for the comparison to be meaningful.

## Done when

- [ ] C++ `RateMonotonicPriorityVec` (or its replacement) produces important-first
      group-locked DM; `SeedIncumbentFromRMFast` uses it.
- [ ] Python `_important_priority_order` sorts by deadline; docstrings + tests
      updated to DM; `pytest Gen_Taskset/tests/` green.
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (DEBUG build;
      `--clean-first` if header layout changes — [[sp-opt-test-build-debug-config]]).
- [ ] P0.6 `goal.md` refs updated RM → DM; P0.8 records updated.
- [ ] `dev_log.md` + memory (`p09-...`, `p06-...`, `p08-...`, MEMORY index) updated.
- [ ] `git add` staged; user reviews (no commit).

## Relationship to other tasks

- **Blocks P0.6 + P0.8:** both depend on the DM + group-locked seed PA being in
  place. P0.8's gate certifies under this order; P0.6's static solution seeds from
  it. Land P0.9 BEFORE P0.8's Step 2b gate-wiring and P0.6's walk.
- **Supersedes P0.6's `AssignRMRespectingGroupOrder` helper** (`goal.md:42,281`):
  same important-first block, DM not RM, promoted into the shared first-init.
- **Cross-links:** [[p08-important-task-schedulability]] (the certification this
  aligns with), [[p06-static-solution-fallback-seed]] (same seed point + WCET
  fields), [[important-tasks-design-decision]] (`is_important` labeling).
