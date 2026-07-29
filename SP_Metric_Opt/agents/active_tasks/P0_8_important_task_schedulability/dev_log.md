# P0.8 Important-Task Schedulability — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-28 (P0.9 supersedence — plan docs relabeled RM→DM)

- **P0.9 (DM + important-first group lock) LANDED.** The C++ seed PA is now
  `DeadlineMonotonicPriorityVec`; this task's Python RTA sort key was likewise switched
  period→deadline (`_important_priority_order`, landed 2026-07-28; `pytest Gen_Taskset/tests/`
  43 green). So the certification now agrees with the seed on a DM + important-first
  group lock.
- To keep this task's forward-looking plan consistent with the landed seed, `goal.md` +
  `tasks.md` were relabeled RM→DM on 2026-07-28: the priority order under test is now
  "DM-with-top-lock" / "DM-ordered within the important group", the seed point is
  "DM-grouped + min-TL + WCET", and the ordering-invariance test property is
  "DM-ordering invariance".
- **Historical entries below retain their original RM wording** (point-in-time records,
  not rewritten — same "don't falsify history" treatment as the P2.11 leave). They
  describe what was true when written; the relabeled plan docs above are authoritative.

## 2026-07-26

- Task scaffolded from the user's fall-back design direction (the generation-time
  schedulability lever). Scope + 5 open design decisions (D1–D5) recorded in
  `goal.md`; `tasks.md` checklist written.
- **BLOCKER:** D1 (important-task selection) shared with P0.6/P0.7; D3 (per-core vs
  global RTA for `N_CORES=2`) is THIS task's decisive fork.
- Grounding: the test is a Python fixed-priority RTA (R_i recurrence) for the
  important group under RM-with-top-priority-lock + WCET, scoped to higher-priority
  important-task interference only (the rest group is lower priority → no interference
  — that's the point of the P0.6 priority lock). Generation-time gate in
  `Gen_Taskset/lib/`, retry-on-fail modeled on `uunifast_distribution`
  (`taskset_generator.py:151-170`); NEVER silently emit an unschedulable taskset
  (re-creates the P1.8 substrate). Closes the `feasibility_clamp.py` perf-task gap
  (`_is_perf_task`) for the important set — perf tasks are often the high-weight
  important tasks (P2.13 D1).
- Not started; awaiting D1/D2/D3 resolution.

## 2026-07-27 (final design lock)

- **D2 RESOLVED (WCET value).** WCET per task = the **max ET that task exhibits across all
  generated interval tasksets** (the global max — the SAME value P0.6's offline walk
  uses). For P0.8's Python RTA: perf WCET = TL-grid upper bound
  (`period * FINAL_Et_OVER_PERIOD_RANGE[1]`); non-perf WCET = `execution_time_dist.max_time`
  (= `et_mean + 2*sigma`). User confirmed interval ETs are pre-generated → global max
  computable offline at generation time. Code-verification of the exact acquisition
  mechanism is task #7 (in flight). Consistency requirement: P0.8 uses the SAME WCET
  fields P0.6 uses — if they diverge, the seed P0.6 starts from might not be the point
  P0.8 certified.
- **D3 RESOLVED (per-core).** The generator partitions tasks to cores by `processorId`
  (best-fit-decreasing, `taskset_generator.py:555-562`); the orchestrator runs one
  RunQueue per core (P1.7). So the RTA is per-core: within each core's important subset,
  the R_i recurrence over that core's higher-priority important tasks. A task on core 0
  is NOT interfered by a higher-priority important task on core 1. This matches the
  orchestrator's actual scheduling semantics (not the C++ scorer's global-ish RTA).
- **D4 RESOLVED (retry budget).** 20 retries, then LOUD raise (NEVER silently emit an
  unschedulable taskset — that would re-create the P1.8 substrate). Then propose the
  second lever (generator-logic fix) to the user. (Prior default was 100; tightened to
  20 — if rejection rate is high at 20, the second lever is the right response, not more
  retries.)
- **D5 RESOLVED (ordering).** `feasibility_clamp.py` first, then important-task RTA —
  the RTA tests the final emitted taskset's deadlines (post-clamp relabeling).
- **Seed certification (relationship to P0.6 refined).** P0.6's refined algorithm seeds
  from RM-grouped + min-TL + WCET and skip-and-continues to find the best-SP safe point.
  P0.8 certifies the SEED (the most schedulable point — smallest perf ET = least
  interference), NOT the walk's result (the walk is simulation-start, P0.8 is
  generation-time — P0.8 can't see the walk's output). If even the seed is unschedulable,
  P0.6's walk has no safe start → P0.8's re-generate handles it; P0.6's filter then
  preserves schedulability during the walk (never adopts an unsafe candidate → the
  returned best-SP-safe solution inherits the seed's guarantee). P0.8 = precondition;
  P0.6 = in-walk preservation.
- All D1–D5 now RESOLVED. `goal.md` "Open decisions" + "Done when" + `tasks.md` step 0
  updated. Next: implement `important_task_rta.py` (per-core, global-max WCET) — gated on
  step-0.5 (`is_important` emitted to YAML). Still no code.

## 2026-07-29 (step-0.5 unblocked → Step 1 landed: RTA module + TDD)

- **Step-0.5 prerequisite COMMITTED** (`6eacc8a3 add is_important to tasks`): the shared
  `bool Task::is_important` primitive is in. `important_task_rta.py` can now read the
  important set directly from the generated taskset object (`t.get("is_important")`), no
  duplicate top-X% logic. Unblocks this task's Step 1.
- **Step 1 LANDED (TDD red→green):** `Gen_Taskset/lib/important_task_rta.py` + tests
  `Gen_Taskset/tests/test_important_task_rta.py` (10 tests, all green). The recurrence is a
  pure function — caller supplies per-task WCET (per D2: perf = `period *
  FINAL_Et_OVER_PERIOD_RANGE[1]`, non-perf = `execution_time_max`); the module is
  WCET-source-agnostic. Mirrors the C++ `RTA_LL` recurrence (`RTA_LL.h:65-92`) in spirit,
  NOT in code.
  - `important_tasks_schedulable(tasks, wcets) -> (bool, culprits)`: per-core (D3)
    fixed-point RTA over the important group, RM-with-top-lock priority order
    (`_important_priority_order` sorts important tasks by ascending period; non-important
    excluded — they're lower priority, the priority lock). Returns per-task culprits
    (`task_index`, `core`, `response_time`, `deadline`) so a rejection can be diagnosed.
  - `_rta_one_task`: the `R^{k+1} = WCET_i + Σ ceil(R^k/period_j)·WCET_j` fixed-point loop,
    with a utilization guard (Σ WCET/period ≥ 1 → `inf`, matching `RTA_LL.h:97-100`) and a
    deadline early-exit. Loop bound 1500 (matches C++).
  - **Bug caught by TDD:** initial utilization guard divided the self-term by `deadline_i`
    instead of `period_i`. `test_single_important_task_response_equals_wcet` (WCET=30 >
    deadline=20, but period=100 → util 0.3) tripped it: the guard returned `inf` instead of
    letting the recurrence certify R=30. Fixed: self-term uses `period_i` (utilization =
    WCET/period, matching C++ `Task::utilization()`; a task can be util-light yet miss a
    tight deadline — the guard must not conflate the two). This is exactly the kind of
    deadline-vs-period semantics [[p213-important-task-ddl-vs-sp-metric]] tracks.
  - Tests cover: schedulable, unschedulable (+culprit), boundary R==deadline (PASS, ≤),
    per-core isolation (cross-core HP does NOT interfere — D3), non-important
    non-interference (priority lock), RM-ordering invariance (period-based, not index),
    overload guard, vacuous (no important tasks), single-task, length-mismatch raises.
  - **Smoke vs real generator output:** ran `important_tasks_schedulable` against a real
    N=4/2-core `generate_taskset_parameters` output with the D2 WCET derivation. Got a
    legitimate unschedulable verdict (task_index 1, core 1: util 0.59+0.576=1.166 → overload
    → miss) — exactly the substrate P0.8 exists to catch. Confirms the pure RTA composes
    correctly with real generator data + D2 WCET; de-risks Step 2 (gate-wiring).
- **Full generator suite green:** `pytest Gen_Taskset/tests/` = 36 passed (26 baseline +
  10 new RTA). No regressions.
- **NOT yet done (Step 2 — gate-wiring, separate review increment):** the RTA is a pure
  function; it is NOT yet wired into the generation flow. Step 2 hooks it into
  `orchestrator.run_full_generation_pipeline` post-`feasibility_clamp` (D5: clamp first,
  then RTA) with a retry-regenerate loop (budget 20 per D4, loud raise on exhaustion —
  NEVER silent). Open mechanism questions for Step 2 (to settle with the user, NOT
  unilaterally):
  1. **WCET acquisition (D2 code-verification, task #7):** the smoke test used per-task
     `execution_time_max` for non-perf and `period * FINAL_Et_OVER_PERIOD_RANGE[1]` for perf
     — the taskset_param-level fields available at generation time. But D2 says "global-max
     ET across all generated interval tasksets" (the SAME value P0.6's offline walk uses).
     At `generate_taskset_parameters` time the interval tasksets aren't emitted yet (the
     trace loop in `generate_additional_execution_traces` produces the per-interval
     `Et_actual` stats). So either (a) the gate runs AFTER trace generation on the emitted
     characteristics YAMLs (like `feasibility_clamp` does — but then "regenerate" means
     re-running the whole trace loop, expensive), or (b) the gate uses the
     taskset_param-level WCET (et_mean+2σ / TL-grid upper bound) as a conservative proxy
     and accepts the small divergence from P0.6's cross-interval max. This is a real fork
     that affects the retry placement and cost. **Needs user direction.**
  2. **Retry placement + cost:** regenerating inside `generate_taskset_parameters` is cheap
     (no trace recompute) but can only use the proxy WCET (option b above); regenerating
     after trace generation (option a) matches D2 exactly but re-runs the trace loop per
     retry (expensive at 20 retries). The clamp-then-RTA ordering (D5) also implies the
     gate runs after the clamp, which is post-trace. Tension with the "cheap retry" goal.
  3. **Rejection-rate reporting (tasks.md Step 3):** log attempts-per-accepted by N across
     a smoke sweep; if excessive → propose the second lever (generator-logic fix:
     tighten ET/period draw / enforce `deadline > WCET` for important tasks) to the user.
     Do NOT implement the second lever without user go.
- Next: stage Step 1 (`important_task_rta.py` + `test_important_task_rta.py`), ask user
  review. Step 2 design questions above are deferred to that review — the pure RTA is
  reviewable in isolation now.

## 2026-07-28 (Step 1 staged for review; Step 2 fork grounded + surfaced)

- **Step 1 `git add`-staged** (NOT committed — awaits user review):
  `Gen_Taskset/lib/important_task_rta.py` + `Gen_Taskset/tests/test_important_task_rta.py`.
  Re-confirmed green: `pytest Gen_Taskset/tests/test_important_task_rta.py` = 10 passed;
  full suite `pytest Gen_Taskset/tests/` = 36 passed (no regressions).
- **Grounded the Step 2 WCET-acquisition fork** by reading the pipeline flow:
  - `run_full_generation_pipeline` (`orchestrator.py:367-440`) calls
    `generate_taskset_parameters` (step 1, cheap — no traces) → exports
    `taskset_param.yaml` → `generate_additional_execution_traces` (step 3, the trace
    loop) → `clamp_avg_et_to_period` (step 4, D5 feasibility clamp, post-trace, rewrites
    the emitted characteristics YAMLs in place).
  - The per-interval `Et_actual` stats (`Et_max` etc.) are computed INSIDE the trace loop
    (`orchestrator.py:300-312`), AFTER `generate_taskset_parameters` returns. So at
    `generate_taskset_parameters` time the global-max ET (D2) is NOT yet known — only the
    taskset_param-level distribution params (`et_mean`, `sigma` → `execution_time_max` for
    non-perf; TL-grid upper bound for perf).
  - This is the real fork behind the dev_log's open mechanism question #1. Two sound
    options (recorded in `tasks.md` Step 2):
    - **(A) Gate AFTER trace generation** (post-`clamp_avg_et_to_period`, D5-exact): reads
      the real per-interval max ET from the emitted characteristics YAMLs → matches D2
      exactly. But "regenerate" = re-run the whole trace loop per retry (expensive at
      budget-20). D5's clamp-then-RTA ordering implies post-trace anyway.
    - **(B) Gate INSIDE `generate_taskset_parameters`** (cheap retry, proxy WCET): uses
      the param-level WCET (`et_mean+2σ` / TL-grid upper bound) as a conservative proxy.
      Retry is cheap (no trace recompute). The proxy is an UPPER BOUND on the per-interval
      max → a taskset the proxy certifies is also certified by the true max (safe), but a
      taskset the proxy REJECTS might pass under the true max (false rejection → more
      retries → louder second-lever trigger). Accepts a small divergence from P0.6's
      cross-interval max.
  - Trade = retry-cost (B cheap, A expensive) vs D2-exactness (A exact, B conservative
    proxy). Both are sound; NOT wiring unilaterally.
- **Surfacing to user:** Step 1 review + the (A)/(B) fork. The pure RTA is reviewable in
  isolation now; the fork decides where the gate + retry loop live. Awaiting user
  direction before Step 2 implementation.

## 2026-07-28 (Step 2 placement = (A); seeded-retry prerequisite found)

- **User direction received: option (A)** — gate AFTER trace generation
  (post-`clamp_avg_et_to_period`, D5-exact), reads real per-interval max ET from the
  emitted characteristics YAMLs. Accepts the expensive retry (re-runs the trace loop per
  attempt, budget 20 per D4).
- **Grounded the WCET source for (A)** from `yaml_exporter.py:41-88`:
  - **Perf task** (`performance_records_time` present): WCET = `period *
    FINAL_Et_OVER_PERIOD_RANGE[1]` (TL-grid upper bound; set deterministically at
    exporter line 79, identical across intervals).
  - **Non-perf task**: WCET = **max `execution_time_max` across all emitted
    `taskset_characteristics_interval_*.yaml`** (each interval's `Et_max` from the trace
    loop `orchestrator.py:303-312`). The clamp may pull `execution_time_max` DOWN, so the
    global max MUST be computed **post-clamp** (D5: clamp first, then RTA — consistent).
- **CRITICAL prerequisite found (would be a bug if missed):** `generate_taskset_parameters`
  RE-SEEDS with `cfgs["RANDOM_SEED"]` at the top of EVERY call
  (`taskset_generator.py:379-382`: `np.random.seed(seed); random.seed(seed)`). So a retry
  loop that naively re-invokes `run_full_generation_pipeline` with the same config produces
  a **byte-identical taskset every attempt** → 20 identical attempts → loud raise with zero
  actual re-sampling. The retry is a no-op unless the seed is advanced per attempt
  (e.g. `RANDOM_SEED + attempt`) OR the re-seed is moved out of the per-call path. This is
  a real design fork for Step 2's retry mechanism — NOT decided unilaterally; surfacing to
  the user alongside the (A) confirmation.
- **Remaining Step 2 design questions (to settle with the user):**
  1. Seeded-retry mechanism: advance seed per attempt (`seed + attempt`) vs move the
     re-seed out of `generate_taskset_parameters` (broader blast radius — affects all
     callers' reproducibility). Prefer the former (localized, opt-in for the retry loop).
  2. Retry loop placement: wrap `run_full_generation_pipeline` in the gate (caller-side,
     e.g. a new `run_full_generation_pipeline_with_important_task_gate` wrapper) vs embed
     inside `run_full_generation_pipeline` (changes the canonical entry point's signature
     contract — many callers, see grep). Prefer the wrapper (keeps the canonical pipeline
     as-is; the gate is an opt-in safety layer).
- Next: surface the seeded-retry prerequisite + placement to the user; on direction,
  TDD the gate (red: emit a taskset, fail the RTA, retry advances the seed, eventually
  PASS or loud-raise) then wire.
