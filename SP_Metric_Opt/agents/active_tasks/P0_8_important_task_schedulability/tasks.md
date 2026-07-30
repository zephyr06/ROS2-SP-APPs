# P0.8 — Tasks (working checklist)

> See `goal.md` for scope + the per-core RTA (D3) + global-max WCET (D2) + seed
> certification. Depends on the shared D1 (important-task selection). Blocks P0.6
> (meaningful safe floor) + P0.7 (D4 — exact-guarantee vs re-check).
> **All design decisions D1–D5 RESOLVED 2026-07-27.**

## 0. Design decisions (RESOLVED)
- [x] **D1 RESOLVED (2026-07-27):** top-50% by `sp_weight`, persisted as
      `bool Task::is_important` (generator-set, YAML-emitted; the Python RTA reads it
      directly from the generated taskset).
- [x] **D2 RESOLVED (2026-07-27):** WCET = max ET across all generated interval tasksets
      (global max, same as P0.6). Perf WCET = `period * FINAL_Et_OVER_PERIOD_RANGE[1]`;
      non-perf WCET = `execution_time_dist.max_time` (= `et_mean + 2*sigma`).
- [x] **D3 RESOLVED (2026-07-27):** per-core RTA (generator partitions by `processorId`;
      orchestrator one RunQueue per core, P1.7).
- [x] **D4 RESOLVED (2026-07-27):** retry budget = 20, then loud raise (NEVER silent).
- [x] **D5 RESOLVED (2026-07-27):** clamp first, then RTA.
- [ ] Record decisions in `dev_log.md` + memory; sync with P0.6/P0.7 owners. **Coordinate
      with P0.6: use the SAME seed point (DM-grouped + min-TL + WCET) and SAME WCET fields
      (global-max ET per task) P0.6 uses** (see `goal.md` "Relationship to P0.6").

## 1. Python fixed-priority RTA for the important group (per-core) — LANDED, staged for review
- [x] TDD red→green: 10 tests in `test_important_task_rta.py` (schedulable, unschedulable
      +culprit, boundary R==deadline PASS via ≤, per-core isolation D3, non-important
      non-interference / priority lock, DM-ordering invariance, overload guard, vacuous,
      single-task, length-mismatch raises).
- [x] `important_task_rta.py`: `important_tasks_schedulable(tasks, wcets) ->
      (bool, culprits)` — per-core (D3) fixed-point RTA over the important group,
      DM-with-top-lock priority order, interference from higher-priority important tasks
      on the SAME core only. Pure function: caller supplies WCET (WCET-source-agnostic).
      `_rta_one_task` = `R = WCET + Σ ceil(R/period)·WCET` recurrence with utilization
      guard (self-term uses period, NOT deadline — caught by TDD) + deadline early-exit,
      loop bound 1500 (matches C++ `RTA_LL`).
- [x] Covers both task types via caller-supplied WCET (per D2: non-perf =
      `execution_time_max`, perf = `period * FINAL_Et_OVER_PERIOD_RANGE[1]`).
- [x] TDD green: `pytest Gen_Taskset/tests/test_important_task_rta.py` = 10 passed.
      Full suite `pytest Gen_Taskset/tests/` = 36 passed (no regressions).
- [x] Smoke vs real generator output: ran the RTA against a real N=4/2-core
      `generate_taskset_parameters` output with the D2 WCET derivation → legitimate
      unschedulable verdict (task_index 1, core 1: util overload → miss). Confirms the
      pure RTA composes with real generator data + D2 WCET; de-risks Step 2.
- [x] Files `git add`-staged (NOT committed — awaits user review). **NEXT: user review of
      Step 1, then settle the Step 2 design fork (below) before wiring the gate.**

### Step 2 design fork (NEEDS USER DIRECTION — do NOT wire unilaterally)
The pure RTA is WCET-source-agnostic, but D2 says "global-max ET across all generated
interval tasksets" (the SAME value P0.6's offline walk uses). At
`generate_taskset_parameters` time the interval tasksets are NOT emitted yet — the trace
loop in `generate_additional_execution_traces` (`orchestrator.py:300-365`) computes the
per-interval `Et_actual` stats (`Et_max` etc.) AFTER `generate_taskset_parameters`
returns. So the WCET-acquisition + retry-placement fork is:
- **(A) Gate AFTER trace generation (post-`feasibility_clamp`, D5-exact):** matches D2
  exactly (reads the emitted characteristics YAMLs' real per-interval max ET). But
  "regenerate" = re-run the whole trace loop per retry (expensive at budget-20 retries),
  and the clamp-then-RTA ordering (D5) implies post-trace anyway.
- **(B) Gate INSIDE `generate_taskset_parameters` (cheap retry, proxy WCET):** uses the
  taskset_param-level WCET (non-perf `et_mean+2σ` / perf TL-grid upper bound) as a
  conservative proxy. Retry is cheap (no trace recompute) but accepts a small divergence
  from P0.6's cross-interval max — the proxy is an upper bound on the per-interval max,
  so a taskset the proxy certifies is also certified by the true max (conservative =
  safe), but a taskset the proxy REJECTS might pass under the true max (false rejection →
  more retries → louder second-lever trigger).
Both are sound; the trade is retry-cost vs D2-exactness. **RESOLVED 2026-07-28: user chose
(A)** — post-trace gate, D2-exact, accepts expensive retry.

### Step 2 prerequisites (RESOLVED 2026-07-28 via user direction)
- **Seeded-retry mechanism:** `generate_taskset_parameters` RE-SEEDS `RANDOM_SEED` at the
  top of every call (`taskset_generator.py:379-382`) → a naive retry produces a
  byte-identical taskset ×20 (no-op). **RESOLVED: advance seed per attempt**
  (`RANDOM_SEED + attempt`) — localized to the retry wrapper; `generate_taskset_parameters`
  keeps its per-call re-seed (other callers' reproducibility untouched).
- **Retry-loop placement:** **RESOLVED: wrapper function** — new
  `run_full_generation_pipeline_with_important_task_gate` calls the canonical pipeline,
  runs the post-clamp RTA, loops on failure; keeps `run_full_generation_pipeline`'s
  contract intact for the ~15 existing callers.

## 2a. WCET acquisition from emitted characteristics (option A) — LANDED, staged
- [x] TDD red→green: `test_important_task_gate.py` (7 tests) for
      `compute_wcets_from_characteristics(dir_path, cfgs) -> {gid: wcet}`:
      non-perf = MAX `execution_time_max` across interval YAMLs (D2 global max); perf =
      `period * FINAL_Et_OVER_PERIOD_RANGE[1]` (TL-grid upper bound, NOT the on-disk
      `execution_time_max` which is a grid bound — closes the `feasibility_clamp`
      perf-skip gap); post-clamp (reads on-disk clamped value — D5); reads per-interval
      FULL files ONLY (not the global k=0 copy or per-processor splits — no
      double-count); keyed by `gid` (stable identity, not per-processor-local `id`);
      missing interval files → loud raise (NEVER silent).
- [x] Full suite green: `pytest Gen_Taskset/tests/` = 43 passed (36 + 7 new; the one
      transient integration failure was a pre-existing flaky test-ordering issue,
      passes in isolation + on clean re-run — NOT my change).

## 2b. Gate wrapper + seed-advancing retry loop — LANDED (staged, awaits commit)
- [x] TDD red: `test_gate_passes_first_try` + `test_gate_retries_until_schedulable`
      + `test_gate_raises_on_budget_exhaustion` +
      `test_gate_advances_seed_so_retries_re_sample` (4 red cases; RED because
      `_run_pipeline_with_cfgs` + the wrapper did not exist yet).
- [x] Implemented `run_full_generation_pipeline_with_important_task_gate`
      (`orchestrator.py`): loads cfgs ONCE, advances `cfgs["RANDOM_SEED"] =
      base_seed + attempt` per retry, calls `_run_pipeline_with_cfgs` (the body)
      directly, then `_load_emitted_tasks_by_gid` + `_wcets_from_loaded_tasks` +
      `important_tasks_schedulable` post-clamp; budget 20 (`IMPORTANT_TASK_GATE_MAX_ATTEMPTS`,
      D4); loud `RuntimeError` on exhaustion (NEVER silent — message includes
      final culprits + attempt count + seed range + remediation hint).
- [x] Split `run_full_generation_pipeline` → thin shell (loads+validates cfgs,
      delegates) + `_run_pipeline_with_cfgs(cfgs, ...)` (the body, takes a
      REQUIRED pre-loaded cfgs — NO default args, per user preference). The ~8
      existing callers of the shell are untouched. The gate calls the body
      directly because the shell reloads cfgs from the config FILE each call,
      which would discard the advanced seed → byte-identical taskset every retry
      (the no-op-retry bug the gate exists to prevent).
- [x] Refactor (DRY): extracted `_load_emitted_tasks_by_gid(dir_path) -> {gid: task}`
      (I/O + gid dedup, worst-case-ET representative) +
      `_wcets_from_loaded_tasks(tasks_by_gid, tl_grid_upper) -> {gid: wcet}`
      (pure WCET rule) in `important_task_rta.py`; `compute_wcets_from_characteristics`
      is now a thin composition (public API + 7 existing tests unchanged).
- [x] **Key-normalization correctness fix:** the emitted C++-format YAML uses key
      `important` (`yaml_exporter.py:73`) but the RTA reads `is_important`
      (`important_tasks_schedulable` → `t.get("is_important")`). The reader sets
      `task["is_important"] = task.get("important", False)` on every loaded task
      — WITHOUT this the gate feeds emitted-form tasks to the RTA, which sees NO
      important tasks → vacuously "schedulable" → a hollow gate (the exact silent
      hole P0.8 prevents). Caught by design review before first run.
- [x] Returns a report: `{"schedulable": True, "attempts_used": int, "culprits": []}`
      on success (culprits empty — the passing attempt had no misses); raises on
      exhaustion.
- [x] TDD green: `pytest Gen_Taskset/tests/test_important_task_gate.py` = 11 passed
      (7 WCET + 4 wrapper); `pytest Gen_Taskset/tests/` = 47 passed (was 43; +4
      wrapper tests, no regressions).
- [x] Test strategy = mock `_run_pipeline_with_cfgs` (monkeypatch) writing
      synthetic characteristics YAMLs (unschedulable attempt 0 / schedulable
      attempt 1 / never-schedulable for exhaustion); REAL
      `compute_wcets_from_characteristics` + `important_tasks_schedulable` run on
      the synthetic YAMLs (RTA path exercised, expensive trace gen NOT).
      End-to-end real-pipeline coverage DEFERRED (not this step).

## 2. Generation-time gate
- [ ] TDD red: a generation integration test — emit a taskset, assert it passes the
      important-task RTA gate.
- [ ] Wire the RTA into the generation flow (post-`feasibility_clamp`, per D5):
      on FAIL, regenerate (retry pattern modeled on `uunifast_distribution`).
- [ ] Bounded retries = 20 (per D4); on exhaustion, LOUD raise (no silent unschedulable
      taskset — avoids re-creating the P1.8 substrate).
- [ ] TDD green: `pytest Gen_Taskset/tests/test_integration.py` — every emitted
      taskset passes the gate.

## 3. Rejection-rate reporting + second-lever decision
- [ ] Log the rejection rate (attempts per accepted taskset, by N) across a smoke
      generation sweep.
- [ ] If rejection rate excessive → propose a generator-logic fix (second lever:
      tighten ET/period draw / enforce `deadline > WCET` for important tasks) to the
      user. Do NOT implement without user go.

## 4. Verification + records
- [x] `pytest Gen_Taskset/tests/` green (47 passed — full generator test suite).
- [x] Confirm the gate covers the important-task types the C++ scorer will actually
      see (cross-check a generated taskset's important tasks against
      `feasibility_clamp.py`'s perf-task skip — the gap this task closes).
      [perf → `period * FINAL_Et_OVER_PERIOD_RANGE[1]`; non-perf → global-max
      `execution_time_max`; both in `_wcets_from_loaded_tasks`.]
- [x] Confirm the WCET fields match P0.6's (global-max ET per task — D2 consistency).
      [`_load_emitted_tasks_by_gid` keeps the worst-case-ET representative per gid.]
- [x] `dev_log.md` (this folder + top-level) + memory updated.
- [x] `git add` staged; user reviews (no commit). [3 files: `orchestrator.py`,
      `important_task_rta.py`, `test_important_task_gate.py`.]
- [ ] **DEFERRED (Step 2 generation-time gate wiring):** end-to-end real-pipeline
      test in `test_integration.py` (emit a real taskset, assert the gate passes)
      + wiring the wrapper into prod entry points (`run_generator.py`,
      `run_sim_experiments.py`) — behavior change for the prod pipeline, separate
      user-reviewed step. The wrapper existing + unit-tested is the unit of value
      here (Step 2b); wiring it in is Step 2.
- [ ] **DEFERRED (Step 3):** rejection-rate reporting + second-lever generator-logic
      fix proposal (user-go only).
