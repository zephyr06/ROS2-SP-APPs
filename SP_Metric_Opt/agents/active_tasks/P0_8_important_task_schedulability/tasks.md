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

## 2b. Gate wrapper + seed-advancing retry loop — NEXT
- [ ] TDD red: a retry test — a config whose first draw is unschedulable, second draw
      (advanced seed) schedulable → wrapper returns PASS after 2 attempts.
- [ ] TDD red: budget exhaustion → loud raise (NEVER silent).
- [ ] TDD red: seed advancement actually re-samples (assert two attempts produce
      DIFFERENT tasksets — guards the no-op-retry bug).
- [ ] Implement `run_full_generation_pipeline_with_important_task_gate`: wrap the
      canonical pipeline, run `compute_wcets_from_characteristics` +
      `important_tasks_schedulable` post-clamp, loop on fail with `RANDOM_SEED +
      attempt`, budget 20 (D4), loud raise on exhaustion.
- [ ] Return a report (attempts used, culprits on the final pass).

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
- [ ] `pytest Gen_Taskset/tests/` green (full generator test suite).
- [ ] Confirm the gate covers the important-task types the C++ scorer will actually
      see (cross-check a generated taskset's important tasks against
      `feasibility_clamp.py`'s perf-task skip — the gap this task closes).
- [ ] Confirm the WCET fields match P0.6's (global-max ET per task — D2 consistency).
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).
