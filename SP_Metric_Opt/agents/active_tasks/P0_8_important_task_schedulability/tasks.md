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
- [x] TDD red: a generation integration test — emit a taskset, assert it passes the
      important-task RTA gate. (Step 2b mock tests + the Step 2 e2e real-pipeline
      test below.)
- [x] Wire the RTA into the generation flow (post-`feasibility_clamp`, per D5):
      on FAIL, regenerate (retry pattern modeled on `uunifast_distribution`).
      **PROD-WIRED (Step 2, this session):** `run_generator.py` +
      `run_sim_experiments.py` route through the gate BY DEFAULT
      (`--no-important_tasks_schedulability_check` opt-out). The gate IS the generation flow now.
- [x] Bounded retries = 20 (per D4); on exhaustion, LOUD raise (no silent unschedulable
      taskset — avoids re-creating the P1.8 substrate).
- [x] TDD green: `pytest Gen_Taskset/tests/test_integration.py` — every emitted
      taskset passes the gate. (`test_gate_end_to_end_real_pipeline_certifies_
      schedulable_taskset` runs the REAL pipeline through the gate + independently
      re-checks the certificate; `pytest Gen_Taskset/tests/` = 48 passed.)

## 2d. Step 2 prod-wiring + end-to-end real-pipeline test — LANDED (working tree, git add-only — awaits user commit)
- [x] `run_generator.py`: generation branch → gate BY DEFAULT; `--no-important_tasks_schedulability_check`
      opt-out; surfaces `attempts_used`. `--gen_path_for_taskset` branch untouched.
- [x] `run_sim_experiments.py`: both call sites via `_generate_taskset` helper
      (gate-vs-plain by the flag); **seed-collision fix** — per-taskset seed step
      widened to `IMPORTANT_TASK_GATE_MAX_ATTEMPTS` (20) when gate ON (was +1 →
      retry window collided with the next taskset's draw → silent duplicates);
      legacy +1 when OFF (bit-identical pre-gate).
- [x] E2E test `test_gate_end_to_end_real_pipeline_certifies_schedulable_taskset`
      in `test_integration.py`: REAL pipeline (no monkeypatch) through the gate;
      asserts `schedulable=True` + `attempts_used>=1` + `culprits==[]` AND an
      INDEPENDENT re-run of the RTA primitives on the emitted YAMLs reproduces
      `ok=True` (guards against a hollow gate). Asserts `n_important>=1` (no
      vacuous RTA).
- [x] Smoke: gated `taskset_cfg_paper_4.json` → certified attempts_used=1, exit 0;
      opt-out → plain pipeline, exit 0.

## 2c. Config-tuning round (make the gate pass within budget on the REAL config) — LANDED + COMMITTED `7c8748c0`
- [x] Diagnose why the committed gate (`3d2360ed`) rejects real tasksets:
      (A) perf WCET was `period×0.9` (TL-grid MAX) → overload;
      (B) non-perf WCET `et_mean+2σ` with σ/et∈[0.5,0.6] → 2.0–2.2× amplifier;
      (C) env cap 0.45 → env WCET/period ≈0.99. (Cap-raise NOT the cause —
      real-config `total/n ≤ 0.75` means it never fires.)
- [x] Perf WCET rule → `execution_time_mu` (= et_mean): faithful (sim runs
      `min(et_mean, TL)`, TL is a downward cap) + sound (tightest safe). Dropped
      `tl_grid_upper`/`cfgs` params from `_wcets_from_loaded_tasks` +
      `compute_wcets_from_characteristics`; TL grid now irrelevant to the verdict.
      Loud `KeyError` on missing `execution_time_mu` (no fallback).
- [x] Env cap 0.45→0.27 + variance [0.5,0.6]→[0.3,0.4] → env WCET/period ≤0.486.
- [x] (3a) no-inflation: cpu_util [0.5,1.5]→[0.5,1.0]; DROP the proportional
      redistribution block (raises non-env `u_i` above drawn = inflation; also
      inflates perf `execution_time_mu` = perf WCET). Strictly safe (only lowers load).
- [x] `DEADLINE_MODE=implicit` (deadline=period → RM≡DM).
- [x] Synced diagnostic `measure_important_utilization.py` + 7 WCET tests
      (renamed perf-WCET test, dropped `cfgs`/`tl_grid_upper` from call sites).
- [x] Verify: `pytest Gen_Taskset/tests/` = 47 passed. Diagnostic et_mean rule
      → N=4 0.883 / N=8 0.588 / N=16 0.923 mean max-core util (was 2.197/3.008).
      Faithful gate `measure_gate_rejection_rate --samples 2 --ns 4 8 16` →
      N=4/8 attempt 1, N=16 ≤3 attempts, 0 rejections/0 raises.
- [x] **COMMITTED** as `7c8748c0` "update some configs to generate schedulable
      task sets" (12 files, +864/−81). Criterion relaxed ≤3 → budget-20
      (user: "keep 20 in exp").

## 3. Rejection-rate reporting + second-lever decision
- [x] Log the rejection rate (attempts per accepted taskset, by N) across a smoke
      generation sweep. (`measure_gate_rejection_rate --samples 5 --ns 4 8 16
      --n_sec 100`: 15 independent draws, base seeds 1000–1400. Result:
      N=4/8 100% first-try, N=16 80% first-try (max 3 attempts); **0 rejections,
      0 raises**. Report saved at
      `simulation_experiments/important_task_gate_rejection/ns4-8-16_s5_dur100/
      gate_rejection_rate.json`.)
- [x] If rejection rate excessive → propose a generator-logic fix (second lever:
      tighten ET/period draw / enforce `deadline > WCET` for important tasks) to the
      user. Do NOT implement without user go. **NOT NEEDED — rejection rate = 0**
      across N=4/8/16; the config-tuning round (`7c8748c0`) already brought
      generation within budget. Second lever not proposed.

## 4. Verification + records
- [x] `pytest Gen_Taskset/tests/` green (48 passed — full generator test suite;
      was 47 before the Step 2 e2e test).
- [x] Confirm the gate covers the important-task types the C++ scorer will actually
      see (cross-check a generated taskset's important tasks against
      `feasibility_clamp.py`'s perf-task skip — the gap this task closes).
      [perf → `period * FINAL_Et_OVER_PERIOD_RANGE[1]`; non-perf → global-max
      `execution_time_max`; both in `_wcets_from_loaded_tasks`.]
- [x] Confirm the WCET fields match P0.6's (global-max ET per task — D2 consistency).
      [`_load_emitted_tasks_by_gid` keeps the worst-case-ET representative per gid.]
- [x] `dev_log.md` (this folder + top-level) + memory updated.
- [x] `git add` staged; user reviewed + COMMITTED. Gate wrapper = `3d2360ed`;
      config-tuning round = `7c8748c0` (12 files: `orchestrator.py`,
      `important_task_rta.py`, `taskset_generator.py`,
      `taskset_cfg_paper_base.json`, `taskset_cfg_paper_16.json`,
      `test_important_task_gate.py`, `measure_gate_rejection_rate.py`,
      `measure_important_utilization.py`, this folder's `dev_log.md`/`tasks.md`,
      top-level `dev_log.md`, `.gitignore`).
- [x] **Step 2 prod-wiring + e2e test + Step 3 measurement LANDED (this session,
      git add-only — awaits user commit):** `run_generator.py` +
      `run_sim_experiments.py` (gate default-ON + `--no-important_tasks_schedulability_check`
      opt-out + seed-collision fix) + `test_integration.py` e2e test +
      rejection-rate report. `pytest Gen_Taskset/tests/` = 48 passed; gate smoke
      = certified attempts_used=1.
- [x] **Step 3 DONE:** rejection-rate = 0 across N=4/8/16 (15 draws); second lever
      NOT needed.
