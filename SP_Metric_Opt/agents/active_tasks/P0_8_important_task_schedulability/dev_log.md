# P0.8 Important-Task Schedulability — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-29 (Step 2 prod-wiring + Step 3 rejection-rate — FINISH P0.8)

- **User go: "finish p0.8, fully finish it."** This lands the two remaining
  DEFERRED items, making the gate's guarantee *enforced* in production (not just
  existing + unit-tested). git add-only (user commits).
- **Step 2 — prod wiring (LANDED):**
  - **`Gen_Taskset/executable/run_generator.py`:** the generation branch now
    routes through `run_full_generation_pipeline_with_important_task_gate` BY
    DEFAULT (the canonical CLI entry point). The gate forwards the same kwargs
    as the plain shell (its body IS `_run_pipeline_with_cfgs`, the same body the
    shell calls) and surfaces `attempts_used` so a multi-attempt emit is visible,
    not silent. New `--important_tasks_schedulability_check` flag (default ON =
    gate runs; `--no-important_tasks_schedulability_check` opts out) replaces
    the old `--no_important_task_gate` negative-phase opt-out.
    ON) for diagnostics / non-paper configs where an uncertified emit is
    acceptable. The `--gen_path_for_taskset` branch is untouched (it only
    APPENDS traces to an existing taskset; the gate is a generation-time
    guarantee, not a trace-append guarantee).
  - **`simulation_experiments/run_sim_experiments.py`:** both generation call
    sites (verbose + quiet) now go through a single `_generate_taskset` helper
    that picks gate-vs-plain by `--important_tasks_schedulability_check`
    (same default ON).
    **Seed-collision fix:** the loop previously set
    `RANDOM_SEED = base_seed + idx` (step 1); the gate's internal `+0..19`
    retry window for taskset *k* would then collide with taskset *k+1*'s
    starting draw (retry attempt *a* on taskset *k* == taskset *k+a*'s draw →
    byte-identical tasksets across the batch — silent duplicates). Widened the
    step to `IMPORTANT_TASK_GATE_MAX_ATTEMPTS` (20) when the gate is ON; kept the
    legacy +1 when OFF (bit-identical to pre-gate behavior). The base seed
    remains a sound staleness-baseline key for `_should_generate` (the gate is
    deterministic given a base seed).
  - **Smoke:** `run_generator.py` on `taskset_cfg_paper_4.json` (gated) →
    `[P0.8 gate] ... certified schedulable ... (attempts_used=1)`, exit 0.
    Opt-out (`--no-important_tasks_schedulability_check`) → plain pipeline, no gate message,
    exit 0. Both CLIs expose the flag.
- **Step 2 — end-to-end REAL-pipeline gate test (LANDED, in
  `test_integration.py`):** `test_gate_end_to_end_real_pipeline_certifies_
  schedulable_taskset` runs the REAL pipeline (no monkeypatch on
  `_run_pipeline_with_cfgs`) through the gate on `test_standard_4.json`
  (n_sec=100, n_sec-stable verdict) and asserts (1) the gate returns
  `schedulable=True` with `attempts_used >= 1`, `culprits == []`, AND (2) the
  certificate is GENUINE — independently re-loading the emitted tasks
  (`_load_emitted_tasks_by_gid`), deriving WCETs (`_wcets_from_loaded_tasks`),
  and re-running `important_tasks_schedulable` reproduces `ok=True`. Closes the
  gap the mock-based gate tests (`test_important_task_gate.py`) leave open: a
  hollow gate (e.g. the `important`→`is_important` normalization regression, or
  a WCET rule reading the wrong field) would pass the mock tests but fail here,
  because the emitted YAMLs are whatever the real generator produced. Probed:
  2 of 4 tasks important (top-50% by sp_weight, as designed), real WCETs (e.g.
  task 3 = 195ms < 1000ms period), independent re-check `ok=True`. The test
  explicitly asserts `n_important >= 1` so a vacuous RTA (no important tasks →
  trivially "schedulable") fails loudly.
- **Step 3 — rejection-rate measurement (LANDED, measurement not code):** ran
  `measure_gate_rejection_rate --samples 5 --ns 4 8 16 --n_sec 100` (15
  independent draws, base seeds 1000–1400):

  | N | samples | passed | raised | 1st-try % | reject % | mean att | max att |
  |---|---------|--------|--------|-----------|----------|----------|---------|
  | 4 | 5 | 5 | 0 | 100% | 0% | 1.0 | 1 |
  | 8 | 5 | 5 | 0 | 100% | 0% | 1.0 | 1 |
  | 16| 5 | 5 | 0 | 80% | 0% | 1.4 | 3 |

  **0 rejections, 0 raises, max 3 attempts** — well within budget-20. Reproduces
  + extends the config-tuning round's verdict with independent seeds. The second
  lever (generator-logic fix) is NOT needed (rejection rate = 0). Report saved:
  `simulation_experiments/important_task_gate_rejection/ns4-8-16_s5_dur100/
  gate_rejection_rate.json`.
- **Verification:** `pytest Gen_Taskset/tests/` = **48 passed** (was 47; +1 e2e
  gate test). The one transient `test_integration_pipeline` failure on the first
  run is the PRE-EXISTING flaky test-ordering issue (documented in the
  config-tuning entry + memory): `assert 1000 <= 50` from random/tmp_path state
  leakage between tests; passes in isolation + on clean re-run — NOT this change.
- **Files edited (working tree, git add-only — awaits user commit):**
  `Gen_Taskset/executable/run_generator.py`,
  `simulation_experiments/run_sim_experiments.py`,
  `Gen_Taskset/tests/test_integration.py`, this folder's `dev_log.md`/`tasks.md`/
  `goal.md`, top-level `agents/dev_log.md`. (The Step 3 JSON report is
  untracked output, not source.)
- **Status:** P0.8 now FULLY DONE — Steps 1 + 2a + 2b + config-tuning (committed
  `3d2360ed` + `7c8748c0`) + Step 2 prod-wiring + e2e test + Step 3 measurement
  (this entry). The gate is enforced in production (default ON) with an opt-out.
  Closable as "shipped" once the user commits this batch.

## 2026-07-29 (config-tuning round — make the gate pass within budget on the REAL paper config)

- **Problem (active task #4, prior session):** the gate as committed in
  `3d2360ed` is a correct loud-raise certifier, but the REAL paper config
  (`taskset_cfg_paper_base.json`) made it reject too often under the faithful WCET
  rule — i.e. the gate would burn its 20-attempt budget and raise on real
  tasksets. Root cause is NOT the gate (the gate stays a loud-raise certifier);
  it is the generation config + the perf-WCET rule. Three-layer diagnosis:
  - **A. Perf WCET was `period * FINAL_Et_OVER_PERIOD_RANGE[1]` (= period×0.9, the
    TL-grid MAX).** Probe (`measure_important_utilization.py`) of important-perf
    max-core WCET-util showed overload at N=4/8/16 under this rule.
  - **B. Non-perf WCET = `et_mean + 2σ`** with `SIGMA_OVER_Et_RANGE=[0.5,0.6]` → a
    2.0–2.2× amplifier on env/util — the dominant overload driver.
  - **C. Env cap `MAX_UTIL_PER_ENV_TASK=0.45`** landed env WCET/period at ~0.99
    (AT the line). (The prior session's "uunifast silent cap-raise" attribution
    was a CORRECTION: in the real paper config `total/n ≤ 0.75` → the cap-raise
    never fires; overload is B+A+env-cap, not the cap-raise.)
- **RESOLVED + IMPLEMENTED + COMMITTED `7c8748c0` ("update some configs to
  generate schedulable task sets", 12 files +864/−81):** all three proposals landed.
  1. **Perf WCET = `execution_time_mu` (= et_mean), NOT the middle-TL option and
     NOT the grid MAX.** User reasoning: the C++ sim runs
     `execution_time = min(et_mean, assigned_TL)` — the TL is a DOWNWARD cap,
     never an inflator → `runtime_ET ≤ et_mean ≤ WCET` for ANY operating point →
     et_mean is the faithful + sound operating-point WCET (tightest safe rule).
     The min-TL clamp (the seed P0.6 walks from) discharges the et_mean concern at
     that operating point. Code: `important_task_rta.py`
     `_wcets_from_loaded_tasks(tasks_by_gid)` — DROPPED the `tl_grid_upper` param;
     perf WCET = `float(task["execution_time_mu"])` with a loud `KeyError` on
     missing (no fallback — `execution_time_mu` is guaranteed on emitted perf
     tasks; missing = hand-authored/legacy misconfig). `compute_wcets_from_characteristics
     (dir_path)` — DROPPED the `cfgs` param + grid derivation (the TL grid is now
     IRRELEVANT to the verdict; stays `[0.05,0.9]` in config). Gate call site
     (`orchestrator.py`) updated to match.
  2. **Env cap 0.45→0.27 + variance [0.5,0.6]→[0.3,0.4]** (`taskset_cfg_paper_base.json`).
     Env WCET/period = `u_env·(1+2σ/et)`; σ/et∈[0.3,0.4] → amplifier 1.6–1.8;
     u_env=0.27 → env WCET/period ≤ 0.486 < period (was ~0.99, the overload driver).
  3. **(3a) no-inflation — shortfall DROPPED, NOT redistributed.**
     `CPU_UTIL_RANDOM_RANGE` [0.5,1.5]→[0.5,1.0] (lower target). In
     `taskset_generator.py` the env-cap block KEEPS the env `u_i` clamp-down but
     DROPS the proportional redistribution block (it raised non-env `u_i` above
     drawn = the inflation 3a forbids; AND it inflated PERF `u_i` → inflated
     `execution_time_mu` = perf WCET → spurious rejections). Dropping only LOWERS
     realized load (gate never penalizes lower load → strictly safe). Cosmetic
     cpu_util over-report left as-is with a comment (the gate does not read
     `cpu_util`; the one contract consumer uses a config without
     `MAX_UTIL_PER_ENV_TASK` so the assertion holds). (3b) normal-only
     redistribution NOT chosen (3a only, user direction).
  4. **`DEADLINE_MODE=implicit`** added to `taskset_cfg_paper_base.json`
     (deadline=period → makes RM≡DM; purely config-read at
     `taskset_generator.py:546`).
- **Diagnostic sync:** `measure_important_utilization.py` synced — `_wcet_of`
  dropped `tl_grid_upper`, perf reads `Et_mean` in-memory (= `execution_time_mu`
  emitted); removed the misleading `--perf-cap` CLI flag (perf WCET is now et_mean
  not the grid).
- **Test sync:** `test_important_task_gate.py` — `_emitted_task` +`execution_time_mu`;
  renamed `test_wcet_perf_task_is_tl_grid_upper_bound`→`test_wcet_perf_task_is_et_mean`;
  updated `test_wcet_mixed_perf_and_non_perf`; dropped `cfgs` from all 7 WCET call
  sites.
- **Verification (re-confirmed this session):**
  - `pytest Gen_Taskset/tests/` = **47 passed** (no regressions).
  - Diagnostic `measure_important_utilization --ns 4 8 16 --seeds 5` (et_mean rule):
    mean max-core util N=4 0.883 / N=8 0.588 / N=16 0.923 (was N=8 2.197 / N=16
    3.008 under the old period×0.9 rule); N=4/8 fully cleared, N=16 1/5 seeds
    util-overloads (1.506, all non-perf on one core).
  - **Faithful gate verification (REAL gate + REAL RTA, this session):**
    `measure_gate_rejection_rate --samples 2 --ns 4 8 16 --n_sec 100` →
    N=4/8 pass on attempt 1; N=16 passes within 3 attempts; **0 rejections, 0
    raises.** Beats even the original ≤3-retry criterion.
- **Files edited (COMMITTED `7c8748c0`, 12 files +864/−81):** `important_task_rta.py`,
  `orchestrator.py`, `taskset_generator.py`, `taskset_cfg_paper_base.json`,
  `taskset_cfg_paper_16.json`, `measure_important_utilization.py`,
  `measure_gate_rejection_rate.py`, `test_important_task_gate.py`, this folder's
  `dev_log.md`/`tasks.md`, top-level `dev_log.md`, `.gitignore`. Criterion
  RELAXED from ≤3 to budget-20 (user: "10 is okay, keep 20 in exp").
- **Status:** config-tuning round COMMITTED `7c8748c0`. Step 2 prod-wiring (gate
  into `run_generator.py`/`run_sim_experiments.py`) + Step 3 rejection-rate
  reporting remain DEFERRED (user-go).

## 2026-07-29 (Step 2b refactor — kill triplicated path/config scaffolding + fix dir_path=None regression)

- **User review caught duplication + a regression it caused.** The split of
  `run_full_generation_pipeline` into shell + `_run_pipeline_with_cfgs` had
  triplicated the same scaffolding across the shell, the body, and the gate:
  `OPT_SP_PROJECT_PATH = os.path.dirname(...)` (×3), config-path resolution
  (relative→absolute + `FileNotFoundError`, ×2), `load_generation_config` +
  `validate_trajectory_config` (×2), and `dir_path` None-default /
  relative→absolute + `makedirs` (×2). The duplication also caused a **behavior
  regression**: the original `run_full_generation_pipeline` resolved
  `dir_path=None` → `TaskData/<cfg>_gen_1` inside the body; after the split
  that resolution lived only in the gate, so the shell forwarded `None` to
  `_run_pipeline_with_cfgs`, which raises `ValueError`. The canonical CLI
  (`run_generator.py --cfg_file X` with no `--dir_path`) passes `dir_path=None`
  → it would have broken. (No test exercised the `None`-default path — all
  ~22 callers pass `dir_path=` explicitly, which is why the suite stayed green
  and the regression hid.)
- **Fix = extract three shared helpers** (single source of truth, used by all
  three entry points): `_resolve_config_path(cfg_file) -> str`,
  `_load_and_validate_cfgs(cfg_file) -> dict`, `_resolve_dir_path(dir_path,
  cfg_file) -> str` (does the None-default + relative→absolute + `makedirs`).
  Hoisted `OPT_SP_PROJECT_PATH` to a module constant. The shell now calls
  `_resolve_dir_path` before the body → `dir_path=None` is resolved to the
  `TaskData/<cfg>_gen_1` default again (regression fixed). The body keeps its
  `dir_path is None` → raise guard (a caller that forgets the helper fails
  loudly, not silently) and a relative-path safety net. The gate calls the same
  three helpers, so config + dir resolution can no longer diverge between the
  shell and the gate.
- **Verify:** `pytest Gen_Taskset/tests/` = 47 passed (no regressions).
  Regression fix confirmed end-to-end: `run_full_generation_pipeline(...,
  dir_path=None)` on `test_standard_4.json` now completes (auto-resolves to
  `TaskData/test_standard_4_gen_1`) instead of raising — matches the original
  behavior.
- Files staged (same 3 code files as Step 2b, updated): `orchestrator.py`,
  `important_task_rta.py` (unchanged this pass), `test_important_task_gate.py`
  (unchanged this pass).

## 2026-07-28 (Step 2b — gate wrapper + seed-advancing retry loop LANDED)

- **Step 2b LANDED + staged (NOT committed, awaits user review):** the generation-time
  gate `run_full_generation_pipeline_with_important_task_gate` in `orchestrator.py`.
  Wraps the canonical pipeline with a seed-advancing retry loop: each attempt advances
  `cfgs["RANDOM_SEED"] = base_seed + attempt`, runs the pipeline body, derives WCETs
  (D2), runs `important_tasks_schedulable` (per-core DM-within-important RTA, D3)
  post-clamp (D5), and on PASS returns a report; on exhaustion (budget 20, D4) raises
  `RuntimeError` loudly with the final culprits + attempt count + seed range — NEVER
  silently emits an unschedulable taskset (the exact P1.8 substrate this prevents).
- **Design decision (user, 2026-07-28 — "make a choice based on your judgement"):**
  the seed-advancement blocker was that `run_full_generation_pipeline` reloads cfgs
  from the config FILE every call (`load_generation_config` at the top of the shell),
  so mutating an in-memory `cfgs["RANDOM_SEED"]` between retries would be discarded
  → byte-identical taskset every retry (the no-op-retry bug). Resolved by SPLITTING
  the function (no default args — user explicitly dislikes default args making
  failures harder to capture): `run_full_generation_pipeline` is now a thin shell
  (resolves+loads cfgs, validates, delegates); the body is `_run_pipeline_with_cfgs(cfgs, ...)`
  taking a REQUIRED pre-loaded cfgs. The ~8 existing shell callers are untouched. The
  gate loads cfgs ONCE and calls the body directly, so the advanced seed takes effect.
- **Test-fixture strategy (user, 2026-07-28):** mock `_run_pipeline_with_cfgs`
  (monkeypatch) writing synthetic characteristics YAMLs (unschedulable attempt 0 /
  schedulable attempt 1 / never-schedulable for exhaustion); the REAL
  `compute_wcets_from_characteristics` + `important_tasks_schedulable` run on the
  synthetic YAMLs (RTA path exercised, expensive trace generation NOT). 4 TDD red→green
  tests: `test_gate_passes_first_try`, `test_gate_retries_until_schedulable`,
  `test_gate_raises_on_budget_exhaustion`, `test_gate_advances_seed_so_retries_re_sample`.
- **DRY refactor (folded into the green step):** extracted `_load_emitted_tasks_by_gid(dir_path) -> {gid: task}`
  (I/O + gid dedup, worst-case-ET representative) + `_wcets_from_loaded_tasks(tasks_by_gid, tl_grid_upper) -> {gid: wcet}`
  (pure WCET rule) in `important_task_rta.py`; `compute_wcets_from_characteristics` is
  now a thin composition of the two. Public API + the 7 existing WCET tests unchanged.
  The wrapper uses the same reader + WCET fn — single disk read, no duplicated logic.
- **Key-normalization correctness fix (caught by design review before first run):** the
  emitted C++-format YAML uses key `important` (`yaml_exporter.py:73`) but the RTA reads
  `is_important` (`important_tasks_schedulable` → `t.get("is_important")`). The reader
  normalizes `task["is_important"] = task.get("important", False)` on every loaded task
  — without this the gate would feed emitted-form tasks to the RTA, which sees NO
  important tasks → vacuously "schedulable" → a hollow gate (the exact silent hole P0.8
  exists to prevent).
- **Verification:** `pytest Gen_Taskset/tests/test_important_task_gate.py` = 11 passed
  (7 WCET + 4 wrapper); `pytest Gen_Taskset/tests/` = 47 passed (was 43; +4, no
  regressions). 3 files staged: `orchestrator.py`, `important_task_rta.py`,
  `test_important_task_gate.py`.
- **DEFERRED (Step 2 — wiring):** end-to-end real-pipeline test (`test_integration.py`)
  + wiring the wrapper into prod entry points (`run_generator.py`,
  `run_sim_experiments.py`) — behavior change for the prod pipeline, separate
  user-reviewed step. The wrapper existing + unit-tested is the unit of value (Step 2b).
- **DEFERRED (Step 3):** rejection-rate reporting + second-lever generator-logic fix
  proposal (user-go only).

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
