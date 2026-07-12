# P1.8 — Tasks (working checklist)

> See `goal.md` for the anomaly, the data, the leading hypothesis, and the open
> design decisions. **FILED 2026-07-12 (task + read-only investigation
> STARTED; NO implementation)** per the user's two-part directive ("first add
> the task, then share investigation while working on finding reasons and root
> causes"). Steps below are the execution plan; investigation steps are
> already partly underway (see `dev_log.md`). One review-and-commit cycle per
> `agent_coding_rules.md`. TDD: tests first when the fix is greenlit.

## Step 0 — Confirm the root cause (IN PROGRESS — read-only)

- [x] Aggregate the per-taskset SP; isolate that taskset_1 dominates the
      flip (DONE 2026-07-12 — Finding 1 in `goal.md`).
- [x] Confirm the failure is analytic-SP, not actual-schedule (zero actual
      misses on taskset_1 under both arms) (DONE — Finding 2).
- [x] Identify the unschedulable tasks (WCET > deadline) in taskset_1
      (DONE — Finding 3: task 0 WCET 27.22 > DDL 18; task 2 WCET 21.40 > DDL 14).
- [x] **Structural: which tasks have TL freedom in taskset_1?** (DONE — Finding
      4: only task 3 has `timePerformancePairs`; tasks 0/1/2 are `{-1}`-only
      under BOTH arms. Refutes H1a-as-stated; the SP→0 is RTA over the raw ET
      dist, not an adopted TL ≥ deadline.)
- [x] **Evaluate the user's pessimistic-RTA reasoning** (DONE — Finding 5 +
      verdict: mechanism correct, wrong sign for the observed flip on taskset_1.
      The "adopt tighter TL" channel can only fire on task 3 and pushes INCR_WCET
      slightly *down*; it cannot explain INCR_WCET >> INCR. The WCET collapse is
      also value-copy-confined to the optimizer's internal search (does NOT reach
      the scored SP). Re-pointed root cause to the PRIORITY ASSIGNMENT (H1c).)
- [x] **Confirm whether `FiniteDist` truncates the Gaussian at max_et**
      (RESOLVED — `Probability.cpp:18-43`: YES, truncated; upper-tail mass dumped
      onto the `max_val` bin. So 5b is FALSE — the Gaussian tail does NOT exceed
      the WCET point mass; both arms' task-1 dist tops out at the same max_et.)
- [x] **Generator feasibility sweep: is WCET>DDL / mu>period a one-off or
      systemic?** (DONE 2026-07-12 — Finding 6: SYSTEMIC. 9/10 tasksets have ≥1
      WCET>DDL task; 2/10 have mu>period (the user's flag); 5/10 have a
      deterministic certain-miss task (mu>DDL, σ=1.0). Strongly supports H1d.
      Necessary-but-not-sufficient: WCET>DDL endemic yet only taskset_1 collapses
      → H1d is the substrate, H1c is the collapse mechanism. Cross-recorded to
      P1.7's "D1 separate pass" follow-up note.)
- [ ] **Trace the generator source: does it enforce ANY feasibility constraint?
      Where does `per_core_cpu_util` calibrate ET?** Confirm in source (Python
      generator under `simulation_experiments/` and/or C++ `RegularTasks.cpp`)
      that no bound on mu-vs-period or WCET-vs-deadline is enforced, and locate
      the ET/period/deadline assignment + the `per_core_cpu_util` calibration
      math. READ-ONLY. (Explore agent dispatched 2026-07-12; citation pending.)
- [x] **Does the WCET collapse reach the scored SP, or only the search?**
      (RESOLVED — ONLY the search. `Optimize_w_TL_ScratchOrIncre` takes
      `const DAG_Model&` and value-copies into `dag_tasks_`; the collapse mutates
      the copy. The scored SP at `:553` reads the orchestrator's uncollapsed
      `dag_tasks`. `ApplyTaskConfigurations` sets only `priority`+`setExecutionTime`,
      NOT `execution_time_dist`. Scored ET dists are IDENTICAL across arms.)
- [ ] **Dump the adopted `priority_vec` (PA) for INCR vs INCR_WCET on taskset_1,
      interval-by-interval.** The PA is now the prime differentiator (ET dists
      identical; TL differs only on task 3). Confirm INCR commits to a PA that
      worsens RTA interference on the WCET>deadline tasks 0/2. Instrument
      `DeterminePrioritiesAndBudgets`/`ApplyTaskConfigurations` or a debug print
      of `res.priority_vec` + `res.id2time_limit` in `SimulateInterval` before
      the `:553` score. READ-ONLY (debug print, no optimizer logic change).
- [ ] **Dump per-task `rtas[i]` (response-time dists) for INCR vs INCR_WCET on
      taskset_1.** Confirms which task(s) flip from schedulable to certain-miss
      under INCR's PA. Instrument `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`) or
      print inside `ObtainSP_TaskSet`. READ-ONLY.
- [ ] **Dump INCR's adopted TLs + priorities on taskset_1, interval-by-
      interval.** Confirm whether INCR adopts TL ≥ deadline for task 0 / task 2
      (→ the SP→0 mechanism). The adopted config is in `res_opt_` /
      `ResourceOptResult`; instrument `DeterminePrioritiesAndBudgets` or read
      the per-interval YAML if exported. (NO code change to the optimizer —
      debug print or post-hoc read of exported artifacts.)
- [ ] **Dump the `timePerformancePairs` TL grid INCR sees vs the grid
      INCR_WCET sees** for the unschedulable tasks. Confirms/denies H1a (the
      WCET ablation changes the enumerated TL options → offers a TL < deadline
      INCR lacks, or vice versa).
- [ ] **Compute by hand** `ObtainSP_TaskSet_And_TimeLimits` for taskset_1
      under (i) INCR's adopted TLs and (ii) a TL < deadline for the
      unschedulable task. Confirms whether a TL < deadline would have rescued
      INCR's SP (→ H1a/H1b: INCR failed to find it) or whether the taskset is
      unschedulable even with the best TL (→ H1d: generator defect, INCR's ≈0
      is honest).
- [ ] **Disambiguate H1a vs H1b vs H1c vs H1d** with the above evidence.
      Record the verdict in `dev_log.md` + `goal.md`'s hypothesis section.
- [ ] Check whether Q3's verdict actually moved on this N=4 run (Q3 is scoped
      to large N=8/10; this run is N=4 — the gate may not be at its
      verdict-moving sizing). Determines urgency.

## Step 1 — Settle design decisions with the user (RESOLVED 2026-07-12 — see Step 1b)

- [x] D1 — Is WCET > deadline a generator bug (→ F1) or intentional stress
      (→ F2/F4)? **RESOLVED: generator bug (systemic, 9/10 tasksets; zero
      guard in source). Fix = F1.** User greenlit F1.
- [x] D3 — Is INCR's ≈0 the bug (→ F2/F3) or is INCR_WCET's 0.383 the bug
      (→ F4)? **RESOLVED: INCR's ≈0 is HONEST** (scored ET support genuinely
      crosses the deadline via `CompressDeadlineMissProbability`); F1 removes
      the substrate.
- [~] D2 — Forbid TL ≥ deadline in the optimizer (F2)? **MOOTED:** F1 removes
      the unschedulable substrate; the optimizer never faces a TL≥DDL dilemma
      on a clean taskset.
- [ ] D4 — Re-run scope: full eval suite, or N=4 / taskset_1 probe first?
      (User: "i'll do it myself" — user runs the A/B.)

## Step 1b — F1 design: post-generation clamp pass (USER GREENLIT 2026-07-12)

> **Supersedes the earlier ≤5-retry regenerate-loop draft.** User's simpler
> design (verbatim): "after task set and all intervals' yaml files are
> generated, we go through all of them once. for each task whose avg ET
> exceeds 0.95*period, we just clamp avg ET back to 0.95*period. if that
> happens, we also re-generate deadline as period. otherwise, nothing
> changes." See `dev_log.md` 2026-07-12 (cont'd) "user's SIMPLER design" for
> the full evaluation + the min/max/perf-task mechanical findings.

- [x] Evaluate the design (makes sense? mechanical gaps?). **DONE** — sound in
      goal; one gap closed: clamping `mu` alone is a no-op on the scored SP
      because `FiniteDist` truncates the Gaussian at `execution_time_max`
      (`Probability.cpp:18-43`), so the scored support bound is `max`, not
      `mu`. → the clamp must also pull `execution_time_max` (and `min` if it
      exceeds the clamped max) down to `0.95*period` on the same task. Perf
      tasks (`performance_records_time` present) are SKIPPED — their
      min/max are TL-grid bounds, not ET-support bounds.
- [x] Record design into tasks list. **DONE** (this section + `dev_log.md`).

### Step 1b-TDD — red test first

- [x] New `Gen_Taskset/tests/test_feasibility_clamp.py`: synthetic
      characteristics YAML with (a) a deterministic task `mu=21.4, min=max=21.4,
      period=20, deadline=14` (clamp fires → `mu==max==19.0`, `deadline==20`),
      (b) a perf task `mu=518, min=25, max=450, period=500` (SKIP — unchanged),
      (c) a clean task `mu=10, period=100` (untouched). RED before green.
      **DONE 2026-07-12** — 7 tests (deterministic-fire / perf-skip /
      clean-untouched / mixed / never-raise-max / walks-all-files+skips-param /
      preserves-perf-records-string). RED confirmed (ModuleNotFoundError) before
      impl; GREEN after.
- [x] Regression assertion in `test_integration.py` (or the new test): every
      non-perf task in the pipeline output has `execution_time_mu <=
      0.95*period`. **DONE 2026-07-12** — added the mu AND max invariant
      assertion to `test_integration_pipeline` (gated on non-perf); passes
      against a real pipeline run.

### Step 1b-impl — the clamp pass

- [x] New module `Gen_Taskset/lib/feasibility_clamp.py`:
      `clamp_avg_et_to_period(yaml_dir, et_over_period_cap=0.95)` — walks every
      `taskset_characteristics*.yaml` in `yaml_dir`; for each non-perf task
      with `execution_time_mu > cap*period`: clamp `mu` → `cap*period`,
      `execution_time_max` → `min(max, cap*period)`, `execution_time_min` →
      `min(min, execution_time_max)`, `deadline` → `period`. Rewrite with
      `SpaceSeparatedListDumper` (byte-compatible). Perf tasks skipped.
      **DONE 2026-07-12.**
- [x] Wire into `orchestrator.run_full_generation_pipeline` AFTER
      `generate_additional_execution_traces`, BEFORE return. **DONE 2026-07-12**
      — step 4 in `run_full_generation_pipeline`.
- [x] In-source comment explaining the `max`/`min` clamp necessity (the
      `FiniteDist`-truncates-at-`max` trace) so the next reader doesn't
      "simplify" it back to mu-only. **DONE 2026-07-12** — module docstring
      "WHY mu ALONE IS NOT ENOUGH" section.

### Step 1b-verify

- [x] `pytest Gen_Taskset/tests/` green (new test + no regressions).
      **DONE 2026-07-12** — 21/21 Gen_Taskset tests green; 291/291
      `tests/python/` green (no downstream regressions).
- [x] Re-scan the N=4 run's clamp-firing count: confirm only taskset_1 task 2
      changes (1 task) in the regenerated output. **DONE 2026-07-12** —
      read-only scan of all 10 tasksets' interval_0: exactly 1 task fires
      (taskset_1 gid=2 task_3, `mu=21.4 > 0.95*20=19.0`, `min==max==mu`).
      Clamp-on-copy of the real taskset_1 interval_0 confirms post-state:
      gid=2 `mu=min=max=19.0, deadline=20`; gid=0/1 untouched (mu under cap);
      gid=3 skipped (perf). Matches the design's surgical prediction.
- [x] `dev_log.md` + `MEMORY.md`: record the clamp design + the
      mu-alone-is-a-no-op finding. **DONE 2026-07-12** (this update +
      `dev_log.md` milestone + `MEMORY.md` pointer + memory file).

## Step 2 — TDD: test reproducing the collapse (NOT STARTED; after greenlit fix)

- [ ] A test (in `tests/testScheduleSimulate.cpp` or a new
      `testIncrVsIncrWcet.cpp`) that constructs a taskset with a WCET > deadline
      task, runs INCR, and asserts INCR's SP is NOT driven to ~0 / INCR ≥
      INCR_WCET. Red before green (reproduces the taskset_1 collapse on the
      current code).
- [ ] If F1 (generator): test that the generator rejects/clamps WCET > deadline.
- [ ] If F2 (optimizer TL constraint): test that the optimizer never adopts TL
      ≥ deadline for any task.
- [ ] If F4 (scoring honesty): test that INCR_WCET reports the same
      infeasibility as INCR on the unschedulable taskset.

## Step 3 — Implement the chosen fix (NOT STARTED; after Step 1 + 2)

- [ ] F1 (generator): `sources/` generator path (`RegularTasks.cpp` /
      `per_core_cpu_util` calibration) — clamp ET_max ≤ deadline or reject +
      regenerate.
- [ ] F2 (optimizer): `sources/Optimization/OptimizeSP_TL_Incre.cpp` — hard
      constraint in `PerformCoordinateDescentForTaskConfigOpt` /
      `timePerformancePairs` enumeration: never adopt TL ≥ deadline.
- [ ] F3 (incumbent escape): if H1c confirmed — the P1.2-class fix; coordinate
      with P1.2.
- [ ] F4 (scoring): `sources/Safety_Performance_Metric/SP_Metric.cpp` or
      `ApplyWCETAblationIfRequired` — make the WCET ablation not mask
      infeasibility.
- [ ] Whichever fix: in-source comment; update the Q3 gate fixtures in
      `tests/python/test_evaluation_suite.py` if the expected SP values shift.

## Step 4 — Verify + re-run (NOT STARTED)

- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) — 16/16 ctest
      green; `tests/python/` green.
- [ ] User re-runs the A/B (per D4 scope). Confirm INCR ≥ INCR_WCET on
      taskset_1 + the aggregate; Q3 verdict checked.
- [ ] Record the before/after in `dev_log.md`.

## Step 5 — Docs (NOT STARTED)

- [ ] `agents/overall_tasks.md` — mark P1.8 row DONE with the root-cause +
      fix verdict.
- [ ] Top-level `agents/dev_log.md` — append the P1.8 milestone.
- [ ] Memory: add a P1.8 entry (root cause + fix); `MEMORY.md` pointer.
- [ ] `git add` the P1.8 unit; hand to user for review (no commit).

## Standing constraints

- **No implementation until the root cause is confirmed (Step 0) AND the user
  greenlights a fix (Step 1).** The 2026-07-12 work = task filing + read-only
  investigation only.
- No `git commit` (user's task; `git add` only).
- No running the A/B myself (user runs `run_end_to_end.sh`).
- Do NOT decide D1–D4 unilaterally — settle with the user (per
  `agent_coding_rules.md`).
- Don't conflate with P1.1/P1.2 — P1.8 may share a fix (if H1c), but it asks a
  distinct question (WCET-ablation masking an INCR failure on an unschedulable
  taskset). Confirm overlap before merging.
