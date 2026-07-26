# P2.13 — Tasks (working checklist)

> See `goal.md` for the reframed trigger (actual DDL-miss vs configured SP
> threshold), the code grounding, the threshold-knob findings, and open
> decisions D1–D5.
> **Reframed 2026-07-25.** Two deliverables: (1) config-value edit to the
> SP-threshold knobs; (2) empirical analysis (analytic miss-prob vs threshold,
> mean relative difference, reported number). **No implementation until the user
> greenlights; this is the plan.**

## Step 0 — Settle decisions (NOT STARTED)

- [ ] **D1 — Feasibility of important-task DDLs (perf-task clamp gap).** Check
      `deadline` vs `execution_time_max` (WCET) for the top-weight task in each
      of the 5 tasksets. P1.8's `feasibility_clamp.py` skips perf-record tasks;
      the important tasks in the N=10 run ARE perf tasks → may have
      `ddl < wcet` → analytic miss-prob ≈ 1.0 → penalty branch in both arms →
      flat SP is *correct* (generator owns it). Read-only on existing run.
- [ ] **D5 — How to get per-task analytic `ddl_miss_chance`?** Pick (a) C++
      logging point (faithful, needs rebuild+rerun) or (b) Python re-impl of
      `GetDDL_MissProbability` over the RTA dist (no rebuild, binning-divergence
      risk). **Recommendation: (b) first for a read-only pass, (a) if borderline.**
- [ ] **D2 — Tie-stable "important" definition?** Less central under the
      reframing (analytic check takes the top-weight task per taskset regardless
      of tie). Defer.
- [ ] **D3 — Metric shape?** Only if analysis shows analytic miss-prob >
      threshold yet SP ≈ 1. File as new task; NOT here.
- [ ] **D4 — P2.6 cross-link.** Defer sim-RT cross-check to after P2.6.

## Step 1 — Config edit: SP-threshold knobs (NOT STARTED; on user greenlight)

> Pure config-value change. No code change. NOT committed (`git add` on user go).
> Affects `taskset_cfg_paper_base.json` (the template all 4 paper_* configs
> INCLUDE) + the `suggest` defaults in `taskset_generator.py:33-34`.

- [ ] `SP_THRESHOLD_RANGE`: `[0.5, 0.9]` → `[0.001, 0.9]` in
      `Gen_Taskset/task_sets_config/templates/taskset_cfg_paper_base.json`.
- [ ] `SP_THRESHOLDS_SET`: scrub `1.0` → `[0.2, 0.4, 0.6, 0.8]` (or add a sub-0.1
      strict value like `0.05`) in the same template. **The `1.0` is the real
      absurdity** (always in the reward branch, unpenalizable).
- [ ] Mirror both `suggest` defaults in `Gen_Taskset/lib/taskset_generator.py:33-34`
      so a fresh config (interactive prompt) gets the new values.
- [ ] **Design suggestion:** also add a value-range assertion in
      `validate_config_integrity` (`taskset_generator.py:92-117`) — reject
      `sp_threshold >= 1.0` (and maybe `> 0.9`) at validation time, so the
      absurdity can't silently recur. Currently it checks *presence* only.
      (Optional; ask user.)
- [ ] Note: editing the fallback range alone would NOT change a re-run (the SET
      path is primary). Both knobs edited together.
- [ ] Does NOT touch `parameters.yaml`. Does NOT touch `TIME_LIMIT` or the
      `ReoptimizationUseSubIncrementalWalk` flag.

## Step 2 — Obtain per-task analytic `ddl_miss_chance` (NOT STARTED)

> Depends on D5. Option (b) first (read-only, no rebuild):

- [ ] For each of the 5 tasksets, parse `taskset_param.yaml` → per task:
      `Et_mean`, `Et_sigma`, `execution_time_min`, `execution_time_max`,
      `deadline`, `sp_threshold`, `sp_weight`, `time_limit_task`.
- [ ] Reconstruct the scored `FiniteDist` in Python the way C++ does
      (`RegularTasks.cpp` + `Probability.cpp:18-43`): Gaussian(μ,σ) truncated to
      `[min, max]`, binned, upper-tail mass dumped on the `max` bin. (Verify the
      exact binning from `Probability.cpp` before trusting the number.)
- [ ] Apply `GetDDL_MissProbability`: sum probability of all bins with
      `value > deadline`. This is the analytic `ddl_miss_chance`.
- [ ] **Caveat:** the scored distribution the metric uses is the *post-time-limit*
      RTA distribution, not the raw task ET distribution. For perf
      (time-limit-optimizable) tasks the optimizer picks a time limit → the RTA
      dist shifts. The `interval_sp_metrics.txt` overall SP is computed on the
      *optimized* config. So a Python re-impl from static `taskset_param.yaml`
      gives the *un-optimized* analytic miss-prob, which may differ from what the
      metric actually scored. → This is the strongest argument for D5 option (a):
      log the *actual* `ddl_miss_chance` the metric computed, post-optimization.

## Step 3 — Per-task analytic miss-prob vs threshold (NOT STARTED)

- [ ] For the top-weight ("important") task in each taskset: record
      `ddl_miss_chance`, `sp_threshold`, the branch (reward if
      `ddl_miss_chance <= sp_threshold`, else penalty), and the relative
      difference `(ddl_miss_chance − sp_threshold) / sp_threshold`.
- [ ] Also record the same for ALL tasks (not just important) for context — the
      weight-dilution point (reason 3) means non-important tasks matter for the
      overall SP.
- [ ] Mean relative difference across the 5 important tasks → the reported number.
- [ ] Cross-check against `miss_rate_per_task.txt` (empirical) to quantify the
      analytic-vs-empirical gap per task (the P2.6 signal).

## Step 4 — Written characterization + reported number (NOT STARTED)

- [ ] `dev_log.md` entry: the mean relative-difference number; per-task table
      (task, weight, threshold, analytic miss-prob, branch, empirical miss,
      ddl-vs-wcet feasibility). Which facet (generator / metric / selection)
      owns the disconnect, with evidence.
- [ ] Verdict: are important tasks "safe" by the metric's own definition
      (analytic miss-prob ≤ threshold)? If yes → flat SP is correct, the
      empirical `Important_Miss_Rate` swing is a separate (generator/sim) signal.
      If no (analytic miss-prob > threshold yet SP ≈ 1) → metric under-penalizes
      → D3, file new task.
- [ ] If a fix is warranted → file as NEW task (P2.14+ or P1.x), one-line pointer
      from this task's `dev_log.md`. Do NOT implement here.

## Step 5 — Docs (NOT STARTED)

- [ ] `agents/overall_tasks.md` — P2.13 row updated (reframed + config-edit
      scope); mark DONE when characterized.
- [ ] Top-level `agents/dev_log.md` — append the reframing milestone + later the
      verdict + the reported number.
- [ ] Memory `p213-important-task-ddl-vs-sp-metric.md` — update from "filed" to
      "characterized" with the verdict + number; `MEMORY.md` pointer updated.
- [ ] `git add` the P2.13 unit + the config edit (separate commits if user wants
      the config edit landed first); hand to user for review (no commit).

## Standing constraints

- **No implementation until the user greenlights the plan.**
- **No new simulation** for Steps 2–3 if D5 option (b) suffices — reuse the
  existing N=10 run + `taskset_param.yaml`. A fresh A/B with the new threshold
  range (Step 1 applied) is the user's to run.
- **No `git commit`** (`git add` only, on user go).
- **No `parameters.yaml` edit** (the `ReoptimizationUseSubIncrementalWalk` flag
  stays 0; `TIME_LIMIT` stays as-is).
- **No gate moves; no `SP_Func` change in this task** (D3, if warranted, is a new
  task).
- Don't conflate with P1.8 (generator clamp — separate; note its perf-task skip
  gap) or P2.6 (sim-RT SP column — separate, deferred). P2.13 *uses* their
  conclusions.
