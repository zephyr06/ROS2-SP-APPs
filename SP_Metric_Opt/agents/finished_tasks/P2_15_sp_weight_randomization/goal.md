# P2.15 — Randomize `sp_weight` per task (drop the hardcoded perf/non-perf 2:1 split)

## The Goal

Replace the hardcoded two-tier `sp_weight` base assignment (`2.0` for
time-limit/perf tasks, `1.0` for normal/env tasks) with **per-task random
sampling** from a new continuous config parameter `SP_WEIGHT_RANGE: [0.1, 1.0]`,
mirroring exactly what P2.14 did for `sp_threshold` / `SP_THRESHOLD_RANGE`.

### Motivation (User Direction)

> "we should randomly generate sp weights from 0.1 to 1.0 too, make sp weight
> range an input parameter, similar to SP_THRESHOLD_RANGE, then generate sp
> weight for each task randomly. it doesn't make sense to always assign high sp
> weights to TL tasks."

The current logic (`taskset_generator.py:594/599`) hardcodes
`sp_weight_base = 2.0` for `time_limit_task` (perf) tasks and `1.0` for
normal/env tasks. This bakes in the assumption that **perf/TL-optimizable tasks
are always the "important" tasks** — they always count twice as much in the SP
objective. That assumption is unjustified:

- It couples "importance" to "has a time-limit knob," which is a generator
  artifact, not a domain statement about which tasks matter.
- It undermines P2.13's analysis: P2.13 picks the top-`sp_weight` task as the
  "important" task, but under the 2:1 split the top-weight task is *by
  construction* a perf task — so "important" is synonymous with "perf task," and
  the question "are important tasks safe?" collapses to "are perf tasks safe?"
  (which is exactly the perf-task clamp gap P2.13 D1 flagged).
- It is the weight-side analogue of the P2.14 absurdity (where `SP_THRESHOLDS_SET`
  hardcoded `1.0` = "always safe"). P2.14 killed the hardcoded threshold; P2.15
  kills the hardcoded weight.

Random uniform sampling means a perf task *may* get a high weight, but is not
guaranteed one; a normal/env task *may* become the highest-weight ("important")
task. "Importance" becomes a random per-task property, decoupled from task type.

## Deliverables

1. **Generator & Schema Update** (`Gen_Taskset/lib/taskset_generator.py`):
   - Add `SP_WEIGHT_RANGE` to `REQUIRED_CONFIG_PARAMS` (suggest `[0.1, 1.0]`),
     directly parallel to `SP_THRESHOLD_RANGE`.
   - Sample `sp_weight` per task: `random.uniform(wt_min, wt_max)` in the
     static-properties loop (line 542), replacing the dead `= 1.0` placeholder.
   - Remove the `sp_weight_base = 2.0` / `1.0` assignment (lines 594/599) and
     read `t.sp_weight` at serialization (line 614) instead of
     `float(sp_weight_base)`. The perf/non-perf branch (579-599) keeps its
     other duties (execution bounds + perf records) but no longer sets weight.
   - **Keep** the `SP_WEIGHTS_SUM` normalization (lines 628-633) unchanged —
     it is the scale contract (bounded, comparable magnitude across taskset
     sizes); randomization changes the *ratios*, normalization fixes the *sum*.

2. **Configuration Templates & Test Fixtures:**
   - Add `"SP_WEIGHT_RANGE": [0.1, 1.0]` to every config P2.14 touched:
     `taskset_cfg_paper_base.json`, `tests/speed_test/taskset_N8/generator_config.json`,
     `Gen_Taskset/tests/test_configs/test_standard_{4,6,8}.json`.

3. **Test Suite Updates:**
   - Existing `sp_weight` assertions all survive (positivity + sum-equals-
     `SP_WEIGHTS_SUM`); no forced edits. Optionally add an observable ratio
     bound (see D6).

4. **Verification:**
   - `pytest Gen_Taskset/tests tests/python` (Python-only change; C++ ctest N/A
     — no `SP_WEIGHT` reference in `sources/` beyond reading the YAML value,
     same justification as P2.14 Step 4).

## Code Grounding

- `Gen_Taskset/lib/taskset_generator.py:24-46` — `REQUIRED_CONFIG_PARAMS`
  registry (where `SP_WEIGHT_RANGE` is added, next to `SP_THRESHOLD_RANGE`
  line 33 + `SP_WEIGHTS_SUM` line 36).
- `Gen_Taskset/lib/taskset_generator.py:541-543` — the static-properties loop
  where `sp_weight = 1.0` (dead placeholder) and `sp_threshold` are set; this
  is where `sp_weight` sampling moves.
- `Gen_Taskset/lib/taskset_generator.py:579-599` — the perf/non-perf branch
  that hardcodes `sp_weight_base = 2.0` (perf) / `1.0` (normal/env); the
  weight lines are removed, the rest of the branch stays.
- `Gen_Taskset/lib/taskset_generator.py:614` — `'sp_weight': float(sp_weight_base)`
  → becomes `'sp_weight': float(t.sp_weight)` (mirrors line 615
  `'sp_threshold': float(t.sp_threshold)`).
- `Gen_Taskset/lib/taskset_generator.py:628-633` — `SP_WEIGHTS_SUM`
  normalization (UNCHANGED; preserves the scale contract).
- `Gen_Taskset/lib/gmm_model.py:80-81` — constructor defaults
  `self.sp_weight = 1.0` (overwritten by generator; no change needed).

## Downstream (C++ consumption — UNCHANGED, semantically safe)

`sp_weight` is consumed purely as a **linear multiplier**; no C++ code assumes
the 2:1 ratio or any specific magnitude:

- `sources/Safety_Performance_Metric/ParametersSP.cpp:25-27` — loads
  `sp_weight` from YAML into `weights_node` (reads whatever value is written).
- `sources/Safety_Performance_Metric/SP_Metric.cpp:12-16` —
  `ObtainSP(...) = SP_Func(ddl_miss_chance, threshold) * weight` (linear).
- `sources/Optimization/OptimizeSP_Incre.cpp:81-87` —
  `effective_weight = sp_parameters.weights_node[task_id] * perf_coeff`;
  `sp_lost += effective_weight - sp_cur` (linear).
- `sources/Optimization/OptimizeSP_Incre.cpp:154-159` — `sum_sp_weights` used
  as a ceiling/reference scalar (linear; the comment marks the old method).

→ The change is **generator-only**; no `sources/` edit, no ctest re-run needed
(same N/A justification as P2.14 Step 4: no `SP_WEIGHT` reference in `sources/`
C++ code beyond reading the YAML field, and no C++ test consumes any of the
edited JSON configs).

## Design Decisions (settle with the user BEFORE implementation)

- **D1 — Drop the perf/non-perf weight distinction entirely?** YES (user
  direction: "it doesn't make sense to always assign high sp weights to TL
  tasks"). All tasks draw `sp_weight` from the same `uniform(0.1, 1.0)`. A perf
  task is no longer guaranteed to be high-weight. (If the user later wants a
  *bias* rather than a hard 2:1 split — e.g. perf tasks drawn from a shifted
  range — that is a separate knob; this task does pure uniform.)

- **D2 — Keep `SP_WEIGHTS_SUM` normalization?** YES. It is the scale contract
  (bounded, comparable magnitude across taskset sizes); the metric's absolute
  scale depends on it. Randomization sets the *ratios*; normalization fixes the
  *sum*. Both coexist cleanly: `base ~ U(0.1,1.0)` per task, then rescale so
  `Σ = SP_WEIGHTS_SUM`. The ratio `max/min` is preserved by normalization
  (bounded by `1.0/0.1 = 10×`), so no task can vanish or dominate absolutely.

- **D3 — Is `[0.1, 1.0]` the right range?** User specified it. The `0.1` floor
  guarantees every task contributes (>0 → survives the `sp_weight > 0.0` test
  and avoids div-by-zero in normalization if all happened to be 0, which
  `U(0.1,1.0)` makes impossible anyway). The `1.0` cap bounds the max/min ratio
  at 10×. After normalization the absolute values rescale, but the 10× ratio
  ceiling holds.

- **D4 — Backward compat / config migration.** `SP_WEIGHT_RANGE` is a new
  REQUIRED param → `validate_config_integrity` will prompt/raise on any config
  missing it. So every config P2.14 touched needs the key added. Mirror P2.14
  Step 2 exactly (same 5 config files). A stale config missing the key fails
  loudly (the P1.3/P1.4/P2.14 pattern, not a silent alias).

- **D5 — Where to sample (loop placement).** Sample `sp_weight` in the
  static-properties loop (line 542, next to `sp_threshold`) and read
  `t.sp_weight` at serialization (line 614). This mirrors how `sp_threshold` is
  handled (sampled line 543, read line 615) and removes the now-dead
  `sp_weight_base` local. The perf/non-perf branch at 579-599 keeps its other
  duties (execution bounds + perf records) — it stops being about weight.

- **D6 — Test impact / new assertions.** All existing `sp_weight` assertions
  survive unchanged:
  - `test_specifications.py:90` `sp_weight > 0.0` ✓ (`U(0.1,1.0) > 0`)
  - `test_specifications.py:105-106` `Σ sp_weight == SP_WEIGHTS_SUM` ✓
    (normalization preserved)
  - `test_integration.py:96-97` same sum invariant ✓
  - `test_aggregate.py` / `test_feasibility_clamp.py` use synthetic fixtures
    (not the generator) ✓
  - *Optional new assertion* (mirrors P2.14's threshold-range check): because
    normalization preserves ratios, `max(sp_weight)/min(sp_weight) <= 10.0`
    is an observable invariant from the YAML. But it is a `<=` bound (actual
    ratio is usually < 10), slightly fragile; alternatively rely on the
    existing sum + positivity checks. **Recommendation:** add the ratio bound
    as a soft check with a tolerance, OR skip it (the sum + positivity checks
    already pin the contract). Settle with the user.

- **D7 — C++ impact.** None (see "Downstream" above). Generator-only; ctest N/A.

- **D8 — Relationship to P2.13 / P2.14.** This is the **weight-side analogue**
  of P2.14 (threshold-side). Cross-link:
  - **P2.13:** its "important task" = top-`sp_weight` task. Under the 2:1 split
    that is *by construction* a perf task; under P2.15 it is random. P2.13's
    analysis (D1: are important-task DDLs feasible?) should be re-run against
    randomized-weight tasksets once P2.15 lands — the perf-task clamp gap
    (`feasibility_clamp.py` skips perf tasks) may or may not still bite the
    "important" task depending on the random draw. P2.15 does NOT block P2.13's
    config edit (P2.13 edits `SP_THRESHOLD_RANGE`, P2.15 edits `SP_WEIGHT_RANGE`
    — independent keys).
  - **P2.14:** structural twin. P2.15 reuses P2.14's verification pattern
    (Python-only, ctest N/A, same 5 config files).

- **D9 — Empirical analysis step?** P2.13/P2.14 each carry a Step 5 empirical
  analysis. The user's directive here is purely the generation change (no
  "report a number" ask). So P2.15 is scoped to generation + tests only; the
  empirical "does randomizing weights change which tasks are important / move
  SP?" question is deferred to P2.13's analysis re-run (which can now run
  against randomized-weight tasksets). Placeholder noted in `tasks.md` Step 5
  but not a gate.

## Scope (what this task IS / IS NOT)

**IS:**
- A new `SP_WEIGHT_RANGE` config param + per-task uniform sampling, replacing
  the hardcoded 2:1 base-weight split.
- Generator + config-template + test-fixture edits (Python-only).
- Structurally parallel to P2.14.

**IS NOT:**
- A change to the `SP_WEIGHTS_SUM` normalization (kept).
- A change to C++ weight consumption (none needed; linear multiplier).
- A change to `SP_Func` / the metric shape.
- A re-run of P2.13's analysis (deferred to P2.13, against new tasksets).
- A perf-task weight *bias* (pure uniform; a biased range would be a separate
  knob).

## Guardrails (standing constraints)

- **No `git commit`** (`git add` only when the user asks).
- **No implementation until the user greenlights the plan.** This filing is
  task-description + design suggestions + implementation plan only.
- Settle D1–D6 with the user before implementing; D7–D9 are context/cross-links.

## Done when

- [ ] `SP_WEIGHT_RANGE` added to `REQUIRED_CONFIG_PARAMS` (suggest `[0.1, 1.0]`).
- [ ] `sp_weight` sampled per task `random.uniform(wt_min, wt_max)` in the
      static-properties loop; `sp_weight_base` 2:1 split removed.
- [ ] All 5 P2.14 config files carry `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [ ] `pytest Gen_Taskset/tests tests/python` green (existing assertions
      survive; optional D6 ratio-bound check added if user wants).
- [ ] `agents/overall_tasks.md` + memory updated.
- [ ] `git add` staged; user reviews (no commit).

## Reference docs

- `Gen_Taskset/lib/taskset_generator.py:24-46, 541-543, 579-599, 614, 628-633`
  — weight knobs + sampling + normalization.
- `Gen_Taskset/lib/gmm_model.py:80-81` — `sp_weight` default.
- `sources/Safety_Performance_Metric/SP_Metric.cpp:12-16` — `ObtainSP` (linear × weight).
- `sources/Optimization/OptimizeSP_Incre.cpp:81-87, 154-159` — `effective_weight` + `sum_sp_weights`.
- Sibling: [`P2_14_sp_threshold_set_removal/`](../P2_14_sp_threshold_set_removal/) —
  the structural twin (threshold-side).
- Cross-link: [`P2_13_important_task_ddl_vs_sp_metric/`](../P2_13_important_task_ddl_vs_sp_metric/)
  — "important task" = top-`sp_weight` task; P2.15 makes that label random.
- Memory [`p213-important-task-ddl-vs-sp-metric`] — perf-task clamp gap (D1).
