# P2.15 — Dev Log

## 2026-07-25 — Task filed (planning only, no implementation)

Filed P2.15 as the weight-side analogue of P2.14. Trigger: user direction —
"we should randomly generate sp weights from 0.1 to 1.0 too, make sp weight
range an input parameter, similar to SP_THRESHOLD_RANGE... it doesn't make
sense to always assign high sp weights to TL tasks."

**Current logic (to be replaced):** `taskset_generator.py:594/599` hardcodes
`sp_weight_base = 2.0` for `time_limit_task` (perf) tasks and `1.0` for
normal/env tasks, then normalizes so `Σ = SP_WEIGHTS_SUM` (lines 628-633). The
2:1 base ratio makes perf tasks always count twice as much in the SP objective
— coupling "importance" to "has a TL knob," a generator artifact.

**Proposed logic:** add `SP_WEIGHT_RANGE: [0.1, 1.0]` (parallel to
`SP_THRESHOLD_RANGE`); sample `sp_weight = random.uniform(wt_min, wt_max)` per
task in the static-properties loop (line 542, next to `sp_threshold`); remove
the 2:1 split; keep `SP_WEIGHTS_SUM` normalization (scale contract). "Importance"
becomes a random per-task property, decoupled from task type.

**Verified semantically safe (generator-only):** `sp_weight` is consumed in C++
purely as a linear multiplier — `ObtainSP` × `weight` (`SP_Metric.cpp:12-16`),
`effective_weight = weight * perf_coeff` (`OptimizeSP_Incre.cpp:81-87`),
`sum_sp_weights` as a ceiling scalar (`:154-159`). No C++ code assumes the 2:1
ratio or any specific magnitude → no `sources/` edit, ctest N/A (same
justification as P2.14 Step 4).

**Test impact:** all existing `sp_weight` assertions survive unchanged —
positivity (`test_specifications.py:90`) + `Σ == SP_WEIGHTS_SUM`
(`:105-106`, `test_integration.py:96-97`); `test_aggregate.py` /
`test_feasibility_clamp.py` use synthetic fixtures. Optional D6 ratio-bound
check (`max/min <= 10.0`) deferred to user decision.

**Cross-links:**
- P2.14 — structural twin (threshold-side).
- P2.13 — "important task" = top-`sp_weight` task; under P2.15 that label is
  random (no longer by-construction a perf task). P2.13's D1 feasibility check
  (perf-task clamp gap in `feasibility_clamp.py`) should be re-run against
  randomized-weight tasksets once P2.15 lands.

**Status:** FILED (goal.md + tasks.md + this log). Awaits user sign-off on
D1–D6 before implementation. No `git commit`; `git add` on user go.

## 2026-07-25 — Implemented (Steps 1–5; user greenlit on D1–D6 assumptions)

User said "start implementation." Proceeded on the filed assumptions (D1 pure
uniform YES; D3 range `[0.1, 1.0]`; D6 SKIP the optional ratio-bound assertion —
the existing sum + positivity checks already pin the contract, matching P2.14's
approach).

**Step 1 — generator (`Gen_Taskset/lib/taskset_generator.py`):**
- Added `SP_WEIGHT_RANGE` (`suggest [0.1, 1.0]`) to `REQUIRED_CONFIG_PARAMS`,
  line 36 (between `MAX_TIME_LIMIT_OPTIONS` and `SP_WEIGHTS_SUM`, mirroring the
  registry order: threshold range → weight range → weight sum).
- Sampling loop (lines 542-552): added a P2.15 comment block + `wt_min`/`wt_max`
  extraction, then `taskset_param[i].sp_weight = random.uniform(wt_min, wt_max)`
  replacing the dead `= 1.0` placeholder. `sp_weight` is now sampled right next
  to `sp_threshold`.
- Deleted `sp_weight_base = 2.0` (perf) and `sp_weight_base = 1.0` (normal/env)
  from the perf/non-perf branch — the rest of the branch (execution bounds +
  perf records) is untouched.
- Serialization line 621: `'sp_weight': float(sp_weight_base)` →
  `'sp_weight': float(t.sp_weight)` (mirrors line 622's `t.sp_threshold`).
- `SP_WEIGHTS_SUM` normalization (635-640): UNCHANGED — scale contract preserved.

**Step 2 — configs:** added `"SP_WEIGHT_RANGE": [0.1, 1.0]` to the same 5 config
files P2.14 touched (`taskset_cfg_paper_base.json`, `taskset_N8/generator_config.json`,
`test_standard_{4,6,8}.json`), placed right after `SP_THRESHOLD_RANGE`.

**Step 2 bonus — gate-driven test-config edits:** the new REQUIRED key tripped
`validate_config_integrity` on 9 test files that build inline config dicts (the
non-interactive raise lists `SP_WEIGHT_RANGE` as missing). Added the key next to
each inline `SP_THRESHOLD_RANGE` in: `tests/python/test_{coordinate_system,
correlation_serialization,simulation_pipeline,uunifast,physical_units_pipeline,
taskset_generator}.py` and `Gen_Taskset/tests/test_{generators,specifications,
integration}.py`. (`tests/python/test_yaml_exporter.py` bypasses the gate — it
calls `convert_taskset_parameters_to_cpp_yaml` directly with a 2-key `cfgs` —
not edited, confirmed passing.) This is the same shape of fallout P2.14 had
(P2.14 edited 9 test files for `SP_THRESHOLD_RANGE`).

**Step 3+4 — verification:**
- `pytest Gen_Taskset/tests tests/python` → **372 passed, 0 failed**. Before the
  inline-config edits this was 331 passed / 41 failed — all 41 were the integrity
  gate raising on the missing `SP_WEIGHT_RANGE` key, not logic failures.
- Existing `sp_weight` assertions survive unchanged: positivity (`test_specifications.py:90`)
  and `Σ == SP_WEIGHTS_SUM` (`:105-106`, `test_integration.py:96-97`).
- End-to-end sanity (`test_standard_8`, seed 42): `Σ sp_weight = 5.000000`;
  max/min ratio 6.376 (within the 10× ceiling from `[0.1, 1.0]`); the top-weight
  task was a **non-perf** task (task_4, w=1.213) while the single perf task
  (task_6) was mid-range (w=0.746). Under the old 2:1 split the perf task would
  have been guaranteed the highest weight → confirms "importance" is now random,
  decoupled from task type. This is the P2.15 design goal made observable.
- C++ ctest N/A (Python-only; `sp_weight` is a pure linear multiplier in C++ —
  `ObtainSP × weight`, `effective_weight = weight * perf_coeff`, `sum_sp_weights`
  ceiling — no `sources/` code assumes the 2:1 ratio; same N/A justification as
  P2.14 Step 4).

**Step 5 — records:** updated this log + `tasks.md` (all steps checked) +
`agents/overall_tasks.md` + memory `p215-sp-weight-randomization.md`.

**Cross-link to P2.13:** "important task" = top-`sp_weight` task. Under the old
2:1 split it was *by construction* a perf task; under P2.15 it is random. P2.13's
D1 feasibility check (perf-task clamp gap in `feasibility_clamp.py`, which skips
perf tasks) should be re-run against randomized-weight tasksets once generated —
the gap may or may not still bite the "important" task depending on the random
draw. P2.15 does NOT block P2.13's config edit (independent keys:
`SP_THRESHOLD_RANGE` vs `SP_WEIGHT_RANGE`).

**Status:** IMPLEMENTED, working tree (NOT committed). Awaits user review.
`git add` staged on user go; no `git commit` (per standing guardrail).

## 2026-07-25 — CLOSED (committed `771be079`; moved to finished_tasks)

- **Status updated:** the prior entry's "working tree, NOT committed" is superseded —
  P2.15 was committed at `771be079` (code + `taskset_generator.py` + 5 configs + 9
  inline-config test edits + this folder's 3 records). `pytest Gen_Taskset/tests
  tests/python` → 372/372 green; the "importance is now random" sanity check
  (top-weight task = non-perf) holds.
- **Moved to `agents/finished_tasks/P2_15_sp_weight_randomization/`.**
- **Cross-link reminder (carried forward to P2.13):** P2.13's D1 feasibility check
  (perf-task clamp gap in `feasibility_clamp.py`) should be re-run against randomized-
  weight tasksets — under P2.15 the "important" (top-`sp_weight`) task is no longer
  by-construction a perf task, so the clamp gap may or may not bite depending on the
  random draw.

