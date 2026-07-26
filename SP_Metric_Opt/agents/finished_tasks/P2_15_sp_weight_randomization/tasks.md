# P2.15 — Tasks (working checklist)

> See `goal.md` for the motivation, scope, deliverables, and code grounding.
> Structurally parallel to P2.14 (the threshold-side twin).

## Step 1 — Code & Schema Update
- [x] `Gen_Taskset/lib/taskset_generator.py`: Add `SP_WEIGHT_RANGE` to `REQUIRED_CONFIG_PARAMS` (`[0.1, 1.0]`), next to `SP_THRESHOLD_RANGE` / `SP_WEIGHTS_SUM`. — landed line 36.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Sample `sp_weight` per task in the static-properties loop: `taskset_param[i].sp_weight = random.uniform(wt_min, wt_max)` (replaces the dead `= 1.0` placeholder; mirrors `sp_threshold` sampling). — landed line 551.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Remove `sp_weight_base = 2.0` and `sp_weight_base = 1.0` from the perf/non-perf branch (the rest of the branch — execution bounds + perf records — stays). — both lines deleted.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Change `'sp_weight': float(sp_weight_base)` → `'sp_weight': float(t.sp_weight)` (mirrors `'sp_threshold': float(t.sp_threshold)`). — landed line 621.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Confirm `SP_WEIGHTS_SUM` normalization is UNCHANGED (scale contract preserved). — confirmed, lines 635-640 byte-identical to pre-P2.15.

## Step 2 — Configuration Templates & Test Fixtures
- [x] `Gen_Taskset/task_sets_config/templates/taskset_cfg_paper_base.json`: Add `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [x] `tests/speed_test/taskset_N8/generator_config.json`: Add `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_4.json`: Add `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_6.json`: Add `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_8.json`: Add `"SP_WEIGHT_RANGE": [0.1, 1.0]`.
- [x] **Bonus (gate-driven):** 9 test files build inline config dicts that flow through `validate_config_integrity` and would trip the new REQUIRED key. Added `"SP_WEIGHT_RANGE": [0.1, 1.0]` next to each inline `SP_THRESHOLD_RANGE` in: `tests/python/test_{coordinate_system,correlation_serialization,simulation_pipeline,uunifast,physical_units_pipeline,taskset_generator}.py`, `Gen_Taskset/tests/test_{generators,specifications,integration}.py`. (`tests/python/test_yaml_exporter.py` bypasses the gate — not edited, confirmed passing.)

## Step 3 — Test Suite Updates
- [x] Confirm existing `sp_weight` assertions survive (positivity `> 0.0` + `Σ == SP_WEIGHTS_SUM`): `test_specifications.py:90,105-106`, `test_integration.py:96-97`. — both survive unchanged.
- [x] (D6 — decided SKIP) No new ratio-bound assertion added. The sum + positivity checks already pin the contract; a `max/min <= 10.0` bound is a `<=` ceiling (actual ratio is usually < 10) and slightly fragile. Relied on the existing checks, matching P2.14's threshold approach (which also added no new range assertion beyond what the gate enforces).

## Step 4 — Verification
- [x] Run `pytest Gen_Taskset/tests tests/python` → **372 passed, 0 failed** (was 331 passed / 41 failed before the 9 inline-config edits; the 41 were all the integrity gate raising on the missing key, not logic failures).
- [x] End-to-end sanity (test_standard_8, seed 42): `Σ sp_weight = 5.000000` (= `SP_WEIGHTS_SUM`); max/min ratio 6.376 (within 10× ceiling); the top-weight task was a **non-perf** task (task_4, w=1.213) while the single perf task (task_6) was mid-range (w=0.746) — confirms "importance" is now random, decoupled from task type.
- [x] C++ ctest: N/A (Python-only change — no `SP_WEIGHT` reference in `sources/` C++ code beyond reading the YAML field; no C++ test consumes any of the edited JSON configs. Same N/A justification as P2.14 Step 4.)

## Step 5 — Cross-link & Records (not a gate; deferred empirical work)
- [x] Note in P2.13 that "important task" is now random under P2.15 (no longer by-construction a perf task); P2.13's D1 feasibility check should be re-run against randomized-weight tasksets once generated. — recorded in dev_log + memory.
- [x] Record the change in `dev_log.md` + update `agents/overall_tasks.md` + memory.
