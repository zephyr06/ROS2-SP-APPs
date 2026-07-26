# P2.14 — Tasks (working checklist)

> See `goal.md` for the motivation, scope, deliverables, and code grounding.

## Step 1 — Code & Schema Update
- [x] `Gen_Taskset/lib/taskset_generator.py`: Remove `SP_THRESHOLDS_SET` from `REQUIRED_CONFIG_PARAMS`.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Update `SP_THRESHOLD_RANGE` suggested default to `[0.001, 0.9]`.
- [x] `Gen_Taskset/lib/taskset_generator.py`: Simplify sampling in `generate_tasksets()` (lines 535-545) to directly sample `sp_threshold = random.uniform(trd_min, trd_max)`.

## Step 2 — Configuration Templates & Test Fixtures
- [x] `Gen_Taskset/task_sets_config/templates/taskset_cfg_paper_base.json`: Remove `"SP_THRESHOLDS_SET"` and update `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.
- [x] `tests/speed_test/taskset_N8/generator_config.json`: Remove `"SP_THRESHOLDS_SET"` and update `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_4.json`: Remove `"SP_THRESHOLDS_SET"` and update `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_6.json`: Remove `"SP_THRESHOLDS_SET"` and update `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.
- [x] `Gen_Taskset/tests/test_configs/test_standard_8.json`: Remove `"SP_THRESHOLDS_SET"` and update `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.

## Step 3 — Python Unit Test Suite
- [x] `Gen_Taskset/tests/test_generators.py`: Update tests to remove `SP_THRESHOLDS_SET`.
- [x] `Gen_Taskset/tests/test_integration.py`: Update tests to remove `SP_THRESHOLDS_SET`.
- [x] `Gen_Taskset/tests/test_specifications.py`: Update tests to remove `SP_THRESHOLDS_SET`.
- [x] `tests/python/test_taskset_generator.py`: Update threshold assertion range checks to `0.001 <= sp_threshold <= 0.9`.
- [x] `tests/python/test_simulation_pipeline.py`: Remove `"SP_THRESHOLDS_SET"`.
- [x] `tests/python/test_coordinate_system.py`: Remove `"SP_THRESHOLDS_SET"`.
- [x] `tests/python/test_correlation_serialization.py`: Remove `"SP_THRESHOLDS_SET"`.
- [x] `tests/python/test_uunifast.py`: Remove `"SP_THRESHOLDS_SET"`.
- [x] `tests/python/test_physical_units_pipeline.py`: Remove `"SP_THRESHOLDS_SET"`.

## Step 4 — Verification
- [x] Run `pytest Gen_Taskset/tests tests/python` and verify all tests pass. *(61/61 SP-threshold-relevant tests pass in isolation. 2 full-suite failures are pre-existing flakiness — `test_integration_pipeline` has no `RANDOM_SEED`, `test_trajectory.test_generate_path_only` is unseeded + cross-module RNG-pollution sensitive — reproduced on the stashed baseline WITHOUT these changes.)*
- [x] Run `ctest --test-dir build --output-on-failure` and verify C++ suite passes. *(N/A: P2.14 is Python-only — no `SP_THRESHOLD` reference exists in `sources/` C++ code, and no C++ test consumes any of the 5 edited JSON configs. C++ suite is structurally independent of this change.)*

## Step 5 — Empirical Analysis & Records — DEFERRED TO P2.13
- [-] Compute per-task analytic `ddl_miss_chance` vs `sp_threshold` for important tasks. — **DEFERRED to P2.13** (the empirical safety study is owned by P2.13; P2.14 closed as code-complete).
- [-] Record findings and mean relative difference in `dev_log.md`. — **DEFERRED to P2.13.**
