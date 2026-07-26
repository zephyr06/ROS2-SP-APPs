# P2.14 — Removal of SP_THRESHOLDS_SET & SP_THRESHOLD_RANGE Harmonization

## The Goal

Eliminate `SP_THRESHOLDS_SET` from the taskset generator configuration schema, templates, and unit tests. Harmonize `sp_threshold` generation to rely exclusively on a single continuous parameter: `SP_THRESHOLD_RANGE: [0.001, 0.9]`.

### Motivation (User Direction)
- In `taskset_generator.py` (lines 541-544), `SP_THRESHOLDS_SET` took priority whenever non-empty, completely overriding and bypassing `SP_THRESHOLD_RANGE`.
- Having both a discrete option set (`SP_THRESHOLDS_SET`) and a continuous range (`SP_THRESHOLD_RANGE`) created redundant mechanisms and user confusion.
- Furthermore, `SP_THRESHOLDS_SET` contained `1.0`, which allowed tasks to accept 100% DDL miss as "safe" (unpenalizable).
- Removing `SP_THRESHOLDS_SET` ensures a clean, continuous sampling mechanism: `sp_threshold = random.uniform(0.001, 0.9)`.

## Deliverables

1. **Generator & Schema Update:**
   - Remove `SP_THRESHOLDS_SET` from `REQUIRED_CONFIG_PARAMS` in `Gen_Taskset/lib/taskset_generator.py`.
   - Update suggested value for `SP_THRESHOLD_RANGE` to `[0.001, 0.9]`.
   - Simplify sampling in `generate_tasksets()` to draw `sp_threshold = random.uniform(trd_min, trd_max)` from `SP_THRESHOLD_RANGE`.

2. **Configuration Templates & Test Fixtures:**
   - Remove `"SP_THRESHOLDS_SET"` from `Gen_Taskset/task_sets_config/templates/taskset_cfg_paper_base.json` and set `"SP_THRESHOLD_RANGE": [0.001, 0.9]`.
   - Remove `"SP_THRESHOLDS_SET"` from `tests/speed_test/taskset_N8/generator_config.json` and `Gen_Taskset/tests/test_configs/*.json`.

3. **Test Suite Updates:**
   - Update unit test assertions in `Gen_Taskset/tests/` and `tests/python/` to assert `sp_threshold` is within `[0.001, 0.9]` range rather than checking discrete membership in `SP_THRESHOLDS_SET`.

4. **Empirical Safety Analysis:**
   - Compute analytic `ddl_miss_chance` vs `sp_threshold` for important tasks across existing tasksets, calculate relative difference `(ddl_miss_chance − sp_threshold) / sp_threshold`, and record in `dev_log.md`.

## Code Grounding

- `Gen_Taskset/lib/taskset_generator.py:24-47`: `REQUIRED_CONFIG_PARAMS` registry
- `Gen_Taskset/lib/taskset_generator.py:535-545`: `generate_tasksets()` sampling logic
- `Gen_Taskset/task_sets_config/templates/taskset_cfg_paper_base.json`: Base configuration template
