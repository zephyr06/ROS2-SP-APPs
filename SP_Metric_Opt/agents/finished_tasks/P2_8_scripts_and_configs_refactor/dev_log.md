# P2.8 — Dev Log

## 2026-07-24 — Task filed (investigation complete, no code yet)

Filed by inventorying `scripts/` (4 shell scripts + `generate_taskset_6.py` +
`lib/common.sh`) and `simulation_experiments/configs/` (4 JSON files), and
tracing every config/script reference across `simulation_experiments/*.py`,
`scripts/*.sh`, and `tests/`.

**Findings captured in `goal.md`:**

1. **"How the other 2 scripts are used" (user question):** `run_interval_sweep.sh`
   + `run_paper_figures.sh` are **standalone single-stage entry points**, NOT
   sourced helpers of the two top-level scripts. The main pipeline calls the
   stage modules directly (`run_end_to_end_experiments.py` → `compare_optimizers`
   / `interval_sweep` / `aggregate_across_tasks`). So "move to `lib/`" is a
   category mismatch — `lib/` is for sourced helpers (`common.sh`). Real
   disposition = keep-as-standalone vs delete (D3). `run_paper_figures.sh` is the
   closest existing thing to the user's goal (a) "generate all figures," but it
   misses Fig 2 (sweep) — a gap to close.

2. **`--steps` is stale (D2):** `run_end_to_end_experiments.py` has no `--steps`
   flag (P22 removed step selection), yet `simulation_only_config.json`'s
   `_comment` + the two thin scripts' comments reference `--steps simulate` /
   the deleted `run_simulation.sh`. Broken references — cleanup needed; removing
   `simulation_only_config.json` moots its case.

3. **Eval-suite config coupling (D1, CRUX):** `evaluation_suite_config.json`
   carries the gate-specific 10-scheduler set + `eval_*` keys that
   `paper_simulation_config.json` (renamed `experiment_config.json`) lacks.
   `evaluation_suite.py:611` + `run_evaluation_suite.sh:39` default to it.
   Removing it is NOT a pure deletion — it breaks the eval-suite default. D1
   offers α/β/γ; recommendation γ (flag-gated `--eval_ns` + fold `eval_*` into
   `paper_simulation_config.json`).

4. **Top-level script duplication (D6):** `run_evaluation_suite.sh` is ~90%
   duplicated with `run_end_to_end.sh` (build + delegate). Merge options i/ii;
   recommendation i (single script + `--eval_ns`).

**Status:** Phase 0 design decisions (D1, D3, D6) open — awaiting user before any
code. No files changed. Gate = bit-identical figures + same north-star verdict
(refactor, not correctness).

## 2026-07-24 — Phase 1 (configs) + Phase 2 (script rename/delete) landed

User instruction ("delete relevant files, rename relevant files") = finish the
file-level ops of Phase 2 on top of the already-staged Phase 1 config work.

**Phase 1 (configs) — already staged + green:**
- `git mv experiment_config.json → paper_simulation_config.json`;
  `evaluation_suite_config.json → gate_eval_config.json`;
  `INCR_ET_Profiling.json → incr_et_profiling.json` (case-normalized).
- `git rm` the three redundant configs (`simulation_only_config.json`,
  `n4_perf_test.json`, `bf_n8_verify_config.json`).
- `experiment_config_loader.py` `DEFAULT_CONFIG_PATH` + docstrings updated;
  Python help/docstring refs in `interval_sweep.py` / `aggregate_across_tasks.py`
  / `run_end_to_end_experiments.py` / `plotting_config.py` updated.
- **D1 RESOLVED = β** (keep dedicated eval config, renamed — NOT folded into
  `paper_simulation_config.json`): `gate_eval_config.json` carries the gate
  10-scheduler set + `eval_*` keys the suite needs; folding them would make
  paper-figure runs also run the 10-scheduler gate set. Pinned by
  `TestGateEvalConfigSurvives` in `tests/python/test_experiment_config_loader.py`.
- Stale `experiment_config.json` reference FAILS LOUDLY (no silent alias) —
  pinned by `TestStaleNameFailsLoudly`.

**Phase 2 (scripts) — done this session:**
- `git mv run_end_to_end.sh → run_simulation_and_plot_figures.sh` (D4).
- `git mv run_evaluation_suite.sh → run_simulation_plot_eval_ns.sh` (D4).
- `git rm run_interval_sweep.sh` (D3 = resolution b — subsumed by the pipeline's
  sweep stage; the sweep is now reachable only via the pipeline).
- Cross-references updated to the renamed scripts:
  - `run_simulation_plot_eval_ns.sh` delegates to `run_simulation_and_plot_figures.sh`
    (E2E_WRAPPER + header line + usage block + env-var doc).
  - `run_simulation_and_plot_figures.sh` usage examples → new self-name.
  - `run_paper_figures.sh` comment: stale `run_simulation.sh` + `run_interval_sweep.sh`
    refs → `run_simulation_and_plot_figures.sh`; Fig 2 note → "pipeline's sweep
    stage" (D2 cleanup).
  - Python user-facing messages/docstrings: `evaluation_suite.py` (4 sites),
    `aggregate_across_tasks.py` (1 site).
  - Config `_comment` strings in `gate_eval_config.json` + `incr_et_profiling.json`
    (Use: lines + "mirroring …" refs).
  - Sibling tasks: `P0_3/goal.md` + `tasks.md` (run command + Files list),
    `P2_1/goal.md` + `tasks.md` (sweep now via the pipeline), `P1.19` rerun-mode
    test docstring.
- `lib/common.sh` unchanged (sourced helper, name-neutral).

**D6 DEFERRED:** the two top-level scripts were NOT merged into one
`--eval_ns` script; the eval script delegates to the renamed pipeline instead.
Merge / `lib/common.sh` factoring left for a later pass (the user's "delete +
rename" instruction was the file-level op, not the merge).

**Phase 2 stretch DEFERRED:** `run_paper_figures.sh` not yet extended to render
Fig 2 (Fig 2 is the pipeline's sweep-stage output, not a figures-only step —
see goal.md). Goal (a) "one script for ALL figures" still has that gap.

**Verify (this session):**
- `bash -n` on all 3 scripts: SYNTAX_OK.
- `json.load` on all 3 configs: valid JSON.
- `pytest tests/python/test_experiment_config_loader.py
  test_run_end_to_end_rerun_mode.py test_run_end_to_end.py` → **40 passed**.
- `DRY_RUN=1` both renamed scripts: pipeline resolves
  `paper_simulation_config.json`; eval script resolves `gate_eval_config.json`
  AND delegates to `run_simulation_and_plot_figures.sh`.

**NOT done (out of scope for the rename/delete pass):**
- bit-identical figure re-run (semantics unchanged by a pure rename; TDD pins
  layout + defaults, not figure bytes).
- north-star verdict re-run (deferred with D6).
- `agents/overall_tasks.md` + top-level `dev_log.md` rename-map update (Phase 3
  last item — pending).

**Gate status:** rename/delete is functionally complete and green; D6 + Fig 2
extension + the two Phase-3 "re-run" items remain.

## 2026-07-24 — D1 RE-RESOLVED β→α (fold gate_eval into paper config) + D6 resolved

User: "make `run_evaluation_suite.sh` import code from `run_end_to_end`...
reduce repeated code, make the 2 share the same input config json. after it,
clear the config json that's no longer used." → this is D6 (dedup the scripts)
PLUS a reversal of D1 (which β had kept `gate_eval_config.json` separate
*because* folding is result-changing).

**Surfaced before coding (the gap in "IIUC eval = pipeline + north-star"):**
the two configs are NOT near-clones. They differ in (1) scheduler set
(paper: 4 main `INCR_Reopt_10/BF/RM/CFS`; gate: 8 main incl. 5 `INCR_Reopt_X`
period arms the E3 gate needs), (2) task-count list, (3) `run_name_prefix`
(`""` vs `evalsuite`). The eval suite does NOT simulate — it reads whatever
the pipeline simulated. So merging to ONE config = picking ONE scheduler set.
Asked the user; they picked **"paper set; drop gate_eval"** (D1=α): single
config = `paper_simulation_config.json`; gate eval runs against the 6-scheduler
paper set; E3 loses its period-arm signal.

**E3 consequence (traced, non-fatal):** `evaluate_e3` → `_e3_at_n` needs ≥2
`INCR_Reopt_X` arms. test_mode simulates only `INCR_Reopt_10` → E3 reports
MISSING at every N (not fatal; Q1/Q2/Q3/E1 still evaluate on BF/INCR_Reopt_10/
CFS/RM). prod_mode's ablation list carries `INCR_Reopt_1/_10/_30/_60` (missing
`_5`) → E3 evaluates over those four arms in prod. No crash; suite emits a
verdict. (Earlier this session a concurrent edit to `paper_simulation_config.json`
added `INCR_Reopt_1/_30/_60` to the prod ablation list — preserved; the
stale-file guard caught the divergence before a block-replace clobbered it.)

**D6 resolved (the script dedup, regardless of the config answer):** the eval
script ALREADY delegated the pipeline stage to `run_simulation_and_plot_figures.sh`
(lines 76-87). The duplicated code was its OWN build block (old lines 67-73),
which double-built (eval builds, then the pipeline it calls builds again).
REMOVED that build block. Eval script is now: header → validate config →
delegate pipeline (builds+simulates+aggregates) → `evaluation_suite.py` →
verdict. `SKIP_PIPELINE=1` (eval-only) needs no build. NOT merged into one
`--eval_ns` script — the dedup (share the pipeline's build) IS the merge.

**Fold done (D1=α):**
- Added `eval_quality_task_counts` / `eval_overhead_task_count` / `eval_period_arms`
  to `paper_simulation_config.json` test+prod mode (faithful verbatim values from
  the deleted gate config, so the gate behaves the same modulo the scheduler set).
- `git rm gate_eval_config.json`. Stale name FAILS LOUDLY (pinned).
- `evaluation_suite.py` `--config_json` default + docstring usage →
  `paper_simulation_config.json` (was `gate_eval_config.json`).
- `run_simulation_plot_eval_ns.sh` `CONFIG_JSON` default →
  `paper_simulation_config.json`; header/comment text updated.
- `incr_et_profiling.json` `_comment` cross-ref updated
  (`gate_eval_config.json` → "single config" note).

**Verify (this session):**
- `bash -n` both scripts: SYNTAX_OK. `json.load` both surviving configs: valid.
- `pytest tests/python/test_experiment_config_loader.py
  test_run_end_to_end_rerun_mode.py test_run_end_to_end.py` → **43 passed**
  (config-loader now 12/12: added `test_folded_gate_eval_config_does_not_exist` +
  `test_loading_via_folded_gate_eval_name_raises` + reframed
  `TestGateEvalConfigSurvives` → `TestPaperConfigCarriesEvalKeys` + a
  `test_paper_config_scheduler_set_is_the_paper_set` consequence-pin).
- `DRY_RUN=1` eval script (both paths): default config resolves to
  `paper_simulation_config.json`; pipeline delegation intact; SKIP_PIPELINE=1
  path no longer builds.

**Remaining DEFERRED:** Fig 2 extension in `run_paper_figures.sh` (goal a);
bit-identical figure re-run (semantics unchanged by pure rename/dedup; TDD pins
layout+defaults not figure bytes); the prod gate run is now heavier than the old
dedicated gate config (paper sizing: 8 N × 6 sched × 10 tasksets vs old 4 N ×
10 sched × 5 tasksets) — a result-changing consequence of sharing the paper
config, accepted by the user's choice.

