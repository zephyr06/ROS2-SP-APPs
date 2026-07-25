# P1.19 — Dev Log

## 2026-07-20 — Filed

User request: "also add a mode arg to decide whether to remove existing
generated task sets to re-run experiments, or remove only existing simulation
results and re-run results, or resume mode which doesn't re-run analysis if
something is done."

Then scope-narrowed: "we can start with adding modes about whether to clear all
simulated task sets and re-run first" → implement ONLY `clear_all` now;
`clear_results` + `resume` deferred (noted as Phase 2/3 of this same task, not
separate tasks).

**Numbering note:** P1.18 was already taken (`P1_18_classify_reuse_per_task_more_types`).
This task is filed as **P1.19** (next free number; `grep` confirmed P1.19 unused
across `agents/`).

**Scope:** Python-only. The e2e orchestrator
(`simulation_experiments/run_end_to_end_experiments.py`) + the shell wrapper
(`scripts/run_end_to_end.sh`). NOT a correctness task — changes WHEN prior
artifacts are cleared, not HOW SP is computed.

**Sibling:** P1.15 owns the harness's loud-failure/crash behavior; P1.19 does
NOT touch crash handling. P1.14 Phase 3 / P1.15 Phase 3 / P1.17 3b are the
consumers that will use `clear_all` to force fresh A/B runs.

### Phase 0 — Inventory DONE

**Existing per-stage reuse/resume guards (read, NOT modified):**
- `compare_optimizers.py`:
  - `--skip_generation_if_exists` (default True): reuses a taskset dir whose
    `generator_config.json` matches the current config.
  - `--on_taskset_config_change` (prompt/regenerate/keep): the `[Y/n]` config-
    drift prompt.
  - `--resume`: skips arms whose `interval_sp_metrics.txt` exists AND is
    non-empty (`_has_complete_metrics`, `:279`).
- The orchestrator already forwards `--resume` to `compare_optimizers` when the
  config has `analysis.enable_resume_from_existing_results: True`
  (`build_simulate_command`, `:162-163`).
- The sweep reuses the main step's tasksets via `--reuse_matching_interval`
  (`build_sweep_command`, `:198`).

**Path shape (confirmed):**
- `build_run_root(output_parent, cfg)` = `<output_parent>/runs/<run_id>`
  (`experiment_config_loader.py:133`).
- `build_run_id` = e.g. `run_test_dur70_interval10_seed1000_tasks4x6` (mode/
  dur/interval/seed/tasks-tag; excludes num_tasksets).
- Simulate writes `<run_root>/sim/<subfolder>/taskset_{idx}/<scheduler>/
  interval_sp_metrics.txt` where subfolder = `tasks{N}_dur{D}_interval{I}_seed{S}`.
- Figures under `<run_root>/figures/`.

So `clear_all` = `shutil.rmtree(<run_root>/sim/)`, run BEFORE any stage,
preempting all the per-stage guards above.

### 1a/1b/1c — DONE (clear_all plumbing)

**`simulation_experiments/run_end_to_end_experiments.py`:**
- Added `import shutil` (after `import os`).
- Added `apply_rerun_mode(rerun_mode, run_root, dry_run, verbose)` after
  `run_command()`. `reuse` → no-op; `clear_all` → `shutil.rmtree(<run_root>/sim/)`;
  respects `--dry_run` (prints intent, touches nothing) + `--verbose` (logs
  "not present" / "removing"). Unknown mode → warn + no-op (defensive; argparse
  choices prevent it). Docstring notes `clear_results` as a future addition.
- Added `--rerun_mode` CLI arg (choices `reuse`/`clear_all`, default `reuse`).
- Called `apply_rerun_mode(args.rerun_mode, run_root, args.dry_run, args.verbose)`
  after `run_root = build_run_root(...)` and BEFORE the binary pre-flight check
  (so a dry-run wipe report doesn't demand the binary exist).
- Print `Rerun: <mode>` in the header when non-default.

**`scripts/run_end_to_end.sh`:**
- Documented `RERUN_MODE` in the env-var comment block (after `CONFIG_JSON`):
  `reuse (default) | clear_all`; notes `clear_all` wipes `<run_root>/sim/`.
- Added `RERUN_MODE="${RERUN_MODE:-reuse}"` to the env-var defaults.
- Added `"Rerun mode:   ${RERUN_MODE}"` to the `print_header "End-to-End Pipeline"`.
- Forward to Python only when non-default (matching `CONFIG_JSON`/`DRY_RUN`):
  `if [[ "${RERUN_MODE}" != "reuse" ]]; then CMD+=(--rerun_mode "${RERUN_MODE}"); fi`.

**Syntax checks DONE:** `bash -n scripts/run_end_to_end.sh` + `python3 -c
"import ast; ast.parse(open('...').read())"` both SYNTAX_OK.

**Dry-run / "not present" branch DONE:** from PROJECT_ROOT,
`python3 -m simulation_experiments.run_end_to_end_experiments --rerun_mode clear_all
--output_parent /tmp/rerun_test --dry_run` printed
`[dry-run] would remove: .../sim` + `Rerun: clear_all`. And with no tree
present: `[rerun_mode=clear_all] .../sim not present; nothing to clear.`

### 1d — DONE (destructive path verified from PROJECT_ROOT)

Ran `python3 -m simulation_experiments.run_end_to_end_experiments --rerun_mode
clear_all --output_parent /tmp/rerun_test` from PROJECT_ROOT (no `cd /tmp`) —
this time the module resolved. Planted a populated tree first:
- `sim/sweep_dummy.txt`
- `sim/tasks6_dur70_interval10_seed1000/taskset_0/INCR/interval_sp_metrics.txt`
- `figures/fig1.png`

**Result:** AFTER the run, `sweep_dummy.txt` + the pre-existing
`taskset_0/INCR/interval_sp_metrics.txt` are GONE. The `sim/` tree present in
AFTER is freshly generated by the simulate stage (`tasks6_.../taskset_1/...` +
`taskset_arm_status.csv`), none of which were planted. So `clear_all` wiped
`<run_root>/sim/` then the stages regenerated everything from scratch —
exactly "clear ALL generated tasksets + re-run."

**Caveat (cosmetic, not a bug):** the orchestrator's own
`[rerun_mode=clear_all] removing .../sim` log line did NOT appear in the
captured piped output. Python's stdout is block-buffered when piped (not a
TTY), so the orchestrator's `print()`s were held in its buffer and flushed at
process exit, interleaving oddly with the subprocess stdout. The *behavior*
(wipe + regen) is proven by the filesystem state; the log line is visible on a
TTY (confirmed by the dry-run + the `Rerun: clear_all` header line, which DO
appear). The 1e TDD pin (below) calls `apply_rerun_mode` directly and captures
its stdout via `redirect_stdout`, so it locks the log-line contract
independently of pipe buffering.

### 1e — DONE (Python TDD pin)

Added `tests/python/test_run_end_to_end_rerun_mode.py` (8 tests, all GREEN):
- `test_reuse_leaves_sim_tree_intact` — `reuse` is a true no-op (planted files
  survive, no "removing" in output).
- `test_clear_all_wipes_sim_tree` — `clear_all` removes the whole `sim/` dir.
- `test_clear_all_keeps_figures_tree` — `clear_all` does NOT touch sibling
  `figures/` (figures are terminal output, not tasksets/sim results).
- `test_clear_all_missing_sim_is_noop` — `clear_all` on a run root with no
  `sim/` is a no-op, not an error ("nothing to clear").
- `test_clear_all_missing_run_root_is_noop` — `clear_all` on a non-existent
  run root is a no-op.
- `test_clear_all_announces_removal_when_verbose` — verbose≥1 logs "removing"
  + "sim".
- `test_clear_all_dry_run_touches_nothing` — `dry_run=True` prints "would
  remove" and removes nothing.
- `test_unknown_mode_warns_and_is_noop` — unknown mode warns + is a no-op
  (defensive; argparse choices prevent this in practice).

**Regression check:** existing `tests/python/test_run_end_to_end.py` still
17/17 green — the new `--rerun_mode` CLI arg + the `apply_rerun_mode` call in
`main()` did not break the mocked-subprocess e2e tests.

**Full-stack wiring check:** `bash -n scripts/run_end_to_end.sh` SYNTAX_OK;
`RERUN_MODE=clear_all DRY_RUN=1 bash scripts/run_end_to_end.sh` prints
`Rerun mode: clear_all` (shell header) → forwards `--rerun_mode clear_all` to
Python → Python prints `Rerun: clear_all` + correctly reports
`[rerun_mode=clear_all] .../sim not present; nothing to clear.` against the
real default run root. End-to-end forwarding confirmed.

### Phase 1 — DONE

`clear_all` mode is implemented, verified (destructive wipe confirmed against a
populated throwaway tree), and pinned (8 TDD tests green, existing e2e tests
green, full-stack shell→Python wiring confirmed). Working tree, NOT committed
(`git add` only per the agent rule; user commits). **Committed `8279ae0d`
("and clear_all mode to run_e2e") on 2026-07-20.**

NEXT (deferred per user scope-narrowing): Phase 2 (`clear_results` — keep
tasksets, wipe only sim outputs) + Phase 3 (`resume` — skip stages whose
outputs already exist).

## 2026-07-24 — Phase 2 (`clear_results`) + close

User: "so can we mark p1_19 as done." Verified the real state vs. the (stale)
tracking before answering:

- Phase 1 `clear_all` — committed `8279ae0d` 2026-07-20 (the tracking still
  said "working tree, NOT committed"; corrected).
- Phase 2a `clear_results` — committed `f0858eef` 2026-07-23 ("add clear_results
  mode to run_e2e"). `apply_rerun_mode` handles `reuse`/`clear_all`/
  `clear_results`; argparse choices match
  (`run_end_to_end_experiments.py:435`). The `clear_results` branch globs
  `tasks*_dur*_interval*_seed*/taskset_*/` and removes every SUBDIR
  (per-scheduler results) while keeping the flat taskset artifact files
  (generator_config.json, *.yaml, path_Et_task_*.txt) and the taskset dir
  itself. So `compare_optimizers`'s `--skip_generation_if_exists` reuses the
  tasksets while `--resume` re-runs every arm — exactly "remove only simulation
  results and re-run results."
- Phase 2b TDD pin — MISSING (8 clear_all tests, 0 clear_results tests). User
  chose "write tests, then close."

### 2b — DONE (clear_results TDD pin)

Added 6 `clear_results` tests to `tests/python/test_run_end_to_end_rerun_mode.py`:
- `test_clear_results_wipes_per_scheduler_dirs` — removes `<taskset>/INCR/`
  and the `interval_sp_metrics.txt` (the `--resume` guard's key file).
- `test_clear_results_keeps_taskset_artifacts` — keeps the flat artifacts
  (`generator_config.json`, `taskset_param.yaml`, `path_Et_task_0.txt`,
  `taskset_characteristics_0.yaml`) + the taskset dir. THIS is the contract
  that distinguishes `clear_results` from `clear_all`.
- `test_clear_results_keeps_figures_tree` — does NOT touch sibling `figures/`.
- `test_clear_results_missing_sim_is_noop` + `test_clear_results_missing_run_root_is_noop`
  — clearing never aborts the pipeline.
- `test_clear_results_dry_run_touches_nothing` — prints "would remove", removes
  nothing.

Enriched the shared `setUp` with flat taskset artifacts (the old single-file
tree had only `taskset_0/INCR/interval_sp_metrics.txt`, so it could pin "removes
results" but NOT "keeps taskset artifacts" — the whole point of `clear_results`).
The enriched tree does not affect the `clear_all` tests (they wipe everything
regardless).

**Result:** suite now 14/14 green (8 clear_all + 6 clear_results). Existing
`test_run_end_to_end.py` still 17/17 green (no regression from the `setUp`
change). Updated the module docstring (`--rerun_mode {reuse, clear_all,
clear_results}`).

### Phase 3 — DEFERRED, reframed (resume behavior already exists)

Investigated the user's hunch "resume mode is actually supported." It is — but
NOT as a `--rerun_mode` choice. Resume is a separate, older, config-gated
mechanism (see `tasks.md` 3a for the full evidence chain):
- `analysis.enable_resume_from_existing_results` (boolean, config JSON) →
  `build_simulate_command` (`run_end_to_end_experiments.py:163`) forwards
  `--resume` to `compare_optimizers`; the sweep forwards the same flag
  (`interval_sweep.py:214`).
- `compare_optimizers` skips arms whose `interval_sp_metrics.txt` exists AND
  is non-empty (`_has_complete_metrics`, `compare_optimizers.py:279`/`:665`).
- The sweep ALSO has its own reuse logic independent of `--resume`:
  `_main_dir_is_fresh` (`interval_sweep.py:101`) skips re-running
  `compare_optimizers` when a fresh `comparison_summary.csv` exists.
- The flag is `true` in 4/6 configs (`experiment_config`,
  `evaluation_suite_config`, `n4_perf_test`, `INCR_ET_Profiling`), `false` in 2
  (`simulation_only_config`, `bf_n8_verify_config`).

So Phase 3 as originally scoped would only ADD: (i) unconditional resume (a
`--rerun_mode resume` choice that ignores the config flag) and (ii) stage-level
skip for aggregate (which currently always regenerates figures — no
skip-if-figures-exist branch). Both are convenience, not correctness; the
common case (partial run, re-invoke same command) already works when the config
flag is on. Deferred unless a consumer needs unconditional resume or a
measurably costly aggregate re-run.

### Close — DONE

P1.19's requested code work (`clear_all` + `clear_results`) is committed and
now fully TDD-pinned (14/14). Phase 3 reframed + deferred with justification
(resume behavior already exists via the config-gated `--resume` flag). Task
moved `active_tasks/` → `finished_tasks/`; `overall_tasks.md` + memory updated.
`git add` only per the agent rule; user commits.
