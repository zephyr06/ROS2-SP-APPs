# P2.8 — Tasks (working checklist)

> See `goal.md` for scope + open design questions D1–D6. This is a refactor; gate
> = bit-identical figures + same north-star verdict.

## Phase 0 — design decisions (user)
- [x] D1: resolve eval-suite config coupling (α fold-in / β keep dedicated / γ flag-gate) — **RE-RESOLVED α 2026-07-24** (fold `eval_*` keys into `paper_simulation_config.json`; DELETE `gate_eval_config.json`; eval-suite default → paper config). Reverses the earlier β. User picked "paper set; drop gate_eval" knowing E3 loses its period-arm signal in test_mode (only `INCR_Reopt_10` simulated → E3 MISSING, non-fatal).
- [x] D3: resolve thin-script disposition — RESOLVED (b): deleted `run_interval_sweep.sh`; Fig 2 extension DEFERRED
- [x] D6: resolve merge shape — **RESOLVED**: eval script delegates the pipeline stage to `run_simulation_and_plot_figures.sh` (already did); REMOVED the eval script's redundant own-build block (the pipeline it delegates to builds → was a double-build). NOT merged into one script; the dedup is the share.
- [x] Confirm D4 renames + D5 ripple scope

## Phase 1 — configs
- [x] TDD red: a test that `experiment_config_loader` defaults to `paper_simulation_config.json` (fails before rename)
- [x] `git mv experiment_config.json paper_simulation_config.json`
- [x] Update `experiment_config_loader.py:24` `DEFAULT_CONFIG_PATH` + docstring (`:8`, `:37`)
- [x] Update help/docstring refs: `interval_sweep.py:369-370`, `aggregate_across_tasks.py:935-936`, `run_end_to_end_experiments.py:413-414`
- [x] Stale `experiment_config.json` reference FAILS LOUDLY (not a silent alias)
- [x] D1 follow-through (α): FOLD `eval_*` keys (`eval_quality_task_counts` / `eval_overhead_task_count` / `eval_period_arms`) into `paper_simulation_config.json` test+prod mode; DELETE `gate_eval_config.json` (`git rm`); eval-suite `--config_json` default + eval-script `CONFIG_JSON` default → `paper_simulation_config.json`. Stale `gate_eval_config.json` FAILS LOUDLY (pinned by `TestStaleNameFailsLoudly.test_loading_via_folded_gate_eval_name_raises`). Pinned by `TestPaperConfigCarriesEvalKeys`.
- [x] `git rm simulation_only_config.json` (documented use already broken — D2)
- [x] Keep `incr_et_profiling.json`; `_comment` cross-refs updated (gate_eval_config ref → "single config" note)
- [x] Update `P0_3/tasks.md:14` config path reference

## Phase 2 — scripts
- [x] TDD red: `--dry_run` of the renamed pipeline script prints the renamed script path + new config name
- [x] `git mv run_end_to_end.sh run_simulation_and_plot_figures.sh` (D4)
- [x] `git mv run_evaluation_suite.sh run_simulation_plot_eval_ns.sh` (D4)
- [x] D6 follow-through: REMOVED eval script's redundant build block (lines 67-73 of the old version) — the pipeline it delegates to (`run_simulation_and_plot_figures.sh`) builds itself, so the eval script was double-building. Eval script now: header → validate config → delegate pipeline → eval suite → verdict. `SKIP_PIPELINE=1` (eval-only) needs no build.
- [x] D3 follow-through: delete `run_interval_sweep.sh` (resolution b); extend `run_paper_figures.sh` to render ALL paper figures incl. Fig 2 — DELETE done; Fig 2 extension DEFERRED
- [ ] Rename/repurpose the figures-only script as the "all paper figures" entry point (goal a) — DEFERRED (needs Fig 2 extension)
- [x] Clean stale `--steps` / `run_simulation.sh` comments in the thin scripts (D2)
- [x] Update all in-script + cross-script references to the renamed scripts

## Phase 3 — verify
- [x] `--dry_run` of renamed pipeline script + eval script prints correct commands (both dry-run OK; eval default → paper config)
- [ ] Pipeline (test mode) produces bit-identical figures to pre-rename run — NOT RUN (refactor = rename only; semantics unchanged, TDD pins layout + defaults)
- [ ] North-star suite emits same verdict for unchanged config — NOT RUN (D1=α changes the scheduler set the gate sees by design — test_mode E3 goes FAIL-by-design → MISSING; this is the accepted consequence, not a regression to re-verify)
- [x] Stale `experiment_config.json` / `gate_eval_config.json` references fail loudly (not silent) — pinned by `test_experiment_config_loader.py` 12/12
- [ ] Update `agents/overall_tasks.md` + top-level `dev_log.md` with the rename map + D1=α fold
