# P2.8 — Tasks (working checklist)

> See `goal.md` for scope + open design questions D1–D6. **Do NOT start
> implementation until D1, D3, D6 are resolved with the user.** This is a
> refactor; gate = bit-identical figures + same north-star verdict.

## Phase 0 — design decisions (user)
- [x] D1: resolve eval-suite config coupling (α fold-in / β keep dedicated / γ flag-gate) — RESOLVED β (keep dedicated eval config, renamed `gate_eval_config.json`; carries gate scheduler set + `eval_*`)
- [x] D3: resolve thin-script disposition (keep both / delete interval_sweep, keep+extend figures) — RESOLVED (b): deleted `run_interval_sweep.sh`; Fig 2 extension DEFERRED
- [ ] D6: resolve merge shape (i single script + `--eval_ns` / ii factor into `lib/common.sh`) — DEFERRED (two scripts kept; eval delegates to renamed pipeline)
- [x] Confirm D4 renames + D5 ripple scope

## Phase 1 — configs
- [x] TDD red: a test that `experiment_config_loader` defaults to `paper_simulation_config.json` (fails before rename)
- [x] `git mv experiment_config.json paper_simulation_config.json`
- [x] Update `experiment_config_loader.py:24` `DEFAULT_CONFIG_PATH` + docstring (`:8`, `:37`)
- [x] Update help/docstring refs: `interval_sweep.py:369-370`, `aggregate_across_tasks.py:935-936`, `run_end_to_end_experiments.py:413-414`
- [x] Stale `experiment_config.json` reference FAILS LOUDLY (not a silent alias)
- [x] D1 follow-through: KEEP dedicated eval config renamed `gate_eval_config.json` (resolution β — it carries the gate scheduler set + `eval_*` keys the suite needs; folding them into `paper_simulation_config.json` would make paper runs run the 10-scheduler gate set). Pinned by `TestGateEvalConfigSurvives`.
- [x] `git rm simulation_only_config.json` (documented use already broken — D2)
- [x] Keep `INCR_ET_Profiling.json` (renamed `incr_et_profiling.json`); its `_comment` already describes INCR-ET profiling (the stale "P0.4 evaluation suite" copy-paste was fixed in the working tree earlier)
- [x] Update `P0_3/tasks.md:14` config path reference

## Phase 2 — scripts
- [x] TDD red: `--dry_run` of the renamed pipeline script prints the renamed script path + new config name
- [x] `git mv run_end_to_end.sh run_simulation_and_plot_figures.sh` (D4)
- [x] `git mv run_evaluation_suite.sh run_simulation_plot_eval_ns.sh` (D4)
- [ ] D6 follow-through: merge eval stage behind `--eval_ns` (per resolution) OR factor shared build+delegate into `lib/common.sh` — DEFERRED (D6 unresolved; two scripts kept, eval delegates to renamed pipeline)
- [x] D3 follow-through: delete `run_interval_sweep.sh` (resolution b); extend `run_paper_figures.sh` to render ALL paper figures incl. Fig 2 — DELETE done; Fig 2 extension DEFERRED (Fig 2 is the pipeline's sweep stage, not a figures-only step — see goal.md note)
- [ ] Rename/repurpose the figures-only script as the "all paper figures" entry point (goal a) — DEFERRED (needs Fig 2 extension; out of scope for the pure rename/delete pass)
- [x] Clean stale `--steps` / `run_simulation.sh` comments in the thin scripts (D2)
- [x] Update all in-script + cross-script references to the renamed scripts (eval suite delegates to renamed pipeline; run_paper_figures + P0.3 + P2.1 + Python messages + config `_comment`s updated)

## Phase 3 — verify
- [x] `--dry_run` of renamed pipeline script prints correct commands (both scripts dry-run OK)
- [ ] Pipeline (test mode) produces bit-identical figures to pre-rename run — NOT RUN (refactor = rename only; semantics unchanged, TDD pins layout + defaults)
- [ ] North-star suite (`--eval_ns` or renamed eval script) emits same verdict for unchanged config — NOT RUN (deferred with D6)
- [x] Stale `experiment_config.json` / `run_end_to_end.sh` references fail loudly (not silent) — pinned by `test_experiment_config_loader.py` 9/9
- [ ] Update `agents/overall_tasks.md` + top-level `dev_log.md` with the rename map
