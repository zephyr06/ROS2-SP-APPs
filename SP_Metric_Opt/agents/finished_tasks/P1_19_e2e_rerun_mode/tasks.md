# P1.19 — Tasks (working checklist)

> Python-only. The only acceptance gate is behavior: the three modes do what
> their name says, `reuse` is a true no-op (bare `./run_end_to_end.sh`
> unchanged), and no stage silently regenerates when the user asked to reuse.
> TDD-first where a behavior is non-trivial; the `clear_all` destructive path
> MUST be pinned by a throwaway-tree test before it is declared done.
> Per the agent rule: `git add` only; the user commits.

## Phase 0 — Inventory (no code changes)

- [x] **0a. Confirm the existing per-stage reuse/resume guards** in
  `compare_optimizers.py` (`--skip_generation_if_exists`, `--on_taskset_config_change`,
  `--resume` + `_has_complete_metrics`) and how the orchestrator already threads
  `--resume`. — DONE 2026-07-20: documented in `dev_log.md` Phase 0. The
  orchestrator forwards `--resume` when `analysis.enable_resume_from_existing_results`
  is True; `compare_optimizers` reuses a taskset whose `generator_config.json`
  matches and skips arms whose `interval_sp_metrics.txt` exists + is non-empty.
- [x] **0b. Confirm `build_run_root` / `build_run_id` path shape** so the
  rerun policy targets the right tree. — DONE 2026-07-20: `run_root =
  <output_parent>/runs/<run_id>`; sim under `<run_root>/sim/`; figures under
  `<run_root>/figures/`. `clear_all` wipes `<run_root>/sim/`.

## Phase 1 — `clear_all` mode (clear ALL generated tasksets + re-run)

- [x] **1a. Add `apply_rerun_mode()` to the Python orchestrator.** `reuse` =
  no-op; `clear_all` = `shutil.rmtree(<run_root>/sim/)`. Runs BEFORE any stage,
  respects `--dry_run` + `--verbose`. Docstring notes `clear_results` as a
  future addition. — DONE 2026-07-20.
- [x] **1b. Add `--rerun_mode` CLI arg** (choices `reuse`/`clear_all`,
  default `reuse`) + thread `args.rerun_mode` into `apply_rerun_mode` after
  `run_root` is built. Print `Rerun: <mode>` in the header when non-default.
  — DONE 2026-07-20.
- [x] **1c. Forward `RERUN_MODE` env var from `run_end_to_end.sh`** (default
  `reuse`; forwarded to Python only when non-default, matching the
  `CONFIG_JSON`/`DRY_RUN` pattern). Document it in the env-var comment block +
  the `print_header` call. — DONE 2026-07-20.
- [x] **1d. Verify the destructive path** against a throwaway tree under
  `/tmp`: populate `<run_root>/sim/` with a fake taskset + sweep artifact, run
  `--rerun_mode clear_all`, confirm `shutil.rmtree` wiped the sim tree (run
  from PROJECT_ROOT so the module resolves — the earlier `cd /tmp` attempt
  broke `sys.path`). Then `rm -rf /tmp/rerun_test`. — DONE 2026-07-20: ran
  `--rerun_mode clear_all --output_parent /tmp/rerun_test` from PROJECT_ROOT
  against a populated tree (planted `sim/sweep_dummy.txt` +
  `sim/tasks6_.../taskset_0/INCR/interval_sp_metrics.txt` + `figures/fig1.png`).
  AFTER: `sweep_dummy.txt` + the pre-existing `taskset_0/INCR/...` are GONE
  (the simulate stage then regenerated a fresh `sim/` tree — `taskset_1/...` +
  `taskset_arm_status.csv`, none of which were planted). Wipe confirmed.
  (Caveat recorded: the orchestrator's own `[rerun_mode=clear_all] removing`
  log line didn't appear in the piped output due to Python stdout
  block-buffering under a pipe — cosmetic, not a bug; visible on a TTY and in
  the dry-run path. The 1e TDD pin below locks the behavior independent of
  buffering.)
- [x] **1e. TDD pin (Python).** Add `tests/python/test_run_end_to_end_rerun_mode.py`:
  for each mode, build a throwaway `run_root` with a fake `sim/` tree, call
  `apply_rerun_mode(mode, run_root, dry_run=False, verbose=0)`, assert
  `reuse` leaves the tree intact, `clear_all` removes `sim/` (and only `sim/`,
  not sibling `figures/`), and an unknown mode warns + is a no-op. Add a
  dry-run assertion (nothing removed, message printed). — DONE 2026-07-20:
  8 tests, all GREEN (`test_reuse_leaves_sim_tree_intact`,
  `test_clear_all_wipes_sim_tree`, `test_clear_all_keeps_figures_tree`,
  `test_clear_all_missing_sim_is_noop`, `test_clear_all_missing_run_root_is_noop`,
  `test_clear_all_announces_removal_when_verbose`,
  `test_clear_all_dry_run_touches_nothing`, `test_unknown_mode_warns_and_is_noop`).
  Existing `test_run_end_to_end.py` still 17/17 green (no regression from the
  new `--rerun_mode` arg / `apply_rerun_mode` call).

## Phase 2 — `clear_results` mode (clear ONLY sim results, keep tasksets)

- [x] **2a. Extend `--rerun_mode` choices** with `clear_results`. In
  `apply_rerun_mode`, walk `<run_root>/sim/` and remove only the per-scheduler
  result artifacts (`<subfolder>/taskset_*/<scheduler>/interval_sp_metrics.txt`
  and the sched dir), NOT the generated taskset dirs (the `taskset_*/` dirs +
  their `generator_config.json` + task YAMLs). This lets `compare_optimizers`'s
  `--skip_generation_if_exists` reuse the tasksets while `--resume` re-runs
  every arm. — DONE 2026-07-23, committed `f0858eef`.
- [x] **2b. TDD pin.** Throwaway tree with taskset dirs + results; assert
  `clear_results` removes results, keeps taskset dirs. — DONE 2026-07-24:
  added 6 `clear_results` tests to `tests/python/test_run_end_to_end_rerun_mode.py`
  (`test_clear_results_wipes_per_scheduler_dirs`,
  `test_clear_results_keeps_taskset_artifacts`,
  `test_clear_results_keeps_figures_tree`,
  `test_clear_results_missing_sim_is_noop`,
  `test_clear_results_missing_run_root_is_noop`,
  `test_clear_results_dry_run_touches_nothing`). Suite now 14/14 green; existing
  `test_run_end_to_end.py` still 17/17 green. Enriched `setUp` with flat
  taskset artifacts (`generator_config.json`, `taskset_param.yaml`,
  `path_Et_task_0.txt`, `taskset_characteristics_0.yaml`) so "keeps taskset
  artifacts" is genuinely pinned (the old single-file tree asserted only the
  metrics-file removal).

## Phase 3 — `resume` mode (don't re-run what's already done) — DEFERRED, reframed

- [ ] **3a. (REFRAMED — not started.)** The original Phase 3 was two things:
  (i) make `--resume` UNCONDITIONAL for the simulate stage via a `--rerun_mode
  resume` choice, and (ii) extend skip-if-done to sweep + aggregate (skip when
  output figures already exist). Investigation 2026-07-24 showed resume
  BEHAVIOR is already supported and config-gated — it is NOT missing:

  - `analysis.enable_resume_from_existing_results` (boolean in the experiment
    JSON) drives `build_simulate_command` (`run_end_to_end_experiments.py:163`)
    to forward `--resume` to `compare_optimizers`; the sweep forwards the same
    flag (`interval_sweep.py:214`).
  - Inside `compare_optimizers`, `--resume` skips arms whose
    `interval_sp_metrics.txt` exists AND is non-empty (`_has_complete_metrics`,
    `compare_optimizers.py:279`/`:665`).
  - The sweep ALSO has its own reuse logic independent of `--resume`:
    `_main_dir_is_fresh` (`interval_sweep.py:101`) skips re-running
    `compare_optimizers` when a fresh `comparison_summary.csv` exists.
  - `enable_resume_from_existing_results` is `true` in 4/6 configs
    (`experiment_config`, `evaluation_suite_config`, `n4_perf_test`,
    `INCR_ET_Profiling`) and `false` in 2 (`simulation_only_config`,
    `bf_n8_verify_config`).

  So what Phase 3 would ADD is: making resume unconditional (ignore the config
  flag via the `rerun_mode` choice) AND stage-level skip for aggregate (which
  currently always regenerates figures — it has no skip-if-figures-exist
  branch). Both are convenience, not correctness; the common-case resume
  (partial run, re-invoke same command) already works when the config flag is
  on. Deferred unless a consumer needs unconditional resume or a measurably
  costly aggregate re-run.
- [ ] **3b. TDD pin.** N/A until 3a is lifted (no behavior to pin yet).

## Phase 4 — Verify + close

- [x] **4a. Full smoke:** `./run_end_to_end.sh` (reuse, default) unchanged;
  `RERUN_MODE=clear_all ./run_end_to_end.sh --dry_run` prints the wipe intent;
  a real `RERUN_MODE=clear_all` test-mode run regenerates the sim tree. — DONE
  2026-07-20 (Phase 1 close). `clear_results` smoke is the 6-test TDD pin
  above (calls `apply_rerun_mode` directly against a populated throwaway tree);
  `bash -n scripts/run_end_to_end.sh` SYNTAX_OK throughout.
- [x] **4b. Update `overall_tasks.md` + memory.** Mark P1.19 done; note the
  deferred Phase 3. — DONE 2026-07-24.
