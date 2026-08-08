# P1.26 — Tasks (working checklist)

> See `goal.md` for scope. Flag is already implemented (uncommitted). This
> checklist = verify byte-identity, run the A/B, report.

## Step 0 — Verify byte-identity at variant 0
- [ ] Confirm `pa_research_heuristic_variant: 0` in `sources/parameters.yaml`.
- [ ] Build test (`build_test` DEBUG) + `cmake --build build_test --target
      check.SP_OPT -j5` green. (RELEASE `build` if speed-test needed.)
- [ ] Confirm no behavior change vs HEAD at flag=0 (the switch `default:`
      returns `Decrease` = original case-3 behavior).

## Step 1 — Baseline A/B (flag=0)
- [ ] `export CONFIG_JSON=simulation_experiments/configs/pa_heuristic_ab.json
      MODE=test SKIP_EVAL=1`; `RERUN_MODE=clear_all ./scripts/run_simulation_plot_eval_ns.sh`
      (or `bash _ab_run.sh baseline`).
- [ ] Snapshot baseline `comparison_summary.csv` + per-taskset
      `interval_sp_metrics.txt` to a `baseline/` copy (variant run overwrites
      the same run dir).
- [ ] Record baseline `Mean_SP_Metric` for `INCR_Reopt_10` at N=8 and N=10 in
      `dev_log.md`.

## Step 2 — Variant A/B (flag=1, remove case 3)
- [ ] Flip `pa_research_heuristic_variant: 0 → 1` in `sources/parameters.yaml`.
- [ ] Rebuild (flag is read at runtime from yaml — confirm whether a rebuild is
      needed; if `GlobalVariables` is populated from yaml at startup, no rebuild).
- [ ] `RERUN_MODE=clear_results` run (reuses tasksets, reruns sim).
- [ ] Snapshot variant summary; record variant `Mean_SP_Metric` at N=8/N=10.

## Step 3 — Compare + report
- [ ] Diff baseline vs variant `Mean_SP_Metric` (and per-taskset SP) at N=8/N=10.
- [ ] Verdict: variant 1 improves / matches / hurts SP. Record in `dev_log.md`
      + report to user.

## Step 4 — (conditional) flag=2 OpenToAll
- [ ] Only if step 3 signal warrants: repeat with flag=2, same procedure.

## Step 5 — Decision + paper
- [ ] **If variant wins:** update paper §6.3 `section_increment_pa` 4-item
      enumerate to match; propose flipping flag default (user-go).
- [ ] **If baseline wins/ties:** record negative result, keep rule, close task.
- [ ] Restore `pa_research_heuristic_variant: 0` (prod default).
- [ ] Milestone to top-level `agents/dev_log.md`.

## Standing constraints
- No `git commit` (user's task; `git add` only).
- Prod stays byte-identical (flag=0) until the A/B converges.
- `MODE=test` for the exploration A/B (prod A/B is the user's call).
