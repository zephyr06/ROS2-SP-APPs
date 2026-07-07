# P0.1 — Tasks (working checklist)

> See `goal.md` for scope. Check items off as you complete them; append detail
> to `dev_log.md`. One commit per functional change; agent runs `git add`
> only, user commits.

## Commit group (a) — INCR_P<n> mode override
- [ ] `git add tests/RunOrchestrator.cpp` (the `MaybeOverrideReoptPeriod` diff)
- [ ] `git add simulation_experiments/configs/p25_period_ab_config.json`
- [ ] Ask user to review + commit
- [ ] Verify `ctest` 16/16 green on the committed tree

## Commit group (b) — env-dependent-ratio follow-ups
- [ ] Check for config/fixture drift after commit `4b44aabc` landed the source
- [ ] If drift found: `git add` the aligned configs/fixtures; ask user to commit
- [ ] If no drift: record "no follow-up needed" in `dev_log.md`

## Commit group (c) — trial-and-error + patience TL rewrite
- [ ] `git add sources/parameters.yaml` (patience params; radii lowered)
- [ ] `git add sources/Utils/Parameters.{h,cpp}`
- [ ] `git add sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`
- [ ] Confirm `testIncreOpt_w_TL` 43/43 + `make check.SP_OPT -j5` green
- [ ] Ask user to review + commit
- [ ] Update `finished_tasks/summary.md`: flip trial_and_error note to "committed @ <sha>"

## Commit group (d) — agent docs
- [ ] `git add` the reorg (if not already committed as its own doc-only commit)
- [ ] `git add` the 4 investigation/active-task docs
- [ ] Ask user to review + commit

## Done when
- [ ] `git status` clean
- [ ] `ctest` 16/16 + `pytest tests/python` green on committed tree
