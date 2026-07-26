# P2.14 — Removal of SP_THRESHOLDS_SET & SP_THRESHOLD_RANGE Harmonization — Dev Log

> Detailed working log for this task. Append chronological entries below.

## 2026-07-25

- **Task FILED (Promoted from user review of P2.13 plan).** User directed: *"add a new task p2_14 instead of keeping records in 2.13"*.
- **Motivation:** `SP_THRESHOLDS_SET` and `SP_THRESHOLD_RANGE` coexisted redundantly in `taskset_generator.py`. `SP_THRESHOLDS_SET` took priority whenever present, completely bypassing `SP_THRESHOLD_RANGE`. Furthermore, `SP_THRESHOLDS_SET` contained `1.0`, treating 100% DDL miss as safe.
- **Action:** Completely remove `SP_THRESHOLDS_SET` and harmonize `sp_threshold` generation to continuous sampling from `SP_THRESHOLD_RANGE: [0.001, 0.9]`.

## 2026-07-25 — CLOSED (code-complete; Step 5 deferred to P2.13)

- **Disposition: closed as code-complete.** Steps 1–4 (code + configs + tests +
  verification) committed at `5c782bad`. Generator-only change: `SP_THRESHOLDS_SET`
  removed from `REQUIRED_CONFIG_PARAMS`; `sp_threshold` now sampled uniformly from
  `SP_THRESHOLD_RANGE: [0.001, 0.9]` — the `1.0` "100% DDL miss is safe" absurdity is
  gone. `SP_THRESHOLD_RANGE` is now the sole knob.
- **Step 5 (empirical `ddl_miss_chance` vs `sp_threshold` safety analysis) DEFERRED
  to P2.13.** That study is owned by P2.13 (important-task DDL vs SP metric); P2.14's
  Step 5 and P2.13's empirical deliverable are the same work, so doing it under P2.14
  would duplicate. Marked `[ ]`→`[-]` DEFERRED in `tasks.md`.
- **Records:** `goal.md` + this `dev_log.md` were untracked through the `5c782bad`
  commit (only `tasks.md` rode along); both are `git add`-ed as part of this move.
- **Moved to `agents/finished_tasks/P2_14_sp_threshold_set_removal/`.**
