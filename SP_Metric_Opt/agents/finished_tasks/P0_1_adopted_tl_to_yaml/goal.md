# P0.1 — Persist adopted TL to YAML — RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded)

> **RESOLVED — no code was ever written for this task.** The functional bug
> this task was filed to fix is **resolved by P0.5 by construction**: P0.5's
> `BuildChallengerFromIncumbent` reconstructs the adopted-TL DAG from `res_opt_`,
> so both diff sides of `FindTaskWithDifferentEt` carry the adopted TL → the
> false-positive class is structurally impossible (runtime-confirmed: P1.1 probe
> `ndiff` 5→0). The demoted *inspectability* remainder — overwrite the taskset
> YAML with the adopted TL after each commit, for post-run debuggability — was
> **never implemented** and is **discarded** by user decision (2026-07-10: "if
> it's about writing down to yaml file about the found time limits from
> optimizers, we can discard it"): the orchestrator clamps job ET to
> `res.id2time_limit` (`SimulationOrchestrator.cpp:461-463`), so no scheduler
> decision honors the stale Gaussian once optimization has run. Below is the
> original (now-superseded) plan, kept for the rationale trail.
>
> **Authoritative resolution record:** `agents/finished_tasks/summary.md`;
> rationale in memory `p05-subsumes-tl-init-bug.md`.

---

# P0.1 — Persist adopted TL to YAML (close the "ET comes from YAML, not last opt" bug)

**Priority:** P0 (root-cause fix for the P1.1 residual; user: "that is a bug ... focus on fixing this issue first")
**Status:** RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded — no code written)
**Predecessor investigation:** `agents/active_tasks/P1_1_p25_residual_investigation/` (esp. `investigation_summary.md`, `dev_log.md` YARDSTICK CORRECTION section)

## The bug

A TL-optimizable task's effective ET during interval N−1 IS the TL the optimizer
adopted that interval — **not** the YAML Gaussian. The YAML `execution_time_dist` /
`performance_records_time` were generated *without* optimization results, so they
are a cold-start reference only, vestigial once a TL is applied
(`UpdateExtDistBasedOnTimeLimit` replaces `execution_time_dist` with a point dist
at the adopted TL).

The runtime currently treats the YAML Gaussian as the prior. Two concrete harms
(P1.1 findings):

1. **`FindTaskWithDifferentEt` false positives.** `OptimizeIncre_w_TL`
   (`OptimizeSP_TL_Incre.cpp:387`) was cold-starting the descent from the
   Gaussian-mean TL (`InitializeTimeLimitsFromETConfig()`), so the update side of
   the diff was point dists at the Gaussian-mean TL while the baseline side
   carried the adopted TL → every perf-pair task whose `adopted-TL ≠ Gaussian-mean-TL`
   got false-flagged as "changed," spawning redundant `ObtainSP_DAG` variations.
   (A descent-start-TL fix was TDD-applied in P1.1 — start from
   `ReconstructTimeLimitVecFromResOpt()` + option-set guard — but it is the
   *workaround*; the *root cause* is that the prior is not persisted where the
   cold start reads it.)
2. **Reopt-path cold start.** `ReOptimizePeriodic` (`OptimizeSP_TL_Incre.cpp:535`)
   *also* starts its from-scratch descent from `InitializeTimeLimitsFromETConfig()`
   — the Gaussian-mean TL, not the carried adopted TL. That path does not feed a
   diff (`OptimizeFromScratch` does not call `FindTaskWithDifferentEt`), so it is a
   search-quality concern rather than a false-positive source — but it is the same
   bug, same lever.

## The chosen fix — overwrite the YAML

Per user (2026-07-08): for **ease of debug**, persist the adopted TL back into the
taskset YAML so that the file *is* the source of truth for the prior ET. That
removes the "the YAML lies about the prior" footgun at its root: every reader
(`InitializeTimeLimitsFromETConfig`, `FindTaskWithDifferentEt`, the reopt cold
start, the orchestrator's `setExecutionTime(GetAvgValue())`) then sees the
adopted-TL-derived ET directly, with no special-case "read from `res_opt_`
instead" plumbing.

Concretely: after each interval's optimization commits its adopted TL config
(`UpdateRecords` → `res_opt_.id2time_limit`), write the adopted TL (and the
ET it implies) back to the taskset YAML the run loaded from, so the next interval
reads the carried-forward state from the file. This makes the on-disk taskset a
faithful record of the running system's prior — inspectable, diffable, and the
single source the cold-start paths already read.

> **Scope guard.** This is the *root-cause* fix. The P1.1 descent-start-TL
> workaround (start from `ReconstructTimeLimitVecFromResOpt()`) should be
> RE-EVALUATED once this lands: if the YAML now carries the adopted TL, the
> Gaussian-mean cold start may no longer be wrong, and the workaround may become
> redundant (or may still be needed for the first interval / edge cases). Do not
> assume — re-derive against the corrected file.

## Approach (TDD, per project convention)

1. **Pick the write site.** The adopted TL is finalized in `UpdateRecords`
   (`OptimizeSP_TL_Incre.cpp`, the compare-and-keep commit) and in
   `SeedStateFromIncumbent` (the baseline seed). Decide whether to write at the
   commit site only, or also at the seed. Record the decision + rationale in
   `dev_log.md`.
2. **Add a YAML writer for the adopted TL → ET.** Reuse `yaml-cpp` (already used
   to read tasksets — see `sources/TaskModel/DAG_Model.cpp`,
   `sources/TaskModel/RegularTasks.cpp`). The writer updates, in place, the
   taskset YAML the run loaded from: for each TL-optimizable task, set the ET to
   the adopted TL's implied ET (point dist via `GetUnitExecutionTimeDist(TL)`,
   matching `UpdateExtDistBasedOnTimeLimit`) so the file reflects what the runtime
   actually ran. Non-TL-optimizable tasks (TL=−1) are left as their raw Gaussian.
   Write atomically (temp file + rename) so a crash mid-write cannot corrupt the
   taskset.
3. **TDD red→green.** Add a test that loads a taskset YAML, runs one interval so
   an adopted TL is committed, reloads the YAML, and asserts the reloaded ET
   equals the adopted TL's implied ET (not the original Gaussian mean). Fails
   before the writer; passes after.
4. **Re-run the P1.1 probe.** INCR_P10, N=8 `taskset_0`, interval 0→1. Confirm
   `ndiff` drops toward the corrected ground truth of 2 (only gaussian-only
   tasks flag). Compare against the P1.1 workaround's behavior; record whether the
   workaround is still needed.
5. **Full suite.** `testIncreOpt_w_TL` + `ctest` green. Note any stale test
   expectations that calibrated to the Gaussian-mean-prior and update them
   (structural invariants, not start-TL-specific counts — see the P1.1 precedent
   for `PerformCoordinateDescent_SkipsMinusOneOnlyTaskInMixedSet`).

## Files (expected to touch)

- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` — write site (UpdateRecords /
  SeedStateFromIncumbent) + possibly the cold-start readers.
- `sources/Optimization/OptimizeSP_Base.{h,cpp}` — the YAML writer helper
  (`WriteTimeLimitToYamlOSM` at `OptimizeSP_Base.cpp:175` is the existing precedent
  for writing TL to YAML; generalize / add a sibling for the taskset file).
- `sources/TaskModel/DAG_Model.cpp`, `sources/TaskModel/RegularTasks.cpp` — the
  YAML read paths the writer must round-trip against.
- `tests/testIncreOpt_w_TL.cpp` — the new TDD test + any stale-expectation updates.

## Done when

- Adopted TL (and its implied ET) is persisted to the taskset YAML after each
  interval's commit, atomically.
- New TDD test passes (reloaded YAML ET == adopted-TL-implied ET).
- P1.1 probe re-run: `ndiff` reconciled with ground truth (recorded in
  `dev_log.md`).
- `testIncreOpt_w_TL` + `ctest` green.
- Decision recorded: keep or remove the P1.1 descent-start-TL workaround.
- Milestone appended to top-level `agents/dev_log.md`.

## Out of scope

- Fix C (per-variation `ObtainSP_DAG` scoring) — wrong lever, per P1.1.
- Fix D (`GetAvgValue` band / `FiniteDist::approx_equal` tolerance) — inert today
  (zero production callers; `operator!=` hardcodes `tolerance=1e-1` and does not
  delegate to `approx_equal`), and solves the wrong problem regardless.
- Refactor of `OptimizePA_Incre_with_TimeLimits` data members — that is
  **P0.5** (optimizer iteration redesign, full scope = flow + state; absorbs the
  former P0.1b). See
  [`../P0_5_optimizer_iteration_redesign/goal.md`](../P0_5_optimizer_iteration_redesign/goal.md).
  P0.1 is the prerequisite behavior fix; P0.5 re-derives the flow against the
  corrected prior.
