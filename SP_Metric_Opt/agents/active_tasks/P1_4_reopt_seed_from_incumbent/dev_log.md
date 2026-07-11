# P1.4 — Reopt Seed TL: Carried Incumbent, Permanently — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task created from the user's directive: the reopt descent's seed TL must be
  **algorithm-derived** (the optimizer's own prior result), not a value read
  from the YAML taskset or the generator's drawn distribution. The user
  accepted the "previous found Time limit config" option permanently, with the
  measured ~0.3–0.8 SP pt cost at high reopt frequency (P1.3 A/B).
- **Finding recorded in `goal.md`:** `InitializeTimeLimitsFromETConfig` (the
  current "off"/default reopt seed) is YAML-derived — `et_dist_.GetAvgValue()`
  resolves through the `FiniteDist` built from YAML `mu/sigma/min/max`
  (`RegularTasks.cpp:61-106`), and the generator writes `mu` independent of the
  `performance_records_time` option grid
  (`taskset_generator.py:244-245` / `orchestrator.py:309`). So the current seed
  is a derived function of YAML content, not an algorithm output. Disqualified
  by the new constraint.
- **Closest algorithmic option = `ReconstructTimeLimitsFromResOpt`** — the
  carried incumbent in `res_opt_` (the optimizer's own `CommitIncumbent` write
  from the P0.5 redesign). Already implemented behind
  `GlobalVariables::ReoptStartFromAdoptedTL` (commit `8cbbbc12`), already gated
  by `IfInitialized()` (auto-fallback at interval 0 / INCR_SCRATCH), already
  A/B-measured (P1.3). Making it permanent = flipping the default to true.
- **Design decision: choice (a)** — keep the knob, default true. Minimal blast
  radius; `_ADOPTED` arms become no-op aliases; the flag is a documented
  opt-out for ablation. Chosen over (b) (remove the knob + arms) for the
  smaller, reviewable diff and to preserve A/B comparability with historical
  run dirs. If the user prefers (b), revisit before Step 4.
- Next: TDD — rewrite the 3 `ReOptimizePeriodic_*` tests for the new default,
  confirm they fail, then flip the default in `Parameters.cpp`.

- **Implemented choice (a)** (keep the knob, default true) — flipped
  `ReoptStartFromAdoptedTL` `false`→`true` in `Parameters.cpp:20`; rewrote the 3
  `ReOptimizePeriodic_*` tests TDD red→green (49/49 + 16/16 ctest); updated all
  in-source comments + config + memory to call `_ADOPTED` arms no-op aliases.
  Staged (17 files). Handed to user.

- **User redirected to choice (b):** "always use reopt-start-from-adopted-tl,
  remove the flag, clean related choice code and unused code." The design rules
  ("don't make things optional if not needed" / "ruthlessly prune unused
  features") favor (b); the A/B data is no longer needed since the policy is
  permanent, so the knob + the `_ADOPTED` arms are dead weight.

- **Choice (b) executed:**
  - Removed `GlobalVariables::ReoptStartFromAdoptedTL` entirely from
    `Parameters.h/.cpp` + the `parameters.yaml` note block.
  - `OptimizeSP_TL_Incre.cpp:478`: the flag ternary → unconditional
    `IfInitialized() ? ReconstructTimeLimitsFromResOpt() :
    InitializeTimeLimitsFromETConfig()`. The `IfInitialized()` gate still
    auto-falls-back at interval 0 / INCR_SCRATCH (irreducible).
  - `RunOrchestrator.cpp` `MaybeOverrideReoptPeriod`: dropped the `_ADOPTED`
    suffix handling; a trailing suffix after the digits is now a **hard error**
    (loud stderr, no flag set) — so a stale `_ADOPTED` config can't silently
    dispatch to an empty `ResourceOptResult` (the P1.3 trap). `--help` Usage
    line updated.
  - `SimulationOrchestrator.cpp` `IsINCRPeriodVariant`: simplified to plain
    `INCR_P<n>` (no trailing suffix); history comment added.
  - `p25_period_ab_config.json`: removed the 4 `_ADOPTED` arms from both modes
    (10→6 arms each); rewrote `_comment` + both mode `_comment`s.
  - Tests: renamed `StartsFromAdoptedTLByDefault` → `StartsFromAdoptedTL`
    (dropped the `ASSERT_TRUE(flag)`); **deleted**
    `OptOutStartsFromGaussianMeanTL` (the false path is gone); stripped the
    flag mention from `Interval0FallsBackToGaussianMean` (logic unchanged). Net
    49 → 48 tests.

- **Verified:** `cmake --build build --target check.SP_OPT -j5` (DEBUG) → 16/16
  ctest green, `testIncreOpt_w_TL` 48/48 green. `git add` staged the (b) unit.
  No `release/` rebuild done by me — the user rebuilds + re-runs the A/B (now
  the 6-arm config) to confirm on the loaded tasksets.
