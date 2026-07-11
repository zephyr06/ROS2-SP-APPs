# P1.3 — `ReoptStartFromAdoptedTL` A/B Regression: Stale-Binary Root Cause

**Priority:** P1 (active investigation; blocks the P25 `_ADOPTED` A/B being trustworthy)
**Status:** CLOSED 2026-07-11. Root cause FOUND (stale binary) + Step 2 DONE
(`release/` rebuilt & verified fresh 2026-07-11 11:26; functional probe
confirmed the stale fall-through fingerprint is GONE — `INCR_P1_ADOPTED` SP
0.322→0.729) + Step 3 DONE (user re-ran the full A/B on the fresh binary; the
`_ADOPTED` arms are now distinct per-period and within 0.3–0.8 SP pts of their
plain twins — P1 0.5793→0.5710, P10 0.5597→0.5527, P30 0.5521→0.5491, P60
0.5456→0.5456 byte-identical). Verdict: stale-binary artifact; code sound, no
source change. The result (incumbent seed slightly worse at high reopt
frequency) seeded P1.4, where the user required the incumbent seed anyway on
algorithmic grounds. Code itself verified sound.
**Reference docs:** `agents/active_tasks/P1_2_reopt_incumbent_degradation/`,
memory `reopt-tl-init-adopted-arms.md`, commit `8cbbbc12` (add ReoptStartFromAdoptedTL)

## The user's report

> "initial simulation experiments show that adding ReoptStartFromAdoptedTL cause
> INCR-P1-adopted, etc to have much worse performance than the version without
> INCR-P1. i need to understand why that happens, and how to fix it"

i.e. the `INCR_P<n>_ADOPTED` arms (commit `8cbbbc12`) collapse far below their
plain `INCR_P<n>` twins in the first A/B run.

## Root cause (FOUND — not a code defect)

**The A/B ran against a stale `release/` binary that predates commit `8cbbbc12`.**

The code is sound. The regression is an artifact of running the new config
against the old binary, where `INCR_P<n>_ADOPTED` is an unrecognized mode string
that falls through the dispatch to an empty result.

Evidence chain:
- `release/libSP_OPT.so` built **2026-07-08 20:27**, contains **0** `"adopted"`
  strings; `release/tests/RunOrchestrator` same date, **0** `_ADOPTED` strings.
- Commit `8cbbbc12` (add `ReoptStartFromAdoptedTL`) authored
  **2026-07-11 10:47:09** — AFTER both binaries were built.
- The run dir
  `p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6x8/` was created
  **2026-07-11 10:46:29** — before the commit, against the stale binary.
- `scripts/run_end_to_end.sh:37` defaults to `BIN_DIR=release`, so the e2e run
  picked up the stale `release/` build with no rebuild step.
- The DEBUG build (`build/libSP_OPTDebug.so`, 2026-07-11 10:40) DOES contain the
  code (6 `"adopted"` strings) — proving the source has it; only `release/` is
  stale.

## Why the stale binary produces the collapse (the mechanism)

Under the pre-`8cbbbc12` code, `INCR_P<n>_ADOPTED` hits TWO parsing mismatches
and then falls through the dispatch:

1. **`IsINCRPeriodVariant` rejects it** (`SimulationOrchestrator.cpp:28`).
   Pre-commit, the loop `for (i = prefix.size(); i < mode.size(); i++) if
   (mode[i] < '0' || mode[i] > '9') return false;` — the `_` of `_ADOPTED` is
   non-digit → returns **false**. So the arm is NOT recognized as an INCR
   variant. (Post-commit, the loop stops at the first non-digit and then
   accepts an optional `_ADOPTED` suffix — correct.)

2. **`MaybeOverrideReoptPeriod` sets the period but not the flag**
   (`tests/RunOrchestrator.cpp:21`). Pre-commit, `std::stoi(digits)` where
   `digits = mode.substr(prefix.size())` = `"1_ADOPTED"`. `std::stoi` parses the
   leading `"1"` and stops at `_` (no throw) → `ReoptimizationPeriod = 1`.
   The `ReoptStartFromAdoptedTL` field does not exist in the old binary. (Verified
   with a standalone `stoi("1_ADOPTED")==1` probe.) So the period IS overridden
   but the flag is silently absent.

3. **Dispatch fall-through** (`SimulationOrchestrator.cpp:296`,
   `DeterminePrioritiesAndBudgets`). The INCR branch is
   `scheduler_mode_ == "INCR" || IsINCRPeriodVariant(...)`. With the mode string
   literally `INCR_P1_ADOPTED` (not `"INCR"`) and `IsINCRPeriodVariant` false,
   the INCR branch is skipped. The string matches none of `BF / INCR_SCRATCH /
   INCR_NO_TL / INCR_WCET / RM / RM_FAST / RM_SLOW` either → every branch is
   skipped → `res` returns the default-constructed `ResourceOptResult` with an
   **empty `priority_vec` and empty `id2time_limit`**.

4. **`ApplyTaskConfigurations`** (`SimulationOrchestrator.cpp:393`) then sets
   every task's ET to its Gaussian mean (`task.setExecutionTime(...)`) but
   assigns **no priorities** (the loop over `res.priority_vec` is empty). The
   schedule is a fixed, period-independent, degenerate config.

## The empirical signature that confirms it

All four `_ADOPTED` arms (P1 / P10 / P30 / P60) produce **byte-for-byte
identical** per-interval SP traces — `diff` of `interval_sp_metrics.txt` across
P1_ADOPTED vs P10_ADOPTED vs P60_ADOPTED is **empty** (verified on taskset_0,
tasks4 A/B run). This is the decisive fingerprint:

- If `ReoptStartFromAdoptedTL` were genuinely active, P1 (reopt every interval)
  and P60 (reopt once) could NOT produce identical traces — the flag acts at
  reopt intervals, which differ by period.
- Byte-identity across periods means the period is being set (stoi parses it)
  but **never used** — exactly the dispatch-fall-through signature.

Supporting symptoms (tasks4 taskset_0):
- `INCR_P1_ADOPTED` mean SP `0.322` vs plain `INCR_P1` `0.672`.
- `_ADOPTED` `miss_rate` exactly `0.000000` across all periods (total_jobs far
  smaller: `10500` vs plain's `32348` — a degenerate schedule that completes
  almost nothing / has huge slack).
- taskset_1 `_ADOPTED` trace is identical to plain for intervals 0–5, then
  **collapses to literal `0`** from interval 6 onward (with `e-16` float-zero
  ghosts) while plain stays healthy at ~0.18–0.2.
- Aggregate `comparison_summary.csv`: every `_ADOPTED` row is the exact same
  `Mean_SP_Metric=0.427563, Std=0.237511, miss=0` regardless of period.

## The code is sound (verified, not just asserted)

The post-`8cbbbc12` source is correct by inspection AND by the test suite on the
DEBUG build:
- `MaybeOverrideReoptPeriod` (`tests/RunOrchestrator.cpp:21-56`): splits the
  digit run from the optional `_ADOPTED` suffix correctly; sets the flag only
  when the suffix matches exactly.
- `IsINCRPeriodVariant` (`SimulationOrchestrator.cpp:28-42`): accepts
  `INCR_P<n>` or `INCR_P<n>_ADOPTED`, rejects bare `INCR_P` / `INCR_P_ADOPTED` /
  `INCR_P10X`.
- The reopt branch (`OptimizeSP_TL_Incre.cpp:473-476`): the guarded choice
  `(ReoptStartFromAdoptedTL && IfInitialized()) ? Reconstruct... :
  Initialize...` — `IfInitialized()` auto-falls-back at interval 0 /
  INCR_SCRATCH.
- 49 `testIncreOpt_w_TL` + 16/16 ctest green on `build/libSP_OPTDebug.so`
  (2026-07-11 10:40), including the 3 new
  `ReOptimizePeriodic_StartsFrom*{GaussianMean,AdoptedTL,Interval0Fallback}`
  tests at `tests/testIncreOpt_w_TL.cpp:1206-1331`.

## The fix

1. **Rebuild `release/`** so the binary catches up to commit `8cbbbc12`:
   `cmake --build build --config Release -j5` (or whatever the release build
   invocation is — confirm against the project's build convention; the DEBUG
   constraint `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` is for the test lib only).
   Verify: `strings release/libSP_OPT.so | grep -ci adopted` must be > 0, and
   `strings release/tests/RunOrchestrator | grep -i _ADOPTED` must list the new
   Usage line.
2. **Re-run the A/B** (user-run): `MODE=test CONFIG_JSON=simulation_experiments/configs/p25_period_ab_config.json ./scripts/run_end_to_end.sh` (default `BIN_DIR=release`).
3. **Re-read `comparison_summary.csv`** — expect the `_ADOPTED` arms to no longer
   be byte-identical across periods and to be competitive with (not collapsed
   below) their plain twins. The real A/B signal (adopted-TL reopt start vs
   Gaussian-mean reopt start) is only readable after this.

If, AFTER the rebuild + re-run, the `_ADOPTED` arms are STILL worse than plain,
THEN this becomes a real code investigation (re-open; the flag's interaction
with `ResetIncumbentBaseline(true)` / `ReconstructTimeLimitVecFromResOpt` under a
changed DAG would be the suspect). But the current evidence says stale binary,
not code bug.

## Files

- `sources/Optimization/OptimizeSP_TL_Incre.cpp:469-484` — `ReOptimizePeriodic`,
  the guarded TL-init choice (the actual code change in `8cbbbc12`).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:28-42`
  (`IsINCRPeriodVariant`), `:296` (dispatch) — post-commit correct.
- `tests/RunOrchestrator.cpp:21-56` (`MaybeOverrideReoptPeriod`) — post-commit
  correct.
- `scripts/run_end_to_end.sh:37` — `BIN_DIR="${BIN_DIR:-release}"` (no rebuild;
  the reason the stale binary was used).
- `release/libSP_OPT.so`, `release/tests/RunOrchestrator` — the stale artifacts
  (2026-07-08); `build/libSP_OPTDebug.so` (2026-07-11 10:40) has the code.
- `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6x8/`
  — the run that exhibited the regression (tasks4 aggregated; tasks6/tasks8
  partial).

## Done when

- `release/` rebuilt; verified `strings release/libSP_OPT.so | grep -ci adopted`
  > 0 and the RunOrchestrator binary lists the `_ADOPTED` Usage line.
- A/B re-run by the user; `comparison_summary.csv` re-read; `_ADOPTED` arms no
  longer byte-identical across periods and no longer collapsed to ~0.32 SP.
- A recorded verdict: if the collapse is gone → close (stale-binary artifact,
  no code change). If it persists → escalate to a real code investigation with
  the new evidence.
- Milestone to top-level `agents/dev_log.md`.

## Out of scope

- Any source change before the rebuild + re-run confirms the collapse persists.
  The evidence is unambiguous that the code is correct and the binary is stale.
- `git commit` — user's standing constraint (`git add` only).
- Running the full A/B suite myself — the user runs `run_end_to_end.sh`.
- Re-litigating P1.2 (structural corruption) — different question; the `_ADOPTED`
  A/B is the P0.5 symmetric-fix probe, not the P1.2 hazard probe.
