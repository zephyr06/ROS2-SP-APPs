# P1.3 — Tasks (working checklist)

> See `goal.md` for the full diagnosis. **Root cause found: stale `release/`
> binary.** No source change until the rebuild + re-run confirms otherwise.

## Step 1 — Confirm the stale-binary diagnosis is complete (DONE)
- [x] Read the A/B run that exhibited the regression
      (`p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6x8/`, tasks4
      aggregated).
- [x] Confirm all four `_ADOPTED` arms produce byte-identical interval SP traces
      across P1/P10/P30/P60 (`diff` empty) — the dispatch-fall-through
      fingerprint.
- [x] Confirm `release/libSP_OPT.so` + `release/tests/RunOrchestrator` predate
      commit `8cbbbc12` (built 2026-07-08; commit 2026-07-11 10:47) and contain
      0 `"adopted"` strings.
- [x] Confirm `build/libSP_OPTDebug.so` (2026-07-11 10:40) DOES contain the code
      (6 `"adopted"` strings) — source is correct, only `release/` is stale.
- [x] Trace the mechanism: pre-commit `IsINCRPeriodVariant` rejects `_ADOPTED`
      (non-digit `_`) → dispatch fall-through → empty `ResourceOptResult` →
      degenerate period-independent schedule.
- [x] Verify the post-commit parser + dispatch + reopt branch are correct by
      inspection AND by the test suite (49 `testIncreOpt_w_TL` + 16/16 ctest
      green on the DEBUG build, incl. the 3 new `ReOptimizePeriodic_*` tests).

## Step 2 — Rebuild `release/` (user-run; or confirm the build invocation) — DONE
- [x] Rebuild the release binary so it catches up to commit `8cbbbc12`.
      **DONE 2026-07-11** — `release/` rebuilt (binary mtime 11:26:24, after
      the 10:47 commit); `release/CMakeCache.txt` = `CMAKE_BUILD_TYPE=Release`.
- [x] Verify the rebuilt artifacts: `strings release/libSP_OPT.so | grep -ci
      adopted` = **3** (was 0); `strings release/tests/RunOrchestrator` lists
      the new `_ADOPTED` Usage line; mangled symbol
      `_ZN15GlobalVariables23ReoptStartFromAdoptedTLE` present in both binaries
      (the flag variable is linked in, not just the help string).
- [x] Functional probe (3-way + 2 controls) on the fresh binary: stale
      `INCR_P1_ADOPTED` SP 0.322 → fresh **0.729**; the stale fall-through
      fingerprint (all 4 `_ADOPTED` arms byte-identical) is **gone**. The 5
      fresh probes being mutually identical is a trivial-8-interval-taskset
      artifact (0% miss, SP ceiling — even `INCR_SCRATCH` matches), not a flag
      problem; the real signal needs the loaded full A/B.

## Step 3 — Re-run the A/B (user-run) — DONE
- [x] `MODE=test CONFIG_JSON=simulation_experiments/configs/p25_period_ab_config.json ./scripts/run_end_to_end.sh`
      (default `BIN_DIR=release`) — run by the user 2026-07-11.
- [x] Re-read `comparison_summary.csv` for tasks4. The `_ADOPTED` arms are no
      longer byte-identical across periods and are within 0.3–0.8 SP pts of
      their plain twins (P1 0.5793→0.5710, P10 0.5597→0.5527, P30 0.5521→0.5491,
      P60 0.5456→0.5456 byte-identical). Gap shrinks with reopt period.

## Step 4 — Verdict — DONE
- [x] **The collapse is gone.** Recorded the negative result; P1.3 closed as a
      stale-binary artifact, no source change. Milestone appended to top-level
      `agents/dev_log.md`. The result (incumbent seed slightly worse than the
      YAML seed at high reopt frequency) seeded P1.4, where the user required
      the incumbent seed permanently on algorithmic grounds (the seed must be
      algorithm-derived, not read from YAML).

## Standing constraints
- No `git commit` (user's task; `git add` only).
- No running the A/B suite myself (user runs `run_end_to_end.sh`).
- No source change before Step 3's verdict — the evidence says stale binary.
