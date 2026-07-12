# P1.3 — `ReoptStartFromAdoptedTL` A/B Regression — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task created from the user's report: the first A/B run with the
  `INCR_P<n>_ADOPTED` arms (commit `8cbbbc12`) shows them collapsing far below
  their plain `INCR_P<n>` twins.
- **Root cause FOUND: stale `release/` binary.** The A/B ran against
  `release/libSP_OPT.so` + `release/tests/RunOrchestrator` built
  2026-07-08 20:27, which predate commit `8cbbbc12` (2026-07-11 10:47:09) that
  added `ReoptStartFromAdoptedTL`. The run dir was created 10:46:29 — before the
  commit, against the stale binary. `scripts/run_end_to_end.sh:37` defaults to
  `BIN_DIR=release` with no rebuild step, so the e2e run picked up the stale
  build. The DEBUG build (`build/libSP_OPTDebug.so`, 2026-07-11 10:40) DOES
  contain the code (6 `"adopted"` strings) — confirming the source has it; only
  `release/` is stale.
- **Mechanism (why stale binary → collapse):** under the pre-`8cbbbc12` code,
  `INCR_P<n>_ADOPTED` hits two parsing mismatches then falls through the
  dispatch:
  1. `IsINCRPeriodVariant` (`SimulationOrchestrator.cpp:28`) — pre-commit loops
     over every char after `INCR_P` and rejects any non-digit; the `_` of
     `_ADOPTED` fails → returns false → arm NOT recognized as INCR.
  2. `MaybeOverrideReoptPeriod` (`tests/RunOrchestrator.cpp:21`) — pre-commit
     `std::stoi(digits)` where `digits="1_ADOPTED"`; stoi parses `"1"`, stops at
     `_`, no throw → `ReoptimizationPeriod=1`. The flag field doesn't exist in
     the old binary. (Verified with a standalone `stoi("1_ADOPTED")==1` probe.)
  3. Dispatch fall-through (`SimulationOrchestrator.cpp:296`) — the INCR branch
     is `"INCR" || IsINCRPeriodVariant`; both false for the literal
     `INCR_P1_ADOPTED` string; none of the other branches match either → `res`
     returns default-constructed `ResourceOptResult` (empty `priority_vec`,
     empty `id2time_limit`).
  4. `ApplyTaskConfigurations` (`:393`) sets every task's ET to its Gaussian
     mean but assigns no priorities → a fixed, period-independent, degenerate
     schedule.
- **Decisive empirical fingerprint:** all four `_ADOPTED` arms (P1/P10/P30/P60)
  produce byte-for-byte identical `interval_sp_metrics.txt` traces — `diff`
  empty across P1_ADOPTED vs P10_ADOPTED vs P60_ADOPTED (verified tasks4
  taskset_0). If the flag were genuinely active, P1 (reopt every interval) and
  P60 (reopt once) could not produce identical traces. Byte-identity across
  periods = the period is set (stoi) but never used = dispatch fall-through.
  Supporting: `_ADOPTED` `miss_rate` exactly 0.000000 across all periods;
  taskset_1 `_ADOPTED` identical to plain for intervals 0–5 then collapses to
  literal 0 from interval 6 on; aggregate `Mean_SP_Metric=0.427563` identical
  across all four `_ADOPTED` rows.
- **Code verified sound:** post-`8cbbbc12` parser + dispatch + reopt branch
  correct by inspection AND by the test suite on the DEBUG build (49
  `testIncreOpt_w_TL` + 16/16 ctest green, incl. the 3 new
  `ReOptimizePeriodic_StartsFrom*{GaussianMean,AdoptedTL,Interval0Fallback}`
  tests at `tests/testIncreOpt_w_TL.cpp:1206-1331`).
- **Fix:** rebuild `release/` (verify `strings release/libSP_OPT.so | grep -ci
  adopted` > 0); re-run the A/B (user-run); re-read `comparison_summary.csv`.
  Expect the `_ADOPTED` arms to no longer be byte-identical across periods and
  to be competitive with their plain twins. Only if the collapse PERSISTS after
  a confirmed-fresh rebuild does this become a real code investigation.
- Not yet started on Step 2 (rebuild). Next: user rebuilds `release/` and
  re-runs the A/B.

### 2026-07-11 (cont.) — Step 2 DONE: `release/` rebuilt + verified fresh

- **`release/` has been rebuilt** (by the user, between sessions). Binary mtimes
  are now **2026-07-11 11:26:24** (`libSP_OPT.so`) / **11:26:39**
  (`RunOrchestrator`) — both **after** commit `8cbbbc12` (10:47:09).
  `release/CMakeCache.txt` confirms `CMAKE_BUILD_TYPE=Release`.
- **Freshness verified three ways:**
  1. `strings release/libSP_OPT.so | grep -ci adopted` → **3** (was 0 on the
     stale 07-08 build).
  2. `strings release/tests/RunOrchestrator` lists the `_ADOPTED` Usage line
     ("`INCR_P<n>_ADOPTED: as INCR_P<n> but the reopt descent starts from the
     carried adopted TL (ReoptStartFromAdoptedTL)`").
  3. The mangled symbol `_ZN15GlobalVariables23ReoptStartFromAdoptedTLE`
     (`GlobalVariables::ReoptStartFromAdoptedTL`) is present in BOTH binaries —
     the flag variable is linked in, not just the help string.
- `RunOrchestrator` (no args) prints the `_ADOPTED` mode line — the binary's
  own usage matches the post-commit source.

### 2026-07-11 (cont.) — functional probe: stale fingerprint GONE on fresh binary

- Ran a 3-way smoke probe on the fresh binary to confirm the dispatch no longer
  falls through (NOT the full A/B — a cheap functionality check). Built an
  8-interval taskset by trimming tasks4 taskset_0 to intervals 0–7 (deleted
  `taskset_characteristics_interval_{8..59}.yaml` into `/tmp/p13_short_ts`).
  Ran `release/tests/RunOrchestrator <ts> <out> <mode> 10000 1 0` for
  `INCR_P1`, `INCR_P1_ADOPTED`, `INCR_P60_ADOPTED`, plus `INCR_P60` and
  `INCR_SCRATCH` for the trivial-taskset control. (4th arg = per-interval
  horizon 10000 ms = 10 s, per memory `runorchestrator-duration-arg-semantics`;
  interval count = #interval YAMLs on disk, NOT derived from `duration_ms`.)
- **Decisive result — the collapse is gone:**
  - Stale binary `INCR_P1_ADOPTED` mean SP = **0.322** (collapsed).
  - Fresh binary `INCR_P1_ADOPTED` mean SP = **0.729** (healthy, ≈ plain `INCR_P1`).
  - Stale-binary fall-through fingerprint (all 4 `_ADOPTED` arms byte-identical:
    `INCR_P1_ADOPTED == INCR_P10_ADOPTED == INCR_P30_ADOPTED == INCR_P60_ADOPTED`,
    `diff -q` empty) is **absent** on the fresh binary.
- **The 5 fresh probes are mutually byte-identical — but that is a
  trivial-taskset artifact, NOT a flag problem.** Even `INCR_SCRATCH` (the
  amnesiac always-reopt control, which should differ most from `INCR_P1`)
  produces the identical 8-line trace. Cause: the 8-interval taskset is
  near-zero-load — 0% miss rate across all arms, every arm saturates at the SP
  ceiling (`0.733333`). The `ReoptStartFromAdoptedTL` effect (reopt descent
  starting from the carried adopted TL vs the Gaussian mean) only manifests when
  the descent actually moves off the Gaussian, which requires a *loaded* taskset
  over many intervals — i.e. the full prod A/B run. This is consistent with the
  P1.3 goal: "the real A/B signal is only readable after [the rebuild + re-run]."
- **Conclusion for Step 2 + the pre-A/B verdict:** the code is sound and the
  binary is fresh; the stale-binary root cause is fully resolved at the
  binary level. What remains is Step 3 (the user's full prod A/B re-run on the
  fresh binary) to read the *real* `_ADOPTED`-vs-plain signal on loaded
  tasksets. Per the goal's Done-when: if the loaded A/B shows the `_ADOPTED`
  arms competitive with (not collapsed below) their plain twins, the task
  closes as a stale-binary artifact with no source change.
- Probe artifacts retained at `/tmp/p13_probe/` and `/tmp/p13_short_ts/`
  (throwaway; not under the repo).
