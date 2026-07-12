# P1.4 — Reopt Seed TL: Use the Carried Incumbent Permanently

**Priority:** P1 (active; correctness/policy of the reopt descent start)
**Status:** DONE 2026-07-11 (working tree, staged, NOT committed). The carried
incumbent is the **permanent, unconditional** reopt seed. Implemented **choice
(b)** — the `ReoptStartFromAdoptedTL` knob was REMOVED entirely (not kept as an
ablation opt-out): `ReOptimizePeriodic` now does
`IfInitialized() ? ReconstructTimeLimitsFromResOpt() : InitializeTimeLimitsFromETConfig()`
unconditionally. The `INCR_P<n>_ADOPTED` A/B arms and their parsing were removed
too (a stale `_ADOPTED` config now fails LOUDLY in `MaybeOverrideReoptPeriod`
rather than silently dispatching to an empty result — the P1.3 trap). TDD
red→green: 48/48 `testIncreOpt_w_TL` (one opt-out test deleted; the flag-default
assertion dropped) + 16/16 ctest green on the DEBUG build.
**Reference docs:** memory `reopt-tl-init-adopted-arms.md`,
[`P1_3_adopted_tl_regression/`](../P1_3_adopted_tl_regression/) (the A/B that
measured this), commit `8cbbbc12` (the knob's original introduction).

## The user's directive

> "the initial ET must come from an algorithm rather than reading from yaml or
> the task set generation logic, what would be the closest option now?"

followed by:

> "okay, let's use previous optimization results, permanently, create an active
> task, detail this findings and reasonings, and then implement this task"

So: the reopt descent's **seed TL** must be **algorithm-derived** (the
optimizer's own prior result), not a value read out of the YAML taskset
characterization or the generator's drawn distribution. The chosen option is
**"always use previous found Time limit config"** — implemented by making the
carried-incumbent seed (`ReconstructTimeLimitsFromResOpt`) the **permanent**
reopt seed, not a toggleable A/B variant.

## Background — the two reopt seed policies

`ReOptimizePeriodic` (`sources/Optimization/OptimizeSP_TL_Incre.cpp:453-484`)
runs the TL descent from a starting vector. That start vector is the "seed TL".
Two sources were available:

1. **`InitializeTimeLimitsFromETConfig()`** (`:178-192`) — per task, picks the
   `timePerformancePairs` entry whose `time_limit` is closest (abs-diff argmin
   via `Find_Close_ExecutionTime`) to `et_dist_.GetAvgValue()`. This is the
   **YAML-derived** seed: `et_dist_` is built from the YAML taskset's
   `execution_time_mu/sigma/min/max` (`RegularTasks.cpp:61-106`), and those
   values originate in the generator's drawn `Et_mean`
   (`Gen_Taskset/lib/taskset_generator.py:244-245`) or the trace mean
   (`orchestrator.py:309`). The seed value is therefore a *derived function of
   YAML content* — deterministic, but the seed value is read out of the
   characterization file, not produced by the optimizer.

2. **`ReconstructTimeLimitsFromResOpt()`** — returns the TL vector held in
   `res_opt_`, the optimizer's own carried incumbent state (written by
   `CommitIncumbent`, the single durable store from the P0.5 redesign). This is
   the **algorithm-derived** seed: the value is the optimizer's prior output,
   not a YAML field. On reopt intervals 1+ an incumbent always exists
   (`IfInitialized()` true); at interval 0 / INCR_SCRATCH there is no incumbent
   and the guard auto-falls-back to policy 1 (unavoidable — the very first solve
   has no prior state to seed from).

Commit `8cbbbc12` wired the choice behind `GlobalVariables::ReoptStartFromAdoptedTL`
(bool, default **false**): false → policy 1 (YAML-derived, the pre-existing
behavior); true → policy 2 (incumbent, the `INCR_P<n>_ADOPTED` A/B arms).

## Why policy 1 fails the new constraint (the finding)

The user's constraint — "initial ET must come from an algorithm, not from yaml
or the task-set-generation logic" — disqualifies policy 1 outright:

- `InitializeTimeLimitsFromETConfig` calls `dag_tasks_.tasks[i].execution_time_dist.GetAvgValue()`.
- `execution_time_dist` is a `FiniteDist` constructed in `ReadTaskSet`
  (`RegularTasks.cpp:61-106`) from the YAML fields `execution_time_mu`,
  `execution_time_sigma`, `execution_time_min`, `execution_time_max` — 4 of 5
  inputs are YAML fields.
- For the perf-pair (TL-optimizable) tasks, the generator writes `mu` from its
  drawn `Et_mean` (`taskset_generator.py:244-245`,
  `Et_mean = period * uniform(0.1, 0.3)`) or from the trace mean
  (`orchestrator.py:309`, `np.mean(interval_ets)`) — **independent of the
  `performance_records_time` option grid**, which is a uniform ramp over
  `[period*0.05, period*0.9]` (`taskset_generator.py:580-594`).
- So `GetAvgValue()` resolves (via the discretized Gaussian) to a value tied to
  the generator's mu draw, and `InitializeTimeLimitsFromETConfig` snaps that to
  the nearest option. The seed is a derived function of YAML content — exactly
  what the constraint forbids.

This was confirmed empirically (3 real generated tasks): in every case
`mu != option-average` and `nearest-to-mu != nearest-to-option-midpoint`, so the
current "off" seed is neither "the middle option" nor an algorithmic value — it
is the option nearest the YAML/Gaussian mean.

## Why policy 2 is the closest algorithmic option (the reasoning)

Ranking the candidate seeds by how much of the seed *value* is the optimizer's
own output vs read from YAML:

| Seed | Seed-value origin | Algorithmic? |
|------|-------------------|--------------|
| `InitializeTimeLimitsFromETConfig` (current "off") | `et_dist_.GetAvgValue()`, `et_dist_` from YAML | ❌ reads YAML ET dist |
| Positional middle/smallest/largest option | YAML `performance_records_time` value, picked by index | ⚠️ algorithmic *rule*, YAML *value* |
| Random option | RNG over the YAML option list | ⚠️ algorithmic, but non-deterministic |
| **Carried incumbent (`res_opt_`)** | the optimizer's own prior `CommitIncumbent` write | ✅ fully algorithm-produced |

The carried incumbent is the only seed whose **value** is an algorithm's output,
not a YAML field. It is already implemented (`8cbbbc12`), already gated
correctly (`IfInitialized()` → auto-fallback at interval 0 / INCR_SCRATCH), and
already A/B-measured (P1.3). Making it permanent = flipping the default of
`ReoptStartFromAdoptedTL` to **true** (or, per the design rules' "don't keep
features that aren't needed", folding the knob away so the incumbent seed is
unconditional — see Implementation below).

## The measured tradeoff (the honest cost)

P1.3's A/B (`comparison_summary.csv`, tasks4 re-run on the fresh binary) showed
the incumbent seed is **slightly worse** than the YAML-derived seed:

| Arm pair | Plain (YAML seed) | `_ADOPTED` (incumbent seed) | Δ SP |
|----------|-------------------|------------------------------|------|
| P1  | 0.579318 | 0.571043 | −0.0083 |
| P10 | 0.559728 | 0.552747 | −0.0070 |
| P30 | 0.552093 | 0.549053 | −0.0030 |
| P60 | 0.545583 | 0.545583 |  0.0000 (byte-identical) |

The gap **shrinks toward zero as the reopt period grows** (P60 is a no-op —
reopt once, no carried-incumbent-vs-YAML divergence to accumulate). At high
reopt frequency (P1) it is ~0.8 SP pts. The user has accepted this cost in
exchange for the algorithmic-purity guarantee ("let's use previous optimization
results, permanently").

## The interval-0 caveat (irreducible)

On the bootstrap interval (interval 0) and for `INCR_SCRATCH` (amnesiac — fresh
optimizer each interval), there is **no incumbent** (`IfInitialized()` false),
so the incumbent seed is impossible and the guard falls back to
`InitializeTimeLimitsFromETConfig`. This is irreducible for *any* seed policy:
the very first solve has no prior optimizer state, so something external must
supply the seed. "Fully algorithmic" therefore holds for **reopt intervals 1+**,
not interval 0. The P0.5 redesign already isolated this correctly via the
`IfInitialized()` gate; no change needed.

## Implementation

The change is intentionally small (the `IfInitialized()` gate already does the
right thing). **Choice (b) was chosen** — remove the knob + the `_ADOPTED`
arms entirely, making the incumbent seed unconditional. (Choice (a) — keep the
knob default-true as an ablation opt-out — was implemented first and then
replaced by (b) at the user's direction: "always use reopt-start-from-adopted-tl,
remove the flag, clean related choice code and unused code." The design rules
"don't make things optional if not needed" / "ruthlessly prune unused features"
favor (b); the A/B data is no longer needed since the policy is permanent.)

1. **TDD — tests first** (`tests/testIncreOpt_w_TL.cpp:1199+`). DONE:
   - `ReOptimizePeriodic_StartsFromAdoptedTLByDefault` → renamed to
     `ReOptimizePeriodic_StartsFromAdoptedTL`; dropped the `ASSERT_TRUE(flag)`
     (flag is gone); keeps the incumbent-start assertion on interval 1+.
   - `ReOptimizePeriodic_OptOutStartsFromGaussianMeanTL` → **DELETED** (the
     `false` path no longer exists; the opt-out is gone).
   - `ReOptimizePeriodic_Interval0FallsBackToGaussianMean` → unchanged in
     spirit (interval 0 still falls back); the flag mention in the body comment
     was stripped. Logic unchanged.
   - Net: 49 → 48 tests in `testIncreOpt_w_TL`.
2. **Remove the knob.** `sources/Utils/Parameters.cpp` (drop the
   `ReoptStartFromAdoptedTL` definition + comment), `Parameters.h` (drop the
   extern + comment), `parameters.yaml` (drop the note block). DONE.
3. **Unconditional seed.** `sources/Optimization/OptimizeSP_TL_Incre.cpp:478`:
   the ternary `(GlobalVariables::ReoptStartFromAdoptedTL && IfInitialized()) ?
   Reconstruct... : Initialize...` became `IfInitialized() ?
   ReconstructTimeLimitsFromResOpt() : InitializeTimeLimitsFromETConfig()`.
   Comment updated to state the policy is permanent + unconditional. `.h:132`
   `starting_time_limits` origin comment updated (uniform incumbent seed for
   both paths). DONE.
4. **Remove `_ADOPTED` parsing + arms.** DONE:
   - `tests/RunOrchestrator.cpp` `MaybeOverrideReoptPeriod`: dropped the
     `_ADOPTED` suffix handling; a trailing suffix after the digits is now a
     **hard error** (loud stderr + no flag set) so a stale `_ADOPTED` config
     can't silently dispatch to an empty `ResourceOptResult` (the P1.3 trap).
     Header comment + `--help` Usage line updated.
   - `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`
     `IsINCRPeriodVariant`: simplified to `INCR_P<n>` with no trailing suffix;
     history comment added. Both dispatch sites (`:277`, `:297`) unchanged (they
     call the same function).
   - `simulation_experiments/configs/p25_period_ab_config.json`: removed the 4
     `_ADOPTED` arms from both `test_mode` and `prod_mode` `main_scheduler_list`
     (10→6 arms each); rewrote the `_comment` + both mode `_comment`s.
5. **Docs.** All in-source comments updated to match (see files above). DONE.
6. **Memory.** Updated `reopt-tl-init-adopted-arms.md` to record the (b)
   removal. DONE.
7. **Build + test.** `cmake --build build --target check.SP_OPT -j5` (DEBUG);
   48/48 `testIncreOpt_w_TL` + 16/16 ctest green. DONE.

## Done when

- The `ReOptimizePeriodic_*` tests reflect the unconditional incumbent seed
  (and pass). ✓
- `ReoptStartFromAdoptedTL` is GONE (not just defaulted true); the
  `_ADOPTED` arms are GONE. ✓
- All comments/docs/configs updated to match. ✓
- DEBUG build + `cmake --build build --target check.SP_OPT -j5` green. ✓
- `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated. ✓
- `git add` staged; user reviews and commits. ✓ (staged)

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Running the full A/B myself — user runs `run_end_to_end.sh`. (Under choice
  (b) the `_ADOPTED` arms are gone; the next A/B runs the 6-arm config and the
  incumbent seed is the only reopt seed.)
- Re-litigating P1.1 (the residual ET-growth gap) or P1.2 (structural
  corruption) — different questions; this task is a seed-policy change only.
- Changing the interval-0 / INCR_SCRATCH fallback — irreducible (no incumbent
  to seed from).
