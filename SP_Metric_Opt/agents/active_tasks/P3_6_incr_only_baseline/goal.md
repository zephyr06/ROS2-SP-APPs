# P1.6 — New Baseline: Pure Incremental, RM-Fast Bootstrap, No Periodic Reopt

**Priority:** P1 (active investigation — a new A/B arm for the incr-vs-scratch
comparison, not a correctness fix)
**Status:** PLANNING ONLY 2026-07-11 (task folder created; **NO implementation**).
The user explicitly asked to "only add this active task without implementation."
**Follow-up 2026-07-11 (plans only, still no implementation):** the user directed
that when implemented, the arm should mirror `INCR_SCRATCH`'s dispatch shape (a
dedicated `scheduler_mode_` branch — settles design decision D2), and that it will
"likely be a baseline to add to paper" (so a paper-grade baseline, not just a
throwaway A/B probe — see D5). This file records the finding (what the arm is),
the reasoning (why it's a clean baseline), the **INCR_P1 vs INCR_SCRATCH**
distinction (the key context — see Background), and the open design decisions to
settle BEFORE coding.
**Reference docs:** [`P1_4_reopt_seed_from_incumbent/`](../P1_4_reopt_seed_from_incumbent/)
(the reopt-seed policy this arm inherits), memory
[`reopt-tl-init-adopted-arms.md`](../../../) (the carried-incumbent seed),
[`P1_2_reopt_incumbent_degradation/`](../P1_2_reopt_incumbent_degradation/)
(the structural-corruption question this arm informs).

## The user's directive

> "add a new active task, which is adding a new baseline where we mainly use
> incremental optimization to perform per-interval optimization, and we start
> the RM-Fast rather than re-optimization in the first interval, and never call
> re-optimization periodically. only add this active task without implementation.
> purpose is to compare performance between incr and scratch"

So: a new scheduler arm that is **pure incremental** — `OptimizeIncre_w_TL`
(warm-started from the carried incumbent) runs every interval, **no periodic
`ReOptimizePeriodic`** is ever called, and the **first interval bootstraps from
RM-Fast** (the cheap RM-priorities + smallest-TL heuristic) instead of from a
from-scratch re-optimization. The purpose is **A/B data**: compare this pure-
incremental arm against `INCR_SCRATCH` (the amnesiac control) — and, secondarily,
against `INCR` (which does periodic reopt) — to quantify what carrying the
incumbent via warm-start buys vs. re-solving from RM-Fast each interval.

## Background — the three relevant arms today (and the INCR_P1-vs-INCR_SCRATCH key)

`Optimize_w_TL_ScratchOrIncre` (`sources/Optimization/OptimizeSP_TL_Incre.cpp:291`)
routes each interval on `reoptimization_interval_count_ % ReoptimizationPeriod`:

| Arm | Interval 0 | Intervals 1+ | Carries incumbent? |
|-----|-----------|--------------|--------------------|
| `INCR` (period N) | `ReOptimizePeriodic`: RM-fast seed **+ from-scratch descent** | `OptimizeIncre_w_TL` except every Nth → `ReOptimizePeriodic` (compare-and-keep) | yes |
| `INCR_SCRATCH` | fresh optimizer → `ReOptimizePeriodic`: RM-fast seed + from-scratch descent, then **discard** | same (amnesiac — fresh optimizer each interval) | **no** |
| **NEW (this task)** | **RM-fast seed only** (no descent) → commit as incumbent | `OptimizeIncre_w_TL` every interval, **never** `ReOptimizePeriodic` | yes |

The interval-0 RM-fast seed is already implemented: `ResetIncumbentBaseline(true)`
(`OptimizeSP_TL_Incre.cpp:424`) takes its `else` branch at interval 0 —
`SmallestTimeLimitVec()` (= `timePerformancePairs[0].time_limit`, i.e. the
orchestrator's `RM_FAST` TL, `SimulationOrchestrator.cpp:366-385`) +
`RateMonotonicPriorityVec()` (RM order, tie-break avg-ET asc) →
`SeedStateFromIncumbent(...)` → `CommitIncumbent`. The new arm = run that seed
step, then **skip** the `PerformCoordinateDescentForTaskConfigOpt(...,
from_scratch=true)` descent that `ReOptimizePeriodic` runs afterwards, and never
call `ReOptimizePeriodic` again.

### INCR_P1 vs INCR_SCRATCH — the key distinction (resolved 2026-07-11)

The user asked what the difference is. **Both arms run the from-scratch descent
`ReOptimizePeriodic` every interval** — that is *not* the difference. The
difference is **optimizer lifetime / whether the incumbent is carried**:

- **`INCR_P1`** (`INCR` with `ReoptimizationPeriod=1`) uses the **persistent**
  `incr_optimizer_` (built once before the loop, `SimulationOrchestrator.cpp:296`).
  `res_opt_` survives across intervals → at interval 1+ `ResetIncumbentBaseline(true)`
  takes the **`if (IfInitialized())` branch** (`OptimizeSP_TL_Incre.cpp:427-436`):
  it re-evaluates the **carried** {pa, tl} under the new DAG and commits that as
  the baseline. Compare-and-keep measures the search against the **running best**.
- **`INCR_SCRATCH`** builds a **fresh** `scratch_opt` each interval
  (`SimulationOrchestrator.cpp:334`) and discards it. `res_opt_` is empty on
  every entry → `ResetIncumbentBaseline(true)` takes the **`else` branch**
  (`OptimizeSP_TL_Incre.cpp:437-447`) **every** interval: a synthetic
  RM+min-TL baseline. Compare-and-keep measures against that **synthetic RM
  baseline**, not the prior interval's solution.

Net: **`INCR_P1` weakly dominates `INCR_SCRATCH` in SP** — the carried incumbent
is never worse than RM-fast, and is sometimes strictly better. `INCR_SCRATCH` is
kept only as the **amnesiac (no-memory) control**: it isolates the value of
carrying the incumbent forward by *not* carrying it. (The in-source comment at
`SimulationOrchestrator.cpp:328-333` states this dominance but still names
`prev_optimizer_`, the member P0.5 removed — the wording is stale, the logic is
not; it now runs through `res_opt_`.)

This distinction matters for the NEW arm: the new arm **carries the incumbent**
(like `INCR_P1`) but **never pays the from-scratch descent** (unlike both). So
the three-way contrast is:

| Property | `INCR_P1` | `INCR_SCRATCH` | **NEW arm** |
|----------|-----------|----------------|-------------|
| Carries incumbent (warm-start) | ✅ | ❌ (fresh each interval) | ✅ |
| Runs from-scratch descent each interval | ✅ (every interval, P=1) | ✅ (every interval) | ❌ (interval 0 = RM-fast seed only, then pure incremental) |
| Compare-and-keep baseline | running best | synthetic RM | (no compare-and-keep — pure 1-D `OptimizeIncre` walk) |

So the NEW arm is the **cheapest warm-start path**: it keeps `INCR_P1`'s
incumbent-carrying (the source of its dominance over `INCR_SCRATCH`) but drops
the from-scratch descent `INCR_P1` pays every interval. The A/B then asks: does
the descent earn its cost, or does the pure 1-D incremental walk — warm-started
from the carried incumbent — reach (nearly) the same SP at far lower scheduler
ET? That is exactly the "incr vs scratch" question the user named, sharpened:
*scratch* here = `INCR_SCRATCH` (amnesiac descent), *incr* = the NEW arm
(warm-started, no descent).

## Why this is a clean, well-motivated baseline (the reasoning)

The new arm is the **cheapest possible warm-start path**: bootstrap for free
with the RM-fast heuristic at interval 0, then let the 1-D incremental
coordinate descent (`OptimizeIncre`) do all the work, never paying for a full
from-scratch reopt. Against the two existing warm-start/reopt arms it isolates
two effects the current A/B conflates:

1. **vs `INCR_SCRATCH`** (the user's "incr vs scratch") — isolates the value of
   **carrying the incumbent via warm-start**. Both arms bootstrap from RM-fast;
   the sole difference is that the new arm reuses the prior interval's adopted
   solution (warm-start) while `INCR_SCRATCH` throws it away and re-solves from
   a synthetic RM baseline each interval. Holding the seed fixed makes this a
   clean read of warm-start value. Expected: NEW ≥ INCR_SCRATCH in SP (never
   worse — the carried incumbent dominates synthetic RM) at lower ET (no
   from-scratch descent after interval 0).
2. **vs `INCR_P1`** (the natural reopt sibling — same incumbent-carrying, but a
   from-scratch descent every interval) — isolates whether the **periodic
   from-scratch descent earns its cost**. Both carry the incumbent; the difference
   is that `INCR_P1` pays for a wide-radius compare-and-keep search every
   interval to bound priority drift, while the new arm never does. If NEW ≈
   INCR_P1 in SP at lower ET, the descent is not earning its cost — which feeds
   directly into P1.2 (does reopt commit structurally-worse permutations?) and
   the P25 ET-vs-period story. (Against higher-period `INCR_P<n>`, the contrast
   softens: those pay the descent only every Nth interval, so NEW-vs-INCR_P10/P30/P60
   measures the residual value of occasional reopt, not per-interval reopt.)

The arm also de-risks a P1.2 hazard: with no `ReOptimizePeriodic` ever, the
memoryless from-scratch `OptimizeFromScratch` (K=2 shared, the suspected
structural-corruption vector) never runs — so the new arm's incumbent evolves
**only** through the 1-D `OptimizeIncre` walk. Any structural degradation
observed in `INCR` but absent here points squarely at the reopt path.

## What the arm is NOT

- Not `ReoptimizationPeriod = ∞` alone. A very large period still routes
  `count==0` → `ReOptimizePeriodic`, which runs the **from-scratch descent at
  interval 0**. The new arm additionally **skips that interval-0 descent** — the
  first real optimization is the interval-1 incremental warm-started from the
  RM-fast seed. (Whether skipping the interval-0 descent is material is itself
  one of the things the A/B will show; see Open questions.)
- Not a reopt-seed change. It inherits the P1.4 incumbent seed
  (`ReconstructTimeLimitsFromResOpt`) for the incremental path unchanged; the
  only seed in play is the interval-0 RM-fast bootstrap, which is
  `IfInitialized()`-gated and irreducible (no prior exists at interval 0).
- Not a correctness change. No bug is being fixed; this is a measurement arm.
- Not `INCR_SCRATCH` in optimizer lifetime. "Implement it similar to
  `INCR_SCRATCH`" means the same dispatch *shape* (a dedicated `scheduler_mode_`
  branch), NOT the fresh-`scratch_opt`-each-interval lifetime. The new arm MUST
  use the persistent `incr_optimizer_` to carry the incumbent — that is the whole
  point (warm-start vs amnesiac). `INCR_SCRATCH`'s fresh-each-interval lifetime
  is precisely the amnesia the new arm exists to contrast.

## Open design decisions (settle BEFORE Step 1 — do NOT decide unilaterally)

Per `agent_coding_rules.md` ("Ask users if you're not certain about design
choices, don't make design decisions yourself"), these are flagged for the user.
**Update 2026-07-11:** the user's "implement it similar to `INCR_SCRATCH`" +
"likely a baseline to add to paper" directives **settle D2 (branch) and elevate
D5 (paper-grade, not a throwaway probe)**. D1, D3, D4 remain open.

1. **Mode-string name** (OPEN). Candidates: `INCR_PURE` (pure incremental),
   `INCR_RM` (incremental from RM seed), `INCR_NO_REOPT` (descriptive),
   `INCR_ONLY` (incremental only). The name must not collide with existing modes
   (`INCR`/`INCR_P<n>`/`INCR_SCRATCH`/`INCR_NO_TL`/`INCR_WCET`/`RM`/`RM_FAST`/
   `RM_SLOW`/`BF`) and should read clearly in `comparison_summary.csv`, output
   subdir names, and (since this is paper-bound) in the paper's baseline table.
   **Recommend `INCR_PURE`** (short, signals "no reopt"), but the user picks.
2. **Implementation shape** (SETTLED 2026-07-11 → **(a) new `scheduler_mode_`
   branch**). The user directed "implement it similar to `INCR_SCRATCH`," which
   is a dedicated `scheduler_mode_ == "INCR_SCRATCH"` branch in
   `DeterminePrioritiesAndBudgets` (`SimulationOrchestrator.cpp:320-337`). The
   new arm gets its own `scheduler_mode_ == "<D1 name>"` branch there, plus a
   clause in the pre-loop construction condition (`:293`). NOT a
   `GlobalVariables` flag — the design rules ("don't make things optional if not
   needed") plus the user's "like INCR_SCRATCH" both point to a named branch.
   The interval-0 "seed-only, skip descent" needs a small new entry point
   regardless (see D3).
3. **Interval-0 mechanics — the one real code question** (OPEN). The current
   interval-0 bootstrap lives *inside* `ReOptimizePeriodic` (seed + descent are
   not separable from the outside — `OptimizeSP_TL_Incre.cpp:453-488`). To "start
   RM-fast without re-optimization," the new arm needs either:
   - a new method (e.g. `BootstrapIncumbentFromRMFast()`) that runs only the
     `ResetIncumbentBaseline(true)` interval-0 seed step + `CommitIncumbent` and
     skips `PerformCoordinateDescentForTaskConfigOpt`, called once at interval 0;
     then `OptimizeIncre_w_TL` for all later intervals; OR
   - reuse `ReOptimizePeriodic` but with the descent short-circuited (messier —
     `ReOptimizePeriodic` always descends).
   - **Recommend the new `BootstrapIncumbentFromRMFast()` method** — small,
     testable, and keeps `ReOptimizePeriodic` untouched. The TDD test asserts
     interval 0 produces the RM-fast incumbent (RM priorities + smallest TL)
     with NO descent evals run, and interval 1+ takes the incremental path.
   - **Mirroring-`INCR_SCRATCH` nuance:** `INCR_SCRATCH`'s branch builds a fresh
     `scratch_opt` each interval and calls `ReOptimizePeriodic`. The new arm
     differs on BOTH axes — it must use the **persistent** `incr_optimizer_`
     (to carry the incumbent across intervals, like `INCR`, NOT fresh-each-time
     like `INCR_SCRATCH`) AND call the seed-only bootstrap at interval 0 +
     `OptimizeIncre_w_TL` after. So "similar to `INCR_SCRATCH`" = same dispatch
     *shape* (a dedicated branch), NOT the fresh-optimizer lifetime.
4. **Config placement** (OPEN). Add the arm to `p25_period_ab_config.json` both
   modes (alongside `INCR_P1`/`INCR_P10`/`INCR_P30`/`INCR_P60`/`INCR_SCRATCH`/
   `BF`), or a separate A/B config? **Recommend adding to the p25 config** so
   the incr-vs-scratch comparison reads from the same `comparison_summary.csv`
   as the period sweep (the user's existing A/B harness). Since it is paper-bound,
   it should also appear in whichever config feeds the publication figure run
   (P0.3) — confirm with the user whether that is the same p25 config or a
   separate prod-mode list.
5. **Paper-baseline status + E3 gating** (SETTLED 2026-07-11 → **paper-grade
   baseline; A/B probe, NOT an E3 gate**). The user said it will "likely be a
   baseline to add to paper," so it is a paper-grade arm (polish the name, the
   in-source comment, and the config `_comment`; ensure deterministic). Like
   `INCR_SCRATCH`, it is **not** an E3 gate — E3 (the P0.4 project-evaluation
   suite north-star) stays on the plain `INCR` arms. The new arm reports into
   `comparison_summary.csv` for the figure/analysis, not the pass/fail gate.

## The expected A/B read

Once implemented + the user re-runs the A/B, `comparison_summary.csv` gains a
row for the new arm. The comparison table (avg SP + `Mean_Scheduler_Execution_Time_s`):

| Pair | Question | Expected signal |
|------|----------|-----------------|
| NEW vs `INCR_SCRATCH` | does warm-start (carrying incumbent) beat amnesiac from-scratch? | NEW ≥ INCR_SCRATCH in SP (warm-start should weakly dominate); ET cheaper (no from-scratch descent after interval 0) |
| NEW vs `INCR_P1` | does periodic reopt earn its cost? | if NEW ≈ INCR_P1 in SP at lower ET → reopt not earning its cost (feeds P1.2); if INCR_P1 > NEW → reopt bounds drift the incremental walk can't |

## Done when (implementation phase — NOT started)

- [ ] Design decisions 1, 3, 4 settled with the user (D2 + D5 settled
      2026-07-11).
- [ ] TDD: tests first in `tests/testIncreOpt_w_TL.cpp` — interval 0 produces
      the RM-fast incumbent with zero descent evals; interval 1+ takes the
      incremental path; no `ReOptimizePeriodic` call across a multi-interval
      run; the incumbent is **carried** across intervals (interval N's adopted
      solution seeds interval N+1 — the arm uses the persistent optimizer, NOT
      fresh-each-interval like `INCR_SCRATCH`).
- [ ] New `scheduler_mode_ == "<D1 name>"` branch wired in
      `SimulationOrchestrator.cpp` dispatch (`DeterminePrioritiesAndBudgets`)
      + a clause in the pre-loop construction condition (`:293`) so the
      persistent `incr_optimizer_` is built.
- [ ] New `BootstrapIncumbentFromRMFast()` method (or D3-chosen equivalent) in
      `OptimizeSP_TL_Incre.{h,cpp}` — runs the `ResetIncumbentBaseline(true)`
      interval-0 seed step only, skips the descent.
- [ ] `p25_period_ab_config.json` updated (arm added to both modes + comments);
      confirm with user whether it also goes in the P0.3 prod-mode list.
- [ ] `cmake --build build --target check.SP_OPT -j5` (DEBUG) green; existing
      48/48 `testIncreOpt_w_TL` + 16/16 ctest still green.
- [ ] `agents/overall_tasks.md` + top-level `agents/dev_log.md` updated.
- [ ] `git add` staged; user reviews, rebuilds `release/`, re-runs the A/B.

## Out of scope

- **No implementation now** — the user asked for the task folder only.
- `git commit` — user's standing constraint (`git add` only).
- Running the A/B myself — user runs `run_simulation_and_plot_figures.sh`.
- Re-litigating P1.1 (residual ET growth) or P1.4 (seed policy) — this arm
  inherits P1.4's seed unchanged; it asks a different question (reopt value +
  warm-start value).
- Changing the interval-0 RM-fast seed itself — it is `IfInitialized()`-gated
  and irreducible (no prior exists at interval 0 for any policy).
