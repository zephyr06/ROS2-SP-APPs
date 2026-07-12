# P1.6 — Pure Incremental (RM-Fast Bootstrap, No Reopt) Baseline — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-11

- Task **created (PLANNING ONLY — no implementation)** per the user's
  directive: "add a new active task, which is adding a new baseline where we
  mainly use incremental optimization to perform per-interval optimization, and
  we start the RM-Fast rather than re-optimization in the first interval, and
  never call re-optimization periodically. only add this active task without
  implementation. purpose is to compare performance between incr and scratch."
- **Investigated the dispatch + bootstrap flow** to specify the arm accurately:
  - `Optimize_w_TL_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:291`) routes on
    `reoptimization_interval_count_ % ReoptimizationPeriod == 0` →
    `ReOptimizePeriodic` (from-scratch + compare-and-keep), else →
    `OptimizeIncre_w_TL` (warm-started incremental). `count==0` →
    `ReOptimizePeriodic`, the bootstrap.
  - `ReOptimizePeriodic` (`:453`) does TWO things at interval 0: (1) seed the
    RM-fast incumbent via `ResetIncumbentBaseline(true)`'s `else` branch
    (`:437`) — `SmallestTimeLimitVec()` (= `timePerformancePairs[0].time_limit`,
    i.e. the orchestrator's `RM_FAST` TL, `SimulationOrchestrator.cpp:380-383`)
    + `RateMonotonicPriorityVec()` → `SeedStateFromIncumbent` →
    `CommitIncumbent`; (2) the full from-scratch descent
    `PerformCoordinateDescentForTaskConfigOpt(..., from_scratch=true)`.
  - The new arm = run step (1) only at interval 0 (seed RM-fast, **skip the
    descent**), then `OptimizeIncre_w_TL` every interval after, **never**
    `ReOptimizePeriodic`. So it is NOT just `ReoptimizationPeriod=∞` (that
    still runs the interval-0 descent); it additionally skips that descent.
- **Wrote `goal.md`** — the finding (what the arm is, with the
  interval-0/descent distinction), the reasoning (clean incr-vs-scratch
  comparison: isolates warm-start value vs `INCR_SCRATCH`, reopt value vs
  `INCR`; de-risks the P1.2 structural-corruption hazard by never running the
  memoryless from-scratch search), 5 open design decisions flagged for the
  user (mode name, branch-vs-flag, interval-0 mechanics, config placement,
  E3-gating), the expected A/B read, and Done-when / Out-of-scope.
- **Wrote `tasks.md`** — Step 0 (settle design decisions with the user, NOT
  started) through Step 4 (build + test + index), all marked NOT STARTED,
  TDD-first per the coding rules.
- **No source code touched.** No `git add`. Folder is `goal.md` + `tasks.md` +
  this `dev_log.md` only. Next action is the user's: greenlight implementation
  (and settle the 5 design decisions), or adjust the task scope.

## 2026-07-11 (follow-up — plans only, still no implementation)

- User follow-up: "we should implement it similar to INCR_sCRATCH, it'll likely
  be a baseline to add to paper. only update plans without implementation. also,
  difference between INCR-P1 and INCR-SCRATCH??"
- **Answered the INCR_P1-vs-INCR_SCRATCH question** by reading the dispatch
  (`OptimizeSP_TL_Incre.cpp:291-307, 424-488`) + orchestrator branches
  (`SimulationOrchestrator.cpp:293-337`). Both arms run the from-scratch descent
  `ReOptimizePeriodic` every interval — that is NOT the difference. The
  difference is optimizer lifetime / incumbent-carrying:
  - `INCR_P1` uses the **persistent** `incr_optimizer_` → `res_opt_` survives →
    `ResetIncumbentBaseline(true)` takes the `if (IfInitialized())` branch
    (`:427-436`) at interval 1+ → compare-and-keep measures against the
    **running best**.
  - `INCR_SCRATCH` builds a **fresh** `scratch_opt` each interval
    (`SimulationOrchestrator.cpp:334`) and discards it → `res_opt_` empty →
    `ResetIncumbentBaseline(true)` takes the `else` branch (`:437-447`) every
    interval → compare-and-keep measures against a **synthetic RM baseline**.
  - Net: `INCR_P1` weakly dominates `INCR_SCRATCH` in SP; `INCR_SCRATCH` is the
    amnesiac (no-memory) control. (In-source comment at `:328-333` states this
    but still names `prev_optimizer_`, the member P0.5 removed — stale wording,
    not stale logic; it now runs through `res_opt_`.)
- **Settled D2 + D5** per the user's directive:
  - D2 → **(a) new `scheduler_mode_` branch** (mirror `INCR_SCRATCH`'s dispatch
    shape; NOT a `GlobalVariables` flag).
  - D5 → **paper-grade baseline; A/B probe, NOT an E3 gate** (E3 stays on plain
    `INCR`).
- **Flagged the persistent-vs-fresh nuance** (the one subtlety in "similar to
  INCR_SCRATCH"): the new arm mirrors `INCR_SCRATCH`'s dispatch *shape* (a
  dedicated branch) but MUST use the **persistent** `incr_optimizer_` to carry
  the incumbent (like `INCR`) — NOT the fresh-`scratch_opt`-each-interval
  lifetime (that is `INCR_SCRATCH`'s amnesia, the exact thing the new arm
  contrasts). Recorded in `goal.md` "What the arm is NOT" + the three-way
  contrast table + `tasks.md` Step 2.
- **Updated `goal.md`**: added the INCR_P1-vs-INCR_SCRATCH subsection + a
  three-way contrast table (NEW arm vs INCR_P1 vs INCR_SCRATCH across
  incumbent-carry / per-interval descent / compare-and-keep baseline); rewrote
  the reasoning to name `INCR_P1` (not generic `INCR`) as the reopt sibling and
  note the softening contrast at higher periods; marked D2+D5 SETTLED in the
  design-decisions section; added the persistent-optimizer clarification to
  Done-when + "What the arm is NOT."
- **Updated `tasks.md`**: Step 0 now shows D2+D5 checked (SETTLED), D1/D3/D4
  open; Step 1 + Step 2 + Standing constraints carry the persistent-optimizer
  nuance.
- **No source code touched.** No `git add`. D1 (mode name), D3 (interval-0
  mechanics), D4 (config placement) remain open for the user.
