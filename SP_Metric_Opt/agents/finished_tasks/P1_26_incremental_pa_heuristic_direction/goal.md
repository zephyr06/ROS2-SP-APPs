# P1.26 — Incremental PA Re-Search Direction Heuristic Exploration

**Priority:** P1 (active investigation; potential paper §6.3 + code change)
**Status:** ACTIVE — flag implemented (uncommitted), A/B pending.
**Reference:** memory `explore-incremental-pa-heuristic-direction.md`;
paper §6.3 `section_increment_pa` (the 4-item enumerate that mirrors the code).

## The heuristic under scrutiny

`AnalyzePriorityChangeStatus` (`sources/Optimization/OptimizeSP_Incre.cpp:299`)
drives the 1D priority re-search direction for a single changed task in the
sub-incremental path `OptimizeIncre_SingleTask` (`:324`). It returns one of
`Increase` / `Decrease` / `OpenToAll`, which `FindPriorityVec1D_Variations`
(`:~250`) turns into a search range (index 0 = highest priority):

- `Increase` → scan `[0, old_idx]` (toward HIGHER priority).
- `Decrease` → scan `[old_idx, size-1] (toward LOWER priority).
- `OpenToAll` → full range `[0, size-1]`.

The current 4-scenario rule (weight = `if_highest_weight_unique`):

| # | ET change | weight            | direction  |
|---|-----------|-------------------|------------|
| 1 | ET↑       | highest-unique    | `Increase` |
| 2 | ET↑       | not-highest       | `Decrease` |
| 3 | ET↓       | highest-unique    | `Decrease` |  ← under scrutiny
| 4 | ET↓       | not-highest       | `Increase` |

## The concern (user, 2026-08-06)

**Case 3 "doesn't make sense":** the *most important* task got *faster*, yet the
rule scans DOWNWARD (lower priority), which can only *increase* its response time
/ miss probability. The in-code rationale (`:306` region: "it requires less
resources, so assign lower priority") treats the top-weight task like a generic
less-important task — but for the uniquely-highest-weight task, lowering priority
risks the safety term that matters most. Pre-flagged by the standing TODO at
`:288` ("re-evaluate this heuristic, i feel we can do better with trial-and-error
walk").

Removing the rule = collapse case 3 to case 4's behavior: the whole ET-decreased
branch becomes weight-agnostic `Increase`.

## The change (already implemented, uncommitted)

Runtime flag `pa_research_heuristic_variant` in `sources/parameters.yaml:42`,
declared `sources/Utils/Parameters.h:51`, loaded `sources/Utils/Parameters.cpp:36`
(try/catch fallback default 0). `AnalyzePriorityChangeStatus` consults it ONLY for
the ET↓ + highest-weight sub-case (`:310` switch):

- `0` = baseline: `Decrease` — **prod byte-identical** (default; original flow).
- `1` = remove the rule: `Increase` (ET↓ uniformly weight-agnostic).
- `2` = `OpenToAll` (scan the full priority range).

Every other sub-case is unaffected. Default 0 leaves control flow unchanged, so
SP is bit-identical to HEAD at variant 0.

## Experiment plan

Focused e2e A/B isolating `AnalyzePriorityChangeStatus`. Config
`simulation_experiments/configs/pa_heuristic_ab.json` runs ONLY `INCR_Reopt_10`
(the prod arm) at N=8 and N=10, 5 tasksets, 300s sim, interval 10s. Driver
`_ab_run.sh` (repo root).

Procedure (identical tasksets across both flag values — `RERUN_MODE` controls
this):

1. **Verify byte-identity at variant 0:** build test + `make check.SP_OPT -j5`
   green; confirm flag default 0.
2. **Baseline:** flag=0, `RERUN_MODE=clear_all` → generates tasksets + runs.
   **Snapshot** `comparison_summary.csv` (+ per-taskset `interval_sp_metrics.txt`)
   to a `baseline/` copy — the variant run writes the same run dir and would
   overwrite.
3. **Variant:** flip flag→1 in `parameters.yaml`, `RERUN_MODE=clear_results`
   (reuses tasksets, reruns sim only). Snapshot variant summary.
4. **Compare** `Mean_SP_Metric` (and per-taskset SP) baseline vs variant.
5. **(if signal warrants)** flag=2 (`OpenToAll`) the same way.
6. **Restore** flag→0 (prod default) before handing back.

Metric of record: `Mean_SP_Metric` in `comparison_summary.csv` (normalized SP,
`mean_sp_norm`). Sign of interest: does removing case 3 (scan UP for a faster
top-weight task) improve, match, or hurt SP?

## Files

- `sources/Optimization/OptimizeSP_Incre.cpp` — `AnalyzePriorityChangeStatus`
  (`:299-322`), the flag switch (`:310-317`), the TODO (`:288`),
  `FindPriorityVec1D_Variations` (`:~250`), `OptimizeIncre_SingleTask` (`:324`).
- `sources/Utils/Parameters.{h,cpp}` + `sources/parameters.yaml:42` — the flag.
- `simulation_experiments/configs/pa_heuristic_ab.json` — focused A/B config.
- `_ab_run.sh` (repo root) — A/B driver.
- Paper §6.3 `section_increment_pa` — the 4-item enumerate mirroring the code
  (update only if the A/B picks variant 1/2).

## Done when

- Variant 0 confirmed byte-identical (ctest green, default unchanged).
- Baseline (flag=0) and variant (flag=1) A/B run on identical tasksets;
  `Mean_SP_Metric` compared at N=8 and N=10.
- A recorded verdict: removing case 3 improves / matches / hurts SP.
- If variant wins: paper §6.3 enumerate updated to match; flag default flipped
  (user-go). If baseline wins or ties: record negative result, keep rule, close.
- Flag restored to 0; milestone to top-level `agents/dev_log.md`.

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Changing the flag default in `parameters.yaml` to non-0 before the A/B
  converges (prod must stay byte-identical until a decision).
- Re-architecting `AnalyzePriorityChangeStatus` beyond the 3-variant switch
  (e.g. a full trial-and-error walk per the `:288` TODO) — that is a separate,
  larger task; this exploration only tests removing case 3.
- Touching the ET-increased branch or the not-highest-weight ET-decreased case.
