# PW.1.2 — Tasks (working checklist)

> See `goal.md` for scope. Second slice of the code read: priority assignment +
> TL (task-config) optimization. Mostly a reading task.

## Read the optimization sources
- [x] `OptimizeSP_BF.{h,cpp}` — brute-force PA
- [x] `OptimizeSP_Incre.{h,cpp}` — `FindTaskWithDifferentEt`, `FindEnvTaskWithDifferentEt`, `RemoveOneTask`, `GetProrityIndex`
- [x] `OptimizeSP_TL_BF.{h,cpp}` — `EnumeratePA_with_TimeLimits`
- [x] `OptimizeSP_TL_Incre.{h,cpp}` — `OptimizePA_Incre_with_TimeLimits`; `OptimizeIncre_w_TL` (virtual), `RunIntervalDescent`, `ReconstructTimeLimitVecFromResOpt`, `IntervalDescentMode`
- [x] `PriorityBuilders.{h,cpp}` — `SortKey`, `GroupLock`, `TimeLimitPolicy`, `PriorityBuilderConfig`, `BuildPriorityPlan`
- [x] Config: `DEADLINE_MODE=implicit`; DM priority building (commit `0b9dae4a`)

## Write `sketch_optimization.md` (items 1–3 from goal.md)
> For each item state BOTH the **algorithm** (what) AND the **motivation** (why it exists / what tension it resolves), tied to the concrete symbol/file.

- [x] 1. Priority assignment — (a) brute force `n!`, (b) modified Audsley + beam search (**heuristic**, not "optimal"), (c) incremental ±1 (four scenarios); DM seed + important-first group lock. *Why: SP non-monotone in ordering; BF=optimal small-N/offline, Audsley+beam=tractable, incremental=bounded online cost.*
- [x] 2. TL coordinate descent: ordering, δ→3 candidates, tie-break; O(M^N)→O(M·N); 1-task-ET-diff property (`\agent` theorem note → proof vs stated); `eq: incremental_configuration`. *Why: sidesteps M^N grid; TL trades safety vs performance.*
- [x] 3. Env-task reframing (`\sen` note): TL-optimizable = special env-dependent task → `FindEnvTaskWithDifferentEt` path. *Why: unifies TL-flex with ET-change handling under one incremental mechanism.*
- [x] Each item tied to the concrete symbol/file it came from
- [x] Modified-Audsley + beam search labeled heuristic, NOT "optimal"

## Verification
- [x] Each of 1–3 states both algorithm AND motivation
- [x] `\agent` theorem question has a recommendation (proof or stated property) → stated property backed by structural invariant
- [x] A reader could rewrite methodology §7/§8 using only this sketch + existing `.tex`, without re-reading C++
