# PW.1.2 — Tasks (working checklist)

> See `goal.md` for scope. Second slice of the code read: priority assignment +
> TL (task-config) optimization. Mostly a reading task.

## Read the optimization sources
- [ ] `OptimizeSP_BF.{h,cpp}` — brute-force PA
- [ ] `OptimizeSP_Incre.{h,cpp}` — `FindTaskWithDifferentEt`, `FindEnvTaskWithDifferentEt`, `RemoveOneTask`, `GetProrityIndex`
- [ ] `OptimizeSP_TL_BF.{h,cpp}` — `EnumeratePA_with_TimeLimits`
- [ ] `OptimizeSP_TL_Incre.{h,cpp}` — `OptimizePA_Incre_with_TimeLimits`; `OptimizeIncre_w_TL` (virtual), `RunIntervalDescent`, `ReconstructTimeLimitVecFromResOpt`, `IntervalDescentMode`
- [ ] `PriorityBuilders.{h,cpp}` — `SortKey`, `GroupLock`, `TimeLimitPolicy`, `PriorityBuilderConfig`, `BuildPriorityPlan`
- [ ] Config: `DEADLINE_MODE=implicit`; DM priority building (commit `0b9dae4a`)

## Write `sketch_optimization.md` (items 1–3 from goal.md)
> For each item state BOTH the **algorithm** (what) AND the **motivation** (why it exists / what tension it resolves), tied to the concrete symbol/file.

- [ ] 1. Priority assignment — (a) brute force `n!`, (b) modified Audsley + beam search (**heuristic**, not "optimal"), (c) incremental ±1 (four scenarios); DM seed + important-first group lock. *Why: SP non-monotone in ordering; BF=optimal small-N/offline, Audsley+beam=tractable, incremental=bounded online cost.*
- [ ] 2. TL coordinate descent: ordering, δ→3 candidates, tie-break; O(M^N)→O(M·N); 1-task-ET-diff property (`\agent` theorem note → proof vs stated); `eq: incremental_configuration`. *Why: sidesteps M^N grid; TL trades safety vs performance.*
- [ ] 3. Env-task reframing (`\sen` note): TL-optimizable = special env-dependent task → `FindEnvTaskWithDifferentEt` path. *Why: unifies TL-flex with ET-change handling under one incremental mechanism.*
- [ ] Each item tied to the concrete symbol/file it came from
- [ ] Modified-Audsley + beam search labeled heuristic, NOT "optimal"

## Verification
- [ ] Each of 1–3 states both algorithm AND motivation
- [ ] `\agent` theorem question has a recommendation (proof or stated property)
- [ ] A reader could rewrite methodology §7/§8 using only this sketch + existing `.tex`, without re-reading C++
