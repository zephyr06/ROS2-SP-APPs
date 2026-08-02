# §8 — Task Configuration Optimization

> Draft: `section8_task_config_opt.tex`. Status: row-by-row DONE.
> See `overall_revision_plan.md` for conventions.

## High-level change

§8 is the single highest-rewrite-density section (MOST STALE). The whole section
is built on a δ-radius local-search framing the code does NOT use, plus two
unresolved `\sen`/`\agent`/`\rkwprev` note threads. The new organizing idea: a
TL-optimizable task is a *special kind of env-dependent task* (the `\sen`
reframing, content change 3), so the §7.3 incremental mechanism handles both
ET-change and TL-flex under one path. The δ-radius equation must be replaced with
the patience-bounded outward coordinate descent the code actually runs; the
`\agent` algorithm note must be promoted to body prose; and the 1-task-ET-diff
property must be stated (not proven as a theorem).

## Subsection rows

### §8 lede + env-task reframing — DRIFT → implement `\sen` directive (content change 3)
- **draft claim:** two strategies (brute-force + incremental); opens with two
  unresolved `\sen` notes (4-5): "treat tasks that need to optimize TLs as a
  special type of env-dependent task, then trigger incremental optimization
  after assuming tasks' ET change" + "modify the paper's scope and definition …
  more precise and consistent."
- **code reality:** this is EXACTLY what the code does —
  `FindEnvTaskWithDifferentEt` (`OptimizeSP_TL_Incre.cpp`) treats TL-flexible
  tasks as a special env-dependent task and triggers the incremental walk by
  assuming their ET changed. The `\sen` directive IS the section-8 conceptual
  reframing (content change 3). See `sketch_optimization.md` §env-task.
- **action:** **REWRITE** §8 lede to implement the `\sen` reframing (delete the
  `\sen` notes, promote their content into body prose): define TL-optimizable
  tasks as a special kind of env-dependent task; the incremental mechanism (§7.3)
  then handles both ET-change and TL-flex under one path. This is the section's
  new organizing idea.

### §8 smooth assumption + `eq: incremental_configuration` — DRIFT (STALE)
- **draft claim** (9): "incremental strategy assumes environment changes smoothly
  … influence on optimal config changes smoothly." `eq: incremental_configuration`
  (22-25): `‖λ − λ^(k)‖ ≤ δ`, "δ controls tradeoff between run-time complexity and
  solution quality." Example (27-30): δ=150 → "only 2 candidates: 200, 300."
- **code reality:** the code does NOT use a δ-radius local search. TL
  optimization is a **patience-bounded outward coordinate-descent walk over the
  FULL grid** (`WalkOneTaskWithTimeLimitOptions` `OptimizeSP_TL_Incre.cpp:647-694`,
  patience 0/1, `parameters.yaml:30-31`). There is no δ cap; the walk expands
  outward until SP stops improving (patience bound). The "3 candidates"/"δ-radius"
  framing is STALE (`RecordCloseTimeLimitOptions` exists only in tests, per
  `sketch_optimization.md`).
- **action:** **REPLACE** the δ-radius framing entirely: delete
  `eq: incremental_configuration` (22-25) and the δ=150 Example (27-30); rewrite
  as patience-bounded outward coordinate descent over the full TL grid. State the
  actual stopping rule (walk expands to neighbors while SP strictly improves,
  patience bound halts after K non-improving steps). Drop the "smooth assumption"
  (9) — replace with the dynamic/continuous framing. Cross-link
  `section: relax_smooth_assumption` (10) — verify that target still exists or
  update the `\ref`.
- **Ryan:** (related) 1.4 assumptions/regimes — the smoothness assumption is
  being removed.

### §8 coordinate-descent algorithm — PROMOTE `\agent` note to body (VERIFY content)
- **draft claim:** the actual algorithm is currently buried in an `\agent` margin
  note (32-39): linear-time sequential coordinate descent; init λ^(0) from closest
  valid TL option; prioritized task ordering (weight desc, deadline asc, ID asc);
  coordinate descent over each task's options keeping others fixed;
  resource-aware tie-break (min sum of TLs); O(M^N) → O(M·N).
- **code reality:** MATCHES code — `BuildSerializedTaskQueue`
  (`OptimizeSP_TL_Incre.cpp:435-484`) orders tasks (weight desc, deadline asc, ID
  asc); `WalkOneTaskWithTimeLimitOptions` does the per-task coordinate descent;
  tie-break minimizes total TL; O(M·N) complexity holds.
- **action:** **PROMOTE** the `\agent` note (32-39) into body prose (delete the
  `\agent` wrapper, keep the content as a numbered algorithm description).
  **VERIFY** each step still matches code after the §8 lede rewrite. State the
  O(M^N)→O(M·N) reduction as a proposition (see complexity row below).
- **Ryan:** (none directly — but this is the substance Ryan 1.2/2.4 want made
  rigorous).

### §8 complexity + solution-quality notes — RESOLVE (`\rkwprev`/`\Sen`/`\agent`)
- **draft claim:** `\rkwprev` (41) asks "can we prove anything about complexity or
  solution quality?"; `\Sen` (42) defers complexity to §10 and says quality is
  "very hard due to black-box probabilistic RTA"; `\agent` (45-47) claims
  "initializing a challenger from the champion guarantees each incremental
  optimization only handles 1 task with ET difference" and says "we should add a
  theorem with proof to this claim."
- **code reality:** complexity IS in §10 (matches `\Sen`). The 1-task-ET-diff
  property is enforced *structurally* by the RTA cache: `RTACache` throws on
  `|diff|>1` (`RTA_Cache.cpp:358`, `ComputeTaskSetDifference`) — the single-change
  invariant is a precondition the cache asserts, not a proven theorem. Per
  `sketch_optimization.md` recommendation: state it as a **stated property**
  (backed by the structural invariant), NOT a full formal theorem-with-proof (the
  proof would require formalizing the cache's move model, out of scope for
  T-ASE).
- **action:** **RESOLVE** the three notes: (1) delete `\rkwprev`/`\Sen`/`\agent`
  wrappers; (2) state the O(M·N) complexity as a proposition with a one-line
  argument (forward-pointer to §10 for full analysis); (3) state the
  1-task-ET-diff property as a **stated property / observation** backed by the
  single-change invariant the RTA cache enforces (cite the invariant, not a
  theorem). Do NOT add a formal theorem+proof (PW.1.2 recommendation).
- **Ryan:** 1.4 (honest claim calibration — don't overclaim a proof that isn't
  there).
