# P0.6 Static Solution — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-28 (P0.9 supersedence — plan docs relabeled RM→DM)

- **P0.9 (DM + important-first group lock) LANDED** for the seed PA system-wide.
  The seed's plain-RM sort is now `DeadlineMonotonicPriorityVec` (group-locked DM),
  and the seed bootstrap is `SeedIncumbentFromDMFast` / `BootstrapIncumbentFromDMFast`.
- To keep this task's forward-looking plan (`goal.md` + `tasks.md`) consistent with
  the landed seed, those two files were relabeled RM→DM on 2026-07-28: the planned
  helper is now `AssignDMRespectingGroupOrder` (extracted from the now-deadline-based
  orchestrator sort; the bare `DM`/`DM_FAST`/`DM_SLOW` branches call it with an empty
  group = behavior-identical, the static solution calls it with `important_ids`), the
  seed point is "DM-grouped + min-TL + WCET", and within-group ordering is "DM-ordered".
- **Historical entries below retain their original RM wording** (point-in-time records,
  not rewritten — same "don't falsify history" treatment as the P2.11 leave). They
  describe what was true when written; the relabeled plan docs above are authoritative.

## 2026-07-26

- Task scaffolded from the user's fall-back design direction. Scope + 4 open design
  decisions (D1–D4) recorded in `goal.md`; `tasks.md` checklist written.
- **BLOCKER:** D1 (important-task selection rule) must be settled with the user before
  any code — it is shared with P0.7 (fall-back) and P0.8 (schedulability). Candidates
  (a) top-X% by `sp_weight` [existing convention], (b) by `sp_weight * perf_coefficient`,
  (c) by strict `sp_threshold`, (d) union/intersection. Recommendation to bring: (a)
  for v1, (c) as a follow-up filter post-P2.13.
- Code grounding confirmed: RM period-sort at `SimulationOrchestrator.cpp:356-367`
  (triplicated at 368/388 → candidate for the `AssignRMRespectingGroupOrder` helper);
  `INCR_WCET` WCET-semantics path at 348-355 (`use_wcet_execution_time = true`);
  offline compute site between optimizer construction (300-302) and the interval loop
  (304-308); ET bracket at `DeterminePrioritiesAndBudgets` 316-322 (static solution
  stays OUTSIDE so it doesn't inflate the online ET metric).
- Not started; awaiting D1 resolution + P0.8's schedulability guarantee.

## 2026-07-27

- **ALGORITHM REFINEMENT (user direction).** The static solution is no longer "RM-grouped
  + flat scalar WCET, no optimization." New algorithm: (1) seed RM-grouped PA + min-TL
  (`SmallestTimeLimitVec`); (2) under `use_wcet_execution_time = true`, run the incremental
  TL walk from that seed; (3) early-stop the walk on the first candidate TL that improves
  global SP but breaks important-task schedulability (inline RTA, R_i ≤ deadline_i); (4)
  return the committed incumbent. `goal.md` + `tasks.md` rewritten around this.
- **Key code grounding confirmed during the refinement:**
  - The existing TL walk is **TL-only** — `WalkOneTaskWithTimeLimitOptions`
    (`OptimizeSP_TL_Incre.cpp:500+`) mutates only `time_limits[task_idx]`; `opt_pa_` is
    inherited from the seed and never changed. So "fix priority, walk TL" is the walk's
    natural behavior — **no new fix-priority flag needed** (the existing
    `disable_time_limit_opt` is the inverse: fix TL, walk PA).
  - `SeedIncumbentFromRMFast` (`OptimizeSP_TL_Incre.cpp:758-766`) already seeds RM PA +
    min-TL — but plain RM (`RateMonotonicPriorityVec`), not grouped. P0.6 needs the grouped
    variant (`AssignRMRespectingGroupOrder`).
  - Under WCET mode, non-perf tasks collapse to a point mass (TL=-1, no freedom); only perf
    tasks retain TL freedom → the walk adjusts perf-task TLs only, from min (safest) upward.
  - The early-stop guard is genuinely NEW — `IsBetterTimeLimitOption` adopts any SP-better
    TL; there is no schedulability guard in the walk today.
  - Under WCET mode the RTA dist is a point mass, so DDL-miss (0/1) ≡ R_i ≤ deadline_i →
    the guard's RTA form and the DDL-miss form coincide (D6).
- **New open decisions added:** D5 (guard shape: virtual hook / wrapper / post-walk
  filter), D6 (guard quantity: RTA vs DDL-miss — equivalent under WCET), D7 (HALT vs
  SKIP-AND-CONTINUE on first unsafe candidate — user said "we'll stop" → default HALT).
- **P0.8 coordination:** P0.8's RTA must certify the SAME seed point P0.6 seeds from
  (RM-grouped + min-TL + WCET). If even the seed is unschedulable for important tasks, the
  walk has no safe start → P0.8's re-generate handles it. P0.6's guard preserves
  schedulability DURING the walk. Recorded in `goal.md` "Relationship to P0.8."
- Still not started; awaiting D1 (+ D5/D6/D7) resolution + P0.8.

## 2026-07-27 (later)

- **D1 RESOLVED.** The cross-cutting "how to decide important tasks" question is settled:
  **50% of the tasks in a taskset are important, indicated by SP weights; the rest are
  non-important.** The label is **persisted** as `bool is_important` on the C++ `Task`
  class — set at generation, read by all consumers. This is candidate (a) (top-X% by
  `sp_weight`) with **X = 50** (overriding the old 10%) and the label persisted rather
  than recomputed per consumer.
- **Why persisted (grounding):** `sp_weight` is NOT on `Task` today — it lives in
  `ParametersSP::weights_node` (`ParametersSP.h:51`), loaded from YAML
  (`ParametersSP.cpp:25-29`). The generator assigns it at `taskset_generator.py:551`
  (P2.15 continuous uniform) and `yaml_exporter.py:68` emits it. So the natural place to
  SET the bool is the generator, right after `sp_weight` assignment: sort by weight desc,
  mark the top-`ceil(N/2)` = `(N+1)//2` `is_important = true`, emit `important:` to YAML.
  C++ `Task` reads it at `ReadTaskSet` construction. This promotes "important" from a
  post-hoc *analysis* label (`compute_important_task_miss_rate`, `utils.py:182-220`, sort
  + `ceil(N*0.10)`) to a **generation-time property** — unifying P0.6 (priority lock +
  early-stop guard), P0.7 (safety check), P0.8 (Python RTA), and the analysis path on ONE
  definition (all read the bool → no drift). It also gives P0.8 the important set directly
  from the generated taskset — no duplicate top-X% logic in Python.
- **Tie-breaking:** `sp_weight` continuous uniform [0.1, 1.0] → ties measure-zero → clean
  50% boundary. If a tie lands exactly on the boundary, break by task id (deterministic).
- **Config migration:** the old `important_task_top_percentage = 0.10`
  (`paper_simulation_config.json:66`) + `minimum_important_tasks_count = 1` (line 67) are
  ANALYSIS-config knobs. Under the new rule the fraction is a GENERATION parameter —
  default `IMPORTANT_TASK_RATIO: 0.5` in the generator CONFIG_SPECS; the analysis path
  reads the bool and drops its own pct (ruthless-prune the dead knobs). Exact knob
  placement is an implementation detail, NOT a blocker.
- **Recorded as a SHARED enabling change** in `tasks.md` step 0.5 (new): add `bool
  is_important` to `Task` + generator labeling + `ReadTaskSet` parse + analysis migration,
  landed BEFORE the P0.6 walk (and pairing with P0.8's Python pass). TDD: a taskset of N
  tasks has exactly `ceil(N/2)` important; the important set = the top-weight half; ties
  broken by id. D1 checkbox in `goal.md` "Done when" + `tasks.md` section 0 marked done.
- **Remaining open:** D2 (perf-task WCET + min-TL seed fields, shared with P0.8), D3
  (per-taskset), D4 (in-memory vs file), D5/D6/D7 (guard shape/quantity/halt). D5/D6/D7
  have defaults from the 2026-07-27 refinement; D2 is the next one to confirm with the
  user. Still not started on code.

## 2026-07-27 (final design lock)

- **BUDGET ASYMMETRY (user direction).** "overall, we have high budget for offline
  analysis, so we can try more 'walk' during offline analysis, just find a safe solution
  with good performance. during online, budget is very tight, use this to adjust your
  design." This splits the prior single "early-stop HALT guard" into two:
  - **Offline (THIS task, ample budget):** the walk **skip-and-continues** on unsafe
    candidates — keep exploring to find the **best-SP safe** point (not just the first
    safe point). Returns a stronger safe floor.
  - **Online (P0.7, tight budget):** the guard **halts** on the first unsafe candidate,
    adopts the incumbent-so-far, compares its SP vs `static_solution_`'s SP, picks the
    higher-SP winner. The HALT semantics live in P0.7, NOT here.
  - `goal.md`/`tasks.md` rewritten: D7 = skip-and-continue offline; the HALT guard is
    P0.7's trigger (b). The filter hook renamed `ShouldAdoptCandidate` (was
    `ShouldStopWalk` — the old name implied halt).
- **D2 RESOLVED (WCET value).** WCET per task = the **max ET that task exhibits across all
  generated interval tasksets** (global max, NOT the task's own `execution_time_dist.max_time`).
  User confirmed interval ETs are pre-generated → global max computable offline. This is
  what makes P0.7's ET-jump trigger (a) safe by construction: any interval's jumped ET ≤
  the global max the static solution was computed at. The existing
  `ApplyWCETAblationIfRequired` (`OptimizeSP_TL_Incre.cpp:830-840`) collapses to per-task
  `execution_time_dist.max_time`; P0.6 adds a `ComputeGlobalMaxWCETPerTask(dag_tasks_vecs_)`
  precompute that takes the max across intervals per task id, then collapses to that point
  mass. Exact acquisition mechanism code-verified separately (task #7).
- **D3/D4 RESOLVED.** D3 = per-taskset (orchestrator runs one taskset per worker invocation
  → per-taskset == once per run; no cross-taskset reuse). D4 = in-memory `static_solution_`
  member (the fall-back reads it inside `SimulateInterval`); file dump is inspectability-only,
  default off.
- **D5/D6 RESOLVED.** D5 = virtual hook `ShouldAdoptCandidate(candidate_sp,
  candidate_schedulable)` on the walk (the walk's existing virtuals
  `CallOptimizerGivenTimeLimits`/`OptimizeIncreSingleTask` suggest the pattern). D6 = RTA
  form (R_i ≤ deadline_i) offline — under WCET the RTA dist is a point mass, so DDL-miss
  is 0/1 ≡ R_i ≤ vs > deadline_i (the two coincide offline). The online guard (P0.7), which
  runs OUTSIDE WCET mode, uses the real DDL-miss-chance from the SP metric.
- **Two online triggers clarified (P0.7 scope, recorded here for cross-reference):**
  (a) ET-jump, BEFORE optimization — optimizer holds the old dag; new dag comes in; if any
  task's Gaussian `et_mean` ≥ 1.5× saved old dag's → use `static_solution_` directly for
  that interval (skip the online walk); (b) during-walk early-stop guard — first candidate
  with important-task DDL-miss-chance > its SP threshold → HALT, adopt incumbent-so-far,
  compare its SP vs `static_solution_`'s SP, pick higher-SP winner.
- **A/B purpose clarified (user).** {INCR-family + full fall-back chain (static solution +
  2 triggers + compare)} vs {today's INCR-family, no fall-back}, user re-runs prod, metric
  = SP the fall-back costs. Quantifies the performance penalty of adding the fall-back.
- All D1–D7 now RESOLVED. `goal.md` "Open decisions" + "Done when" updated; `tasks.md`
  step 0 (design) all checked. Next: step 0.5 (`bool is_important`) — gated on the
  parallel P0.8 owner syncing the same seed point. Still no code; awaiting step-0.5 start.
