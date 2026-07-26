# P2.16 — Dev Log

## 2026-07-26 — Task filed; Step 1 in progress

**Context.** P2.11 landed two merge rounds — Phase 2 (`76c45114`, behavior change: legacy
reopt arm + `ReoptimizationUseSubIncrementalWalk` flag deleted; merged sub-incremental walk
+ Type-E queue + patience+1 = unconditional reopt) and Phase 1b-1e (`bd9f5912`,
behavior-neutral: two descent bodies → `RunIntervalDescent` + `SeedBaselineAndArmCache` +
`enum class IntervalDescentMode`; old bodies = 1-line virtual delegating test seams). HEAD
is now `d1d05827` (test taskset N8→N10) on top of `bd9f5912`.

The merges left readability artifacts: a dead `TraverseTimeLimitOptions` decl (its
`UpdateRecords` debug-print label is the only `.cpp` occurrence), a confusing
`OptimizeSingleTaskTimeLimit_Impl` suffix (orphaned by the Phase 2a wrapper deletion), dense
inline history comments re-narrating merged P2.11 state, and a duplicated TL-sum tie-break
in `UpdateRecords`. This task is the cleanup stage P2.11 deferred.

**Scope boundary (parallel agent).** Another agent is concurrently editing
`agents/agent_coding_rules.md`, `agents/overall_tasks.md`,
`simulation_experiments/configs/p211_reopt_ab_config.json`, `tests/RunSpeedTest.cpp`. This
task touches ONLY `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` +
`tests/testIncreOpt_w_TL.cpp` (all clean at HEAD `d1d05827`). No git operations; agent
`git add`s only; user commits each step.

**Plan approved** via plan mode (`/home/zephyr/.claude/plans/mutable-wobbling-sunrise.md`):
4 steps (the original Step 3 — flipping `OptimizeWithTimeLimitOptDisabled`'s `from_scratch`
arg — was dropped per user "Keep as-is, skip Step 3"; steps renumbered).

**Dead-code verified.** `grep -rn TraverseTimeLimitOptions sources/ tests/` → only the
header decl (`OptimizeSP_TL_Incre.h:84`) + the stale print label
(`OptimizeSP_TL_Incre.cpp:134`). No definition, no callers. (P2.10's checklist line "delete
`TraverseTimeLimitOptions`" was never executed because the P2.10 rename was reverted.)

**Step 1 started.** Removing the decl + the `if (GlobalVariables::debugMode){…}` print
block in `UpdateRecords` (`OptimizeSP_TL_Incre.cpp:131-136`).

**Step 1 DONE 2026-07-26** (working tree, staged, NOT committed).
- Removed the 2-line `TraverseTimeLimitOptions` decl from `OptimizeSP_TL_Incre.h`.
- Removed the 6-line `if (GlobalVariables::debugMode){…std::cout…"TraverseTimeLimitOptions:"…}`
  block in `UpdateRecords` (`OptimizeSP_TL_Incre.cpp`).
- `grep -rn TraverseTimeLimitOptions sources/ tests/` → empty.
- Sanity: `GlobalVariables::` still used 7× elsewhere in the `.cpp` (no orphan include);
  `std::cout` now 0× in the `.cpp` but no `<iostream>` include was direct (transitive via
  another header) → nothing to prune.
- `cmake --build build_test --target check.SP_OPT -j5 --clean-first` → **17/17 ctest green**
  (30.48s; `testIncreOpt_w_TL` 2.92s).
- `git add`ed the 2 files (staged = -10 lines: 3 header + 7 cpp). NOTE: `tests/RunSpeedTest.cpp`
  was already staged by the parallel agent → unstaged it to keep this step's staged set clean
  (only my 2 files). **Awaiting user review + commit.**

**Step 1 COMMITTED** as `d860aac5 "slight refactor"` (-10 lines: 3 header + 7 cpp). Two commits
landed on top of `d1d05827`: `6d0c7f7d` (parallel agent's `RunSpeedTest.cpp`) + `d860aac5` (mine).
HEAD now `d860aac5`; my 2 files clean.

## 2026-07-26 — Rescoped to Phase A → Phase B → Phase C

**User pivoted** from the original 4-step plan to a name-ACCURACY audit + inheritance break:
- Flagged `EvaluateTimeLimitConfig_SubIncremental` ("not really limited to TL-changed tasks — env
  tasks call it too") and `EvaluateTimeLimitConfig_ScratchOrIncre` ("bad function, used in 2 places
  for very different purposes") as misleading; asked for a broad accuracy pass + proposals.
- After my feedback (collision w/ base `OptimizeIncre_SingleTask`; cross-file cascade for
  `_w_TL_ScratchOrIncre`; etc.) the user gave **their** rename decisions (see `tasks.md` Phase A).
- Then: "first work on pure name change, after done, break the inheritance, update related doc."
  → **Phase A (pure name changes) → Phase B (break inheritance) → Phase C (docs).** Original
  Steps 3 (trim history comments) & 4 (extract `SumTimeLimitsExcludingSentinel`) DEFERRED.

**Name decisions (user, verbatim where given):**
| old | new | rationale |
|---|---|---|
| `EvaluateTimeLimitConfig_SubIncremental` | `OptimizeIncreSingleTask` | drops "TimeLimit" — it *runs* the incremental opt; env tasks call it too |
| `EvaluateTimeLimitConfig_ScratchOrIncre` | `CallOptimizerGivenTimeLimits` | "bad function, 2 different purposes" |
| `OptimizeOneTaskTimeLimit` | `OptimizeOneTaskWithTimeLimit` | additive clarity |
| `OptimizeSingleTaskTimeLimit_Impl` | `WalkOneTaskWithTimeLimitOptions` | drop `_Impl`; pairs w/ `WalkSerializedTaskQueue` |
| `Optimize_w_TL_ScratchOrIncre` | `DispatchReoptOrIncremental` | role = counter-driven reopt/incremental dispatch |
| `PerformCoordinateDescentForTaskConfigOpt` | REMOVE | "only one function call" (`.cpp:792`) |
| `K` (TL-subclass sigs) | `beam_search_width` | coding rule "no very short names like K" |
| `K` in `OptimizeIncreSingleTask` | DROP | unused (`(void)K;` at `:171`) |

**Inheritance break (Phase B).** User: "i'm thinking about to not let
`OptimizePA_Incre_with_TimeLimits` inherit from `OptimizePA_Incre`, as this is not really helpful.
if you really need inheritance, consider to use `OptimizePA_Base`. or just remove the inheritance."
This is the *real* fix for the `OptimizeIncreSingleTask` vs base `OptimizeIncre_SingleTask` name
proximity (same-instance callability confusion). Structural — filed as Phase B, planned before
editing; Phase A accepts the residual proximity.

**Scope expansion (vs original 2-file).** `DispatchReoptOrIncremental` cascade reaches
`SimulationOrchestrator.cpp` (3 call sites) + `testINCRTimeout.cpp` (2) + `testIncreOpt_w_TL.cpp`
(~10) + comment-only stragglers in `testScheduleSimulate.cpp` + `parameters.yaml`. Neither
`SimulationOrchestrator.cpp` nor `testINCRTimeout.cpp` is in the parallel agent's set.

**Reference map (grep-verified 2026-07-26, HEAD `d860aac5`):**
- `OptimizeSP_TL_Incre.h`: decls at `:94,:108,:120,:149,:189,:202`; comment refs `:195,:196,:199,:223,:227,:236`.
- `OptimizeSP_TL_Incre.cpp`: defs at `:134,:167,:502,:552,:579,:592`; CoutError strings `:159,:191`;
  call sites `:363,:369,:411,:563,:567,:570,:624,:792`; comment refs `:381,:556,:647,:773`.
- `testIncreOpt_w_TL.cpp`: override decls `:1083,:1091,:1176,:1683,:1732,:2016`; recursive/calls
  `:1088,:1097,:1181,:1690,:1739,:2033`; direct `_Impl` calls `:2099,:2125,:2151,:2174,:2198,:2219,:2241`;
  `Optimize_w_TL_ScratchOrIncre` calls `:1299,:1310,:1343,:1345,:1347,:1363,:1389,:1400,:1423`; many comments.
- `SimulationOrchestrator.cpp`: calls `:325,:334,:342`; comments `:24,:286,:293`.
- `testINCRTimeout.cpp`: calls `:95,:154`; comments `:28,:104,:119,:154,:167`.
- `testScheduleSimulate.cpp`: comment-only `:758,:803`.
- `parameters.yaml:15`: comment-only (names `PerformCoordinateDescentForTaskConfigOpt`).

**Phase A plan:** Stage A1 (`.h`/`.cpp` internal) → A2 (`testIncreOpt_w_TL.cpp`) → A3 (cross-file +
stragglers). Each stage = `git add` + 17/17 ctest gate + user commit. NOTE: A1 alone will leave the
build broken (test overrides not yet renamed) — A1g gate is a self-consistency check on the 2 source
files only; full green comes at A2h.

**Phase A NOT YET STARTED.** Beginning Stage A1 now.

## 2026-07-26 — Stage A1 + A2 DONE (working tree, staged for review)

**Clarified scope with user.** User message bundled "continue P2.16" + "integrate rta cache
into incremental+TL optimizer". Verified the RTA cache is ALREADY integrated into the TL
optimizer (committed under P1.12/P1.13): TL subclass owns `RTACache rta_cache_`
(`OptimizeSP_TL_Incre.h:266`); `OptimizeIncreSingleTask` routes the carried-PA re-score through
`rta_cache_.Evaluate` (`:214`) + passes `std::ref(rta_cache_)` into the base primitive (`:231`);
`CommitIncumbent` advances the champion (`:690-693`); rejects restore via `cache_backup` (`:236`);
`SeedBaselineAndArmCache` arms it (`:388,:418`); `ResetIncumbentBaseline` clears it (`:721-722`).
User confirmed: **finish P2.16 Phase A; no cache work.** P2.16 guardrails (`goal.md:72`) forbid
cache/algorithm changes anyway.

**Found working tree mid-Stage-A1.** The uncommitted diff already contained Stage A1 (the `.h`/`.cpp`
internal renames: `OptimizeSingleTaskTimeLimit_Impl`→`WalkOneTaskWithTimeLimitOptions`,
`OptimizeOneTaskTimeLimit`→`OptimizeOneTaskWithTimeLimit`,
`EvaluateTimeLimitConfig_SubIncremental`→`OptimizeIncreSingleTask` (+ drop unused `K`),
`EvaluateTimeLimitConfig_ScratchOrIncre`→`CallOptimizerGivenTimeLimits`,
`PerformCoordinateDescentForTaskConfigOpt` REMOVED + inlined `RunIntervalDescent(Reopt,…)` at
its sole caller, `K`→`beam_search_width` in all TL-subclass sigs). The previous session had ALSO
started Stage A2 (renamed the virtual override signatures + `MakeScratchOrIncreEval`) but left it
**incomplete and build-breaking**: 7 `opt.OptimizeSingleTaskTimeLimit_Impl(` call sites
(`:2101`–`:2243`) still used the old name (header renamed it → unresolved).

**Completed A2 this session:**
- 7 `OptimizeSingleTaskTimeLimit_Impl` call sites → `WalkOneTaskWithTimeLimitOptions`
  (build-breaking; `replace_all` on the call token).
- 3 `_Impl` comment refs (`:1934,:1991,:1992`) → `WalkOneTaskWithTimeLimitOptions` / `RunIntervalDescent(Reopt)`.
- `OptimizeOneTaskTimeLimit` comment ref (`:1167`) → `OptimizeOneTaskWithTimeLimit`.
- All `EvaluateTimeLimitConfig_ScratchOrIncre` comment refs (×11) → `CallOptimizerGivenTimeLimits`.
- All `EvaluateTimeLimitConfig_SubIncremental` comment refs (×4) → `OptimizeIncreSingleTask`.
- `PerformCoordinateDescentForTaskConfigOpt` comment refs (`:970,:1144,:1162,:1280,:1285,:1961`)
  → repointed to `RunIntervalDescent(Reopt)` / "the reopt descent" (kept the P2.11 5.5 regression
  narrative intact; only the dead symbol name swapped).

**A2f deferred to A3.** `Optimize_w_TL_ScratchOrIncre` → `DispatchReoptOrIncremental` was NOT
applied in A1/A2 — the `.h`/`.cpp` kept the old name. Renaming it cascades into
`SimulationOrchestrator.cpp` (3 call sites), `testINCRTimeout.cpp` (2), `testIncreOpt_w_TL.cpp`
(~10), `testScheduleSimulate.cpp` (comments), `parameters.yaml` (comment). Folded into Stage A3
as one cross-file unit so A1/A2 stay a clean 3-file review/commit set. `tasks.md` A3 restructured
accordingly (A3a renames the decl/def first, A3b–A3e the callers).

**Build gate (A2h).** `cmake --build build_test --target check.SP_OPT -j5 --clean-first` →
**17/17 ctest green** (20.43s; `testIncreOpt_w_TL` 2.09s). Behavior-neutral rename verified by
the existing `RunIntervalDescent_Incremental_MatchesWrapperSP` + `*BitIdentical*` guards.

**Staged for review.** `git add` of the 3 P2.16 files (`OptimizeSP_TL_Incre.{h,cpp}` +
`testIncreOpt_w_TL.cpp`). NOTE: the parallel agent's modified set (`agent_coding_rules.md`,
`overall_tasks.md`, `p211_reopt_ab_config.json`) is NOT staged — only my 3 files. **Awaiting user
review + commit.** NEXT: Stage A3 (cross-file `DispatchReoptOrIncremental` cascade).

## 2026-07-26 — Stage A1 + A2 COMMITTED `6d068ec9`; speed-test question answered

**Committed** as `6d068ec9 "code refactor, rename functions"` (-167/+144 across the 3 P2.16 files).
HEAD now `6d068ec9`. Working tree clean of P2.16 files.

**User Q: "is this purely function rename, with no actual code change?"** → NO, two behavior-neutral
*structural* edits alongside the renames, both authorized in the user's Phase A decision table and
pinned by `*BitIdentical*` guards:
1. `OptimizeIncreSingleTask`: dropped unused `K` param (was discarded via `(void)K;`).
2. `PerformCoordinateDescentForTaskConfigOpt`: REMOVED (1 caller); `RunIntervalDescent(Reopt,…)`
   inlined at the sole caller in `ReOptizePeriodic`.

**User Q: release speed test shows ~2× larger total running time + some ET increases — code cause?**
→ NO P2.16 cause. The 2× is the **N8→N10 taskset change** (`d1d05827`, pre-P2.16), not the refactor:
- Run 1 = `taskset_N8` (wall 65.9/66.1 s); Run 2 = `taskset_N10` (wall 149.0/138.1 s). 149/66 ≈ 2.26×.
- Wall time is simulation-dominated: scheduler ET is 0.011–0.024 s/interval vs 6–15 s/interval wall
  (~600–1000×). P2.16 touches only the optimizer; the ROS2-style sim loop is untouched.
- The two structural edits are wall-time-neutral by construction: dropping a dead param cannot
  change runtime; inlining a 1-caller wrapper removes call overhead (marginally faster, never slower).
- SP bit-identical: N10 SP = 0.496673 (both Reopt_1 and Reopt_10).
- ET vs P2.12 baseline (0.042/0.013 s/act): N8 Reopt_1 = 0.0174 (faster), N10 Reopt_10 = 0.0132
  (matches). No regression.
- Clean A/B = run the SAME taskset before/after `6d068ec9` (e.g. checkout HEAD~). Offered to hold
  A3 until user does this; awaiting direction.

**NEXT:** Stage A3 (cross-file `Optimize_w_TL_ScratchOrIncre`→`DispatchReoptOrIncremental` cascade)
once user confirms direction.
