# P1.10 — Tasks (working checklist)

> See `goal.md` for scope + the D1–D5 open design questions. **This task blocks
> [[P1.9]]** (the RTA cache) — P1.9 stays ON HOLD until P1.10's single-change
> invariant is proven in code.

## Phase 0 — Decisions (user, before any code)

> **D2 REVISED 2026-07-15** (premise correction, see `goal.md` "Code-grounded
> findings"): `FindTaskWithDifferentEt` cancels env changes (both diff sides
> derive from the same `dag_tasks_`), so Type-E cannot come from it — it needs a
> dedicated cross-interval pre-TL DAG diff.
>
> **D2 SIMPLIFIED + API DESIGN 2026-07-15** (`api_design.md`): the owner member
> `dag_tasks_` is always pre-TL (TLs go into transient `dag_tasks_cur`), so the
> cross-interval pre-TL DAG diff Type-E needs IS just
> `FindTaskWithDifferentEt(dag_tasks_prev_interval_pre_tl_, dag_tasks_)` — **reuse
> the existing function with new STATE** (one retained pre-TL DAG snapshot), no new
> diff function. The 2026-07-15 API design proposes signatures + purposes for 7
> functions + 1 state member, with proposed answers to D1/D2/D3/D4 (D5 deferred).

- [ ] **API design reviewed** (`api_design.md`) — signatures + purpose-of-each-step
      for: `OptimizeIncre_SingleTask` (A, sub-incremental primitive), Type-E via
      reused `FindTaskWithDifferentEt` (B), `CollectTLFlexibleTaskIds` (C),
      `BuildSerializedTaskQueue` (D, merged + sorted together),
      `EvaluateTimeLimitConfig_SubIncremental` (E, shared eval entry),
      `PerformSerializedTaskQueueOptimization` (F, loop driver),
      `OptimizeSingleTaskTimeLimit_Impl`/`_SubIncremental` (G, eval-injected Type-L
      refactor); new state `dag_tasks_prev_interval_pre_tl_`. Open decision points
      listed at the bottom of `api_design.md`.
- [ ] **Extension idea reviewed** (`goal.md` "Extension idea") — per-interval task
      subset with cycling (top-K_subset + rotating start offset). NOT part of the
      first cut; implement after the core loop + invariant proof.

- [x] **D1 sub-point 1 DECIDED 2026-07-16** (the redundant-eval removal; the D1 *main*
      question below stays open). The redundant `:238-239`-duplicate SP-eval is removed
      at the **generator** level: `FindPriorityVec1D_Variations` (`OptimizeSP_Incre.h:75`
      / `.cpp:180`) gains a default `bool exclude_opt_pa = true` that skips emitting the
      carried-position variation (`i == old_priority_index`), which would reconstruct
      `opt_pa_` and re-evaluate it to the `:238-239` baseline SP. **Inverts** the prior
      "sub-incremental drops `:238-239`" proposal — keep `:238-239` (genuine new-env SP),
      drop the carried-pos variation instead (uniformly correct at baseline AND TL steps).
      **Bit-identical for the existing `OptimizeIncre`** (inherits `true`; the carried-pos
      eval gives SP_base and the `:282` adopt test is strict `>`, so an equal SP never
      displaces `opt_sp_`). Staged in the working tree, NOT built/tested/committed (code
      work paused per user). See `dev_log.md` + `api_design.md` 2026-07-16.
- [x] **D2 DECIDED 2026-07-17 — AMENDED (same session, later): structural filter
      supersedes "caller normalizes."** ⚠ The "caller-ensures-same-ET" decision below
      is SUPERSEDED — it rested on a fragile premise (`FiniteDist::operator!=` is a
      10%-relative `approx_equal` `Probability.cpp:415-417`; a TL-flexible task's dist
      carries the raw YAML mu/min/max with no adopted-TL override at read
      `RegularTasks.cpp:75-79` → can compare unequal across intervals for TL/grid
      reasons, not env). User directed: *"rename the method as
      `FindEnvTaskWithDifferentEt`, and explicitly filter out tasks with TL options in
      the implementation."* ⇒ **TWO NEW FREE FUNCTIONS LANDED+TESTED (NOT committed)**
      in `OptimizeSP_Incre.h`/`.cpp`: `FindTasksWithFlexibleTimeLimits(dag)` → IDs with
      non-empty `timePerformancePairs` (mirrors `RecordTimeLimitOptions`'s `{-1}`
      sentinel `OptimizeSP_TL_BF.cpp:29-30`); `FindEnvTaskWithDifferentEt(prev, cur)` →
      `FindTaskWithDifferentEt(prev, cur)` MINUS the TL-flexible set. **Phase 2 Type-E
      diff now calls `FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)`,
      NOT the raw `FindTaskWithDifferentEt`.** `FindTaskWithDifferentEt` itself stays
      UNCHANGED for ONE caller only — `OptimizeIncre`'s `.cpp:282` live TL-walk site,
      where the diff MUST flag the TL-walked (TL-flexible) task so its 1D priority is
      re-searched each TL step (filtering there empties the diff + stops the mid-walk
      re-search = behavior change to the live path; a single filtered fn can't serve
      both). 3 new tests in `tests/testOptimizeIncrePA.cpp` prove the filter drops the
      TL-flexible perf-pair mover (TSP task 0) + keeps the env mover (SLAM task 3);
      `cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN,
      `testOptimizeIncrePA` + `testIncreOpt_w_TL` unchanged → no live-path behavior
      change. See `api_design.md` "D2 amendment" block + `dev_log.md` §2026-07-17
      "D2 AMENDED". *(Pre-amendment record below, kept for provenance.)*
  - **D2 (pre-amendment 2026-07-17):** **`FindTaskWithDifferentEt` stays UNCHANGED.** The
      user's authoritative correction: "`FindTaskWithDifferentEt`'s implementation
      does its job; the **caller** of `FindTaskWithDifferentEt` should make sure
      `dag_tasks_updated` and `dag_tasks` have the same ET for tasks with flexible ET."
      This **overtURNS the earlier idea of adding a mask/exclusion parameter** to the
      function (no signature/behavior change; `OptimizeIncre`'s `.cpp:282` call site
      untouched; base `OptimizeIncre` stays TL-unaware — TL-awareness is the caller's
      job). The caller-ensures-same-ET property holds **by construction at the capture
      site**: Type-E = `FindTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)`
      where `dag_tasks_prev_pre_tl` is a **LOCAL** `dag_tasks_` captured BEFORE the
      `:313`/`:452` absorb (both sides pre-TL env DAGs → under "TL-flexible tasks have
      no env dependence by design," TL-flexible ETs are equal on both sides → never
      flagged → the surviving diff is pure env, with direction). **No mask, no
      normalization pass, no retained member** — this RETIRES the previously-proposed
      `dag_tasks_prev_interval_pre_tl_` member (`api_design.md` "New state — NONE").
      **Sub-point still OPEN (coupled to D5):** the local capture must happen at BOTH
      absorb entries (`OptimizeIncre_w_TL :313` AND `ReOptimizePeriodic :452`) for the
      first post-reopt interval's Type-E diff to not compare against a stale
      pre-reopt prev-DAG. If D5 = incremental-only first cut, only `:313` is captured
      and the first post-reopt interval suppresses Type-E (acceptable). See
      `api_design.md` "D2 correction" block. *(SUPERSEDED by the amendment above.)*
- [x] **D3 DECIDED 2026-07-17** — sort the merged E+L queue by **task weight,
      descending** (simple, uniform key; no E/L tier). High-weight tasks optimized
      first. User's "first use a simple sorting function based on tasks' weights."
      The earlier "reuse `TaskSortingHeuristic`" proposal (weight/threshold/id
      composite, `OptimizeSP_TL_Incre.h:51`) is superseded — weight alone for the
      first cut; the full heuristic may return in a future cut.
- [x] **D4 DECIDED 2026-07-17** — **running-adopted single champion** = `res_opt_`
      (the P0.5 single durable incumbent: PA/TL/SP). Seeded at interval start from the
      previous interval's committed results; each serialized step compares-and-keeps
      via the existing `UpdateRecords`/`CommitIncumbent`
      (`OptimizeSP_TL_Incre.cpp:105`/`:379`); new champion vs keep-old treated
      identically (the strict-`>` adopt test governs). Matches the existing pattern;
      interval-start champion (each step diffs against the original) NOT adopted.
- [x] **D5 DECIDED 2026-07-17** — **incremental-only first cut.** The serialized
      loop's primitive (`OptimizeIncre_SingleTask`) is **warm-started 1D** — it maps
      cleanly onto the incremental path (same warm-started character: replace
      `OptimizeIncre` with `OptimizeIncre_SingleTask`), but does NOT fit reopt. Reopt
      (`ReOptimizePeriodic`) is **deliberately memoryless about the PA**: each
      candidate eval calls `OptimizeFromScratch(K)` (full beam, empty start), NOT
      `OptimizeIncre`; the champion is the *yardstick* (compare-and-keep via
      `UpdateRecords`), not the search's starting point. Plugging the warm-started
      primitive into reopt would make reopt warm-started → defeating its
      escape-incumbent-structure purpose. That is a **semantic change** to what reopt
      IS, not a loop rewrite. Additionally, **Type-E does not apply to reopt at all**
      — Type-E is "an env-changed task whose priority is frozen and needs re-search,"
      but reopt already re-searches ALL priorities every candidate via
      `OptimizeFromScratch`, so nothing is frozen; Type-E is purely an
      incremental-path concept. ⇒ First cut = rewrite the incremental path only;
      `ReOptimizePeriodic` stays memoryless full-beam (the periodic "cover
      everything" backstop, also the role the extension idea casts for it). Serialize
      reopt later ONLY if A/B shows the post-reopt interval actually hurts AND we're
      willing to change reopt's character.
- [x] **D2 sub-point DISSOLVED 2026-07-17** (was "capture at both entries vs
      incremental-only"; coupled to D5). **Moot — capture at `:313` only.** Premise
      corrected: reopt updates `dag_tasks_` to its own env at `:452` just as
      incremental does at `:313` (both are the ONLY writers of the `dag_tasks_`
      member; `BuildChallengerFromIncumbent`/`UpdateExtDistBasedOnTimeLimit`/
      `ResetIncumbentBaseline` all use fresh LOCALs, never mutate the member). So at
      the start of any interval T, before its own absorb, `dag_tasks_` already holds
      interval T-1's env (whichever path T-1 took) → capturing
      `dag_tasks_prev_pre_tl = dag_tasks_` before `:313` gives T-1's env, and the
      Type-E diff T-1→T is correct **across the reopt boundary for free**. NOT stale,
      NOT suppressed. (And reopt doesn't compute Type-E anyway — see D5.) The earlier
      "stale pre-reopt snapshot" framing was based on the wrong premise that reopt
      doesn't update `dag_tasks_`; corrected here + in `api_design.md` + `dev_log.md`.
- [x] **Dedup policy DECIDED 2026-07-17** (#5) — a task is **NOT allowed** to be both
      env-changed (Type-E) AND TL-flexible (Type-L). The two sets are disjoint by
      generator design (TL-flexible tasks have no env dependence by construction);
      if a task appears in BOTH at runtime → **RaiseError** (a contract violation, not
      an optimization choice — this case is not considered in this project). NOT a
      silent winner-pick; if a winner were ever needed, TL-flexible wins (user
      direction), but the chosen behavior is to hard-fail rather than pick. Recorded in
      `api_design.md` function D comment + "Open decision points" #5.
- [x] **Baseline eval DECIDED 2026-07-17** (#6) — **(b) dedicated re-score, NOT
      `EvaluateTimeLimitConfig_ScratchOrIncre`.** The baseline re-scores the champion's
      carried `{pa, tl}` under the new env DAG to seed `opt_sp_`; it must **NOT**
      optimize anything (the queue walk does all optimization, in D3 weight order).
      Routing the baseline through `EvaluateTimeLimitConfig_ScratchOrIncre` would call
      `OptimizeIncre`, which performs priority optimization on env-changed tasks
      **before** the queue's sorted order is honored → violates the proposal's core
      "sort tasks, then optimize in order" shape. Baseline =
      `EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_, committed_tl),
      sp_parameters_, opt_pa_)` directly — no `OptimizeIncre`, no
      `BuildChallengerFromIncumbent` rebuild. Recorded in `api_design.md` function F
      comment + "Open decision points" #6.

## Phase 1 — Sub-incremental optimizer (the primitive)

- [x] **Land D1 sub-point 1 (LANDED + TESTED + COMMITTED 2026-07-16, commit
      `33b2270c`):** the `exclude_opt_pa=true` default on
      `FindPriorityVec1D_Variations` (`OptimizeSP_Incre.h:75` / `.cpp:180`) skips
      emitting the carried-position variation (`i == old_priority_index`), which
      reconstructs `opt_pa_` exactly and whose SP-eval duplicated the `:243-244`
      baseline. The 4 old full-range unit tests in `tests/testOptimizeIncrePA.cpp`
      were renamed `*_full_range` and pinned to `exclude_opt_pa=false` (they
      document the range logic the default-true path skips); + 1 new test
      `FindPriorityVec1D_Variations_excludes_carried_pa` asserts the default-true
      exclude behavior (sizes −1, no emitted PA equals the carried PA).
      `cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN —
      the Phase 1 bit-identity gate: `testOptimizeIncrePA` (incl. the e2e
      `GetPriorityAssignments_IncrementalOpt` result assertion) + `testIncreOpt_w_TL`
      (the production TL caller inheriting the new default) both pass unchanged →
      `OptimizeIncre`'s adopted results are bit-identical, only the eval count drops by
      1 per changed task.
- [x] **Implement the sub-incremental primitive per D1 main (LANDED + TESTED +
      COMMITTED 2026-07-17, commit `3d2f9b28`):** `OptimizeIncre_SingleTask(
      dag_tasks_update, task_id, et_increased)` on `OptimizePA_Incre` = the
      extracted former `:274-292` loop body. TRUSTS `opt_sp_` (caller-set), does
      `FindPriorityVec1D_Variations` + `EvaluateSPWithPriorityVec` + strict-`>`
      adopt, mutates `opt_pa_`/`opt_sp_` in place. Does NOT advance `dag_tasks_`.
      `OptimizeIncre` refactored to CALL it in the `:274` loop + gained the
      optional `baseline_sp` arg (default `INT_MIN` = score the carried PA at
      `:243-244`; provided → skip). Behavior-preserving (bit-identical) — the
      `:274` loop now delegates to the primitive instead of inlining the body.
- [x] **TDD: differential test (LANDED + PASSING in `3d2f9b28`):**
      `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange`
      (`tests/testOptimizeIncrePA.cpp:381`) — on the v22→v23 `|diff|==1` pair
      (task 1, increase), two optimizers seeded identically via `OptimizeFromScratch(2)`;
      path A = full `OptimizeIncre(dag_update)`, path B = primitive alone with the
      SAME baseline seed `EvaluateSPWithPriorityVec(dag_update, sp, opt_pa_)`.
      Asserts `pa_full == pa_primitive` (exact) and `optA.opt_sp_ == optB.opt_sp_`.
      Proves the extraction is behavior-preserving (refactor, not new behavior).
- [x] **`cmake --build build --target check.SP_OPT -j5` green + `ctest` 16/16
      (DEBUG build)** — re-verified 2026-07-17 on HEAD `3d2f9b28` + the uncommitted
      D2-amendment working tree: `testOptimizeIncrePA` (incl. the differential
      test) + `testIncreOpt_w_TL` both PASS unchanged → `OptimizeIncre`'s adopted
      results are bit-identical pre/post extraction.

## Phase 2 — Serialized interval search (the loop rewrite)

> **IMPLEMENTED 2026-07-17 (later)** in the working tree, behind compile-time
> flag `GlobalVariables::use_serialized_incremental_opt` (default OFF, NOT
> YAML-backed). 16/16 ctest green (flag OFF = legacy bit-identical; flag ON =
> differential test passes). **NOT committed** — pending user review. See
> `dev_log.md` §2026-07-17 "Phase 2 IMPLEMENTED + TDD GREEN". One bug caught +
> fixed by TDD (the stale-baseline-in-`OptimizeIncre_SingleTask` issue — see
> dev_log). Functions landed: `OptimizeSingleTaskTimeLimit_Impl` (G, eval
> injection), `EvaluateTimeLimitConfig_SubIncremental` (E),
> `CollectTLFlexibleTaskIds` (C), `BuildSerializedTaskQueue` (D),
> `PerformSerializedTaskQueueOptimization` (F), + the flag wired into
> `OptimizeIncre_w_TL` with the local `dag_tasks_prev_pre_tl` capture before
> the `:313` absorb.

- [x] Build the initial champion from `res_opt_` (reuse `BuildChallengerFromIncumbent`
      `:394`); confirm the warm-start source is unchanged — done inside E (the
      sub-incremental eval builds the challenger per candidate; the dedicated
      baseline re-score in F seeds `opt_sp_` directly, #6)
- [x] **Compute Type-E via `FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl,
      dag_tasks_)`** (D2 AMENDED 2026-07-17). `dag_tasks_prev_pre_tl` is a LOCAL
      `dag_tasks_` captured before the `:313` absorb in `OptimizeIncre_w_TL`,
      threaded into F → `BuildSerializedTaskQueue`. One call site; the function
      was already implemented + tested.
- [x] Collect Type-L via `CollectTLFlexibleTaskIds()` (the `{-1}`-sentinel inverse
      of `RecordTimeLimitOptions`, on the derived class — mirrors the landed free
      function `FindTasksWithFlexibleTimeLimits`)
- [x] Merge + sort the queue per D3 (`BuildSerializedTaskQueue`, weight-descending
      `std::stable_sort`; `CoutError` on E∩L overlap per #5)
- [x] Serialized loop body: Type-E → sub-incremental (`EvaluateTimeLimitConfig_SubIncremental`,
      the NEW capability); Type-L → `OptimizeSingleTaskTimeLimit_Impl` walk with
      eval bound to the sub-incremental (skips the redundant challenger-rebuild
      re-score the legacy eval pays each step — the one kept re-score is the
      carried-PA baseline inside E, which `OptimizeIncre_SingleTask` trusts)
- [x] Champion mutation between steps per D4 (`UpdateRecords`/`CommitIncumbent`
      per step; working TL refreshed from `res_opt_` after each step)
- [x] TDD: end-to-end — `TaskSetForTest_robotics_v19.SerializedIncremental_NoWorseThanLegacy`
      (tests/testIncreOpt_w_TL.cpp): bootstrap both optimizers via
      `ReOptimizePeriodic(v19)`, warm-start with v21; asserts serialized SP ≥
      legacy + both reach TSP TL=1000. PASSING. *(SP-eval COUNT characterization
      deferred — the differential asserts SP-equivalence, not eval count; count
      the net Type-L-down/Type-E-up delta separately if needed for the A/B.)*

## Phase 3 — Prove the single-change invariant (unblocks P1.9)

> **INVARIANT CORRECTED 2026-07-17 (Phase 3 instrumentation):** the earlier
> "Type-E step = `|diff|==1`" claim (in `api_design.md` + the old task below)
> was **wrong**. `AssertSingleChangeInvariant` empirically proved a Type-E step
> yields `|diff|==0`: the env move is absorbed into `dag_tasks_` at the `:313`
> absorb BEFORE the champion DAG is built → on BOTH diff sides → cancels
> (`goal.md` premise-correction finding #1). Candidate DAG == champion DAG
> (same committed TL); only the re-searched PA varies. The honest invariant is
> `|diff|<=1` (Type-L: 1 on the walked task; Type-E: 0; baseline: 0, out of
> scope). For P1.9: `|diff|==0` → full RTA reuse, `|diff|==1` → single-task
> patch — never multi-task.

- [x] **Instrument:** `AssertSingleChangeInvariant(champion_dag, candidate_dag,
      task_idx)` (debugMode-gated, `OptimizeSP_TL_Incre.h`/`.cpp`) called inside
      `EvaluateTimeLimitConfig_SubIncremental` at every serialized SP-eval.
      Accepts `|diff|==0` (Type-E) or `|diff|==1` flagging `task_idx` (Type-L);
      throws via `CoutError` on `|diff|>1` or a 1-flagging-different-task
      (champion drift). debugMode-only → cost-free in production.
- [x] **TDD proof:** `TaskSetForTest_robotics_v19.SerializedIncremental_SingleChangeInvariant`
      (`tests/testIncreOpt_w_TL.cpp`) — forces `debugMode=1`, runs v19→v21
      (moves BOTH the TL-flexible TSP task 0 AND the env-only SLAM task 3, per
      the D2 `FindEnvTaskWithDifferentEt` tests), bootstraps via
      `ReOptimizePeriodic` then `OptimizeIncre_w_TL`. A violation throws → test
      aborts (FAIL). Reaching the `EXPECT_GT(res.sp_opt, 0.0)` line means the
      invariant held at EVERY SP-eval across both step kinds. PASSING.
- [x] **Gate:** `cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest`
      16/16 GREEN. `testIncreOpt_w_TL` now 51 tests (was 50; +1 invariant proof).
      The existing `SerializedIncremental_NoWorseThanLegacy` also runs with
      debugMode armed (parameters.yaml `debugMode: 1`) → doubles as a second
      invariant witness on the single-TSP TL-walk case.
- [x] **Recorded** the invariant-hold evidence + the Type-E `|diff|==0` correction
      in `dev_log.md` §2026-07-17 "Phase 3" + `api_design.md` "The single-change
      invariant" (pre-correction text marked SUPERSEDED).
- [x] **Unblock P1.9:** P1.9 STATUS flipped ON HOLD → resume; the cache
      simplification is `|diff|==0` → full RTA reuse, `|diff|==1` → single-task
      patch — never multi-task. (P1.9's own goal.md updated separately.)

## Done when
- [x] D1–D5 answered + dedup policy (#5) + baseline eval (#6) answered
- [x] sub-incremental optimizer (`OptimizeIncre_SingleTask`) implemented +
      committed (`3d2f9b28`); `cmake --build build --target check.SP_OPT -j5` +
      `ctest` 16/16 green (DEBUG); differential TDD green
- [x] serialized loop (Phase 2) implemented + TDD green (NOT committed; flag
      default OFF)
- [x] **single-change invariant proven** (Phase 3, `|diff|<=1` at every SP-eval,
      debugMode-gated instrumentation + TDD); **P1.9 unblocked**
