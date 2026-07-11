# P0.5 — Design: incumbent-state redesign in `OptimizeSP_TL_Incre`

> The decided design for the optimizer iteration-process / incumbent-state
> refactor. This file is the single source of truth for *what* we are building
> and *why*; `goal.md` is the scope, `tasks.md` is the step-by-step execution
> checklist (mirrors the migration phases below), `dev_log.md` is the
> chronological working log.
>
> Status: **design decided by user (2026-07-08)** — see the dev_log entry.
> Supersedes the earlier "propose the target shape, do not decide" framing in
> `goal.md` v1; `goal.md`/`tasks.md` have been updated to match.

## 1. The problem this design solves

`OptimizePA_Incre_with_TimeLimits` (`sources/Optimization/OptimizeSP_TL_Incre.h`)
carries the running system's "best-so-far" solution — the **incumbent** — across
intervals. An incumbent is the 4-tuple `{DAG-with-adopted-TL, SP, PA, TL-config}`.

Today that 4-tuple is **split across four data members that can desync**:

| field | declared | holds | overlaps |
|-------|----------|-------|----------|
| `prev_optimizer_` | `OptimizeSP_TL_Incre.h:184` | a full `OptimizePA_Incre` | DUPLICATES `dag_tasks_` / `sp_parameters_` / `opt_pa_` / `opt_sp_` already held by the derived class |
| `res_opt_` | `OptimizeSP_TL_Incre.h:179` (`ResourceOptResult`) | `id2priority` + `id2time_limit` + `priority_vec` + `sp_opt` | overlaps `opt_pa_` (=`priority_vec`) / `opt_sp_` (=`sp_opt`) |
| `opt_pa_` / `opt_sp_` | `OptimizeSP_Base.h:69-68` (base) | the PA result + best SP | overlapped by both of the above |
| `time_limit_option_for_each_task_` | `OptimizeSP_TL_Incre.h:180` | the per-task TL search window | transient per-call, not incumbent |

Three concrete symptoms this split causes (all currently masked by defensive
copies; all would re-emerge under any refactor that drops one copy):

1. **The `sp_parameters_` desync.** `SeedStateFromIncumbent`
   (`OptimizeSP_TL_Incre.cpp:467-476`) documents that `IfInitialized()` only
   checks `!opt_pa_.empty()`, so `prev_optimizer_` can be "initialized" with an
   **empty** `sp_parameters_` → `OptimizeIncre`'s SP-eval throws
   `_Map_base::at` on `thresholds_node`. It is masked only because `UpdateRecords`
   (`prev_optimizer_ = optimizer`, a full copy, `:134`) usually fires between a
   reopt and the next incremental call. A no-improvement all-{−1} interval never
   fires `UpdateRecords`, so the manual `prev_optimizer_.sp_parameters_ =
   sp_parameters_` line at `:476` is **load-bearing, not decorative**.

2. **The frozen-baseline DAG.** `OptimizeIncre` advances its own `dag_tasks_` to
   the current interval's DAG as a side-effect (`OptimizeSP_Incre.cpp:296`,
   `dag_tasks_ = dag_tasks_update`). That advance only propagates to the
   incumbent via `UpdateRecords`'s copy. The frozen-baseline regression test
   (`OptimizeIncre_AdvancesPrevOptimizerDagTasks`, `testIncreOpt_w_TL.cpp:913`)
   exists *only* to guard this fragile propagation path.

3. **The diff-side cold-start.** `OptimizeIncre_w_TL`'s descent must start from
   the **carried adopted TL** (so `FindTaskWithDifferentEt` does not flag
   unchanged perf-pair tasks). Today that is enforced by a per-site override —
   `ReconstructTimeLimitVecFromResOpt()` at `OptimizeIncre_w_TL:387` — bolted on
   by the P1.1 investigation.

   > **Re-derivation (2026-07-08, Phase 3d) — OVERTURNS the reopt-path half of
   > this symptom.** The original wording also claimed the reopt path
   > (`ReOptimizePeriodic:535`, cold-starting from
   > `InitializeTimeLimitsFromETConfig()`) was "the same class of bug on the
   > other path." That claim is **wrong** under re-derivation. The reopt path
   > uses `from_scratch=true` → `EvaluateTimeLimitConfig_ScratchOrIncre` builds a
   > *fresh* `OptimizePA_Incre` and calls `OptimizeFromScratch` (NOT
   > `OptimizeIncre`), so `FindTaskWithDifferentEt`'s diff **never runs** on the
   > reopt path. The starting `time_limits` on the reopt path is only the
   > from-scratch search's initial TL vector (the walk explores the full option
   > set from there); there is no baseline/update diff invariant to preserve. So
   > the reopt cold-start does **not** need to start at the carried adopted TL,
   > and `InitializeTimeLimitsFromETConfig()` at `ReOptimizePeriodic:575` is
   > correct as-is. Symptom 3 is **incremental-path only**. The redesign still
   > promotes the incremental-path override into the structural norm via
   > `BuildChallengerFromIncumbent` (both diff sides inherit the adopted TL); the
   > reopt path is simply unaffected.

The redesign makes the incumbent **owned once** and makes the cold-start prior
**read from the correct source by construction**, so none of the three symptoms
can recur.

## 2. The decided design

User's decision (2026-07-08), made literal:

- **`res_opt_` becomes the single durable incumbent store.** No parallel cache.
  The carried adopted TL lives in `res_opt_.id2time_limit`; the carried PA lives
  in `res_opt_.priority_vec` / `id2priority`; the carried SP lives in
  `res_opt_.sp_opt`. The base-class `opt_pa_` / `opt_sp_` become thin mirrors
  populated by the commit helper (kept only because `OptimimizePA_Base`'s public
  surface and `CollectResults()` read them).
- **`has_incumbent_` (bool) replaces `prev_optimizer_.IfInitialized()` as the
  interval-0 gate.** `IfInitialized()` checked `!opt_pa_.empty()`, which was the
  root of symptom 1 (it could be true while `sp_parameters_` was empty). An
  explicit bool set only by `CommitIncumbent` cannot lie about its own state.
- **`CommitIncumbent(pa, sp, tl)` — one helper replacing the 8 scattered sync
  assignments** currently spread across `UpdateRecords` (`:128-134`) and
  `SeedStateFromIncumbent` (`:457-476`). Both call sites reduce to a single
  `CommitIncumbent(...)`; the desync class of bug becomes structurally
  impossible because there is exactly one writer.
- **`BuildChallengerFromIncumbent()` — builds a throwaway `OptimizePA_Incre`
  from `res_opt_` each incremental interval.** The challenger is reconstructed
  from the carried adopted-TL DAG (`UpdateExtDistBasedOnTimeLimit(dag_tasks_,
  ReconstructTimeLimitVecFromResOpt())`) + the carried PA + `sp_parameters_`.
  Genuinely transient — your "challenger is transient, incumbent is durable"
  intuition made literal.
- **The DAG-advance side-effect of `OptimizeIncre` becomes harmless.** Today
  `dag_tasks_ = dag_tasks_update` (`OptimizeSP_Incre.cpp:296`) only propagates to
  the incumbent via `UpdateRecords`'s copy — the fragile path symptom 2 guards.
  Under the redesign the challenger is a **throwaway local**; whatever
  `OptimizeIncre` does to its own `dag_tasks_` dies with the local. The adopted
  TL it persists into `res_opt_.id2time_limit` (via `CommitIncumbent`) is the
  real invariant, **reconstructed next interval** from `res_opt_`. The
  frozen-baseline propagation path simply ceases to exist.
- **Zero public-API change.** `Optimize_w_TL_ScratchOrIncre`,
  `ReOptimizePeriodic`, `OptimizeIncre_w_TL`, `EvaluateTimeLimitConfig_ScratchOrIncre`,
  `CollectResults`, `SeedStateFromIncumbent`, `SeedIncumbentBaseline` all keep
  their signatures. The orchestrator (`SimulationOrchestrator.cpp`) and the
  end-to-end tests are untouched at the API level.

### Blast radius

Two source files + one test file (plus a comment-only touch):

- `sources/Optimization/OptimizeSP_TL_Incre.h` — remove `prev_optimizer_`; add
  `has_incumbent_` + `CommitIncumbent` / `BuildChallengerFromIncumbent` helpers;
  document `time_limit_option_for_each_task_` as transient (per-call, not
  incumbent).
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — bodies for the two new
  helpers; rewrite `SeedStateFromIncumbent`, the `UpdateRecords` write-block,
  the `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch, and the
  `SeedIncumbentBaseline` gate.
- `tests/testIncreOpt_w_TL.cpp` — migrate every `prev_optimizer_.*` read (15
  occurrences across 6 tests). The 3 frozen-baseline DAG-ET assertions are
  **rewritten to observe the carried TL via `res_opt_`**, not the gone-away
  `prev_optimizer_.dag_tasks_`. `eval_count_` and `from_scratch_flags` stay
  (documented test seams).
- `sources/Optimization/OptimizeSP_Incre.cpp` — **comment-only** (`:293-295`):
  the comment says "UpdateRecords copies this optimizer into `prev_optimizer_`,
  so the advanced DAG propagates across intervals." That sentence becomes false
  under the redesign; rewrite it to say the challenger is a throwaway local and
  the DAG advance dies with it (the adopted TL in `res_opt_` is what carries).
- `sources/Optimization/OptimizeSP_Base.h` — **read-only reference** for
  `ResourceOptResult` and `OptimimizePA_Base` data members; no change.

## 3. The diff-baseline invariant (what must survive the redesign)

The whole point of carrying an incumbent is the **diff** in
`FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:242`):

```cpp
FindTaskWithDifferentEt(dag_tasks_, dag_tasks_update);
//                    ^baseline      ^update
```

`dag_tasks_` is the **baseline side** (the carried adopted-TL DAG);
`dag_tasks_update` is the **update side** (the current interval's DAG, after
`UpdateExtDistBasedOnTimeLimit` applies the descent's starting TL vector). The
diff flags a perf-pair task **iff its ET actually changed** — i.e. iff its
adopted TL moved between intervals — **only when both sides carry the same
adopted TL**. This is the invariant the P1.1 investigation established (see
`agents/active_tasks/P1_1_p25_residual_investigation/investigation_summary.md`
and memory `p25-ndiff-diff-semantics`).

Under the redesign:

- **Baseline side** = the challenger's `dag_tasks_`, which
  `BuildChallengerFromIncumbent` reconstructs as
  `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
  — i.e. the current raw DAG with the carried adopted TL applied. Reconstructed
  fresh every interval from `res_opt_`.
- **Update side** = `dag_tasks_cur` in
  `EvaluateTimeLimitConfig_ScratchOrIncre` (`:148-149`), built as
  `UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` where `time_limits`
  is the descent's starting vector. For the incremental path that starting
  vector IS `ReconstructTimeLimitVecFromResOpt()` (already the case at
  `OptimizeIncre_w_TL:387` under the P1.1 workaround) → both sides carry the
  adopted TL → the diff invariant holds **by construction**, not by override.

So the redesign **promotes the P1.1 per-site workaround into the structural
norm**: the adopted TL is read from `res_opt_` at exactly one site
(`BuildChallengerFromIncumbent`), and both sides of the diff inherit it. The
workaround at `OptimizeIncre_w_TL:387` and the override at
`ReOptimizePeriodic:535` collapse into "the challenger is always built from the
incumbent."

### What the rewritten frozen-baseline tests prove

Three tests currently read `prev_optimizer_.dag_tasks_.tasks[1].execution_time_dist`
as a window onto whether the carried DAG advanced:

1. `OptimizeIncre_AdvancesPrevOptimizerDagTasks` (`:913`) — asserts the mutated
   `T_noise` ET propagates into the carried DAG after an incremental call.
2. `PerformCoordinateDescent_AllMinusOneOnly_RunsOneEvalAndAdvancesPrevOptimizer`
   (`:1062`) — same propagation, via the zero-work fallback, plus the
   `eval_count_` drop from N to 1.
3. (implicit, via `SeedStateFromIncumbent_WritesFullFourTuple` `:658` and
   `SeedIncumbentBaseline_ReEvalsIncumbentUnderNewDAG` `:706`) — the seeded
   4-tuple is observable through `res_opt_`.

Under the redesign the carried DAG **no longer exists as a storable object**
(the challenger is a throwaway). The invariant these tests guard — "the adopted
TL is carried forward so the next interval's diff is against the right
baseline" — must be re-observed through `res_opt_`:

- **Rewritten observable:** after an incremental call, `res_opt_.id2time_limit`
  holds the adopted TL for the current interval. For `T_noise` (no perf pairs,
  TL always −1) the carried TL is −1; for `T_perf` the carried TL is whatever
  the descent adopted. The "did the baseline advance?" question becomes "does
  `res_opt_.id2time_limit[T_perf]` reflect the current interval's adopted TL?",
  which is exactly what `BuildChallengerFromIncumbent` will reconstruct next
  interval. The `T_noise`-ET-mutation trick is replaced by asserting the carried
  TL on `T_perf` matches the descent's adopted value.

The exact rewrite is specified per-test in `tasks.md` Phase 3. The principle:
**the diff-baseline invariant survives the reconstruction** — if
`BuildChallengerFromIncumbent` reconstructs the same baseline DAG the old
`prev_optimizer_.dag_tasks_` carried, the diff produces the same `ndiff`, and
the frozen-baseline guards (rewritten to observe `res_opt_`) pass.

## 4. Migration plan (TDD, additive → dual-write → flip → remove)

Per `agent_coding_rules.md`: minimal, testable, modular steps; all tests green
after each; behavior-preserving for the structural migration. Each phase is one
review-and-commit cycle.

### Phase 1 — Introduce helpers additively (no behavior change)

- **1a.** Add `has_incumbent_` (bool, default false) and the two helper
  declarations to `OptimizeSP_TL_Incre.h`. Do NOT wire them yet. Build + tests
  green (no call sites changed).
- **1b.** Implement `CommitIncumbent(pa, sp, tl)` as the exact body of the
  `SeedStateFromIncumbent` write-block (`:457-476`) **minus** the
  `prev_optimizer_` lines, plus `has_incumbent_ = true`. Implement
  `BuildChallengerFromIncumbent()` returning a fresh `OptimizePA_Incre`
  constructed from `UpdateExtDistBasedOnTimeLimit(dag_tasks_,
  ReconstructTimeLimitVecFromResOpt())` + `sp_parameters_`, with its
  `opt_pa_`/`opt_sp_`/`dag_tasks_` seeded from `res_opt_`. Still no call-site
  change. Build + tests green (helpers exist, unused).
- **1c.** Add a focused unit test for each helper in isolation (TDD: write the
  test, watch it pass against the additive implementation):
  - `CommitIncumbent` populates `res_opt_` (id2time_limit, priority_vec,
    sp_opt), `opt_pa_`/`opt_sp_`, and sets `has_incumbent_` — and does NOT
    touch `prev_optimizer_` (assert it stays uninitialized).
  - `BuildChallengerFromIncumbent` returns an `OptimizePA_Incre` whose
    `dag_tasks_` equals
    `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
    and whose `opt_pa_`/`opt_sp_` mirror `res_opt_`. Assert against the same
    primitives the old `SeedStateFromIncumbent` used.

### Phase 2 — Dual-write (flip writers one at a time)

- **2a.** Make `SeedStateFromIncumbent` call `CommitIncumbent(pa, sp, tl)`
  internally, then **also** perform the legacy `prev_optimizer_` writes (dual
  write). `has_incumbent_` and `prev_optimizer_.IfInitialized()` must agree.
  Tests green — including the 6 `prev_optimizer_.*`-reading tests, which still
  see the old field populated.
- **2b.** Make `UpdateRecords`'s `should_update` block call `CommitIncumbent`
  for the `res_opt_`/`opt_*` writes, then **also** `prev_optimizer_ = optimizer`
  (dual write). Tests green.
- **2c.** Flip the `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch
  (`:160-165`): replace `OptimizePA_Incre optimizer = prev_optimizer_;` with
  `OptimizePA_Incre optimizer = BuildChallengerFromIncumbent();`. Keep
  `UpdateRecords(optimizer, time_limits)` as the commit (still dual-writing
  `prev_optimizer_` from 2b). The challenger is now transient; the diff baseline
  is reconstructed from `res_opt_`. Tests green — this is the load-bearing flip;
  the frozen-baseline tests must still pass because `BuildChallengerFromIncumbent`
  reconstructs the same DAG the old copy carried.

### Phase 3 — Remove `prev_optimizer_` + migrate tests

- **3a.** Migrate the 6 tests reading `prev_optimizer_.*` to read `res_opt_` /
  `has_incumbent_` / `BuildChallengerFromIncumbent()` instead, per the per-test
  spec in `tasks.md`. The 3 DAG-ET assertions rewrite to observe the carried TL
  (§3). Run the tests against the **dual-write** code (Phase 2) — they must pass
  reading the new observables before any field is removed.
- **3b.** Drop the legacy `prev_optimizer_` writes from `SeedStateFromIncumbent`
  (2a) and `UpdateRecords` (2b). Drop the `prev_optimizer_` member and its
  mentions in the header. Drop the `prev_optimizer_.IfInitialized()` gate in
  `SeedIncumbentBaseline` (`:491`), replacing with `has_incumbent_`. Tests green.
- **3c.** Comment-only fix in `OptimizeSP_Incre.cpp:293-295`: rewrite the
  "UpdateRecords copies this optimizer into `prev_optimizer_`" comment to
  describe the throwaway-challenger / `res_opt_`-carries-the-adopted-TL model.
- **3d.** Re-evaluate the P1.1 descent-start-TL workaround at
  `OptimizeIncre_w_TL:387` and the cold-start at `ReOptimizePeriodic:535`. Under
  the redesign both should derive from `BuildChallengerFromIncumbent` /
  `CommitIncumbent` by construction; record keep-vs-simplify in `dev_log.md`.
  **Re-derive, do not assume** (coding rules).
  - **DECIDED (2026-07-08).** (1) The reopt cold-start at `ReOptimizePeriodic:575`
    (`InitializeTimeLimitsFromETConfig()`) is **NOT a bug** — keep as-is. The
    reopt path uses `from_scratch=true` → `OptimizeFromScratch` (not
    `OptimizeIncre`), so `FindTaskWithDifferentEt`'s diff never runs on it; the
    starting TL is just the search's initial vector, no diff invariant to
    preserve. This overturns §1 symptom 3's "same class of bug on the other
    path" claim (corrected in-place above). (2) The incremental start at
    `OptimizeIncre_w_TL:398` (`ReconstructTimeLimitVecFromResOpt()` + stale-TL
    guard `:411-421`) is **KEPT as-is, NOT folded** into
    `BuildChallengerFromIncumbent`. The call site's vector (update-side descent
    start, guard-applied) and the helper's internal `tl_prev` (baseline
    reconstruction, raw) serve different purposes; folding would drop the guard
    or diverge the baseline. This overturns §6 Q3's "fold" default (corrected
    in-place above). Full reasoning in `dev_log.md`.

### Phase 4 — Verify + close

- `testIncreOpt_w_TL` (44) + `testOptimizeIncrePA` + `ctest` (16) green.
- `make check.SP_OPT -j5` green.
- Re-run the P1.1 probe (INCR_P10, N=8 taskset_0, interval 0→1); confirm `ndiff`
  → ~2 (only gaussian-only tasks flag), reconciled with the corrected ground
  truth. **This probe should pass from P0.5 alone** (the redesign makes the diff
  carry the adopted TL by construction) — no YAML persistence (P0.1) required.
  - **RESULT (2026-07-08): PASS, and stronger than predicted.** `call=0 ndiff=0`
    (was 5 pre-redesign); across all 302 incremental calls `ndiff` is only ever
    0 or 1, never the spurious 5. The 3 perf-pair false positives (tasks 0,1,2)
    vanished. The investigation summary's "ground truth = 2 (tasks 6,7
    gaussian-only)" prediction was itself based on diffing raw Gaussians
    *between intervals*; under the redesign the gaussian-only baseline is
    rebuilt from the *current* `dag_tasks_` each interval, so the inter-interval
    Gaussian drift is no longer in the diff either → `ndiff=0` at call=0, not 2.
    Only a task whose adopted TL actually moved within the current descent flags
    (task 5 at call=1: 23.467→20.35). This is the more-correct semantic. Full
    trace: `simulation_experiments/optimizer_comparison/et_repro/p05_probe_ts0_P10/`.
- Milestone appended to top-level `agents/dev_log.md`.

## 5. Scope boundaries (what this design does NOT do)

- **Does not change the flow shape.** The INCR/REOPT/scratch dispatch
  (`Optimize_w_TL_ScratchOrIncre`), the compare-and-keep guard in `UpdateRecords`
  (strictly-greater SP wins, tie-break lower TL-sum), and the cold-start
  bootstrap in `SeedIncumbentBaseline` all keep their current logic. Whether the
  flow *simplifies* once the incumbent is owned once is a Phase-3d re-derivation,
  not a precondition. The user's design is specifically the **state** refactor.
- **Subsumes the functional TL-init bug (P0.1's core), but not the YAML
  persistence.** By construction `BuildChallengerFromIncumbent` makes both sides
  of `FindTaskWithDifferentEt`'s diff carry the adopted TL → the `ndiff` false
  positives (P1.1 residual) vanish with no YAML write. What remains for P0.1 is
  *inspectability* — the on-disk taskset YAML still shows the Gaussian (generated
  without optimization). That is a debuggability concern, not a runtime bug: the
  orchestrator clamps each job's `execution_time` to `res.id2time_limit` at
  `SimulationOrchestrator.cpp:461-463`, so a perf-pair task's runtime ET is
  governed by the adopted TL, not the Gaussian; the `setExecutionTime(GetAvgValue())`
  reads at `:395`/`:684` only affect gaussian-only (TL=−1) tasks or absent traces.
  P0.1 is therefore demoted to a smaller follow-up (or an optional write site in
  Phase 3d) and is **no longer a predecessor** of P0.5.
- **Does not collapse `ResourceOptResult`'s overlap.** `id2priority` +
  `id2time_limit` + `priority_vec` + `sp_opt` stay as a transport type. The
  overlap with `opt_pa_`/`opt_sp_` is acknowledged (`goal.md` Phase 0 (b)) but
  out of scope — `res_opt_` is the incumbent *store*; thin mirrors in the base
  class are kept for the public surface.
- **Does not touch `PriorityPartialPath`'s per-path copies** (`OptimizeSP_Incre.h:41-42`).
  That is a separate perf concern (deferred to P3.1).
- **Fix C / Fix D are out of scope** (see `goal.md`): the per-variation
  `ObtainSP_DAG` asymmetry and the `FiniteDist::approx_equal` tolerance are not
  revisited here.

## 6. Open questions to confirm before Phase 1 (none blocking design)

These are re-derivation decisions, surfaced per coding rules ("ask users if
you're not certain about design choices"). They do not change the design above;
they affect Phase 3d only.

1. **`opt_pa_` / `opt_sp_` mirror lifetime.** Keep them populated by
   `CommitIncumbent` forever (current public surface reads them via
   `CollectResults`/`opt_pa_`), or eventually route all reads through `res_opt_`
   and remove the mirrors? **Default: keep** (zero public-API change; the design
   says so explicitly).
2. **`has_incumbent_` reset.** Should any path ever set `has_incumbent_ = false`
   again (e.g. an explicit reset for a fresh optimizer), or is it strictly
   one-way false→true for the lifetime of the object? **Default: one-way** — the
   orchestrator constructs a fresh optimizer per INCR_SCRATCH interval, so reset
   is never needed within one object's life.
3. **P1.1 workaround at `:387`.** Keep as-is (it becomes the structural norm
   via `BuildChallengerFromIncumbent`) or fold the `ReconstructTimeLimitVecFromResOpt`
   call entirely into the helper so `OptimizeIncre_w_TL` no longer names it?
   **Default: fold** — the helper owns the reconstruction; the call site should
   not re-name the primitive.

   > **Re-derivation (2026-07-08, Phase 3d) — OVERTURNS the default: KEEP, do
   > not fold.** The call site's `time_limits` and the helper's internal
   > `tl_prev` are **not** the same vector and serve different purposes:
   > - The helper (`BuildChallengerFromIncumbent:510`) uses
   >   `tl_prev = ReconstructTimeLimitVecFromResOpt()` **raw** (no guard) to
   >   build the *baseline* DAG (`UpdateExtDistBasedOnTimeLimit(dag_tasks_,
   >   tl_prev)`).
   > - The call site (`OptimizeIncre_w_TL:398-421`) uses
   >   `ReconstructTimeLimitVecFromResOpt()` **then applies the stale-TL guard**
   >   (`:411-421`, forces −1 when the carried TL is no longer a member of the
   >   current option set) to produce the *update-side* descent start vector.
   >
   > Folding would either drop the guard from the update side (re-introducing
   > the stale-TL-applied-as-point-dist hazard the guard exists to prevent) or
   > push the guard into the helper (making the baseline diverge from the raw
   > reconstruction). They are correctly separate. The call site keeps naming
   > `ReconstructTimeLimitVecFromResOpt()`. (Note: the guard is latent today —
   > `CommitIncumbent` only ever writes a current-option TL — but the asymmetry
   > is a real, intentional separation of baseline-reconstruction vs.
   > descent-start-vector, not a fold candidate.)

These defaults are recorded here so the implementer does not re-litigate them
mid-migration; they can be overturned by user review at Phase 1 sign-off.
