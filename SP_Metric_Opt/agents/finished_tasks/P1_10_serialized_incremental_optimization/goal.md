# P1.10 — Serialized Single-Task Incremental Optimization

> Architectural redesign of the optimizer's per-interval search loop. **This is
> the "more-important issue" that [[P1.9]] (`RTA_Cache`) is ON HOLD behind** —
> recorded 2026-07-15 by the user. P1.9's cache design is deferred until this
> lands, because (per point 3 below) this redesign guarantees **only one task's
> ET and one task's priority change per SP-eval**, which collapses P1.9's cache
> to the trivially-cacheable single-task-patch case.
>
> **Correctness AND perf** (unlike P1.9 which is perf-only): it reshapes the
> search *order*, so it changes what gets evaluated and when — must remain
> bit-identical-in-spirit to the existing search's *result quality* (the TDD
> gate is "no worse SP than the current incremental search," ideally identical
> on a fixed taskset), not necessarily bit-identical in trace.

---

## The problem (why this exists)

> **CORRECTED 2026-07-15** after a source re-read. The earlier framing claimed
> "OptimizeIncre re-walks every env-changed task on every TL step." That is
> **wrong** — env changes cancel in the diff, so env-changed tasks are never
> flagged at all. The two real gaps are stated below. Full code-grounding in the
> "Code-grounded findings (2026-07-15)" section at the bottom.

The incremental interval pays one `EvaluateTimeLimitConfig_ScratchOrIncre`
(`OptimizeSP_TL_Incre.cpp:142`) at the baseline and one per TL step
(`OptimizeSingleTaskTimeLimit :224`). Inside each, the incremental branch
(`:160`/`:161`) builds a throwaway challenger (`BuildChallengerFromIncumbent`
`:394`) and calls `OptimizeIncre` (`OptimizeSP_Incre.cpp:233`). The diff
`FindTaskWithDifferentEt` (`:242-243`) compares the **challenger's** DAG to
**`dag_tasks_cur`** — and *both derive from the same member `dag_tasks_`*
(the new interval's env DAG, absorbed at `OptimizeIncre_w_TL :313`). The only
difference between them is the TL vector: challenger uses `tl_prev =
ReconstructTimeLimitVecFromResOpt()` (committed best), `dag_tasks_cur` uses the
trial `time_limits`. So:

- **Env changes cancel** — they're in both DAGs → never flagged by
  `FindTaskWithDifferentEt`. Env-changed tasks get **no priority re-search at
  all** in the incremental path; their priorities are frozen at the carried
  `res_opt_.priority_vec` and only re-SCORED (`OptimizeSP_Incre.cpp:238-239`).
- At the **baseline** eval, `time_limits == ReconstructTimeLimitVecFromResOpt()`
  → the diff is **empty** (ndiff==0) → the re-search loop (`:269`) runs zero
  iterations → nothing is optimized, the carried `{pa, tl}` is only re-scored
  and force-committed.
- At each **TL step**, `time_limits[task_idx]` differs from `tl_prev[task_idx]`
  → ndiff==1 (just the walked task) → `:269` re-searches that one task's 1D
  priority. **`|diff|==1` already holds** throughout the TL walk — by
  `BuildChallengerFromIncumbent`'s construction (comment at `:388-393`).

So the two real gaps P1.10 addresses are NOT "redundant multi-task re-walk"
(that doesn't happen) but:

1. **Quality gap (the primary one):** env-changed tasks' priorities are FROZEN
   across intervals — re-scored under the new env but never re-searched. As the
   environment drifts, the carried PA can grow stale; only the periodic
   from-scratch reopt (`ReOptimizePeriodic`) ever restructures it. P1.10's
   Type-E step adds a targeted per-env-changed-task 1D re-search the incremental
   path currently lacks entirely.
2. **Perf gap (secondary):** each TL step today pays a **redundant full-RTA
   re-score** at `OptimizeSP_Incre.cpp:238-239` (the challenger's `opt_sp_` was
   already set to `res_opt_.sp_opt` at `:400`, and its DAG == the committed DAG,
   so `:238-239` reproduces the already-known SP) plus a throwaway challenger
   rebuild (`BuildChallengerFromIncumbent` copies the DAG + re-applies TL). A
   sub-incremental entry that trusts the committed `opt_sp_` and goes straight
   to the one-task 1D search skips that re-score RTA per TL step. (The baseline
   re-score is NOT redundant — it genuinely computes the new env's SP — so the
   saving is per-TL-step, not per-baseline.)

## The new idea — serialize the search around one task at a time

User's design (paraphrased + code-grounded, 2026-07-15). **Note:** the code
re-read above revises the *mechanism* of the win but NOT the design's shape —
the serialized single-task queue is still the right structure; what changes is
what each slot *adds* over the status quo.

**Sub-incremental optimizer.** Introduce a *sub-incremental* optimizer that is
**restricted to exactly one task's ET changing** — it does **one** priority
optimization (the 1D search over that single task's priority position), not the
full multi-task `OptimizeIncre` loop. This is the specialization of
`OptimizeIncre` to `|tasks_with_diff_et| == 1`. **Note:** the TL walk *already*
hits the `|diff|==1` case via `BuildChallengerFromIncumbent`; so for a Type-L
step the sub-incremental is largely an *extraction/simplification* of what
`OptimizeIncre`'s `:269` loop does for one task, not a new behavior. The genuine
**new capability** is the Type-E step (env-changed-task re-search), which the
incremental path has NO analog of today.

**Serialized interval search.** At each new interval:

1. **Build the initial champion** from last interval's optimization result
   (`res_opt_`, the carried incumbent — already the warm-start source via
   `BuildChallengerFromIncumbent`).
2. **Collect the two sets of tasks** the interval surfaces:
   - **Type E — env-changed-ET tasks:** tasks whose base `execution_time_dist`
     moved vs last interval (the env disturbance). These are **NOT** surfaced by
     `FindTaskWithDifferentEt` in the incremental path (env cancels in the diff)
     — they must be computed by a dedicated cross-interval diff of the pre-TL
     DAGs. See OPEN D2.
   - **Type L — TL-flexible tasks:** tasks with `timePerformancePairs`
     (non-`{-1}` option set, from `RecordTimeLimitOptions`), i.e. the tasks
     `PerformCoordinateDescentForTaskConfigOpt` walks today.
3. **Serialize over a merged, heuristically-sorted queue** of these tasks (one
   at a time — needs a **sorting heuristic** to decide order; OPEN D3 — whether
   to reuse `TaskSortingHeuristic` `OptimizeSP_TL_Incre.h:51` or define a new
   one that interleaves E and L types).
4. **For each task in the queue:**
   - **If it's a Type-E (env-changed ET) task:** this is the *new* capability —
     the env moved one task's ET vs the champion, so call the sub-incremental
     optimizer to re-search that one task's priority. The status quo does NOT do
     this (env cancels → frozen priority). This is the primary quality win.
   - **If it's a Type-L (TL-flexible) task:** run the **trial-and-error TL
     strategy already implemented** (`OptimizeSingleTaskTimeLimit`'s
     patience-bounded outward walk), **but with the key distinction**: each TL
     step calls the sub-incremental optimizer (one-task ET change) rather than
     the full `OptimizeIncre` entry — this skips the redundant full-RTA re-score
     at `OptimizeSP_Incre.cpp:238-239` (the perf gap) since the sub-incremental
     trusts the committed `opt_sp_` and goes straight to the 1D search.

**The critical distinction from the existing code** (user's words): *"we assume
all other tasks' ET don't change compared against the champion solution, so we
can also call the sub-incremental optimizer. In this case, we don't perform
repeated incremental optimization to the tasks whose ET is changed by the
environment. This is what I call to serialize the search process. This will also
significantly reduce the number of calls to the SP evaluation."*

**Re-interpreted against the code re-read:** the phrase "we don't perform
repeated incremental optimization to the tasks whose ET is changed by the
environment" reads two ways; the code-grounded reading is: today the
incremental path performs **zero** env-changed-task re-search (frozen
priorities), and the serialized design adds a **single** per-task re-search in
its own queue slot (was 0, becomes 1 — an *addition*, not a *reduction* of an
existing multi-task walk). The "reduce the number of SP evaluations" claim maps
to the **Type-L perf gap**: each TL step today pays the redundant `:238-239`
re-score + challenger rebuild; the sub-incremental short-circuits those. Net
SP-eval count falls on the Type-L side and rises (0→1 per env-changed task) on
the Type-E side — the question of whether the net is a reduction depends on the
TL-option-set sizes vs the env-changed-task count (OPEN — settle empirically in
Phase 2).

## Relationship to P1.9 (the cache) — point 3

User: *"we'll revisit the cache design after we finish the change above. at
that time, we can safely assume that only one task's ET and priority changes
each time, this will simplify our cache design and implementation."*

So **P1.9 stays ON HOLD until P1.10 lands.** After P1.10, the cache's reuse
classification is trivial: every SP-eval changes exactly one task →
`RTAReuseClass` is always `RtaReuse`-except-the-one-task → the patcher path
(P1.9 steps 4/5) reduces to a single-task RTA patch every call. The rev-2
`CheckTaskSetRTAReuse` + the whole `EvaluateRTA_WithCache` surface simplifies
considerably. **Do not resume P1.9 design until P1.10's single-change
invariant is proven in code.**

## Out of scope / non-goals

- **NOT** a free rewrite — the search *result* quality must hold (the gate is
  "SP no worse than the current incremental search on the same taskset," and
  the strong form is bit-identical SP where the serialized walk covers the same
  candidate set). The serialization may change the *order* of exploration, so
  early-termination/patience could in principle change the adopted result — the
  TDD differential is the safety net.
- Does **not** change the generator, the simulator, or the SP metric (analytic
  RTA). Pure optimizer-search-architecture change.
- Does **not** touch `OptimizePA_BF` / `OptimizeSP_TL_BF` (the brute-force
  oracle path stays as the differential oracle).
- The `ReOptimizePeriodic` from-scratch path (interval 0 + every
  `ReoptimizationPeriod`-th interval) is **out of scope for the first cut** —
  the serialization applies to the **incremental** (`OptimizeIncre_w_TL`)
  branch. Whether reopt should also serialize is OPEN D5.

---

## Extension idea (filed 2026-07-15 — NOT part of the first cut)

> Per the user: *"another idea … to only optimize a subset of tasks in each interval
> optimization. we can add a heuristic to optimize such as only a few tasks in each
> interval, and cycle through different tasks in different interval."*

**Per-interval task subset with cycling.** Instead of serializing over the **full**
E+L queue every interval, select a **subset** of size ≤ K_subset each interval and
**cycle** which subset is optimized across intervals, so every task is re-optimized
over a few intervals rather than all at once. The point is to bound per-interval
SP-eval cost to O(K_subset) instead of O(|E|+|L|), accepting slower per-task response
to env drift (a task that drifts right after being skipped sits stale for one cycle).

- **Subset heuristic (how to pick the K_subset):** top-K_subset by
  `TaskSortingHeuristic` weight (the existing weight-desc/threshold-asc ranking),
  and/or by env-change magnitude for Type-E tasks (biggest ET mover first — those
  benefit most from a re-search). The serialized queue (`BuildSerializedTaskQueue` in
  `api_design.md`) is already a sorted list, so taking the first K_subset is natural.
- **Cycling (how to rotate across intervals):** round-robin / rotating window over
  the full task set — advance a start offset each interval so the subset selected at
  interval i+1 covers tasks not (or least-recently) optimized at interval i. Goal:
  no task goes more than ⌈N / K_subset⌉ intervals without a re-search.
- **Backstop:** `ReOptimizePeriodic`'s from-scratch reopt (every
  `ReoptimizationPeriod`-th interval) already restructures ALL priorities — that
  remains the periodic "cover everything" safety net, so a cycled subset never lets a
  stale task drift indefinitely.
- **Why it fits the serialized design naturally:** the queue (D) is already a sorted
  list; subset = first K_subset with a rotating start offset. The single-change
  invariant (one task's ET per SP-eval) is preserved — a subset is still a sequence of
  single-task steps.
- **Cost trade-off:** per-interval SP-evals drop from O(|E|+|L|) to O(K_subset), but a
  task that drifts in its off-cycle interval is not re-searched that interval (its
  priority stays frozen, the same as the status-quo incremental path's behavior — so
  the extension is strictly ≥ status-quo quality, never worse, and strictly better on
  the optimized subset). Settle the K_subset value empirically.

**Implement AFTER** the core serialized loop lands + the single-change invariant is
proven (Phase 3, `tasks.md`).

> **ELEVATED 2026-07-17 to its own task — [[P1.11]]**
> (`active_tasks/P1_11_partial_task_subset_optimization/`), with the user's
> percentage-based parameterization: a single knob `IncrementalTaskOptimizationPercentage`
> (X in (0,1], 1.0 = current prod behavior bit-identical), a weight-biased +
> fair selection policy (stale-bucket force-include at `CoverageHorizon=⌈1/X⌉`
> → every task re-optimized within `⌈1/X⌉+1` intervals), persistent
> `intervals_since_last_optimized_` state, and a pure `SelectTaskSubset` free
> fn for unit-testability. Full design + D1–D7 open questions in P1.11's
> `goal.md`. The core serialized loop + invariant this builds on are DONE here.

---

## Open design questions (D1–D5) — for the user

These are the points the prompt leaves open; recorded here for a decisions pass
before any code.

- **D1 — sub-incremental API shape.** New method `OptimizeIncre_SingleTask(dag, task_id, et_increased)` (the body of the existing `OptimizeIncre` loop body at `OptimizeSP_Incre.cpp:269-287` extracted to one task), or a `|diff|==1` fast-path inside the existing `OptimizeIncre`? (The loop body is already single-task-shaped — `FindPriorityVec1D_Variations(opt_pa_, task_id, ...)` — so extraction is mechanical.)
- **D2 — Type-E definition + how it's computed.** `FindTaskWithDifferentEt` compares the challenger's DAG to `dag_tasks_cur` — **both derived from the same member `dag_tasks_`** — so env changes CANCEL and are never flagged (verified, see "Code-grounded findings"). Therefore Type-E **cannot** be derived from `FindTaskWithDifferentEt` at all; it must be a **dedicated cross-interval diff of the pre-TL `execution_time_dist`** (compare this interval's `dag_tasks_` to last interval's, before either side applies TLs). The old framing ("the `FindTaskWithDifferentEt` diff, minus TL-induced changes") is wrong — there is no env signal in that diff to subtract from. Open: the exact comparison key (the raw `execution_time_dist` dist object? its `GetAvgValue()`? both?) and where to store last interval's pre-TL DAG for the diff (it's not currently retained — `OptimizeIncre_w_TL :313` overwrites `dag_tasks_` in place).
- **D3 — the sorting heuristic for the merged queue.** Reuse `TaskSortingHeuristic` (weight-desc, threshold-desc — `OptimizeSP_TL_Incre.cpp:9`), or a new heuristic that also weights Type-E vs Type-L ordering (e.g. env-changed first, since they're the uncontrolled disturbance)? The prompt explicitly calls for "a sorting heuristic to decide which task to go through first."
- **D4 — champion mutation between queue steps.** When a queue step adopts a new {pa, tl} for task X, does step X+1 see that adoption as the new champion (sequential compare-and-keep, the current `UpdateRecords`/`CommitIncumbent` pattern), or does each step diff against the *original* interval champion? The prompt says "assume all other tasks' ET don't change compared against the champion solution" — which champion: the interval-start one, or the running adopted one?
- **D5 — reopt path scope.** Apply serialization to `ReOptimizePeriodic` (the from-scratch branch) too, or incremental-only for the first cut?

---

## STATUS: D1 SUB-POINT 1 DECIDED + STAGED 2026-07-16 (code work paused for docs; P1.9 blocked behind this)

**2026-07-16:** D1 sub-point 1 (the redundant-eval removal) DECIDED — generator-level
default `exclude_opt_pa=true` on `FindPriorityVec1D_Variations`, bit-identical for
`OptimizeIncre`. Staged in the working tree (`OptimizeSP_Incre.h:75` / `.cpp:180`) but
**NOT built/tested/committed** — code work paused per the user ("first update all design
decisions into related task md file"); the 4 unit tests + `ctest` 16/16 gate are pending.
The D1 *main* question (new method vs fast-path) and D2/D3/D4/dedup/D5 remain open.

**2026-07-15:** premise corrected + design recorded (no code written then). **2026-07-15 premise correction:** a source re-read (the
"Code-grounded findings" section) showed the original framing's central claim —
"the incremental path re-walks every env-changed task on every TL step" — was
**factually wrong**: env changes cancel in `FindTaskWithDifferentEt`'s diff
(both sides derive from the same `dag_tasks_`), so env-changed tasks are never
flagged, never re-searched; only re-scored with frozen priorities. The design
*shape* (sub-incremental + serialized Type-E/Type-L queue) is unchanged; what
changes is the *win story*: Type-E steps are a **quality addition** (env-changed
re-search the incremental path never had), Type-L steps are a **perf
simplification** (skip the redundant `:238-239` re-score + challenger rebuild).
D2 in particular is rewritten — Type-E cannot come from `FindTaskWithDifferentEt`
at all.

Next = a decisions pass on **D1–D5 (revised)** (user) → TDD plan → implement the
sub-incremental optimizer + the serialized queue → prove the single-change
invariant → **then** unblock P1.9.

---

## Code-grounded findings (2026-07-15 premise correction)

Verified against source before rewriting the problem statement. These are the
load-bearing facts; if any is overturned the premise correction itself must be
revised.

1. **Both diff sides derive from the same `dag_tasks_`.**
   `EvaluateTimeLimitConfig_ScratchOrIncre :145-146` builds
   `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)`.
   `BuildChallengerFromIncumbent :394-402` builds the challenger from
   `dag_with_tl_prev = UpdateExtDistBasedOnTimeLimit(dag_tasks_, tl_prev)` with
   `tl_prev = ReconstructTimeLimitVecFromResOpt()`. Same `dag_tasks_` (the new
   interval's env DAG, absorbed at `OptimizeIncre_w_TL :313`) on both sides →
   any env change is present in BOTH → cancels in
   `FindTaskWithDifferentEt(challenger.dag_tasks_, dag_tasks_cur)`.

2. **At the baseline eval, ndiff == 0.** The baseline call passes
   `starting_time_limits`, which in the incremental path (`OptimizeIncre_w_TL
   :324`/`:329`) is `ReconstructTimeLimitVecFromResOpt()` — **identical** to the
   challenger's `tl_prev`. So `dag_tasks_cur == dag_with_tl_prev` → empty diff →
   `OptimizeIncre`'s `:269` loop runs zero iterations → **nothing optimized**;
   the carried `{pa, tl}` is only re-scored (`:238-239`) and force-committed
   (`UpdateRecords`'s `opt_sp_>-1.0` gate, satisfied because
   `ResetIncumbentBaseline(false)` only sets `opt_sp_=-1.0` at `:441`). The
   INCR-NDIFF-PROBE seam (`OptimizeSP_Incre.cpp:251-267`, debugMode==1)
   empirically confirms `ndiff 5 → 0` after the descent-start-TL fix.

3. **At each TL step, ndiff == 1.** `OptimizeSingleTaskTimeLimit :223` sets
   `time_limits[task_idx]=val` (trial TL, one task), then calls the eval. The
   challenger is rebuilt from the committed `tl_prev`. Diff = just the walked
   task → `:269` re-searches that one task's 1D priority. The comment at
   `:388-393` states this invariant explicitly. **`|diff|==1` already holds
   throughout the incremental TL walk** — the sub-incremental `|diff|==1` case
   is the *existing* TL-step behavior, not a new one P1.10 introduces.

4. **Env-changed tasks are never re-searched in the incremental path.**
   Consequence of (1)+(2)+(3): env changes cancel → never flagged → frozen at
   `res_opt_.priority_vec`, only re-scored. The only place priorities are ever
   re-SEARCHED in the incremental path is the `:269` loop, which only fires on
   the TL-walked task. The periodic reopt (`ReOptimizePeriodic`, `from_scratch`
   branch) does restructure all priorities via `OptimizeFromScratch` beam search,
   but that's the separate reopt path, not the incremental interval.

5. **The redundant re-score (`OptimizeSP_Incre.cpp:238-239`).** At a TL step the
   challenger's `opt_sp_` was just set to `res_opt_.sp_opt` (`:400`) and its DAG
   equals the committed DAG (`tl_prev`), so `:238-239`'s
   `EvaluateSPWithPriorityVec(dag_tasks_cur, ..., opt_pa_)` recomputes an SP the
   search mostly already knows — the genuinely new information at a TL step is
   the *one* task's new ET, which only the `:269` variations consume. A
   sub-incremental that trusts the committed `opt_sp_` and evaluates only the 1D
   variations skips this re-score RTA per TL step. (At the baseline, `:238-239`
   is NOT redundant — it computes the new env's SP, which is the point of the
   baseline.)

   **2026-07-16 D1 sub-point 1 DECIDED — sharper mechanism, same −1 eval/call.**
   The duplicate is more precisely the **carried-position variation** emitted by
   `FindPriorityVec1D_Variations` (`i == old_priority_index`, always in `[lb,ub]`
   for all 3 `PriorityChangeStatus` cases): it reconstructs `opt_pa_` exactly, so
   evaluating it (at `OptimizeSP_Incre.cpp:279-280`) reproduces `:238-239`'s SP_base.
   **Decided mechanism (inverts the sub-incremental's earlier "drop `:238-239`"
   proposal):** give the generator a default `bool exclude_opt_pa = true`
   (`OptimizeSP_Incre.h:75` / `.cpp:180`) that skips emitting the carried-pos
   variation. Keep `:238-239` as the baseline (it is the genuine new-env SP — not
   redundant at baseline); drop the carried-pos variation instead (it duplicates
   `:238-239` at BOTH baseline and TL steps, so always skipping it is uniformly
   correct). **Bit-identical for the existing `OptimizeIncre`** — the carried-pos
   eval gives SP_base and the `:282` adopt test is strict `>` (`if (sp_eval > opt_sp_)`),
   so an equal SP never displaces `opt_sp_`. The production caller at `:273-277`
   inherits the default `true`. Staged in the working tree, NOT built/tested/committed.
   See `dev_log.md` + `api_design.md` 2026-07-16.
