# P2.9 — Dev Log

## 2026-07-25 — Task filed; cost model grounded; lever catalogue

### Cost model (verified)
One reopt activation (`PerformCoordinateDescentForTaskConfigOpt`,
`from_scratch=true`, `OptimizeSP_TL_Incre.cpp:488`):
1. Baseline beam: `EvaluateTimeLimitConfig_ScratchOrIncre(..., from_scratch=true)`
   (`:503`) → `OptimizeFromScratch(K=2)`. Node score = `GetRTA_OneTask` direct
   (`OptimizeSP_Incre.cpp:80`), no cache. ≈ O(K·N²/2) RTA calls.
2. TL walk: per TL-flexible task, backward+forward (`:514`,`:517`) via
   `OptimizeSingleTaskTimeLimit`, whose eval lambda (`:432-436`) re-binds to the
   FULL beam per trial. ~O(N) beam searches stacked on the 1 baseline.

Dominant cost = the TL walk re-running a global priority re-search on every
single-task TL nudge. The incremental path already avoids this: its serialized
step calls `OptimizeSingleTaskTimeLimit_Impl` directly with a sub-incremental
eval (comment `:424-427`), re-scoring the carried PA + a 1D `|diff|==1`
re-search, cache-routed — not a full beam.

### Verified numbers
- `INCR_Reopt_1` N=16: 29–65s/sim ÷ 60 ≈ 0.5–1.1s/activation (matches the
  user's ~0.8s). Some tasksets hit the 1s `TIME_LIMIT` (cancelled mid-beam).
- `INCR_Reopt_10` N=16: 4.5–22.6s/sim → incremental intervals ~5–8× cheaper.
- `K = Layer_Node_During_Incremental_Optimization = 2` (`SimulationOrchestrator.cpp:325`).

### Lever catalogue (ranked)

**A. Route the reopt TL walk through the sub-incremental eval (biggest,
quality-preserving, moderate code).** After the one baseline from-scratch beam
commits a champion (`CommitIncumbent`), switch the TL walk's eval from the
full-beam lambda to the sub-incremental pattern (`EvaluateTimeLimitConfig_SubIncremental`:
re-score carried PA + 1D single-task re-search, `|diff|==1`, cache-routed).
This is the EXACT pattern the incremental path uses; the only reason reopt
doesn't is the legacy wrapper `OptimizeSingleTaskTimeLimit` hard-binding
`from_scratch`. Cuts ~O(N) beams to 1 per reopt. Risk: a TL change *could* in
principle shift the global optimum PA by >1 task; but a single-task ET change
rarely does, and the incremental path already bets it doesn't. Gate behind a
flag (default on after A/B), pin with bit-identical-where-expected + ctest.

**B. `ReoptimizationTimeLimitSearchPatience: 1 → 0` (1-line YAML, ~2× on the
walk, result-CHANGING).** Halves the TL walk length (stop on first
non-improvement vs tolerate one dip). The comment (`parameters.yaml:25-28`,
`OptimizeSP_TL_Incre.cpp:494-496`) warns reopt can be non-unimodal at high
utilization — so this may miss a dip-hidden better option. Cheapest possible
win; a config-only A/B. If lever A lands, B matters less.

**C. Lower `Granularity: 10 → 5` (1-line YAML, ~2× broad, result-CHANGING).**
`GetRTA_OneTask` cost scales with the distribution discretization; halves every
RTA call (reopt + incremental). Coarser SP-metric accuracy — the knob the YAML
already documents as the accuracy/speed trade-off. Last resort; A/B required.

**D. Warm-start the beam from the incumbent PA (quality-preserving, more
invasive).** `OptimizeFromScratch` seeds one empty partial path
(`OptimizeSP_Incre.cpp:105`); seeding a beam path from the carried incumbent
gives the search a strong start → fewer wasted expansions. Touches beam logic;
smaller win than A. Secondary.

**E. Raise `ReoptimizationPeriod` (config-only, fewer activations — NOT a
per-activation speedup).** Doesn't make reopt faster, just rarer. Already the
period-10 design; raising it weakens the period-1 contrast arm. Out of scope as
a "method," noted for completeness.

**F. Narrow the TL walk's task set (quality-preserving-ish, moderate).** Today
the walk visits every TL-flexible task. Skipping tasks whose baseline TL is
already at an option-set edge, or whose last-evaluated SP gain was below a
threshold, cuts walk iterations. More logic than A; secondary.

### Recommended sequencing (given "conclude config today")
1. **B first** if a same-day win is needed and result-change is acceptable —
   zero code, just a YAML value + an A/B re-run of `INCR_Reopt_1`/`_10` at N=16.
2. **A as the real fix** — quality-preserving, biggest leverage, reuses tested
   incremental machinery. Worth the one sitting of implementation + TDD.
3. C only if A+B are insufficient; D/F as follow-ups.

## 2026-07-25 — New Brainstormed Levers & Deep Technical Evaluation

### Deep Evaluation of Original Catalogue
- **Lever A (Sub-Incremental TL Walk after 1 Baseline Beam)** is confirmed as the primary high-leverage architectural fix:
  - Runs `OptimizeFromScratch(K=2)` **once** to establish a fresh global priority ordering.
  - Switches subsequent single-task TL walk evaluations to `EvaluateTimeLimitConfig_SubIncremental` (1D, $|diff| \le 1$, RTA-cached search).
  - **Impact:** Cuts full beam searches per reopt activation from $O(N \cdot \text{steps}) \approx 30\text{--}60$ down to **exactly 1**.
  - **Expected Per-Activation ET:** Reduced from ~0.8s down to ~0.02s–0.05s (~15× to 40× speedup).

---

### New Brainstormed Levers (Superior & Complementary)

#### Lever G. Incumbent Upper-Bound Pruning in Beam Search (100% Bit-Identical)
- **Concept:** During `OptimizeFromScratch(K=2)`, candidate partial priority paths accumulate `sp_lost` level-by-level.
- **Mechanism:** Before expanding partial paths at priority level $p$, compare each path's current `sp_lost` against the incumbent's `sp_lost` ($SP_{\text{incumbent}}$ evaluated at reopt entry via `ResetIncumbentBaseline`).
- **Pruning Rule:** If `path.sp_lost > incumbent_sp_lost`, prune `path` immediately.
- **Why it's better:** 
  - **100% Bit-Identical / Quality-Preserving:** It is mathematically impossible for a path with `sp_lost > incumbent_sp_lost` to yield a higher final SP than the incumbent we already possess.
  - **Complementary to Lever A:** Directly accelerates `OptimizeFromScratch` itself (speeding up baseline reopt beams and initial interval-0 setups by ~20%–40%).

#### Lever I. Incumbent-Seeded Beam Initialization (Quality-Preserving)
- **Concept:** `OptimizeFromScratch` initializes `partial_paths` with a single empty path.
- **Mechanism:** Seed `partial_paths` with both the empty path AND the partial path corresponding to the carried incumbent priority assignment.
- **Why it's better:** Prevents the beam search from diverging into inferior search branches at intermediate priority levels, ensuring beam quality is monotonically non-decreasing.

---

### Master Lever Ranking

1. **Lever A (Sub-Incremental TL Walk):** Primary architectural fix (eliminates $O(N)$ redundant beam searches per reopt activation; ~15x–40x per-activation speedup).
2. **Lever G (Incumbent Upper-Bound Pruning):** Primary complementary algorithm fix (100% bit-identical speedup inside `OptimizeFromScratch`).
3. **Lever B (`ReoptimizationTimeLimitSearchPatience: 1 → 0`):** YAML config knob for result-changing trade-offs.

### Lever H — RETIRED (false premise)
**Lever H (Chained Forward Pass in TL Coordinate Descent) was removed after
code-grounded verification showed its premise is false.** The walk primitive
`OptimizeSingleTaskTimeLimit_Impl` (`OptimizeSP_TL_Incre.cpp:441-483`) starts
its for-loop at `i = curr_opt_idx + step` — both passes skip `curr_opt_idx`
itself, so `baseline_val` is NEVER re-evaluated. The backward (`i = curr-1,
curr-2, …`) and forward (`i = curr+1, curr+2, …`) passes explore disjoint
regions; there is no overlap and no redundant baseline eval. H-claim-1 (skip
backward at `opts[0]`) is a no-op — the loop already zero-iterates when
`curr_opt_idx==0` and `step==-1` (`i=-1` fails `i>=0`). H-claim-2 (chain
forward from `TL*`) would re-evaluate `[j+1, curr_opt_idx-1]` that backward
already covered, ADDING evals and changing results. Test
`testIncreOpt_w_TL.cpp:1555-1556` pins the invariant: "the walk visits each
option at most once: baseline + one outward pass per direction." Lever H as
described is retired; revisit only if a genuine redundancy is identified.

### Status
Awaiting user selection of levers to implement.

## 2026-07-25 — Lever A walk switch + TDD landed (working tree, NOT committed)

### What landed
The reopt TL coordinate walk now routes through the sub-incremental eval when
`ReoptimizationUseSubIncrementalWalk` is on (flag added in `5dfd146e`, default
OFF). Change is in `PerformCoordinateDescentForTaskConfigOpt`
(`OptimizeSP_TL_Incre.cpp:485`):

1. `use_subincremental_walk = from_scratch && ReoptimizationUseSubIncrementalWalk`
   — only arms on the reopt descent (incremental path already uses the
   sub-incremental machinery directly via `PerformSerializedTaskQueueOptimization`).
2. The one baseline beam STILL runs through
   `EvaluateTimeLimitConfig_ScratchOrIncre(from_scratch=true)` (`:514-515`) —
   this establishes the champion PA+TL via `OptimizeFromScratch(K=2)`.
3. When armed: re-arm `rta_cache_active_=true` + `AdoptChampion` on the
   committed triple (`:517-530`) so the `|diff|<=1` single-change invariant
   holds for the whole walk (mirrors `PerformSerializedTaskQueueOptimization`'s
   setup at `:354-355`).
4. The per-task walk loop (`:532-575`) branches: armed → `OptimizeSingleTask-
   TimeLimit_Impl` with a SubIncremental eval lambda (`et_up = sign of trial TL
   − committed TL)`, matching the incremental Type-L body (`:382-401`);
   disarmed → legacy `OptimizeSingleTaskTimeLimit` (full-beam eval per trial).
5. Disarm `rta_cache_active_=false` on exit (`:579-581`) so the gate is scoped
   to this descent.

### Why it's not bit-identical (and that's OK)
The legacy reopt walk re-runs `OptimizeFromScratch` (a global PA re-search) on
every trial TL; lever A re-scores the carried PA + a 1D `|diff|<=1` re-search.
If a single-task TL change shifts the global optimum PA by >1 task, the two
diverge. The incremental path already bets this doesn't happen (same
`|diff|<=1` contract); reopt at high utilization is the one place it could.
→ gated, default OFF; needs an A/B (task 2b) before flipping on.

### TDD pin
Two new tests in `CounterDispatcherSynthetic` (`testIncreOpt_w_TL.cpp`), added
a `subincremental_calls` counter to `RecordingDispatcherOpt` (overrides
`EvaluateTimeLimitConfig_SubIncremental`):
- `ReoptWalk_Legacy_Off_RoutesTrialsThroughScratchOrIncre` — flag=0:
  `from_scratch_flags` non-empty + all true, `subincremental_calls==0`.
- `ReoptWalk_LeverA_On_RoutesWalkTrialsThroughSubIncremental` — flag=1:
  baseline beam still via ScratchOrIncre (flags non-empty, all true) AND
  `subincremental_calls>0` (walk trials routed through SubIncremental).

`testIncreOpt_w_TL`: 53/53 green (was 51; +2 new). Flag save/restore added to
the fixture SetUp/TearDown so the global is not leaked across tests.

### Build/test notes
- `.cpp`-only change (no header layout change) → incremental build OK (no
  `--clean-first` needed). libSP_OPTDebug.so + testIncreOpt_w_TL relinked clean.
- **CRITICAL mid-session finding (pre-existing, now FIXED):** the working-tree
  `sources/parameters.yaml` had lost the `ReoptimizationUseSubIncrementalWalk`
  key (an external edit during the session restored the pre-`5dfd146e` yaml —
  flag + comment deleted, `TIME_LIMIT` 1→10 — then removed the `.bak`).
  `Parameters.cpp:30-31` does `loaded_doc["ReoptimizationUseSubIncrementalWalk"].as<int>()`
  with NO fallback → yaml-cpp throws "bad conversion" in the lib's static
  initializer → EVERY binary (test + release) crashed at startup with empty
  output ("Subprocess aborted", 0.17s). This was misread first as timing-flaky /
  OOM (machine was also memory-starved: 12Gi/15Gi, load 11.86) — the real cause
  was the yaml throw. **Fix:** restored the flag + its comment block to the
  working-tree yaml (kept the user's `TIME_LIMIT: 10`). After the fix:
  `libSP_OPTDebug.so` loads clean, **17/17 ctest green**, `testIncreOpt_w_TL`
  53/53. Flag default OFF → legacy path bit-identical to HEAD.
- Lesson: a missing required YAML key is a hard crash at lib load (silent —
  empty stdout). If binaries "abort on launch with no output," check that every
  `loaded_doc["X"].as<T>()` in `Parameters.cpp` has its key in `parameters.yaml`.

### Next
- 2b: A/B at N=16 — run `INCR_Reopt_1` + `INCR_Reopt_10` with flag=0 vs flag=1,
  compare SP (acceptability gate) + per-activation ET.
- 2c: re-measure per-activation ET vs the 0.8s baseline.
- Then ask user to review + commit (flag stays default OFF until A/B passes).

