# Potential Ideas Queue — P1.1 Efficiency Optimizations

This file tracks potential speedup ideas for the incremental optimizer, categorized by their current status and priority.

---

## Active / Under Consideration

### 1. Delta-Thresholding for Task ET Changes (ET-Based Heuristic)
* **Concept**: Skip triggering `OptimizeIncre` if the change in execution time is negligible.
* **Mechanism**: In `FindTaskWithDifferentEt`, instead of comparing the old and updated ET distributions with exact inequality (`!=`), check if the relative/absolute change in their average values (or shape) is above a threshold $\epsilon_{ET}$ (e.g., $1\%$). If the change is below $\epsilon_{ET}$, do not flag the task as modified.
* **Status**: **High Priority** (User Preferred).
* **Expected Impact**: **High**. Prevents minor statistical noise or tiny updates from triggering 1D priority re-searches.

### 2. Incremental and Memoized RTA (Processor-Level & Priority-Level)
* **Concept**: Avoid running full RTA on all processors and all tasks when only a single task configuration or priority changes.
* **Mechanism**:
  * **Processor Isolation**: RTA is completely independent across processors. If a task limit or priority changes on CPU $A$, do not re-evaluate tasks on CPU $B$.
  * **Priority Prefix Reuse**: If task $T_i$'s priority is changed (e.g., during 1D search variations), tasks with priority higher than both the old and new position of $T_i$ are unaffected. Cache the running convolved HP task execution time distribution to warm-start RTA from the first affected priority index.
* **Status**: **High Priority**.
* **Expected Impact**: **High**. Eliminates up to 80-90% of convolutions during 1D search.

### 3. Low-Probability Tail Pruning during Convolution
* **Concept**: Filter out state combinations with negligible probability (e.g., $< 10^{-12}$) before sorting and coalescing.
* **Status**: **Interesting / Under Review**.
* **Expected Impact**: **Low-Medium**. Reduces the number of elements processed by `std::sort` and `Coalesce`.

### 4. Coarse-Grained Processor/Task Pruning (Bottleneck Gating)
* **Concept**: Skip walking coordinate descent for tasks on processors that are already performing perfectly.
* **Mechanism**: If a processor has $100\%$ SP (zero deadline misses among all its tasks) and its tasks do not preempt or impact tasks on other overloaded processors, skip the time limit walk for all tasks on this processor.
* **Status**: **Candidate**.
* **Expected Impact**: **High** on multi-core systems where only a subset of processors is overloaded.

### 5. Skip 1D Priority Re-Search on Stable/Small Walks
* **Concept**: Skip evaluating 1D priority variations if the time limit change is too small to affect the priority order.
* **Mechanism**: During the coordinate-descent walk, if the time limit step is small and does not alter the relative order of task average execution times, the optimal priority assignment is highly likely to remain unchanged. We can directly evaluate the SP using the carried optimal priority assignment, skipping the 1D variation generation and evaluation loop entirely.
* **Status**: **Candidate**.
* **Expected Impact**: **Medium-High**. Eliminates the inner 1D variation evaluation loop for the majority of search steps.

### 6. Cheap Deterministic RTA Pre-Filtering
* **Concept**: Filter out highly unschedulable configurations using a fast, deterministic RTA approximation before running full probabilistic RTA.
* **Mechanism**: Compute a simple Worst-Case Response Time (WCRT) or average response time using task average/max execution times. If this cheap filter predicts a severe deadline miss, skip the expensive probabilistic RTA and assign an SP of $0$ immediately.
* **Status**: **Candidate**.
* **Expected Impact**: **Medium**. Speeds up walks that wander into highly overloaded regions.

### 10. Single-Point (Degenerate) Convolution Fast Path — IMPLEMENTED 2026-07-13
* **Concept**: Special-case `FiniteDist::Convolve` when either operand is a single-point distribution (size 1) — a value-shift plus probability-scale, not a full Cartesian product + sort.
* **Mechanism**: A TL-optimized task's ET distribution is replaced by `GetUnitExecutionTimeDist(time_limit)` (`SP_Metric.cpp:75`, `Probability.h:173`) — a degenerate single-point dist (prob 1.0 at `time_limit`). Convolving dist A (size N) with single-point B `{(v, p)}` yields `{(a.value + v, a.probability * p)}`: **O(N), no sort** (A's sorted order is preserved by a uniform shift). A single-pass adjacent coalesce (also O(N)) matches the general path exactly when an operand carries duplicate values. Implemented at the top of `FiniteDist::Convolve` (`Probability.cpp`): the size-1 check dispatches to the `FiniteDist::ConvolveSinglePoint` member (`Probability.h`), which calls the file-scope free function `ShiftAndCoalesce` — `other.size()==1` shifts+scales `this` in place; the symmetric `else if (distribution.size()==1)` branch copies `other` then shifts+scales; the final `else` `CoutError`s the precondition violation (neither operand single-point). Both branches update `min_time`/`max_time`. The general N×M + sort + coalesce path follows unchanged for the multi×multi case.
* **Status**: **IMPLEMENTED 2026-07-13** (TDD; `make check.SP_OPT -j5` 16/16 green). Coverage: 7 new tests in `tests/testProbability.cpp` — `Convolve_SinglePointOther_ShiftsValues`, `..._Self_ShiftsOther`, `..._Zero_IsIdentity`, `..._ScalesProbability` (p<1 case, the AddOnePreemption trigger), `..._CoalescesDuplicateValues`, `..._PreservesOrderAndProbs` (negative shift), and a differential oracle `..._MatchesShiftReference` (independent shift+scale+sort+coalesce reference across p=1.0 and p<1.0 cases).
* **Correction during TDD**: the first implementation assumed the single-point operand always carried p=1.0 (only `GetUnitExecutionTimeDist`'s case) and dropped the probability scale; this broke the pre-existing `FiniteDist.AddPreemption` test (the preemption tail in `AddOnePreemption` is a single mass point with p<1). Fixed by scaling probabilities by p and adding an adjacent coalesce; the `ScalesProbability` + `CoalescesDuplicateValues` tests pin the corrected contract.
* **Expected Impact**: **Moderate** for tasksets where most tasks carry TLs. **Measured ~2.9–3.0× per-Convolve** at Granularity 5/10/20 (e.g. 99.76 ns → 32.89 ns at Granularity=10) via a micro-benchmark — real, but below the earlier "~10×" doc estimate (revised down). The savings are NOT the `std::sort` (near-free on already-sorted small input) but the eliminated temp allocations (`convolved` + `merged`), the N `emplace_back`s, and the final `std::move`. **End-to-end noticeability unmeasured** (compounds across N convolves/RTA and many RTAs/interval, but per-convolve cost at Granularity=10 is already sub-100 ns — the whole RTA may be dominated by other constant factors). **Zero correctness risk** (shift+scale+coalesce is exact, verified against the general path).

### 11. Incremental RTA Patching Across the 1D Priority-Variation Loop and the TL Walk
* **Concept**: The concrete mechanism for Idea 2 — don't recompute the full N-task RTA when only one task's TL or priority changes; patch only the affected entries.
* **Mechanism**: Two hot loops both change ONE task per step:
  - **1D priority loop** (`OptimizeSP_Incre.cpp:278-286`): each variation moves one task to a new priority position. Tasks at priority above BOTH old and new position have an identical HP set → identical RTA; only the moved task and tasks between old/new positions change.
  - **TL walk** (`OptimizeSingleTaskTimeLimit`, `OptimizeSP_TL_Incre.cpp:194-239`): each step changes one task's TL → only that task's RTA and lower-priority tasks that include it as HP change.
  Compute the baseline RTA vector once (the `opt_sp_` baseline eval already does), then for each candidate patch only the changed entries: reuse the HP-prefix convolution (`hp_tasks_et_conv` in `ProbabilisticRTA_TaskSet_SingleCore`, `RTA.cpp:73-83`) up to the first affected priority, recompute from there. Turns each candidate from O(N) convolves into O(k) where k = affected suffix length.
* **Status**: **High Priority** (new 2026-07-12; specifies Idea 2's mechanism).
* **Expected Impact**: **High**. The 1D loop is the innermost hot loop (up to ~N/2 full RTAs per changed task per TL step); patching collapses each to O(k). The processor-isolation half of Idea 2 is already done at the RTA level (`ExtractTaskSetPerProcessor`, `RTA.cpp:87`); this is the live priority-prefix half.

### 12. Eliminate the Per-Eval DAG/TaskSet Copy in `EvaluateSPWithPriorityVec`
* **Concept**: Each SP eval copies the full DAG + TaskSet just to stamp a priority vector onto the tasks; the copies are avoidable.
* **Mechanism**: `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148-164`) does `UpdateTaskSetPriorities` (TaskSet copy + per-task priority write) then `DAG_Model dag_tasks_eval = dag_tasks` (full DAG copy) then `ObtainSP_DAG`. The 1D-loop variations differ ONLY in priority assignment, not in task content. `ProbabilisticRTA_TaskSet` (`RTA.cpp:100-119`) sorts tasks by their `priority` field — it could instead sort indices by the passed PA vector over a read-only task array, avoiding both copies. With N=6 / Granularity=10 the copy is ~6 `FiniteDist` copies (cheap per call), but it runs once per candidate (hundreds per interval) so the constant adds up.
* **Status**: **Candidate** (new 2026-07-12).
* **Expected Impact**: **Low-Medium**. Removes two O(N) allocations + copies per eval; zero correctness risk; low complexity. Best paired with Idea 11 (both touch the eval hot path).

### 13. Multi-Fidelity (Coarse-Granularity) Search RTA
* **Concept**: Search the TL/priority landscape with a cheaper coarse RTA; re-evaluate only the adopted winner at full fidelity and compare-and-keep.
* **Mechanism**: `Granularity` (`parameters.yaml:6`, =10) bounds every `FiniteDist` support size; convolve cost is ~Granularity². Run the optimizer-internal RTA during the TL walk and 1D loop at a coarser granularity (e.g., 5 → ~4× cheaper convolves), but re-evaluate the adopted config in `UpdateRecords` (`OptimizeSP_TL_Incre.cpp:105-140`) at full granularity before committing, with the existing strictly-greater-SP guard. The coarse landscape only needs to RANK candidates correctly, not produce exact SP; the fine re-eval gates adoption.
* **Status**: **Candidate** (new 2026-07-12).
* **Expected Impact**: **Medium-High** (potentially the largest single lever — convolve cost scales with Granularity²). **Risk**: coarse RTA can mis-rank near-tied configs, causing the search to wander; mitigate by keeping the fine re-eval gate. Medium complexity.

### 14. Cache the Static Time-Limit Option Set Across Intervals
* **Concept**: `RecordTimeLimitOptions` rebuilds the per-task TL option vectors every interval, but the option set (from `timePerformancePairs`) is invariant across intervals.
* **Mechanism**: `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:319`) and `ReOptimizePeriodic` (`:456`) call `RecordTimeLimitOptions(dag_tasks_)` each interval, walking every task's `timePerformancePairs`. Those pairs come from the YAML taskset characterization and don't change as ET distributions update — only the ET Gaussian moves within the fixed `[min,max]` support. Compute the option set ONCE (lazy, keyed on task-count / first interval) and reuse. The walk's `FindTimeLimitOptionIndex` / `baseline_val` logic is unchanged.
* **Status**: **Low Priority** (new 2026-07-12).
* **Expected Impact**: **Low** (O(N × |pairs|) per interval, small), but **zero risk** and trivial. A freebie if touching this area.

### 15. Patience-Bounded Local 1D Priority Search (vs. Exhaustive Enumeration)
* **Concept**: Mirror the TL walk's patience-bounded unidirectional strategy in the priority dimension, instead of evaluating every position in the search half.
* **Mechanism**: `FindPriorityVec1D_Variations` (`OptimizeSP_Incre.h:68`, `OptimizeSP_Incre.cpp:180-212`) generates ALL positions in `[lb, ub]` and `OptimizeIncre` (`:278-286`) evaluates each. Apply the same `IsBetterTimeLimitOption` + patience discipline already in `OptimizeSingleTaskTimeLimit` (`OptimizeSP_TL_Incre.cpp:194-239`): step outward from the current priority position, stop after P consecutive non-improving SP evals. The priority landscape is typically unimodal around the current position (Audsley-ish), so a local walk usually suffices.
* **Status**: **Candidate** (new 2026-07-12). Note: distinct from rejected Idea 8 (binary search on the small TL option set); here the set is the priority RANGE and the lever is patience-bounding, not log-search.
* **Expected Impact**: **Medium**. Halves-to-thirds the 1D eval count when the optimum is near the current position. **Risk**: a non-unimodal SP landscape could hide a better far-out position; the reopt path (patience=1) already tolerates one dip for this reason. Low-medium complexity.

---

## Deferred / Rejected

### 7. Avoiding Sort in Convolution (Sorted Merge)
* **Concept**: Convolving two sorted distributions of size $N$ and $M$ produces a set of values that can be generated/merged in sorted order without using `std::sort` on $N \times M$ elements.
* **Status**: **Deferred** due to unclear relative performance gains compared to the implementation complexity.

### 8. Binary or Exponential Search on Time Limit Options
* **Concept**: Avoid linear scanning of time limit options one by one during coordinate descent.
* **Status**: **Rejected**. The option set for time limits is very small/limited, so binary/exponential search overhead is not helpful.

### 9. RTA Short-Circuiting for Guaranteed Deadline Misses
* **Concept**: Stop convolving preemptions once a task is guaranteed to miss its deadline.
* **Status**: **Rejected (Dangerous)**. Short-circuiting RTA calculations corrupts the actual response time distribution of the task. Even if its own SP is 0, its full, correct distribution is needed to analyze preemptions on lower priority tasks.

---

## Evaluation of existing ideas (code-grounded re-assessment, 2026-07-12)

Hot-path facts established by reading the source (so impact estimates below are
calibrated, not guessed):

- **Per-candidate cost = one RTA.** `EvaluateTimeLimitConfig_ScratchOrIncre`
  → `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148`) → `ObtainSP_DAG`
  (`SP_Metric.cpp:89`) → `ObtainSP_TaskSet` (`:53`) → `ProbabilisticRTA_TaskSet`
  (`RTA.cpp:100`). The generated tasksets have **no cause-effect chains**
  (`chains_` empty: `DAG_Model.h:59` only builds chains when
  `numCauseEffectChain > 0`, and the taskset YAML has no chain config), so
  `GetRTDA_Dist_AllChains` returns immediately and the per-eval cost is purely
  the N-task RTA convolution. `Granularity = 10` (`parameters.yaml:6`) → each
  `FiniteDist` ≤ 10 support points → `Convolve` is ≤ 100 emplaces + sort.
- **TL'd tasks are single-point distributions.** `GetUnitExecutionTimeDist`
  (`Probability.h:173`) replaces a perf-pair task's ET with a single-point dist
  once a TL is applied — yet `Convolve` (`Probability.cpp:72-108`) still runs the
  full N×M + sort, never exploiting the degenerate operand (this is Idea 10).
- **Post-P0.5 the diff flags only the walked task.** Per the P0.5 resolution
  (both diff sides carry the adopted TL), `FindTaskWithDifferentEt`
  (`OptimizeSP_Incre.cpp:140-155`) flags ~1 task per interval, not many — which
  changes the evaluation of Delta-Thresholding (Idea 1).
- **Processor isolation is already done** at the RTA level
  (`ExtractTaskSetPerProcessor`, `RTA.cpp:87-119`); the live half of Idea 2 is
  the priority-prefix reuse, now specified as Idea 11.

Per-idea verdicts:

- **Idea 1 (Delta-Thresholding)** — **Downgrade to Low priority.** The premise
  was "minor ET noise triggers spurious 1D re-searches," but post-P0.5 the diff
  already flags only the genuinely-walked task (`ndiff` 5→0 in the P1.1 probe),
  so there is little spurious-triggering left to suppress. The TL walk changes
  TLs deliberately, not by noise. Keep as a cheap guard against Gaussian-resample
  jitter in the *non-perf* tasks, but it is no longer a High-impact lever.
- **Idea 2 (Incremental/Memoized RTA)** — **Remains High priority; now specified
  by Idea 11.** The processor-isolation half is already implemented; the
  priority-prefix half is the live lever and the biggest correctness-safe win.
- **Idea 3 (Tail Pruning)** — **Keep Under Review; weak.** With Granularity=10
  the distributions are tiny and `std::sort` on ≤100 elements is already cache-
  resident; pruning probabilities < 1e-12 saves little and risks SP drift. Only
  interesting if Granularity is raised (which Idea 13 would lower, not raise).
- **Idea 4 (Bottleneck Gating)** — **Keep Candidate; conditional.** Only helps
  multi-core tasksets where a subset of processors is already at 100% SP. The
  generated tasksets pin tasks across few cores; impact depends on the loaded
  taskset's utilization profile. Cheap to add as a skip-guard.
- **Idea 5 (Skip 1D Re-Search on stable walks)** — **Keep Candidate.** Sound: if
  the TL step doesn't change the ET-mean ordering, the Audsley priority is
  unchanged, so the 1D loop is wasted. Pairs naturally with Idea 11 (the patch
  would be a no-op anyway). Low risk.
- **Idea 6 (Cheap Deterministic RTA Pre-Filter)** — **Keep Candidate.** A
  WCRT/avg-RT filter that assigns SP=0 to grossly overloaded configs skips the
  probabilistic RTA for walks that wander into infeasible regions. Overlaps with
  Idea 13 (multi-fidelity); pick one. Risk: a false-zero on a borderline config.
- **Idea 7 (Sorted Merge)** — **Remains Deferred.** With the single-point fast
  path (Idea 10) the sort disappears for the common case anyway, subsuming much
  of this idea's benefit at lower complexity.
- **Idea 8 (Binary search on TL options)** — **Remains Rejected.** The trial-and-
  error walk already replaced exhaustive window enumeration with a patience-bounded
  unidirectional walk (`OptimizeSingleTaskTimeLimit`); the option set is small and
  the walk stops early. Confirmed correct rejection.
- **Idea 9 (RTA Short-Circuit)** — **Remains Rejected (Dangerous).** Confirmed:
  lower-priority tasks convolve the HP distribution, so a mid-RTA abort corrupts
  downstream RTAs. Correct rejection.

**New ranking (High-impact, correctness-safe, low-complexity first):**
1. ~~Idea 10 (single-point Convolve fast path)~~ — **IMPLEMENTED 2026-07-13**
   (measured ~3× per-Convolve, not the ~10× first estimated; see the Idea 10
   entry above). Biggest per-convolve win, zero risk.
2. Idea 11 (incremental RTA patching — specifies Idea 2's mechanism) — biggest
   per-candidate win.
3. Idea 13 (multi-fidelity coarse search) — potentially largest single lever, but
   needs the fine re-eval gate and mis-ranking risk.
4. Idea 15 (patience-bounded 1D priority walk) — medium, mirrors a proven pattern.
5. Idea 12 (drop per-eval DAG copy) — low-medium, freebie on the hot path.
6. Idea 5 / Idea 4 — conditional skips.
7. Idea 14 (cache TL option set) — trivial freebie.
8. Idea 1 (delta-thresholding) — downgraded; cheap guard only.
