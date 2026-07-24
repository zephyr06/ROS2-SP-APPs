# P1.18 — Dev Log

## 2026-07-23 — Merged Architectural Design for Fine-Grained Reuse Classification

### Analysis & Unification of Classification Rules

The Response Time Analysis (RTA) $R_i$ of task $\tau_i$ on core $C$ under partitioned fixed-priority scheduling depends exclusively on:
1. $\tau_i$'s own execution time distribution $C_i$.
2. The execution time distributions of higher-priority tasks on core $C$ ($hp(\tau_i)$).

Tasks with lower priority than $\tau_i$ on core $C$, and tasks on other cores, have **zero impact** on $\tau_i$'s RTA distribution $R_i$.

### Unified Master Rules

Given a single-task change between candidate and champion solutions on core $C$, let $p_{\min} = \min(\text{old\_pos}, \text{new\_pos})$ and $p_{\max} = \max(\text{old\_pos}, \text{new\_pos})$:

#### 1. Rule A: Task ET Changed (`has_et_diff == true`)
- Applies whenever task $X$'s ET changed (handles both ET-only changes and combined ET + priority moves):
- **Tasks at $pos < p_{\min}$**: Higher priority than task $X$ in both champion and candidate orders. Their $hp$ sets do NOT contain $X$, and their own ETs are unchanged $\implies$ **`FullReuse`**.
- **Tasks at $pos \ge p_{\min}$**: Includes task $X$ and tasks with altered HP-ET convolutions $\implies$ **`NoReuse`** (recomputed via rolling prefix seeded from champion checkpoint at $p_{\min}$).

#### 2. Rule B: Pure Priority Move (`has_et_diff == false`)
- Applies when a single task moves priority position from $\text{old\_pos}$ to $\text{new\_pos}$ on core $C$ with no ET change:
- **Tasks at $pos < p_{\min}$**: Higher priority than the move window. HP sets untouched $\implies$ **`FullReuse`**.
- **Tasks at $p_{\min} \le pos \le p_{\max}$**: Inside the priority shift window. HP sets changed $\implies$ **`NoReuse`**.
- **Tasks at $pos > p_{\max}$**: Lower priority than the move window. Their $hp$ set contains the exact same set of tasks (permuted only within $[p_{\min}, p_{\max}]$).
  - *Mathematical Proof of Reuse*: Execution time distribution convolution is commutative ($C_a \ast C_b = C_b \ast C_a$). Thus, the combined HP-ET convolution for any task at $pos > p_{\max}$ is bit-identical before and after the priority move!
  - **Verdict**: **`FullReuse`**!

### Integration with Optimization Flow
In `OptimizePA_Incre`, the optimization process sequentially alternates between updating time limits / execution times (Rule A) and searching 1D priority variations (Rule B).
During 1D priority searches, Rule B restricts RTA recomputations strictly to the narrow window $[p_{\min}, p_{\max}]$. For adjacent priority swaps ($p_{\max} - p_{\min} = 1$), **only 2 tasks on the core are recomputed**, while all other tasks on that core and all other cores are **`FullReuse`**!

### Next Steps
Awaits implementation signal. All rules, mathematical proofs, and pseudocode are documented in `goal.md` and `tasks.md`.

## 2026-07-23 — Phase 1 LANDED: Rule A (ET Changed) fine-grained reuse

### What shipped
- **`TaskSetDifference::has_et_diff`** added (RTA_Cache.h) — the locator-class fact
  distinguishing Rule A (ET changed) from Rule B (pure priority move). Populated in
  `IsSingleTaskChange` (the one place the ET diff is already computed, via
  `FindTaskWithDifferentEt`); `et_task_id != -1` ⇒ `has_et_diff=true`. Carries
  through `ComputeTaskSetDifference` → `ClassifyReusePerTask` with zero extra calls.
  This is the seam the validator recommended over the goal.md pseudocode's re-derive
  (which would double the ET-bake work).
- **`ClassifyReusePerTask` Rule A branch** (RTA_Cache.cpp): on the changed core,
  `pos < p_min ⇒ FullReuse`, `pos >= p_min ⇒ NoReuse`. Rule B left at the v1-safe
  fallback (whole-core NoReuse) pending Phase 2.
- **`Evaluate` unchanged** — verdict-only change. The existing fold-from-0 recompute
  loop is verdict-agnostic: the `RollPrefix` fold at :487-488 runs unconditionally
  (outside the `if (NoReuse)` block), so narrowing NoReuse to the suffix just changes
  WHICH slots get recompute vs seed-keep. The fold re-derives the `[0,p_min)` prefix
  bit-identically (same ETs, same `RollPrefix` sequence as the oracle), so the prefix
  handed to the first NoReuse recompute matches `champion_.hp_prefix_per_core[core][p_min]`.

### Validator's key findings (driving the design)
- **Rule A is unconditionally safe** — top-reuse (pos < p_min) holds because those
  tasks' HP sets are identical in membership/order/ET between champion and candidate,
  so their champion RTA is bit-identical to the oracle.
- **Rule B bottom-reuse (pos > p_max) is RISKY** — `CompressDistributionWithOnlySize`
  (Probability.cpp) is LOSSY once convolved support > Granularity, and its bucket
  boundaries are ORDER-dependent (it interleaves between convolutions). So the
  goal.md commutativity argument (`A*B = B*A`) is invalidated by the interleaved
  compress. Phase 2 must gate bottom-reuse on a wide-ET differential test and fall
  back to NoReuse on divergence — NOT rely on commutativity.
- **Checkpoint-seeding is perf-only, not correctness.** Shipped verdict-only; the
  fold-from-0 loop reproduces the prefix bit-identically. Defer seeding unless
  profiling shows the prefix re-fold is hot.

### TDD arc
- RED: `ClassifyReusePerTask_TLChangeMiddle_...` + updated `..._SuffixOnChangedCore`
  both failed pre-implementation (v1 marked whole changed core NoReuse → t0 NoReuse).
- GREEN post-implementation. `Evaluate_TLChangeMiddle_BitIdenticalToOracle_RuleA`
  (wide-ET middle-task TL change, support past Granularity) passes — the bit-identity
  gate for the prefix fold.
- Full suite **17/17 ctest green** (incl. `testIncreOpt_w_TL`, `testOptimizeIncrePA`
  which exercise the cache through the optimizer).

### Tests touched
- `testRTA.cpp`: `ClassifyReusePerTask_TLChange_SuffixOnChangedCore` assertions
  updated (t0 NoReuse → FullReuse, intended per Rule A); two new tests on the wide-ET
  fixture (classifier + Evaluate bit-identity).

Phase 1 → Phase 2: implement Rule B's `[p_min,p_max]` window + gate bottom-reuse.
