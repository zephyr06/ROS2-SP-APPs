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

## 2026-07-24 — Phase 2 PROBE: is Rule B bottom-reuse (pos > p_max → FullReuse) safe?

### The open question (deferred from Phase 1)
Rule B's `goal.md` pseudocode marks tasks at `pos > p_max` as `FullReuse`, on the
argument that ET convolution is commutative (`A*B = B*A`) so a permuted HP set
yields a bit-identical HP-ET convolution → bit-identical RTA for the bottom task.
The Phase 1 dev_log flagged this as RISKY: `CompressDistributionWithOnlySize` is
LOSSY and the rolling fold `RollPrefix` (= `Compress(prefix); Convolve(prefix, et)`)
interleaves a lossy `Compress` after each `Convolve`, so the intermediate
compress count / bucket boundaries can depend on fold ORDER. The commutativity
argument only holds if the lossy compress is order-invariant — which is NOT
provable in general (intermediate `Compress(A)⊛B` vs `Compress(B)⊛A` fold
different distributions).

### Structural analysis (why it's not provably invariant)
- `Convolve` is commutative & produces a value-sorted, duplicate-merged dist.
- `Compress` (buffer-based, threshold `1/max_size`) re-buckets by accumulated
  PROBABILITY MASS walking the value-sorted dist. Bucket boundaries depend on the
  dist's value→probability layout.
- `RollPrefix` = `Compress(prefix); Convolve(prefix, et_k)`. So the bottom task's
  HP prefix = `Compress(...Compress(Compress(id)⊛et_0)⊛et_1...) ⊛ et_n`. The
  intermediate `Compress` after each `Convolve` is what breaks commutativity:
  folding `et_0` then `et_1` compresses `Compress(id⊛et_0)=et_0` then
  `Compress(et_0)⊛et_1`; folding `et_1` then `et_0` compresses `et_1` then
  `Compress(et_1)⊛et_0`. `Compress(et_0)⊛et_1` vs `Compress(et_1)⊛et_0` convolve
  DIFFERENT distributions → not provably equal.
- BUT the codebase `operator==` is `approx_equal(tol=1e-1)` — the same gate every
  prior bit-identity test (P1.12/P1.17/Phase 1) uses. So "bit-identical" here
  means within 0.1, not exact-bit.

### Empirical probe (3 fixtures × several move shapes)
Added temporary `PROBE_*` tests in `testRTA.cpp` (NOT regression pins —
informational `cout` + `EXPECT_TRUE(true)`). For each candidate PA (a single
priority move vs champion id-order), computed p_min/p_max and compared the
champion-built RTA vs the oracle-built RTA for every bottom task (pos > p_max):

| fixture | move shape | bottom checks | divergences |
|---|---|---|---|
| 3-task wide-Gaussian (1 core) | swap top two | 1 | 0 |
| 4-task wide-Gaussian (1 core) | middle window + others | 3 | 0 |
| 4-task mixed (point-mass + wide, 1 core) | middle window + others | 3 | 0 |

**Total: 9/9 bottom-task checks MATCH** under `operator==` (0.1 tol), including
the critical middle-window case (`swap t1<->t2`, window [1,2], bottom t3) and the
dissimilar-shape fixture (point-mass + wide Gaussian — the shape most likely to
expose order-dependence, since a point-mass convolve is a pure shift while a wide
convolve grows support past Granularity).

### Verdict (pending user decision)
Empirically safe on every shape probed, but NOT a structural proof — a generated
taskset with an unprobed ET shape could still diverge and silently break the
bit-identity gate. The failure mode is severe (silent SP divergence). This is a
correctness-risk design decision → escalate to user (see tasks.md 2a-decision).

Optimizer context (perf value of Rule B): `FindPriorityVec1D_Variations` moves
ONE task across a RANGE of priority positions (not just adjacent swaps), so a
single move can be a long jump → wide window `[p_min,p_max]`. Wider windows =
more bottom tasks to reuse = more perf win, BUT also more HP tasks permuted =
higher divergence risk. Adjacent swaps (window size 2) recompute only 2 tasks
regardless of bottom-reuse; bottom-reuse only helps on wide-window moves.

## 2026-07-24 — GATE REFRAMED: "safe upper bound" (cached ≥ true), not bit-identity

### The user reframing
The cache's correctness gate is **NOT** "bit-identical to the oracle." It is
"always return a SAFE UPPER BOUND — a per-task RTA whose deadline-miss
probability is ≥ the true (oracle-represented) miss probability." Over-estimating
miss-prob is conservative (lower SP → the maximization optimizer never over-claims
safety); under-estimating is the only bug. This flips the Phase 1 verdict that
bottom-reuse was "RISKY."

### Why bottom-reuse is safe under the reframed gate
1. **The SP metric consumes miss-probability** (`GetDDL_MissProbability` = tail
   mass above deadline, RTA.cpp:154-169), NOT max_time. "Safe RTA" = a dist whose
   tail-mass-above-deadline ≥ the true tail-mass.
2. **`SP_Func` is monotonically decreasing in miss-prob** (SP_Metric.h:31-41:
   `RewardFunc=log(threshold-vp+1)` decreases; `PenaltyFunc` negative). So higher
   miss-prob → lower SP → conservative direction for the maximization optimizer.
3. **Every RTA op is stochastically conservative** (mass-upward-or-preserving):
   - `Convolve` (Probability.cpp:113-155) — exact, lossless.
   - `CompressDistribution` (335-380) — uses `item.value` = the MAX value in each
     buffer; comment: *"to be conservative — never underestimate."* Moves mass
     upward, never down.
   - `CompressDeadlineMissProbability` (194-207) — lumps all above-deadline mass
     at `deadline+1` (still above deadline) → preserves miss-prob exactly.
   - `AddOnePreemption` (178-193) — convolves the tail (shifts it up by `et_hp`)
     + conservative compress.
4. **Rule B's bottom task keeps the same HP *set***: a pure priority move only
   permutes the HP set *within* `[p_min, p_max]`; membership + ETs unchanged.
   The true RTA depends on the HP SET (convolution is commutative), so the bottom
   task's true RTA is identical between champion and candidate.
5. **Therefore** the champion's RTA for the bottom task is an oracle output for
   the *same HP set* → `champion_RTA ≥ true_RTA`. The candidate's bottom task has
   that same true RTA → `champion_RTA ≥ candidate's true RTA`. Reusing it verbatim
   is a safe upper bound. ✓

### What happens to the order-dependence flagged in Phase 1
`ResolvePreemptionsAndCompress` (RTA.cpp:9-26) IS order-sensitive (per-task job
counters, guard flips on intermediate `max_time`) — but that only affects
*TIGHTNESS*, not *validity*. Different HP orderings produce different upper bounds
on the same order-independent true fixed point, but ALL are ≥ true (by fact 3).
We need `champion ≥ true`, NOT `champion ≥ oracle(candidate)`. The 9/9 probe MATCH
was testing the wrong gate (bit-identity); under safe-upper-bound we don't need a
match at all — the match is bonus evidence the bound happens to be tight.

### The honest caveat
This relies on the oracle being a valid upper bound for *any* HP ordering — a
property of the oracle ITSELF, inherited unchanged by the non-cached path. The
only residual risk (`CompressDeadlineMissProbability` truncating mid-iteration
stopping the fixed-point loop early) is PRE-EXISTING and identical with or
without the cache; caching Rule B bottom-reuse introduces NO new risk, because the
reused value is just an oracle output for the same HP set. If the oracle ever
under-estimates for some ordering, that's an oracle bug to fix at the source —
not a reason to disable bottom-reuse.

## 2026-07-24 — Phase 2 LANDED: Rule B (Pure Priority Move) window-bounded reuse

### What shipped
- **`ClassifyReusePerTask` Rule B branch** (RTA_Cache.cpp): replaced the v1-safe
  fallback (whole changed core NoReuse) with the window-bounded verdict —
  `p_min <= pos <= p_max → NoReuse`, `pos < p_min OR pos > p_max → FullReuse`.
  Comment records the safe-upper-bound argument (not bit-identity).
- **`Evaluate` UNCHANGED** — verdict-only change, same as Phase 1. The existing
  fold already handles bottom-reuse: the FullReuse bottom task keeps its seeded
  (reindexed-by-task-id) champion RTA while the window tasks recompute against
  the rolling prefix; every task's ET is still folded into `hp_tasks_et_conv` so
  a later NoReuse task's HP set is complete. No loop change needed.

### TDD arc
- **RED:** `ClassifyReusePerTask_PriorityMoveMiddle_WindowNoReuseBottomFullReuse`
  (4-task wide-ET, middle-pair swap pa {0,2,1,3}, p_min=1 p_max=2) failed
  pre-implementation (v1 marked whole changed core NoReuse → t0/t3 NoReuse
  instead of FullReuse). GREEN post-implementation.
- **Safety pin:** `Evaluate_PriorityMoveMiddle_BottomReuseSafeUpperBound_RuleB`
  — for every priority-position slot, `GetDDL_MissProbability(rtas_eval[k],
  deadline) >= oracle`. This was GREEN under v1 (t3 recomputed = trivially safe,
  equality) and STAYS GREEN after Rule B (t3 reused → champion value empirically
  dominates the oracle on this fixture). The gate reference is the lossy oracle,
  NOT the unreachable "true" RTA — and the champion-reused value still dominates
  it, a stronger result than the safe-upper-bound-on-true argument requires.

### Tests
- Full suite **17/17 ctest green** + **63/63 testRTA green**. No existing
  `Evaluate_*PriorityMove*` test broke (recomputing a slot the oracle also
  recomputes stays safe-upper-bound).
- Probe tests (`PROBE_RuleB_*`) left in place as informational/diagnostic
  scaffolding (still print MATCH/DIVERGE per move shape, `EXPECT_TRUE(true)`).
  Pruning them is a refactor step deferred to user review.

### Indexing contract confirmed
`UpdateTaskSetPriorities` re-sorts by pa (OptimizeSP_Base.cpp:71), so both
`OracleRtas` and `Evaluate` outputs are **priority-position indexed** (slot k =
task at candidate priority-position k), consumed that way by
`ObtainSP_Full_From_NodeRTAs`. The probe's `rtas_champion[tid] == rtas_oracle[tid]`
only aligned because the bottom task (never moved) keeps id==position; the new
Evaluate test compares at priority-position via `tasks[k].deadline` (identity-pa
fixture → pos k == task k).

Phase 2 → Phase 3: pytest SP-regression check + benchmark recompute savings.

## 2026-07-24 — Task CLOSED

Rule A (Task ET Changed: prefix reuse) + Rule B (Pure Priority Move: window-
bounded reuse) landed in `RTACache::ClassifyReusePerTask` and committed at
`09d1fca9`. Gate = safe-upper-bound (cached miss-prob >= true), NOT bit-identity;
bottom-reuse (pos > p_max) safe because all RTA ops are stochastically
conservative (Convolve lossless; CompressDistribution uses max value "never
underestimate"; etc.).

Verification:
- 17/17 ctest + 63/63 testRTA green (incl. both new Rule B tests
  `ClassifyReusePerTask_PriorityMoveMiddle_WindowNoReuseBottomFullReuse` +
  `Evaluate_PriorityMoveMiddle_BottomReuseSafeUpperBound_RuleB`).
- 3a2 SUPERSEDED: python unit suite is pure-mock subprocess-stub, cannot observe
  C++ SP — pytest green would prove nothing about Rule B.
- 3b was a perf-only nicety (savings = K−|jump|−1 per priority-move candidate,
  zero on adjacent swaps), deferred to a P1.23-style e2e optimizer-binary run.
  **User ran e2e verification of the optimizer binary 2026-07-24 and results were
  acceptable** — the end-to-end SP signal 3a2 said was required is now supplied.
  Formal recompute-count instrumentation not measured (deferred indefinitely; not
  a gate).

All gates satisfied. Task moved to `finished_tasks/`.

