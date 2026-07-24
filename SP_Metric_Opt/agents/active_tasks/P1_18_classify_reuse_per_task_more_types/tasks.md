# P1.18 — Tasks (working checklist)

> Correctness-gated extension of the cache's reuse classifier. Bit-identical SP
> to the oracle is the only acceptance gate. TDD-first: pin each new verdict
> with a differential test BEFORE implementing it.

## Phase 0 — Design & Formalization (Completed)

- [x] **0a. Formalize Fine-Grained Classification Rules**
  - Documented Rule A (Task ET Changed: $pos < p_{\min} \rightarrow \text{FullReuse}$, $pos \ge p_{\min} \rightarrow \text{NoReuse}$).
  - Documented Rule B (Pure Priority Move: $pos < p_{\min} \rightarrow \text{FullReuse}$, $p_{\min} \le pos \le p_{\max} \rightarrow \text{NoReuse}$, $pos > p_{\max} \rightarrow \text{FullReuse}$).
- [x] **0b. Write Pseudocode for `ClassifyReusePerTask` & `Evaluate` Dispatch**
  - Documented in `goal.md`.

## Phase 1 — Implementation: Rule A (Task ET Changed: Prefix Reuse)

- [x] **1a. (TDD, RED)** Add differential test in `testRTA.cpp` for ET-changed task on core with $\ge 3$ tasks.
  - `ClassifyReusePerTask_TLChangeMiddle_PrefixFullReuseSuffixNoReuse` (wide-ET 3-task, middle-task TL change: t0→FullReuse, t1/t2→NoReuse). Updated `ClassifyReusePerTask_TLChange_SuffixOnChangedCore` to Rule A (t0→FullReuse).
- [x] **1b. Update `ClassifyReusePerTask` for Rule A**
  - Added `has_et_diff` to `TaskSetDifference` (populated in `IsSingleTaskChange`, the seam). Rule A branch: `pos < p_min → FullReuse, pos >= p_min → NoReuse`. Rule B left at v1-safe fallback (whole-core NoReuse) pending Phase 2.
- [x] **1c. Update `Evaluate` Dispatch**
  - Verdict-only change (no loop change needed): the existing fold-from-0 recompute loop is verdict-agnostic — FullReuse tasks keep their seeded champion RTA while still being folded into the rolling `hp_tasks_et_conv`. Confirmed bit-identical to oracle on wide-ET middle-task change (`Evaluate_TLChangeMiddle_BitIdenticalToOracle_RuleA`). Checkpoint-seeding deferred (perf-only, not correctness).

## Phase 2 — Implementation: Rule B (Pure Priority Move: Window-Bounded Reuse)

- [ ] **2a. (TDD, RED)** Add differential test in `testRTA.cpp` for priority move (no ET change) on core with $\ge 4$ tasks.
  - Assert that tasks at positions $< p_{\min}$ AND tasks at positions $> p_{\max}$ get `FullReuse`.
- [ ] **2b. Update `ClassifyReusePerTask` for Rule B**
  - Assign `NoReuse` strictly to tasks in range $[p_{\min}, p_{\max}]$.
  - Assign `FullReuse` to tasks at $pos < p_{\min}$ and $pos > p_{\max}$.
- [ ] **2c. Update `Evaluate` Dispatch**
  - Skip `GetRTA_OneTask` recomputations for tasks at $pos > p_{\max}$.
  - Pass differential tests.

## Phase 3 — Verification & Benchmarking

- [ ] **3a. Run Full Test Suite (`ctest` & `pytest`)**
  - Confirm 17/17 C++ tests and 308/308 Python tests pass without failure.
- [ ] **3b. Benchmark Recompute Savings in `OptimizePA_Incre`**
  - Measure reduction in `GetRTA_OneTask` call counts during sub-incremental walks.
