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

- [x] **2a. (TDD, RED)** Add differential test in `testRTA.cpp` for priority move (no ET change) on core with $\ge 4$ tasks.
  - `ClassifyReusePerTask_PriorityMoveMiddle_WindowNoReuseBottomFullReuse` (4-task wide-ET, middle-pair swap pa {0,2,1,3}, p_min=1 p_max=2: t0→FullReuse, t1/t2→NoReuse, t3→FullReuse). Pinned RED pre-impl (v1 marked whole core NoReuse → t0/t3 wrong).
  - `Evaluate_PriorityMoveMiddle_BottomReuseSafeUpperBound_RuleB` — the safety pin: for every priority-position slot, `GetDDL_MissProbability(rtas_eval[k], deadline) >= oracle`. Green under v1 (t3 recomputed = trivially safe); stays green after Rule B (t3 reused → champion value dominates oracle). Reference = lossy oracle, NOT the unreachable "true" RTA.
- [x] **2b. Update `ClassifyReusePerTask` for Rule B**
  - Replaced the v1-safe fallback (whole changed core NoReuse) with the window-bounded verdict: `p_min <= pos <= p_max → NoReuse`, `pos < p_min OR pos > p_max → FullReuse`.
- [x] **2c. Update `Evaluate` Dispatch**
  - NO loop change needed — the existing verdict-driven fold (skip FullReuse, recompute NoReuse, fold every task's ET into the rolling prefix) already handles bottom-reuse: the FullReuse bottom task keeps its seeded (reindexed-by-task-id) champion RTA while the window tasks recompute against the rolling prefix. Verified by `Evaluate_PriorityMoveMiddle_BottomReuseSafeUpperBound_RuleB`.

## Phase 2 — COMMITTED `09d1fca9` (2026-07-24, by user)

Rule B (Pure Priority Move) window-bounded reuse landed. Working tree clean vs HEAD; Rule B code + both tests verified present in `HEAD:sources/.../RTA_Cache.cpp` / `HEAD:tests/testRTA.cpp`.

## Phase 3 — Verification & Benchmarking

- [x] **3a. Run Full C++ Test Suite (`ctest`)**
  - 17/17 ctest green + 63/63 testRTA green after Rule B (no existing test broke; the v1 whole-core-NoReuse `Evaluate_*PriorityMove*` tests still pass because recomputing a slot the oracle also recomputes stays bit-identical / safe-upper-bound).
- [x] **3a2. Python suite — SUPERSEDED (cannot observe C++ SP)**
  - Premise was false: the Python unit suite is entirely mock-based (`test_run_sim_experiments.py` stubs `subprocess.run`, 21 mock/patch lines; no test loads `libSP_OPT` or runs a `build_test` binary — grep for build/lib refs in `tests/python/` is empty). The C++ optimizer is reached only via subprocess in the real runner, which the unit tests mock out. ∴ pytest green would prove nothing about Rule B. The C++ `ctest` (17/17 + 63/63, incl. both new Rule B tests) IS the SP-regression gate for the cache path and is green. A real SP-level end-to-end signal requires an A/B run of the optimizer binary (→ folded into 3b).
- [x] **3b. Benchmark Recompute Savings in `OptimizePA_Incre` — CLOSED via e2e (deferred-measurement satisfied)**
  - Was deferred (perf-only nicety, NOT a correctness gate). Savings formula = K − |jump| − 1 per priority-move candidate (zero on adjacent swaps); needed a realistic multi-task/multi-core run to measure. Correctness gate was already met: 17/17 ctest + 63/63 testRTA green. **User ran e2e verification of the optimizer binary and results were acceptable (2026-07-24)** — the end-to-end SP signal 3a2 said was required ("a real SP-level end-to-end signal requires an A/B run of the optimizer binary → folded into 3b") is now supplied. Formal recompute-count instrumentation not measured (deferred indefinitely; not a gate). Task CLOSED.
