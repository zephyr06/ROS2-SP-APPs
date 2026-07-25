# P3.1 Efficiency Optimizations — Dev Log
> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-06
- Trimmed `improve_efficiency.md` to reference-style `goal.md` and moved into
  `active_tasks/P3_1_efficiency_optimizations/`. Stripped the obsolete per-TL
  cache item (B) and the inline `Convolve` code block (C — source is the source
  of truth); kept the two live-but-deferred items (PriorityPartialPath→pointers,
  incremental HP-task convolution) with file/line refs. All live items are
  cross-referenced from `overall_tasks.md`'s Deferred/P3 table. Not yet started.

## 2026-07-10
- Moved the challenger-reuse efficiency item from P0.5 Phase-5 issue 5h into
  here. P0.5 5h was "Reuse a single optimizer instance across
  `EvaluateTimeLimitConfig_ScratchOrIncre` calls instead of rebuilding per
  candidate." — a pure efficiency item (perf, not correctness), so it belongs
  in the deferred P3.1 bucket, not the P0.5 redesign. Added as a third
  deferred item in `goal.md` + `tasks.md` with the trade-off: the current
  rebuild-from-champion design (P0.5 5b, decided 2026-07-10) was chosen OVER
  the persistent challenger because the champion tracks the working TL so the
  diff flags only the one task being walked; a persistent challenger would
  drift the diff baseline to non-adopted candidates and flag extras. Pick up
  only if profiling shows the rebuild is a runtime blocker. P0.5 5h marked
  MOVED in P0.5's `tasks.md`.

## 2026-07-12
- Re-evaluated tasks. Confirmed that "Incremental HP-task convolution" is already fully implemented in `ProbabilisticRTA_TaskSet_SingleCore` via the 3-argument `GetRTA_OneTask` overload, and verified active/correct by unit tests. Moved it to the historical/implemented section in `goal.md` and removed it from `tasks.md`.
- Formulated and appended 4 new efficiency optimization ideas to `goal.md` (Memoized/Incremental RTA, RTA Short-Circuiting for guaranteed misses, Sorted-Merge Convolution, and Low-Probability Tail Pruning).

## 2026-07-12 (brainstorm + evaluation pass)
- Read the live hot path end-to-end to calibrate impact estimates:
  `EvaluateTimeLimitConfig_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:142`)
  → `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148`)
  → `ObtainSP_DAG` (`SP_Metric.cpp:89`) → `ObtainSP_TaskSet` (`:53`)
  → `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`); `FiniteDist::Convolve`
  (`Probability.cpp:72-108`); the 1D variation loop (`OptimizeSP_Incre.cpp:278`).
- Established three facts that reshape the evaluation:
  (1) generated tasksets have **empty `chains_`** (`DAG_Model.h:59`; the taskset
  YAML has no chain config) so `GetRTDA_Dist_AllChains` is a no-op → per-eval cost
  is purely RTA convolution, `Granularity=10`;
  (2) TL'd tasks are **single-point distributions** (`GetUnitExecutionTimeDist`,
  `Probability.h:173`) but `Convolve` never exploits this;
  (3) post-P0.5 the diff flags only the walked task (`ndiff` 5→0) — weakens the
  premise of Delta-Thresholding (Idea 1).
- Appended 6 new ideas to `idea_queue.md`: #10 single-point Convolve fast path,
  #11 incremental RTA patching (specifies Idea 2's mechanism), #12 drop per-eval
  DAG/TaskSet copy, #13 multi-fidelity coarse-granularity search, #14 cache the
  static TL option set across intervals, #15 patience-bounded local 1D priority
  search.
- Added an "Evaluation of existing ideas" section to `idea_queue.md` with
  code-grounded per-idea verdicts. Key re-rankings: **Idea 1 downgraded to Low**
  (little spurious triggering left post-P0.5); **Idea 2 stays High but is now
  specified by Idea 11** (processor isolation already done; priority-prefix reuse
  is the live lever); **Idea 7 still Deferred** (Idea 10's fast path subsumes
  much of it at lower complexity). New High-impact / correctness-safe ranking:
  Idea 10 → 11 → 13 → 15 → 12.
- NOTE: this session was brainstorm + evaluation only — NO source changes, NO
  tests run. All ideas remain deferred (perf, not correctness) per the task scope;
  pick up only if P1.1 finds per-activation ET is a paper blocker.

## 2026-07-13
- Implemented Idea 10 (Single-Point Convolve Fast Path) via TDD. Source:
  `sources/Safety_Performance_Metric/Probability.cpp` `FiniteDist::Convolve` —
  added a degenerate-operand fast path ahead of the general N×M + sort +
  coalesce path. When either operand is a single support point `{(v, p)}`,
  convolution is `{(a.value + v, a.probability * p)}` — O(N), no std::sort
  (uniform shift preserves sorted order), plus an O(N) adjacent coalesce to
  match the general path exactly when an operand carries duplicate values.
  Both `other.size()==1` (shift+scale `this` in place) and
  `distribution.size()==1` (copy `other`, shift+scale) branches handled;
  `min_time`/`max_time` updated. The multi×multi path is unchanged.
- Tests: 7 new characterization + differential tests in
  `tests/testProbability.cpp` (`Convolve_SinglePoint*`). RED→GREEN against the
  slow path first (oracle validated), then the fast path kept them green.
  TDD caught a real bug: the first fast path assumed the single-point operand
  always carried p=1.0 (only `GetUnitExecutionTimeDist`'s case) and dropped the
  probability scale — this broke the pre-existing `FiniteDist.AddPreemption`
  test (the `AddOnePreemption` preemption tail is a single mass point with p<1).
  Fixed by scaling probabilities by p and adding the adjacent coalesce; added
  `Convolve_SinglePointScalesProbability` + `..._CoalescesDuplicateValues` to
  pin the corrected contract.
- Verified: `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` (build dir already DEBUG;
  `libSP_OPTDebug.so`) + `cmake --build . --target check.SP_OPT -j5` →
  16/16 ctest green; new tests confirmed to actually run (not silently skipped).
- Idea 10 marked IMPLEMENTED in `idea_queue.md`. Idea 11 (incremental RTA
  patching — the priority-prefix half of Idea 2) remains the next candidate;
  Idea 10's per-convolve speedup compounds inside Idea 11's patched suffixes.

## 2026-07-13 (measurement + refactor)
- Measured the Idea 10 fast path's actual per-Convolve speedup with a
  micro-benchmark (single-point `other` vs the general path, Granularity 5/10/20).
  Result: **~2.9–3.0× per-Convolve** (e.g. 99.76 ns → 32.89 ns at Granularity=10).
  This is real, but well below the earlier "~10×" estimate in the docs. The
  savings are NOT the `std::sort` (near-free on already-sorted small input) —
  they come from eliminating the two temp allocations (`convolved` + `merged`),
  the N `emplace_back`s, the N `push_back`s, and the final `std::move`. End-to-
  end noticeability is unmeasured; per-convolve cost at Granularity=10 is already
  sub-100 ns, so the whole RTA may be dominated by other constant factors.
- Refactored the Idea 10 code per three code-style corrections from the user:
  (1) removed the anonymous `namespace { }` wrapper — `ShiftAndCoalesce` is now a
  plain file-scope free function, sibling to the existing `near`
  (`Probability.cpp:222`) and `CompressDistributionVector` (`:303`) helpers (no
  new namespace introduced);
  (2) `ConvolveSinglePoint`'s second single-point branch is now an explicit
  `else if (distribution.size() == 1)` (not a bare `else`), and the final
  `else { CoutError(...); }` throws on the precondition violation (neither
  operand single-point) rather than silently doing nothing;
  (3) `ConvolveSinglePoint` is a `FiniteDist` class member function
  (`void FiniteDist::ConvolveSinglePoint(const FiniteDist& other)`, declared in
  `Probability.h` next to `Convolve`) operating on `this` — NOT a free function
  taking `FiniteDist& self`. The user's "independent function" instruction applied
  to `ShiftAndCoalesce`, not `ConvolveSinglePoint`. The `Convolve` dispatch is now
  a plain member call `ConvolveSinglePoint(other);`.
- Rebuilt `cmake --build . --target check.SP_OPT -j5` → 16/16 ctest green after
  the refactor (behavior-preserving; the 7 single-point tests +
  `FiniteDist.AddPreemption` are the safety net).
- Corrected the doc overclaim: `idea_queue.md` Idea 10 entry + ranking, and the
  P1.1 `tasks.md` "Done" entry, now state the measured ~3× (not ~10×) and note
  end-to-end noticeability is unmeasured. Idea 11 remains the next candidate.

