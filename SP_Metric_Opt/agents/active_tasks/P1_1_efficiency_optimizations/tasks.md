# P1.1 — Tasks (working checklist)

> See `goal.md` for scope. **Deferred** — perf only, not correctness. Pick up
> only if P1.1 finds per-activation ET is a paper blocker.

- [ ] (only if unblocked) `PriorityPartialPath` → `const DAG_Model*` /
      `const SP_Parameters*` pointers in `sources/Optimization/OptimizeSP_Incre.h`
- [ ] (only if profiling shows it's a blocker) Reuse one challenger across
      `EvaluateTimeLimitConfig_ScratchOrIncre` calls instead of rebuilding from
      `res_opt_` each candidate (`sources/Optimization/OptimizeSP_TL_Incre.cpp`
      incremental branch `:160` / `BuildChallengerFromIncumbent` `:402`). Moved
      from P0.5 Phase-5 issue 5h on 2026-07-10. See `goal.md` for the trade-off
      (the rebuild-from-champion design was chosen over the persistent
      challenger in P0.5 5b — the diff baseline drift risk makes this risky).
- [ ] Implement Delta-Thresholding for Task ET Changes in `FindTaskWithDifferentEt` (avoid triggering `OptimizeIncre` if the ET distribution's change is below a threshold $\epsilon_{ET}$).
- [ ] Implement Processor-Level and Priority-Level RTA Memoization (isolate RTA evaluations per processor; reuse convolved HP prefix when priority variations are evaluated in 1D search).
- [ ] (Optional) Implement Low-Probability Tail Pruning during convolution (filter out state combinations with probability $< 10^{-12}$ to reduce Convolve/Sort/Coalesce overhead).
- [ ] (Optional) Implement Early Stopping in Coordinate Descent (terminate the task-level walk loop early if $P_{CD}$ consecutive tasks result in zero SP improvement).

## Done

- [x] **Idea 10 — Single-Point (Degenerate) Convolve Fast Path** — DONE
      2026-07-13. `FiniteDist::Convolve` (`sources/Safety_Performance_Metric/Probability.cpp`)
      now dispatches to the `FiniteDist::ConvolveSinglePoint` member (declared in
      `Probability.h`) ahead of the general N×M + sort + coalesce path when either
      operand is a single support point `{(v, p)}`. `ShiftAndCoalesce` is the
      file-scope free function that does the O(N) value-shift + probability-scale
      + adjacent coalesce (no `std::sort` — uniform shift preserves sorted order).
      TDD: 7 new tests in `tests/testProbability.cpp` (`Convolve_SinglePoint*`),
      RED→GREEN against the slow path first. `cmake --build . --target check.SP_OPT -j5`
      → 16/16 ctest green.
      - Refactored per code-style feedback: `ShiftAndCoalesce` is a plain file-scope
        free function (no anonymous namespace, matching the `near` /
        `CompressDistributionVector` convention); `ConvolveSinglePoint` is a
        `FiniteDist` member with an explicit `else if (distribution.size() == 1)`
        branch and a `CoutError` precondition-violation path for the final `else`.
      - **Measured impact**: ~2.9–3.0× per-Convolve at Granularity 5/10/20
        (99.76 ns → 32.89 ns at Granularity=10). Below the earlier ~10× estimate —
        the savings come from eliminating the two temp allocations + N emplace/push
        + final `std::move`, NOT the `std::sort` (near-free on already-sorted small
        input). End-to-end noticeability unmeasured (per-convolve cost is already
        sub-100 ns at Granularity=10). Zero correctness risk.

## Done when
- [ ] Either implemented (with `make check.SP_OPT -j5` green + `ctest` 16/16) or
      explicitly let-go with rationale recorded here
