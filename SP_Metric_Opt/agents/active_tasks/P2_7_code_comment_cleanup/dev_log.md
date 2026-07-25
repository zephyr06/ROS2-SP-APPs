# P2.7 — Dev Log

## 2026-07-23 — Task Creation & Comment Policy Definition

### Problem Statement
Iterative modifications by AI agents have introduced bloated, multi-paragraph comments, historical changelog narratives, and embedded task numbers (e.g. `// P1.12 Phase 2b...`) across `RTA_Cache` and `OptimizeSP_TL_Incre`. These make the code noisy, hard to read, and prone to becoming stale.

### Agreed Guiding Rules for Comment Simplification

1. **No Task / Ticket References**: Strip all `P1.x` task IDs from source comments. Task context belongs in commit messages and task logs, not in C++ headers/source files.
2. **Explain "WHY", Not "WHAT"**: Code logic should be self-documenting. Comments must focus on non-obvious domain rationale, invariants, or mathematical bounds.
3. **Conciseness**: Condense long essays into 1–2 crisp lines.
4. **Remove Stale Narratives**: Delete historical notes about fixed bugs or replaced implementations.
5. **Clean Header/Impl Division**: Header files store concise API contracts; implementation files contain minimal inline notes for non-trivial branches.

### Target Files
- `sources/Safety_Performance_Metric/RTA_Cache.{h,cpp}`
- `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`
- `sources/Optimization/OptimizeSP_Incre.{h,cpp}`
- `sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h`

## 2026-07-24 — Phase 1b (audit) + Phase 2a (`RTA_Cache.{h,cpp}`) DONE

### Audit results (1b)
Task-tag counts via `grep -cE 'P[0-9]\.[0-9]'`:
- `RTA_Cache.h` = 6, `RTA_Cache.cpp` = 6 (now 0/0 after 2a)
- `PrioritySwitchAnalysis.h` = 3
- `OptimizeSP_TL_Incre.h` = 13, `OptimizeSP_TL_Incre.cpp` = 24 (heaviest — P1.10/P1.14/P1.25
  changelog narratives)
- `OptimizeSP_Incre.h` = 8, `OptimizeSP_Incre.cpp` = 7

### Phase 2a — `RTA_Cache.{h,cpp}`
- Stripped every task tag. Biggest rewrites: the `champion_` private-member comment block (10-line
  `D1=(b)` / `EvaluateTimeLimitConfig_SubIncremental` changelog → 8 lines stating the
  drift-proof-by-construct invariant + the "raw triple consumed at bake time, not stored" rule) and
  the `ChampionState` docstring (dropped the P1.25 reject-path anecdote, kept the
  "candidate_rta_ is scratch, never needs copying" WHY).
- **Preserved** the high-value WHY comments verbatim: the safe-upper-bound gate rationale in
  `ClassifyReusePerTask` Rule B, the reindex-by-task-id invariant in `Evaluate`, the 3-arg vs 2-arg
  `GetRTA_OneTask` bit-identity reasoning, the `PerCoreOrderFromPa` id==index invariant.
- No code/behavior change. `cmake --build build_test --target check.SP_OPT -j5` → 17/17 ctest green.

### Next
Phase 2b: `OptimizeSP_TL_Incre.{h,cpp}` (37 tags — the heaviest; expect the most narrative
condensing, esp. the P1.10/P1.14/P1.25 changelog blocks in the .cpp).
