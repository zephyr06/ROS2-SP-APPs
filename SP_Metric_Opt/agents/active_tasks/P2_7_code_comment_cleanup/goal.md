# P2.7 — Code Comment Cleanup & Simplification

## The Goal

Over time, agent-driven modifications have accumulated excessively long, conversational, and redundant comments in C++ source files (e.g. `RTA_Cache.{h,cpp}`, `OptimizeSP_TL_Incre.cpp`, `OptimizeSP_Incre.cpp`). Many comments contain historical task references (`// P1.12 Phase 2b...`, `// P1.21: ...`), multi-paragraph changelog narratives, or step-by-step restatements of obvious code logic.

This task reviews and simplifies code comments across the codebase to ensure they are **concise, high-value, easy to maintain, and free of historical noise**.

---

## Guiding Rules for Comment Simplification

1. **No Task / Ticket References**:
   - **Rule**: Strip all task tags like `// P1.10...`, `// P1.12...`, `// P1.21...`.
   - **Rationale**: Production source code should describe the current domain logic. Task histories belong in git commit messages, PRs, and task logs, not inside C++ files where they quickly become stale.

2. **Explain "WHY", Not "WHAT" (Self-Documenting Code)**:
   - **Rule**: Never restate the code in English (e.g. `// Loop over core_tasks` or `// Set committed_ to true`).
   - **Rationale**: Well-named functions and variables self-document the *what*. Comments should strictly explain non-obvious domain rationale, mathematical invariants, or subtle hardware/algorithmic trade-offs.

3. **Conciseness & Brevity**:
   - **Rule**: Replace multi-paragraph narrative essays with 1–2 crisp, focused lines.
   - **Rationale**: Long comments clutter the editor screen, reduce code density, and increase maintenance overhead when logic changes.

4. **Eliminate Stale & Defensive Changelog Anecdotes**:
   - **Rule**: Remove obsolete notes about previous implementations, bugs fixed long ago, or temporary TDD debug logs.
   - **Rationale**: Code comments represent the *living specification* of the current system, not a history book.

5. **Clean Header vs. Implementation Separation**:
   - **Header (`.h`)**: Concise API contracts (purpose, parameters, preconditions, invariants, throw specs).
   - **Implementation (`.cpp`)**: Short, targeted notes near complex mathematical steps or non-obvious control branches.

---

## Targeted Files for Cleanup

1. `sources/Safety_Performance_Metric/RTA_Cache.h`
2. `sources/Safety_Performance_Metric/RTA_Cache.cpp`
3. `sources/Optimization/OptimizeSP_TL_Incre.h`
4. `sources/Optimization/OptimizeSP_TL_Incre.cpp`
5. `sources/Optimization/OptimizeSP_Incre.h`
6. `sources/Optimization/OptimizeSP_Incre.cpp`
7. `sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h`
