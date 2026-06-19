# Idea Queue

This file tracks the proposed ideas for improving the scheduler (specifically INCR) performance, focusing on heuristic-quality improvements with zero or reduced computational overhead.

## Idea 1: Adjacent-Swap Hill Climbing (Heuristic Quality & Speed Improvement)
- **Problem**: Current `FindPriorityVec1D_Variations` removes the changed task and tries reinserting it at all positions in one direction (N/2 candidates). This is expensive and heuristic-directed (may search the wrong direction).
- **Proposed Solution**: Try swapping the changed task with its left and right neighbors. If one improves SP, accept and continue swapping in that direction. Stop when no improvement.
- **Complexity**: O(1) in the best/average case, O(N) in the worst case (number of evaluations: typically 2-3 instead of N/2).
- **Status**: Ready to experiment.

## Idea 2: Fix Beam Search Tiebreaker (Heuristic Quality Improvement)
- **Problem**: When two paths have `sp_lost` within 0.05 tolerance, the tiebreaker assigns low priority to tasks with longer ET. This is backwards for tasks with tight deadlines.
- **Proposed Solution**: Replace with a **schedulability slack** metric: assign low priority to tasks with more relative slack `(deadline - avg_ET) / deadline`.
- **Complexity**: O(1) extra cost (zero additional SP evaluations).
- **Status**: Ready to experiment.

## Idea 3: Reduce Beam Tolerance from 5e-2 to 1e-2 (Heuristic Quality Improvement)
- **Problem**: The `5e-2` tolerance causes many paths to be treated as equal when `sp_lost` is small, causing frequent reliance on the flawed tiebreaker.
- **Proposed Solution**: Reduce tolerance to `1e-2`.
- **Complexity**: O(0) cost.
- **Status**: Ready to experiment.

## Idea 4: Skip Redundant Re-optimization (Overhead Reduction)
- **Problem**: When no task execution times change, the scheduler still executes SP evaluation.
- **Proposed Solution**: Check if `tasks_with_diff_et` is empty and early exit by returning the current optimal priority assignment.
- **Complexity**: Saves O(N) calculations when no changes occur.
- **Status**: Completed.

## Idea 5: Lookahead Adjacent-Swap (Escape Local Optima)
- **Problem**: Adjacent-swap hill climbing can get stuck in local optima if moving a task to its optimal position requires passing through intermediate positions with lower SP.
- **Proposed Solution**: Evaluate 2-step lookahead swaps (swapping two positions ahead). If the 2-step swap improves SP, proceed even if the 1-step swap did not.
- **Complexity**: Slightly higher constant factor (O(2) evaluations per step), but still O(N) worst case.
- **Status**: Proposed.

## Idea 6: Switching Penalty for BR (Dynamic Stability)
- **Problem**: BR runs from scratch and frequently swaps priority orders, creating dynamic preemption backlog transients in simulation.
- **Proposed Solution**: Modify the optimization objective of BR to include a penalty for changing priority assignments relative to the previous interval (e.g. edit distance penalty).
- **Complexity**: O(N!) (same as BR).
- **Status**: Proposed.

## Idea 7: Monotonicity Constraint
- **Problem**: Heuristic updates in INCR can sometimes choose a priority assignment that is worse than the current one due to local heuristic decisions.
- **Proposed Solution**: Always evaluate the previous PA's SP value on the new ET profile. If the optimized PA's SP is lower than the previous PA's SP, reject the optimization and keep the previous PA.
- **Complexity**: 1 extra SP evaluation (negligible).
- **Status**: Proposed.

## Idea 8: Hybrid Scheduler (Dynamic Priority Mode)
- **Problem**: INCR provides stability but may get stuck in local optima, while BR is optimal per-interval but introduces high priority-switching transients.
- **Proposed Solution**: Run INCR as the default scheduler. If a significant shift in task set characteristics (e.g. execution time standard deviation or deadline miss rate exceeds a threshold) is detected, trigger a single BR search to reset the priority assignment.
- **Complexity**: O(N) for INCR in most intervals; occasionally O(N!) when BR is triggered.
- **Status**: Proposed.

## Idea 9: Adaptive Search Depth in INCR
- **Problem**: A fixed 1-step adjacent-swap search scope in INCR might miss multi-position optimizations, while a full search is too costly.
- **Proposed Solution**: Dynamically scale swap search depth based on execution context. Evaluate up to 3-step swaps when deadline miss rate is high, and fall back to 1-step swap when stable.
- **Complexity**: O(d * N) worst case, where depth $d \in \{1, 2, 3\}$.
- **Status**: Proposed.

## Idea 10: Dynamic Switching Threshold (Hysteresis)
- **Problem**: Small fluctuations in task execution times can trigger priority changes, causing preemption overhead and queue backlog transients.
- **Proposed Solution**: Only switch to a newly optimized priority assignment if its expected SP metric improvement over the current assignment exceeds a threshold (e.g., $\Delta SP > 0.05$).
- **Complexity**: O(1) comparison check.
- **Status**: Proposed.

## Idea 11: Priority-Only Optimization (Ablation Method 1: NO_TL)
- **Problem**: Need to isolate the contribution of task configuration (time limit budget) optimization from priority assignment optimization.
- **Proposed Solution**: Fix the task execution time budgets (time limits) at their maximum default limits, and optimize only the priority assignments.
- **Complexity**: Bypasses the coordinate descent/enumeration search over budget levels, reducing overhead.
- **Status**: Completed (implemented as `INCR_NO_TL` and `BR_NO_TL`).

## Idea 12: Deterministic WCET Modeling (Ablation Method 2: WCET)
- **Problem**: Need to measure the benefit of exploiting statistical execution time distributions versus designing for pessimistic Worst-Case Execution Times (WCET).
- **Proposed Solution**: Override task execution time distributions to a constant value equal to their WCET (`execution_time_max`), and simulate/optimize under this deterministic model.
- **Complexity**: Zero variance distribution reduces SP-Metric RTA calculation complexity to deterministic checks.
- **Status**: Completed (implemented as `INCR_WCET` and `BR_WCET`).
