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
