# P1.18 — Upgrade `ClassifyReusePerTask` to consider more optimization types

## The Goal

`RTACache::ClassifyReusePerTask` currently (v1) emits a coarse per-task verdict:
- **`FullReuse`**: Tasks on a *different* core than the single change, or when $|diff| == 0$.
- **`NoReuse`**: *Every* task on the *same* core as the single change, even if its higher-priority task set ($hp$) was completely untouched or mathematically unchanged!

This task upgrades `ClassifyReusePerTask` and `Evaluate` to exploit fine-grained scheduling reuse properties, collapsed into **two unified master rules**:

1. **Rule A — Task Execution Time (ET) Changed (`has_et_diff == true`)**:
   Applies whenever task $X$'s ET changed (regardless of whether $X$ also moved priority or not):
   - Let $p_{\min} = \min(\text{old\_pos}, \text{new\_pos})$.
   - **Tasks at $pos < p_{\min}$**: Higher priority than task $X$ in both champion and candidate orders. Their $hp$ sets and execution times are 100% untouched ($\rightarrow$ **`FullReuse`**).
   - **Tasks at $pos \ge p_{\min}$**: Includes task $X$ and tasks with altered HP-ET convolutions ($\rightarrow$ **`NoReuse`**).

2. **Rule B — Pure Priority Move (`has_et_diff == false`)**:
   Applies when a single task moves priority position from `old_pos` to `new_pos` on core $C$ with no ET change:
   - Let $p_{\min} = \min(\text{old\_pos}, \text{new\_pos})$ and $p_{\max} = \max(\text{old\_pos}, \text{new\_pos})$.
   - **Top Tasks ($pos < p_{\min}$)**: Higher priority than the move window. HP sets untouched ($\rightarrow$ **`FullReuse`**).
   - **Shift Window Tasks ($p_{\min} \le pos \le p_{\max}$)**: Inside the priority move window. HP sets modified ($\rightarrow$ **`NoReuse`**).
   - **Bottom Tasks ($pos > p_{\max}$)**: Lower priority than the move window. Their $hp$ set contains the exact same set of higher-priority tasks with unchanged ETs (permuted only within the window). Because ET convolution is commutative ($A * B = B * A$), their total HP-ET convolution and response times are **bit-identical** ($\rightarrow$ **`FullReuse`**)!

Since `OptimizePA_Incre` sequentially alternates between updating ETs (Rule A) and searching 1D priority variations (Rule B), this fine-grained classification drastically reduces `GetRTA_OneTask` recomputations per walk step (e.g. adjacent priority swaps recompute **only 2 tasks** on the core).

---

## Unified Classification Algorithm & Pseudocode

### 1. `ClassifyReusePerTask` Algorithm

```python
function ClassifyReusePerTask(dag_tasks, pa, tl):
    result = vector of size N, initialized to NoReuse
    if not HasChampion():
        return result  # All NoReuse

    diff = ComputeTaskSetDifference(dag_tasks, pa, tl)

    # Condition 0: Identity (|diff| == 0)
    if diff.changed_task_id == -1:
        fill result with FullReuse
        return result

    # Default for cross-core tasks: FullReuse
    fill result with FullReuse

    changed_core = diff.core
    cand_order = PerCoreOrderFromPa(dag_tasks, pa)
    core_tasks = cand_order[changed_core]  # tasks in candidate priority order

    old_pos = diff.old_pos
    new_pos = diff.new_pos
    p_min = min(old_pos, new_pos)
    p_max = max(old_pos, new_pos)

    has_et_diff = (FindTaskWithDifferentEt(champion.champ_tasks_baked, cand_tasks_baked).size() > 0)

    # Master Rule A: Task ET Changed (handles ET-only & combined ET+priority move)
    if has_et_diff:
        for pos from 0 to core_tasks.size() - 1:
            task_id = core_tasks[pos]
            if pos < p_min:
                result[task_id] = FullReuse
            else:
                result[task_id] = NoReuse

    # Master Rule B: Pure Priority Move (0 ET change)
    else:
        for pos from 0 to core_tasks.size() - 1:
            task_id = core_tasks[pos]
            if pos >= p_min and pos <= p_max:
                result[task_id] = NoReuse
            else:
                result[task_id] = FullReuse

    return result
```

---

### 2. `Evaluate` Dispatch Integration

```python
function Evaluate(dag_tasks, pa, tl):
    if not HasChampion():
        return Initialize(dag_tasks, pa, tl)

    verdict_per_task = ClassifyReusePerTask(dag_tasks, pa, tl)

    # Step 1: Seed candidate_rta_ with FullReuse values indexed by candidate priority position
    candidate_rta_.resize(champion.rta.size())
    for k from 0 to champion.champ_prioritized.size() - 1:
        tid = champion.champ_prioritized[k].id
        cand_idx = candidate_task_id2index[tid]
        candidate_rta_[cand_idx] = champion.rta[k]

    if no task has NoReuse in verdict_per_task:
        return candidate_rta_  # 100% FullReuse

    # Step 2: Recompute only NoReuse tasks on the changed core
    diff = ComputeTaskSetDifference(dag_tasks, pa, tl)
    p_min = min(diff.old_pos, diff.new_pos)

    # Seed rolling prefix from champion checkpoint at p_min
    hp_tasks_et_conv = champion.hp_prefix_per_core[diff.core][p_min]
    hp_tasks = candidate_tasks_on_core[0 ... p_min - 1]

    for pos from p_min to candidate_tasks_on_core.size() - 1:
        task_curr = candidate_tasks_on_core[pos]
        if verdict_per_task[task_curr.id] == NoReuse:
            candidate_rta_[candidate_task_id2index[task_curr.id]] = GetRTA_OneTask(task_curr, hp_tasks, hp_tasks_et_conv)
        
        # Advance rolling HP prefix for subsequent tasks on this core
        hp_tasks.push_back(task_curr)
        RollPrefix(hp_tasks_et_conv, task_curr.execution_time_dist)

    return candidate_rta_
```

---

## Scope & Non-Negotiable Requirements

1. **Bit-Identical Safety Performance (SP) Output**:
   The refined classification must produce exact, bit-identical SP metrics compared to the full oracle `ProbabilisticRTA_TaskSet`. Any metric divergence is a strict failure.
2. **Zero Code Implementation in this Phase**:
   This task currently specifies the design, pseudocode, and TDD plan. Implementation will proceed after plan approval.
