"""Important-task fixed-priority RTA (P0.8 — seed certification).

Generation-time guarantee that every emitted taskset is schedulable for the
important tasks under the static solution's DM-with-top-priority-lock at the
SEED point (DM-grouped priority assignment + min-TL + WCET). See
``agents/active_tasks/P0_8_important_task_schedulability/goal.md``.

The test is the standard exact fixed-priority response-time recurrence, scoped
to the important group:

    R_i = WCET_i + Σ_{j ∈ important, prio(j) < prio(i), same core} ceil(R_i / period_j) · WCET_j
    schedulable_i  ⇔  R_i ≤ deadline_i

(Non-important tasks sit in the LOWER priority slots — they never interfere
with the important group; that is the point of the P0.6 priority lock. So the
important-group test is self-contained: only higher-priority important tasks on
the same core contribute interference.)

Per-core (D3): the generator partitions tasks to cores by ``processorId``
(best-fit-decreasing); the orchestrator runs one RunQueue per core (P1.7), so a
task on core 0 is NOT interfered by a higher-priority important task on core 1.
The RTA runs within each core's important subset.

WCET (D2): per-task WCET = the max ET that task exhibits across all generated
interval tasksets (the global max — the SAME value P0.6's offline walk uses).
For perf tasks this is the TL-grid upper bound (``period *
FINAL_Et_OVER_PERIOD_RANGE[1]``); for non-perf it is ``execution_time_max``
(= ``et_mean + 2*sigma``). P0.8 uses the SAME WCET fields P0.6 uses (D2
consistency). The caller supplies each task's WCET — this module is a pure RTA,
it does not know how WCET was derived.

Priority order (P0.9): Deadline Monotonic — ``AssignDMRespectingGroupOrder``
semantics. Important tasks occupy the top ``n_important`` priority slots,
DM-ordered (shorter DEADLINE = higher priority) within the important group;
non-important tasks fill the lower slots (DM within their group). The generator
emits constrained deadlines (``deadline = period * U(0.5, 1.0)``), so ``D`` can
be ``< T``; for constrained deadlines DM is the optimal fixed-priority
assignment. This RTA MUST rank by deadline (not period/RM) to match the C++
scheduler's seed PA (``DeadlineMonotonicPriorityVec``) — a divergence would make
the certification hollow. Within the important group the recurrence only ever
looks at higher-priority important tasks, so what matters here is the
DM-within-important ordering on each core. (The non-important group's internal
order is irrelevant to the important-group RTA — they are all lower priority.)

This mirrors the C++ ``RTA_LL`` recurrence (``RTA_LL.h:65-92``) in spirit, NOT in
code: generation is Python; a C++ round-trip per taskset is wasteful and
fragile. The recurrence is a small fixed-point iteration.
"""
import glob
import math
import os

import yaml


def _rta_one_task(
    wcet_i: float,
    period_i: float,
    deadline_i: float,
    hp_periods: list,
    hp_wcets: list,
) -> float:
    """Fixed-point response-time recurrence for ONE important task.

    Mirrors ``RTA_LL::ResponseTimeAnalysisWarm_util_nece`` (``RTA_LL.h:65-92``):
    start from the task's own WCET and iterate

        R^{k+1} = WCET_i + Σ ceil(R^k / period_j) · WCET_j   over HP important tasks

    until it converges (``R^{k+1} == R^k``) or exceeds the deadline (early
    exit — once ``R > deadline`` the task is unschedulable; no point iterating
    further). Returns the converged response time (which may exceed the
    deadline — schedulability is the caller's ``R <= deadline`` check).

    A utilization guard up front (Σ WCET_j/period_j + WCET_i/period_i >= 1) lets
    us short-circuit to unschedulable without iterating, matching the C++
    ``ResponseTimeAnalysisWarm`` guard (``RTA_LL.h:97-100``). Utilization is
    WCET/period (NOT WCET/deadline) — ``Task::utilization()`` in the C++ is
    ``execution_time / period``; a task can be utilization-light yet still miss
    a tight deadline, so the self-term must use period to avoid tripping the
    guard on a short-deadline task that the recurrence would otherwise certify.

    Args:
        wcet_i: this task's WCET.
        period_i: this task's period (for the utilization guard).
        deadline_i: this task's deadline (for the early-exit guard only).
        hp_periods: periods of the higher-priority important tasks on this core.
        hp_wcets: WCETs of the higher-priority important tasks on this core
            (same order as ``hp_periods``).

    Returns:
        The task's worst-case response time. ``math.inf`` if the utilization
        guard trips (overload → never schedulable).
    """
    # Utilization guard: if the important subset alone is overloaded on this
    # core, the recurrence diverges. Short-circuit (C++ returns INT32_MAX).
    # Self-term uses period (utilization), NOT deadline — see docstring.
    if period_i <= 0:
        return math.inf
    util = wcet_i / period_i
    for p, w in zip(hp_periods, hp_wcets):
        if p <= 0:
            return math.inf
        util += w / p
    if util >= 1.0 - 1e-9:
        return math.inf

    # Fixed-point iteration. Seed with the task's own WCET (C++ seeds from
    # beginTime, which the caller passes as the WCET for a cold start).
    r = wcet_i
    loop_count = 0
    while True:
        nxt = wcet_i
        for p, w in zip(hp_periods, hp_wcets):
            nxt += math.ceil(r / p) * w
        if nxt == r:
            return nxt
        # Early exit: once we exceed the deadline, further iteration can only
        # grow R (the recurrence is monotonic non-decreasing in R). Done.
        if nxt > deadline_i:
            return nxt
        r = nxt
        loop_count += 1
        if loop_count > 1500:  # matches C++ RTA_LL safety bound
            return nxt


def _important_priority_order(tasks: list) -> list:
    """Return the important tasks DM-ordered (shorter deadline = higher priority).

    The static solution assigns important tasks the top ``n_important`` priority
    slots, DM-ordered within the group (``AssignDMRespectingGroupOrder`` — the
    P0.9 RM→DM switch). Within the important group, "higher priority" = shorter
    DEADLINE (Deadline Monotonic), ties broken deterministically by the task's
    position in the input list (stable, matches the generator's index-based
    tie-break for ``is_important`` labeling).

    Why deadline, not period (RM): the generator emits constrained deadlines
    (``deadline = period * U(0.5, 1.0)``, ``taskset_generator.py:534``), so
    ``D`` can be ``< T``. For constrained deadlines, Deadline Monotonic is the
    optimal fixed-priority assignment — DM certifies every taskset RM does, plus
    more. The C++ scheduler's seed PA runs the same DM order
    (``DeadlineMonotonicPriorityVec``); this RTA MUST match it or the
    certification is hollow (certifying under DM while the scheduler runs RM —
    the exact silent hole P0.8 exists to prevent).

    Non-important tasks are excluded entirely — they sit in the lower priority
    slots and never interfere with the important group (the priority lock).

    Args:
        tasks: list of task dicts, each carrying ``is_important``, ``period``,
            ``deadline``, ``processorId``, plus whatever else the caller needs
            (the WCET is supplied separately). Order is the generator's task
            order (index = stable tie-break identity).

    Returns:
        The important-task subset, sorted by ascending deadline (DM), ties by
        original index. This IS the priority order: index 0 = highest priority
        within the important group.
    """
    important = [
        (i, t) for i, t in enumerate(tasks) if t.get("is_important", False)
    ]
    # DM: shorter deadline = higher priority. Stable sort keeps input order on
    # ties (deterministic by task index).
    important.sort(key=lambda pair: pair[1]["deadline"])
    return [t for _, t in important]


def important_tasks_schedulable(
    tasks: list,
    wcets: list,
) -> tuple[bool, list]:
    """Per-core fixed-priority RTA for the important group at the seed point.

    For each core, takes that core's important tasks in DM-within-important
    priority order (shorter deadline = higher priority — P0.9) and runs the
    ``R_i`` recurrence over the higher-priority important tasks ON THAT CORE. A
    task on core 0 is NOT interfered by a higher-priority important task on
    core 1 (D3 — matches the orchestrator's one-RunQueue-per-core semantics).

    schedulable  ⇔  for every important task τ_i on every core, R_i ≤ deadline_i.

    Args:
        tasks: list of task dicts in the generator's internal representation.
            Each must carry ``is_important`` (bool), ``period`` (number),
            ``deadline`` (number), and ``processorId`` (int). Order is the
            generator's task order (used as the deterministic tie-break).
        wcets: per-task WCET, parallel to ``tasks`` (``wcets[i]`` is
            ``tasks[i]``'s WCET). The caller computes these per D2 (global-max
            ET across interval tasksets: perf = ``period *
            FINAL_Et_OVER_PERIOD_RANGE[1]``, non-perf = ``execution_time_max``).
            This module is WCET-source-agnostic.

    Returns:
        ``(schedulable, culprits)`` where ``schedulable`` is True iff every
        important task meets its deadline, and ``culprits`` is a list of dicts
        ``{"task_index", "core", "response_time", "deadline"}`` for each
        important task that MISSED (empty when schedulable). Culprits are
        reported per-core so a rejection can be diagnosed (which core / which
        task blew the budget).
    """
    if len(tasks) != len(wcets):
        raise ValueError(
            f"tasks and wcets must be parallel (got {len(tasks)} vs {len(wcets)})"
        )

    # DM-within-important priority order, per core. _important_priority_order
    # gives the global DM order; we then bucket by core preserving that order,
    # so within each core the important tasks are DM-ordered (higher-priority
    # important tasks come first).
    important_ordered = _important_priority_order(tasks)

    # Bucket important tasks by core, preserving the global DM priority order.
    # Within a core, list index 0 = highest priority among that core's important
    # tasks. A task is interfered ONLY by earlier-listed important tasks on the
    # SAME core.
    cores: dict[int, list[tuple[int, dict, float]]] = {}
    # We need the original task index to report culprits and to index wcets.
    # Re-walk the ordered list and recover each task's original index.
    index_by_identity = {}
    # Build a stable identity -> original index map. Tasks are dicts; use
    # id() since the caller passes the same objects (no mutation here).
    for orig_i, t in enumerate(tasks):
        index_by_identity[id(t)] = orig_i
    for t in important_ordered:
        core = int(t.get("processorId", 0))
        orig_i = index_by_identity[id(t)]
        cores.setdefault(core, []).append((orig_i, t, wcets[orig_i]))

    culprits = []
    for core, core_tasks in cores.items():
        # core_tasks: DM-ordered (highest priority first), same core.
        for k, (orig_i, t, wcet_i) in enumerate(core_tasks):
            # Higher-priority important tasks on this core = those before k.
            hp = core_tasks[:k]
            hp_periods = [h[1]["period"] for h in hp]
            hp_wcets = [h[2] for h in hp]
            deadline_i = t["deadline"]
            period_i = t["period"]
            r = _rta_one_task(wcet_i, period_i, deadline_i, hp_periods, hp_wcets)
            if r > deadline_i:
                culprits.append({
                    "task_index": orig_i,
                    "core": core,
                    "response_time": r,
                    "deadline": deadline_i,
                })

    return (len(culprits) == 0, culprits)


# --------------------------------------------------------------------------
# WCET acquisition (Step 2, option A — post-trace, D2-exact)
# --------------------------------------------------------------------------

# The pipeline emits the SAME task in several files per interval:
#   taskset_characteristics_interval_{k}.yaml   — the canonical per-interval
#                                                  full taskset (ALL cores).
#   taskset_characteristics.yaml                — the k=0 backward-compat copy
#                                                  (identical to interval_0).
#   taskset_characteristics_i{k}_p{p}.yaml      — per-processor splits (a
#                                                  subset of the interval file,
#                                                  local id restarts at 0).
# The gate reads the per-interval FULL files ONLY. The global copy is
# redundant with interval_0; the per-processor splits are subsets with
# re-numbered local ids. Reading all three would risk double-counting a task
# (and silently corrupting the WCET if a split were ever stale). The interval
# files are the canonical per-interval source.
_INTERVAL_GLOB = "taskset_characteristics_interval_*.yaml"


def _is_perf_task_emitted(task: dict) -> bool:
    """A task is perf iff it carries ``performance_records_time`` (emitted form).

    Mirrors ``feasibility_clamp._is_perf_task``: the generator emits
    ``performance_records_time`` as a space-separated string for perf tasks
    and omits it (or emits "") otherwise; ``bool("")`` and ``bool(None)``
    are both False, so this one check covers both.
    """
    return bool(task.get("performance_records_time"))


def compute_wcets_from_characteristics(dir_path: str, cfgs: dict) -> dict:
    """Derive each task's WCET from the emitted characteristics YAMLs (D2).

    Reads every ``taskset_characteristics_interval_{k}.yaml`` the pipeline
    emitted (the canonical per-interval full taskset — NOT the global k=0 copy
    or the per-processor splits, which are redundant/subset) and computes,
    per task (keyed by ``gid`` — the generator's stable identity, NOT the
    per-processor-local ``id``):

      - **Perf task** (carries ``performance_records_time``): WCET =
        ``period * FINAL_Et_OVER_PERIOD_RANGE[1]`` — the TL-grid upper bound.
        The on-disk ``execution_time_max`` for a perf task is a TL-grid
        BOUND (the exporter forces it to the config range,
        ``yaml_exporter.py:79-80``), NOT the ET support, so it must NOT be
        read as the WCET. This is the gap ``feasibility_clamp`` leaves open
        (it SKIPS perf tasks) that P0.8 closes.
      - **Non-perf task**: WCET = MAX ``execution_time_max`` across all
        interval YAMLs — the global max ET the task exhibits across the
        generated intervals (D2). Computed POST-clamp (the clamp may pull
        ``execution_time_max`` DOWN, so the gate reads the on-disk clamped
        value — D5: clamp first, then RTA).

    Args:
        dir_path: the generation output directory containing the emitted
            ``taskset_characteristics_interval_*.yaml`` files.
        cfgs: the generation config (read for ``FINAL_Et_OVER_PERIOD_RANGE``;
            presence enforced by ``validate_config_integrity``).

    Returns:
        ``{gid: wcet}`` for every task seen across the interval files. A task
        present in multiple intervals collapses to its global max (non-perf)
        or its single TL-grid bound (perf, identical across intervals).

    Raises:
        ValueError: if no ``taskset_characteristics_interval_*.yaml`` files
            exist in ``dir_path`` (the pipeline did not emit its canonical
            output — the gate cannot certify what isn't there; NEVER silent).
    """
    et_over_period_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    tl_grid_upper = et_over_period_range[1]

    interval_files = sorted(glob.glob(os.path.join(dir_path, _INTERVAL_GLOB)))
    if not interval_files:
        raise ValueError(
            f"No taskset_characteristics_interval_*.yaml files in {dir_path!r} — "
            "the pipeline did not emit its canonical per-interval output; the "
            "important-task gate cannot certify a taskset that isn't there."
        )

    wcets: dict[int, float] = {}
    for fpath in interval_files:
        with open(fpath, "r") as f:
            data = yaml.safe_load(f)
        if data is None or "tasks" not in data:
            continue
        for task in data["tasks"]:
            gid = task.get("gid")
            if gid is None:
                # No gid → not an emitted task (defensive); skip rather than
                # raise — a malformed file is the clamp/exporter's problem.
                continue
            period = float(task["period"])
            if _is_perf_task_emitted(task):
                wcet = period * tl_grid_upper
            else:
                wcet = float(task["execution_time_max"])
            # Per-task global max across intervals (non-perf) or the single
            # TL-grid bound (perf — identical across intervals, so max is a
            # no-op). max() handles both uniformly.
            prev = wcets.get(gid)
            if prev is None or wcet > prev:
                wcets[gid] = wcet

    return wcets
