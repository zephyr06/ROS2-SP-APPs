"""Tests for the important-task RTA (P0.8 — seed certification).

The RTA is the generation-time guarantee that every emitted taskset is
schedulable for the important tasks under the static solution's
RM-with-top-priority-lock at the SEED point (RM-grouped PA + min-TL + WCET).
See ``important_task_rta.py`` for the recurrence and
``agents/active_tasks/P0_8_important_task_schedulability/goal.md`` for scope.

These tests construct synthetic tasksets (the generator's internal dict
representation: ``is_important``, ``period``, ``deadline``, ``processorId``,
plus a parallel ``wcets`` list per D2) and assert the recurrence's verdict on:
  - a clearly schedulable taskset (R_i ≪ deadline_i),
  - a clearly unschedulable taskset (one important task's R_i > deadline_i),
  - the boundary R_i == deadline_i (PASS — schedulable is ≤, not <),
  - per-core isolation (a task on core 0 is NOT interfered by a higher-priority
    important task on core 1 — D3),
  - the RM-with-top-lock ordering (important tasks occupy the top slots, RM
    within the group; non-important tasks are lower priority and never
    interfere — the point of the priority lock).

The WCETs are supplied by the caller (per D2: perf = period *
FINAL_Et_OVER_PERIOD_RANGE[1], non-perf = execution_time_max); these tests pass
them directly so the RTA is tested in isolation from WCET derivation.
"""
import math

from Gen_Taskset.lib.important_task_rta import important_tasks_schedulable, _rta_one_task


def _task(
    period: int,
    deadline: int,
    wcet: float,
    is_important: bool = True,
    processor_id: int = 0,
) -> dict:
    """Build one task dict in the generator's internal representation.

    Only the fields the RTA reads are populated (``is_important``, ``period``,
    ``deadline``, ``processorId``); the WCET is supplied via the parallel
    ``wcets`` list, matching how the gate calls the RTA.
    """
    return {
        "is_important": is_important,
        "period": period,
        "deadline": deadline,
        "processorId": processor_id,
    }


# ---------------------------------------------------------------------------
# Schedulable: two important tasks on one core, small WCETs, ample deadline.
# R_1 = WCET_1 (no HP interference) = 5 <= deadline 100.  R_2 = WCET_2 +
# ceil(R_2/period_1)*WCET_1; converges well under deadline 200.
# ---------------------------------------------------------------------------
def test_schedulable_taskset_passes():
    t1 = _task(period=100, deadline=100, wcet=5.0)
    t2 = _task(period=200, deadline=200, wcet=10.0)
    tasks = [t1, t2]
    wcets = [5.0, 10.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    assert schedulable is True
    assert culprits == []


# ---------------------------------------------------------------------------
# Unschedulable: the HP important task alone nearly saturates the core, so the
# LP important task's response time blows past its deadline. Verifies the
# recurrence detects a miss AND names the culprit (per-task diagnostics so a
# rejection can be diagnosed — which core / which task).
# ---------------------------------------------------------------------------
def test_unschedulable_taskset_fails_and_names_culprit():
    # HP important task: period 50, WCET 40 -> 80% util on its own.
    t_hp = _task(period=50, deadline=50, wcet=40.0)
    # LP important task: period 100, WCET 10. Its response time:
    #   R = 10 + ceil(R/50)*40. R=10 -> 10+40=50 -> 10+ceil(50/50)*40=50 -> fixed at 50?
    #   Actually R=50: ceil(50/50)=1 -> 10+40=50. Converges at 50 <= deadline 60. Schedulable!
    # So make the LP deadline tighter to force a miss: deadline 30.
    t_lp = _task(period=100, deadline=30, wcet=10.0)
    tasks = [t_hp, t_lp]
    wcets = [40.0, 10.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    assert schedulable is False
    assert len(culprits) == 1
    c = culprits[0]
    # The culprit is the LP task (the HP task has no interference -> always
    # schedulable here). It is the second task (index 1) on core 0.
    assert c["task_index"] == 1
    assert c["core"] == 0
    assert c["response_time"] > c["deadline"]


# ---------------------------------------------------------------------------
# Boundary: R_i == deadline_i must PASS (schedulable is ≤, not <). This is the
# D6 form P0.6's filter also uses offline; the boundary must not flip to fail.
# ---------------------------------------------------------------------------
def test_boundary_response_equals_deadline_passes():
    # Construct a task whose converged R equals its deadline exactly.
    # HP task: period 100, WCET 30. LP task: WCET 10, period 100, deadline 40.
    #   R = 10 + ceil(R/100)*30. R=10 -> 10+30=40 -> 10+ceil(40/100)*30=40. Fixed at 40 == deadline 40.
    t_hp = _task(period=100, deadline=100, wcet=30.0)
    t_lp = _task(period=100, deadline=40, wcet=10.0)
    tasks = [t_hp, t_lp]
    wcets = [30.0, 10.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    assert schedulable is True, (
        f"boundary R==deadline must pass (≤); got culprits {culprits}"
    )
    assert culprits == []


# ---------------------------------------------------------------------------
# Per-core isolation (D3): a task on core 0 must NOT be interfered by a
# higher-priority important task on core 1. If we put the two important tasks
# on DIFFERENT cores, the LP task's response time is just its own WCET (no HP
# interference), even though the HP task has a shorter period. The same taskset
# on ONE core would miss — confirming the per-core scoping is what saves it.
# ---------------------------------------------------------------------------
def test_per_core_isolation_no_cross_core_interference():
    # On one core this taskset is unschedulable (LP misses). On two cores it
    # passes because the HP task lives on core 1 -> no interference to core 0.
    t_hp = _task(period=50, deadline=50, wcet=40.0, processor_id=1)
    t_lp = _task(period=100, deadline=30, wcet=10.0, processor_id=0)
    tasks = [t_hp, t_lp]
    wcets = [40.0, 10.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    # Per-core: t_lp on core 0 has NO higher-priority important tasks on core 0
    # -> R = 10 <= deadline 30. t_hp on core 1 has no HP either -> R = 40 <= 50.
    assert schedulable is True, f"per-core isolation failed: {culprits}"
    assert culprits == []

    # Sanity-check the inverse: same tasks on ONE core must miss (the LP task's
    # R grows past 30 under HP interference). This confirms the pass above is
    # specifically because of per-core scoping, not because the taskset is
    # trivially schedulable.
    t_hp_same = _task(period=50, deadline=50, wcet=40.0, processor_id=0)
    t_lp_same = _task(period=100, deadline=30, wcet=10.0, processor_id=0)
    schedulable_same, culprits_same = important_tasks_schedulable(
        [t_hp_same, t_lp_same], wcets
    )
    assert schedulable_same is False
    assert any(c["task_index"] == 1 for c in culprits_same)


# ---------------------------------------------------------------------------
# RM-with-top-lock ordering: important tasks occupy the top priority slots,
# RM-ordered within the group (shorter period = higher priority). Non-important
# tasks are LOWER priority and must NEVER interfere with the important group —
# that is the point of the priority lock. So a high-utilization non-important
# task with a short period must not appear in any important task's R_i.
# ---------------------------------------------------------------------------
def test_non_important_tasks_do_not_interfere():
    # Important task: period 200, WCET 20, deadline 60.
    # Non-important task: period 10 (SHORTER than the important task's!), WCET 9
    # -> 90% util. Under plain RM the non-important task would be higher
    # priority and clobber the important task. Under RM-with-top-lock the
    # important task is ABOVE it, so the non-important task never interferes.
    t_imp = _task(period=200, deadline=60, wcet=20.0, is_important=True)
    t_non = _task(period=10, deadline=10, wcet=9.0, is_important=False)
    tasks = [t_imp, t_non]
    wcets = [20.0, 9.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    # The important task has no higher-priority IMPORTANT interference ->
    # R = 20 <= deadline 60. Passes. (If the non-important task interfered,
    # R would be 20 + ceil(20/10)*9 = 20 + 2*9 = 38, still <= 60 but that
    # misses the point — so tighten the important deadline to 25 to make any
    # non-important interference fatal, and assert it still passes.)
    assert schedulable is True
    assert culprits == []

    # Tighter: important deadline 25. With non-important interference R=38>25
    # (miss); without it R=20<=25 (pass). The RTA must report pass.
    t_imp_tight = _task(period=200, deadline=25, wcet=20.0, is_important=True)
    schedulable_tight, culprits_tight = important_tasks_schedulable(
        [t_imp_tight, t_non], wcets
    )
    assert schedulable_tight is True, (
        "non-important task must not interfere with the important group "
        f"(priority lock); got culprits {culprits_tight}"
    )


# ---------------------------------------------------------------------------
# RM-within-important ordering: among important tasks on the same core, the
# shorter-period one is higher priority. Verify the recurrence uses period
# (not task index, not WCET) to rank HP interference by swapping the input
# order — the verdict must be order-invariant (RM sorts them).
# ---------------------------------------------------------------------------
def test_rma_ordering_is_period_based_not_index_based():
    # Two important tasks; give the longer-period one a smaller index. RM must
    # still treat the shorter-period task as higher priority.
    t_long = _task(period=200, deadline=200, wcet=20.0, is_important=True)  # index 0
    t_short = _task(period=50, deadline=50, wcet=10.0, is_important=True)   # index 1
    tasks = [t_long, t_short]
    wcets = [20.0, 10.0]

    # The short task is HP (period 50). The long task's R:
    #   R = 20 + ceil(R/50)*10. R=20 -> 20+10=30 -> 20+ceil(30/50)*10=30. Fixed at 30.
    # 30 <= deadline 200 -> schedulable.
    schedulable, culprits = important_tasks_schedulable(tasks, wcets)
    assert schedulable is True, f"RM ordering failed: {culprits}"

    # Invert input order; same verdict (RM re-sorts).
    schedulable_rev, _ = important_tasks_schedulable(
        [t_short, t_long], [10.0, 20.0]
    )
    assert schedulable_rev is True


# ---------------------------------------------------------------------------
# Utilization guard: if the important subset alone is overloaded on a core
# (Σ WCET/period >= 1), the recurrence diverges. The guard must short-circuit
# to unschedulable (mirrors C++ RTA_LL ResponseTimeAnalysisWarm's util check).
# ---------------------------------------------------------------------------
def test_overloaded_core_is_unschedulable():
    # Two important tasks, combined util > 1 on one core.
    t1 = _task(period=100, deadline=100, wcet=60.0)  # 60%
    t2 = _task(period=100, deadline=100, wcet=50.0)  # 50% -> total 110%
    tasks = [t1, t2]
    wcets = [60.0, 50.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    assert schedulable is False
    # The LP task (index 1) is the one whose recurrence would diverge.
    assert any(c["task_index"] == 1 for c in culprits)


# ---------------------------------------------------------------------------
# No important tasks: trivially schedulable (vacuously). The gate must not
# crash on a taskset with zero important tasks (edge case — the generator
# always marks >= 1 via max(1, ceil(N*ratio)), but the RTA must be robust).
# ---------------------------------------------------------------------------
def test_no_important_tasks_is_vacuously_schedulable():
    tasks = [
        _task(period=100, deadline=100, wcet=10.0, is_important=False),
        _task(period=200, deadline=200, wcet=20.0, is_important=False),
    ]
    wcets = [10.0, 20.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    assert schedulable is True
    assert culprits == []


# ---------------------------------------------------------------------------
# Single important task, no HP interference: R = WCET. Schedulable iff WCET <=
# deadline. Exercises the recurrence's convergence on the trivial fixed point.
# ---------------------------------------------------------------------------
def test_single_important_task_response_equals_wcet():
    t = _task(period=100, deadline=50, wcet=30.0)
    schedulable, culprits = important_tasks_schedulable([t], [30.0])
    assert schedulable is True
    assert culprits == []

    # WCET > deadline -> miss.
    t_miss = _task(period=100, deadline=20, wcet=30.0)
    schedulable_m, culprits_m = important_tasks_schedulable([t_miss], [30.0])
    assert schedulable_m is False
    assert culprits_m[0]["response_time"] == 30.0


# ---------------------------------------------------------------------------
# Length-mismatch defense: tasks and wcets must be parallel.
# ---------------------------------------------------------------------------
def test_length_mismatch_raises():
    import pytest
    with pytest.raises(ValueError):
        important_tasks_schedulable([_task(100, 100, 10.0)], [10.0, 20.0])
