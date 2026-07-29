"""Tests for the important-task RTA (P0.8 — seed certification).

The RTA is the generation-time guarantee that every emitted taskset is
schedulable for the important tasks under the static solution's
DM-with-top-priority-lock at the SEED point (DM-grouped PA + min-TL + WCET).
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
  - the DM-with-top-lock ordering (important tasks occupy the top slots, DM
    within the group — shorter DEADLINE = higher priority; non-important tasks
    are lower priority and never interfere — the point of the priority lock).

Priority model (P0.9): Deadline Monotonic — among important tasks on a core,
the SHORTER-DEADLINE task is higher priority (NOT shorter-period/RM). The
generator emits constrained deadlines (``deadline = period * U(0.5,1.0)``,
``taskset_generator.py:534``) so ``D`` can be ``< T``; for constrained
deadlines DM is the optimal fixed-priority assignment, and it is what the C++
scheduler's seed PA runs (``DeadlineMonotonicPriorityVec``). These tests
therefore construct cases where ``D != T`` so DM and RM orderings DIVERGE —
asserting the RTA ranks by deadline, not period.

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
# D==T here so DM and RM agree on the order. R_1 = WCET_1 (no HP interference)
# = 5 <= deadline 100.  R_2 = WCET_2 + ceil(R_2/period_1)*WCET_1; converges
# well under deadline 200.
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
# Unschedulable: the HP important task interferes enough that the LP important
# task's response time blows past its (tight) deadline. Verifies the recurrence
# detects a miss AND names the culprit (per-task diagnostics so a rejection can
# be diagnosed — which core / which task).
#
# DM-correct construction: the HP task has the SHORTER DEADLINE (20 < 24) so it
# is highest-priority under DM (and also under RM, since its period 50 < 100).
# The LP task's deadline (24) is TIGHT but still > the HP deadline, so DM keeps
# the HP task on top — without this, DM would promote the LP task to HP and the
# miss would vanish.
# ---------------------------------------------------------------------------
def test_unschedulable_taskset_fails_and_names_culprit():
    # HP important task: period 50, deadline 20, WCET 15. Shortest deadline and
    # shortest period -> HP under both DM and RM. R = 15 (no HP) <= 20.
    t_hp = _task(period=50, deadline=20, wcet=15.0)
    # LP important task: period 100, deadline 24, WCET 10. Its response time:
    #   R = 10 + ceil(R/50)*15. R=10 -> 25 -> 10+ceil(25/50)*15=25. Fixed at 25.
    # 25 > deadline 24 -> MISS.
    t_lp = _task(period=100, deadline=24, wcet=10.0)
    tasks = [t_hp, t_lp]
    wcets = [15.0, 10.0]

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
#
# DM construction: both tasks share period 100; DM ranks by deadline, so the
# deadline-30 task is HP, the deadline-40 task is LP. Each converges to EXACTLY
# its deadline (R_hp = 30 == 30; R_lp = 40 == 40) — both boundaries.
# ---------------------------------------------------------------------------
def test_boundary_response_equals_deadline_passes():
    # HP (deadline 30, WCET 30): R = 30 (no HP) == deadline 30. Boundary.
    t_hp = _task(period=100, deadline=30, wcet=30.0)
    # LP (deadline 40, WCET 10): R = 10 + ceil(R/100)*30. R=10 -> 40 -> 40. == 40.
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
# interference). The same taskset on ONE core would miss — confirming the
# per-core scoping is what saves it.
#
# DM-correct: the heavy task (a) has the shorter deadline (50 < 60) and shorter
# period (50 < 100) -> HP under both DM and RM. On one core its interference
# makes (b) miss; on two cores (b) is isolated.
# ---------------------------------------------------------------------------
def test_per_core_isolation_no_cross_core_interference():
    # On one core this taskset is unschedulable (b misses). On two cores it
    # passes because the heavy task (a) lives on core 1 -> no interference to
    # core 0.
    t_a = _task(period=50, deadline=50, wcet=42.0, processor_id=1)   # HP (heavy)
    t_b = _task(period=100, deadline=60, wcet=10.0, processor_id=0)  # LP
    tasks = [t_a, t_b]
    wcets = [42.0, 10.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)

    # Per-core: t_b on core 0 has NO higher-priority important tasks on core 0
    # -> R = 10 <= deadline 60. t_a on core 1 has no HP either -> R = 42 <= 50.
    assert schedulable is True, f"per-core isolation failed: {culprits}"
    assert culprits == []

    # Sanity-check the inverse: same tasks on ONE core must miss (b's R grows
    # past 60 under a's interference). This confirms the pass above is
    # specifically because of per-core scoping, not because the taskset is
    # trivially schedulable.
    #   a (HP): R = 42 <= 50.  b (LP): R = 10 + ceil(R/50)*42 = 94 > 60 -> MISS.
    t_a_same = _task(period=50, deadline=50, wcet=42.0, processor_id=0)
    t_b_same = _task(period=100, deadline=60, wcet=10.0, processor_id=0)
    schedulable_same, culprits_same = important_tasks_schedulable(
        [t_a_same, t_b_same], wcets
    )
    assert schedulable_same is False
    assert any(c["task_index"] == 1 for c in culprits_same)


# ---------------------------------------------------------------------------
# DM-with-top-lock ordering: important tasks occupy the top priority slots,
# DM-ordered within the group (shorter DEADLINE = higher priority). Non-
# important tasks are LOWER priority and must NEVER interfere with the
# important group — that is the point of the priority lock. So a high-
# utilization non-important task with a short deadline must not appear in any
# important task's R_i.
# ---------------------------------------------------------------------------
def test_non_important_tasks_do_not_interfere():
    # Important task: period 200, WCET 20, deadline 60.
    # Non-important task: period 10, deadline 10 (SHORTER than the important
    # task's!), WCET 9 -> 90% util. Under plain DM (or RM) the non-important
    # task would be higher priority and clobber the important task. Under
    # DM-with-top-lock the important task is ABOVE it, so the non-important
    # task never interferes.
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
# DM-within-important ordering: among important tasks on the same core, the
# SHORTER-DEADLINE one is higher priority. This is the discriminating test for
# the P0.9 RM→DM switch — it constructs a case where DM and RM orderings
# DIVERGE (one task has the shorter period, the other the shorter deadline) and
# asserts the RTA ranks by DEADLINE (DM), not period (RM).
#
#   t_a: period 100, deadline 30 (SHORT deadline -> DM highest priority)
#   t_b: period  50, deadline 50 (SHORTER period -> RM highest priority, but
#         LONGER deadline -> DM lower priority)
# Under DM: a is HP. R_a = 10 <= 30; R_b = 25 + ceil(R_b/100)*10 = 35 <= 50
#   -> SCHEDULABLE.
# Under RM (period sort): b is HP. R_b = 25 <= 50; R_a = 10 + ceil(R_a/50)*25
#   = 35 > 30 -> UNSCHEDULABLE (a misses).
# The RTA must use DEADLINE (DM), so it reports schedulable. (Under the old
# period sort this test FAILS — the red that drives the switch.)
# ---------------------------------------------------------------------------
def test_dm_ordering_is_deadline_based_not_period_based():
    t_a = _task(period=100, deadline=30, wcet=10.0, is_important=True)  # DM-HP
    t_b = _task(period=50, deadline=50, wcet=25.0, is_important=True)   # DM-LP
    tasks = [t_a, t_b]
    wcets = [10.0, 25.0]

    schedulable, culprits = important_tasks_schedulable(tasks, wcets)
    assert schedulable is True, (
        f"DM ordering must rank by deadline (not period); got culprits {culprits}"
    )

    # Order-invariant: reverse input -> same DM verdict (sort re-ranks by
    # deadline).
    schedulable_rev, _ = important_tasks_schedulable(
        [t_b, t_a], [25.0, 10.0]
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
